#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.15);
    trajectory_timeout_ = declare_parameter<double>("trajectory_timeout", 0.50);
    lookahead_base_ = declare_parameter<double>("lookahead_base", 0.30);
    lookahead_speed_gain_ = declare_parameter<double>("lookahead_speed_gain", 0.8);
    lookahead_min_ = declare_parameter<double>("lookahead_min", 0.30);
    lookahead_max_ = declare_parameter<double>("lookahead_max", 0.70);
    projection_dt_ = declare_parameter<double>("projection_dt", 0.03);
    projection_forward_window_ = declare_parameter<double>("projection_forward_window", 1.0);
    max_linear_accel_ = declare_parameter<double>("max_linear_accel", 0.50);
    max_yaw_accel_ = declare_parameter<double>("max_yaw_accel", 1.0);
    trajectory_frame_ = declare_parameter<std::string>("trajectory_frame", "world");
    visualization_rate_ = declare_parameter<double>("trajectory_visualization_rate", 20.0);
    visualization_dt_ = declare_parameter<double>("trajectory_visualization_dt", 0.10);

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    trajectory_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "planning/bspline_path", rclcpp::QoS(1).reliable().transient_local());
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    last_visualization_time_ = now() - rclcpp::Duration::from_seconds(1.0);
    RCLCPP_DEBUG(get_logger(), "Closed-loop controller ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  double findProjectionTime()
  {
    const double step = std::max(0.005, projection_dt_);
    const double begin = projection_initialized_ ? projected_t_ : 0.0;
    const double end = projection_initialized_
        ? std::min(traj_duration_, projected_t_ + projection_forward_window_)
        : traj_duration_;
    double best_t = begin;
    double best_distance = std::numeric_limits<double>::infinity();
    for (double t = begin; t <= end + 1e-9; t += step)
    {
      const Eigen::Vector3d point = traj_[0].evaluateDeBoorT(std::min(t, end));
      const double distance = (point.head<2>() - odom_pos_.head<2>()).squaredNorm();
      if (distance < best_distance)
      {
        best_distance = distance;
        best_t = std::min(t, end);
      }
    }
    // Refine the sampled minimum without allowing the projection to move back.
    double left = std::max(begin, best_t - step);
    double right = std::min(end, best_t + step);
    for (int i = 0; i < 8; ++i)
    {
      const double t1 = (2.0 * left + right) / 3.0;
      const double t2 = (left + 2.0 * right) / 3.0;
      const double d1 = (traj_[0].evaluateDeBoorT(t1).head<2>() -
                         odom_pos_.head<2>()).squaredNorm();
      const double d2 = (traj_[0].evaluateDeBoorT(t2).head<2>() -
                         odom_pos_.head<2>()).squaredNorm();
      if (d1 <= d2) right = t2;
      else left = t1;
    }
    projected_t_ = std::max(begin, 0.5 * (left + right));
    projection_initialized_ = true;
    return projected_t_;
  }

  double findLookaheadTime(double start_t, double distance) const
  {
    const double step = std::max(0.005, projection_dt_);
    double accumulated = 0.0;
    double t = start_t;
    Eigen::Vector3d previous = traj_[0].evaluateDeBoorT(t);
    while (t < traj_duration_ && accumulated < distance)
    {
      t = std::min(traj_duration_, t + step);
      const Eigen::Vector3d current = traj_[0].evaluateDeBoorT(t);
      accumulated += (current.head<2>() - previous.head<2>()).norm();
      previous = current;
    }
    return t;
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    previous_command_ = cmd;
    cmd_vel_pub_->publish(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    projected_t_ = 0.0;
    projection_initialized_ = false;
    last_update_time_ = now();
    receive_traj_ = true;
    last_trajectory_time_ = now();
    RCLCPP_DEBUG(get_logger(), "Received trajectory %lld, duration %.3fs",
                 static_cast<long long>(traj_id_), traj_duration_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
    last_odom_time_ = now();
  }

  void cmdCallback()
  {
    const auto current_time = now();
    publishTrajectoryPath(current_time);
    if (!receive_traj_ || !have_odom_ ||
        (current_time - last_odom_time_).seconds() > odom_timeout_ ||
        (current_time - last_trajectory_time_).seconds() > traj_duration_ + trajectory_timeout_)
    {
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    double dt = (current_time - last_update_time_).seconds();
    if (dt <= 0.0 || dt > 0.2) dt = 0.01;
    publishExecutionFrozen(false);
    last_update_time_ = current_time;

    const double nearest_t = findProjectionTime();
    const Eigen::Vector3d nearest = traj_[0].evaluateDeBoorT(nearest_t);
    const Eigen::Vector3d velocity_at_nearest = traj_[1].evaluateDeBoorT(nearest_t);
    const double planned_speed = velocity_at_nearest.head<2>().norm();
    const double lookahead_distance = std::clamp(
        lookahead_base_ + lookahead_speed_gain_ * planned_speed,
        lookahead_min_, lookahead_max_);
    const double lookahead_t = findLookaheadTime(nearest_t, lookahead_distance);
    Eigen::Vector2d tangent = traj_[1].evaluateDeBoorT(lookahead_t).head<2>();
    if (tangent.squaredNorm() < 1e-6)
      tangent = (traj_[0].evaluateDeBoorT(lookahead_t) - nearest).head<2>();
    if (tangent.squaredNorm() < 1e-6)
      tangent = Eigen::Vector2d(std::cos(odom_yaw_), std::sin(odom_yaw_));
    tangent.normalize();

    const double desired_yaw = std::atan2(tangent.y(), tangent.x());
    const double yaw_error = normalizeAngle(desired_yaw - odom_yaw_);
    const double heading_scale = std::clamp(std::cos(yaw_error), 0.0, 1.0);
    const Eigen::Vector2d projection_error = nearest.head<2>() - odom_pos_.head<2>();
    const Eigen::Vector2d vel_world = clampNorm(
        heading_scale * (planned_speed * tangent + kp_pos_ * projection_error),
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist desired;
    desired.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    desired.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    desired.angular.z = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    const Eigen::Vector2d endpoint_error =
        traj_[0].evaluateDeBoorT(traj_duration_).head<2>() - odom_pos_.head<2>();
    if (nearest_t >= traj_duration_ - projection_dt_ && endpoint_error.norm() < finish_dist_)
      desired = geometry_msgs::msg::Twist();

    geometry_msgs::msg::Twist command = desired;
    const Eigen::Vector2d previous_linear(previous_command_.linear.x,
                                          previous_command_.linear.y);
    const Eigen::Vector2d desired_linear(desired.linear.x, desired.linear.y);
    const Eigen::Vector2d limited_delta = clampNorm(
        desired_linear - previous_linear, std::max(0.0, max_linear_accel_) * dt);
    const Eigen::Vector2d limited_linear = previous_linear + limited_delta;
    command.linear.x = std::clamp(limited_linear.x(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(limited_linear.y(), -max_vy_, max_vy_);
    const double max_yaw_step = std::max(0.0, max_yaw_accel_) * dt;
    command.angular.z = previous_command_.angular.z + std::clamp(
        desired.angular.z - previous_command_.angular.z, -max_yaw_step, max_yaw_step);
    previous_command_ = command;

    cmd_vel_pub_->publish(command);
  }

  void publishTrajectoryPath(const rclcpp::Time &current_time)
  {
    if (!receive_traj_ || visualization_rate_ <= 0.0 || visualization_dt_ <= 0.0)
      return;
    if ((current_time - last_visualization_time_).seconds() < 1.0 / visualization_rate_)
      return;
    last_visualization_time_ = current_time;

    nav_msgs::msg::Path path;
    path.header.stamp = current_time;
    path.header.frame_id = trajectory_frame_;
    const double start = std::min(projected_t_, traj_duration_);
    for (double t = start; t < traj_duration_; t += visualization_dt_)
    {
      const Eigen::Vector3d point = traj_[0].evaluateDeBoorT(t);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = point.z();
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    const Eigen::Vector3d endpoint = traj_[0].evaluateDeBoorT(traj_duration_);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = endpoint.x();
    pose.pose.position.y = endpoint.y();
    pose.pose.position.z = endpoint.z();
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
    trajectory_path_pub_->publish(path);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double projected_t_{0.0};
  bool projection_initialized_{false};
  geometry_msgs::msg::Twist previous_command_;
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_trajectory_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_visualization_time_{0, 0, RCL_ROS_TIME};
  std::string trajectory_frame_;
  double visualization_rate_, visualization_dt_;
  double kp_pos_, kp_yaw_;
  double odom_timeout_, trajectory_timeout_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  double lookahead_base_, lookahead_speed_gain_, lookahead_min_, lookahead_max_;
  double projection_dt_, projection_forward_window_;
  double max_linear_accel_, max_yaw_accel_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
