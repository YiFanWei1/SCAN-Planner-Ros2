#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace scan_planner
{
class Go2KinematicSim : public rclcpp::Node
{
public:
  Go2KinematicSim() : Node("go2_kinematic_sim")
  {
    x_ = declare_parameter<double>("init_x", 0.0);
    y_ = declare_parameter<double>("init_y", 0.0);
    z_ = declare_parameter<double>("init_z", 0.3);
    yaw_ = declare_parameter<double>("init_yaw", 0.0);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vz_ = declare_parameter<double>("max_vz", 0.30);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.3);
    const double sim_rate = declare_parameter<double>("sim_rate", 100.0);
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    sync_gazebo_pose_ = declare_parameter<bool>("sync_gazebo_pose", false);
    gazebo_entity_name_ = declare_parameter<std::string>("gazebo_entity_name", "go2");
    gazebo_set_pose_service_ = declare_parameter<std::string>(
        "gazebo_set_pose_service", "/world/scan_demo/set_pose");
    gazebo_sync_rate_ = declare_parameter<double>("gazebo_sync_rate", 30.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base");
    terrain_following_ = declare_parameter<bool>("terrain_following", false);
    terrain_body_clearance_ = declare_parameter<double>("terrain_body_clearance", 0.4);
    terrain_profiles_ = declare_parameter<std::vector<double>>(
        "terrain_profiles", std::vector<double>{});
    terrain_platforms_ = declare_parameter<std::vector<double>>(
        "terrain_platforms", std::vector<double>{});
    terrain_bands_ = declare_parameter<std::vector<double>>(
        "terrain_bands", std::vector<double>{});
    if (terrain_profiles_.size() % 6 != 0)
      throw std::invalid_argument("terrain_profiles must contain groups of six values");
    if (terrain_platforms_.size() % 5 != 0)
      throw std::invalid_argument("terrain_platforms must contain groups of five values");
    if (terrain_bands_.size() % 6 != 0)
      throw std::invalid_argument("terrain_bands must contain groups of six values");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("body_pose", 100);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 20, std::bind(&Go2KinematicSim::cmdCallback, this, std::placeholders::_1));
    last_cmd_time_ = now();
    last_sim_time_ = now();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, sim_rate)),
        std::bind(&Go2KinematicSim::simCallback, this));
    if (sync_gazebo_pose_)
      gazebo_pose_client_ = create_client<ros_gz_interfaces::srv::SetEntityPose>(
          gazebo_set_pose_service_);
    RCLCPP_INFO(get_logger(), "Go2 kinematic simulator ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  void cmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    vx_cmd_ = std::clamp(msg->linear.x, -max_vx_, max_vx_);
    vy_cmd_ = std::clamp(msg->linear.y, -max_vy_, max_vy_);
    vz_cmd_ = std::clamp(msg->linear.z, -max_vz_, max_vz_);
    vyaw_cmd_ = std::clamp(msg->angular.z, -max_vyaw_, max_vyaw_);
    last_cmd_time_ = now();
  }

  void publishOdom(const rclcpp::Time &stamp)
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw_);
    const auto orientation = tf2::toMsg(quaternion);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = z_;
    odom.pose.pose.orientation = orientation;
    odom.twist.twist.linear.x = vx_world_;
    odom.twist.twist.linear.y = vy_world_;
    odom.twist.twist.linear.z = vz_world_;
    odom.twist.twist.angular.z = vyaw_cmd_;
    odom_pub_->publish(odom);

    if (publish_tf_)
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = child_frame_id_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.translation.z = z_;
      transform.transform.rotation = orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void syncGazeboPose(const rclcpp::Time &stamp)
  {
    if (!sync_gazebo_pose_ || !gazebo_pose_client_ || gazebo_request_pending_ ||
        !gazebo_pose_client_->service_is_ready())
      return;
    if (last_gazebo_sync_time_.nanoseconds() != 0 &&
        (stamp - last_gazebo_sync_time_).seconds() < 1.0 / std::max(1.0, gazebo_sync_rate_))
      return;

    auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    request->entity.name = gazebo_entity_name_;
    request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
    request->pose.position.x = x_;
    request->pose.position.y = y_;
    request->pose.position.z = z_;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw_);
    request->pose.orientation = tf2::toMsg(quaternion);
    gazebo_request_pending_ = true;
    last_gazebo_sync_time_ = stamp;
    gazebo_pose_client_->async_send_request(
        request, [this](rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedFuture future) {
          gazebo_request_pending_ = false;
          if (!future.get()->success)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Gazebo rejected pose update for '%s'",
                                 gazebo_entity_name_.c_str());
        });
  }

  void simCallback()
  {
    const auto current_time = now();
    double dt = (current_time - last_sim_time_).seconds();
    last_sim_time_ = current_time;
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    double vx = vx_cmd_, vy = vy_cmd_, vz = vz_cmd_, wz = vyaw_cmd_;
    if ((current_time - last_cmd_time_).seconds() > cmd_timeout_)
      vx = vy = vz = wz = 0.0;
    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    vx_world_ = c * vx - s * vy;
    vy_world_ = s * vx + c * vy;
    x_ += vx_world_ * dt;
    y_ += vy_world_ * dt;
    const double previous_z = z_;
    if (terrain_following_)
      z_ = terrainHeight(x_, y_) + terrain_body_clearance_;
    else
      z_ += vz * dt;
    vz_world_ = dt > 1e-6 ? (z_ - previous_z) / dt : 0.0;
    yaw_ = normalizeAngle(yaw_ + wz * dt);
    publishOdom(current_time);
    syncGazeboPose(current_time);
  }

  double terrainHeight(double x, double y) const
  {
    for (size_t index = 0; index < terrain_platforms_.size(); index += 5)
    {
      if (x >= terrain_platforms_[index] && x <= terrain_platforms_[index + 1] &&
          y >= terrain_platforms_[index + 2] && y <= terrain_platforms_[index + 3])
        return terrain_platforms_[index + 4];
    }
    // x_min, x_max, y_min, y_max, z_at_y_min, z_at_y_max. Unlike the legacy
    // profiles, bands are bounded in Y and therefore support multiple
    // consecutive ascending and descending panels in the same lane.
    for (size_t index = 0; index < terrain_bands_.size(); index += 6)
    {
      const double x_min = terrain_bands_[index];
      const double x_max = terrain_bands_[index + 1];
      const double y_min = terrain_bands_[index + 2];
      const double y_max = terrain_bands_[index + 3];
      if (x < x_min || x > x_max || y < y_min || y > y_max)
        continue;
      const double ratio = (y - y_min) / std::max(1e-6, y_max - y_min);
      return terrain_bands_[index + 4] +
             ratio * (terrain_bands_[index + 5] - terrain_bands_[index + 4]);
    }
    for (size_t index = 0; index < terrain_profiles_.size(); index += 6)
    {
      const double x_center = terrain_profiles_[index];
      const double half_width = terrain_profiles_[index + 1];
      const double y_start = terrain_profiles_[index + 2];
      const double y_end = terrain_profiles_[index + 3];
      const double z_start = terrain_profiles_[index + 4];
      const double z_end = terrain_profiles_[index + 5];
      if (std::abs(x - x_center) > half_width)
        continue;
      if (y <= y_start) return z_start;
      if (y >= y_end) return z_end;
      return z_start + (y - y_start) / (y_end - y_start) * (z_end - z_start);
    }
    return 0.0;
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr gazebo_pose_client_;
  double x_{0.0}, y_{0.0}, z_{0.3}, yaw_{0.0};
  double vx_cmd_{0.0}, vy_cmd_{0.0}, vz_cmd_{0.0}, vyaw_cmd_{0.0};
  double vx_world_{0.0}, vy_world_{0.0}, vz_world_{0.0};
  double max_vx_{0.75}, max_vy_{0.35}, max_vz_{0.30}, max_vyaw_{1.0}, cmd_timeout_{0.3};
  bool publish_tf_{false};
  bool sync_gazebo_pose_{false}, gazebo_request_pending_{false};
  double gazebo_sync_rate_{30.0};
  std::string gazebo_entity_name_, gazebo_set_pose_service_;
  std::string frame_id_, child_frame_id_;
  bool terrain_following_{false};
  double terrain_body_clearance_{0.4};
  std::vector<double> terrain_profiles_;
  std::vector<double> terrain_platforms_;
  std::vector<double> terrain_bands_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_sim_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gazebo_sync_time_{0, 0, RCL_ROS_TIME};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::Go2KinematicSim>());
  rclcpp::shutdown();
  return 0;
}
