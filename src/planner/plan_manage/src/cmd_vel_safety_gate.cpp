#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/data_disp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace scan_planner
{
class CmdVelSafetyGate : public rclcpp::Node
{
public:
  CmdVelSafetyGate() : Node("cmd_vel_safety_gate")
  {
    enabled_ = declare_parameter<bool>("enable_motion_on_start", false);
    max_vx_ = declare_parameter<double>("max_vx", 0.30);
    max_vy_ = declare_parameter<double>("max_vy", 0.20);
    max_vyaw_ = declare_parameter<double>("max_vyaw", 0.50);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.15);
    trajectory_timeout_ = declare_parameter<double>("trajectory_timeout", 0.50);
    planner_heartbeat_timeout_ = declare_parameter<double>("planner_heartbeat_timeout", 0.20);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.15);
    publish_rate_ = declare_parameter<double>("publish_rate", 100.0);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_raw", 20, [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          latest_cmd_ = *msg;
          last_cmd_time_ = now();
          have_cmd_ = true;
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr) {
          last_odom_time_ = now();
          have_odom_ = true;
        });
    trajectory_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10, [this](const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg) {
          last_trajectory_time_ = now();
          trajectory_valid_for_ = trajectory_timeout_;
          if (msg->knots.size() >= 2)
          {
            const auto bounds = std::minmax_element(msg->knots.begin(), msg->knots.end());
            trajectory_valid_for_ += std::max(0.0, *bounds.second - *bounds.first);
          }
          have_trajectory_ = true;
        });
    planner_heartbeat_sub_ = create_subscription<scan_planner_msgs::msg::DataDisp>(
        "planning/data_display", 10, [this](scan_planner_msgs::msg::DataDisp::ConstSharedPtr) {
          last_planner_heartbeat_time_ = now();
          have_planner_heartbeat_ = true;
        });
    emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
        "emergency_stop", 10, [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
          emergency_stop_ = msg->data;
        });
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe", 20);
    status_pub_ = create_publisher<std_msgs::msg::String>("status", 10);
    enable_service_ = create_service<std_srvs::srv::SetBool>(
        "enable_motion", [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                                std_srvs::srv::SetBool::Response::SharedPtr response) {
          enabled_ = request->data;
          response->success = true;
          response->message = enabled_ ? "motion enabled" : "motion disabled; zero command active";
          if (!enabled_)
            latest_cmd_ = geometry_msgs::msg::Twist();
          RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        });
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_)),
        std::bind(&CmdVelSafetyGate::timerCallback, this));
    RCLCPP_WARN(get_logger(), "Velocity safety gate ready; motion is %s",
                enabled_ ? "ENABLED" : "DISABLED");
  }

private:
  static bool finiteCommand(const geometry_msgs::msg::Twist &cmd)
  {
    return std::isfinite(cmd.linear.x) && std::isfinite(cmd.linear.y) &&
           std::isfinite(cmd.linear.z) && std::isfinite(cmd.angular.x) &&
           std::isfinite(cmd.angular.y) && std::isfinite(cmd.angular.z);
  }

  void timerCallback()
  {
    const auto current = now();
    std::string reason = "active";
    if (!enabled_)
      reason = "motion_disabled";
    else if (emergency_stop_)
      reason = "emergency_stop";
    else if (!have_odom_ || (current - last_odom_time_).seconds() > odom_timeout_)
      reason = "odometry_timeout";
    else if (!have_planner_heartbeat_ ||
             (current - last_planner_heartbeat_time_).seconds() > planner_heartbeat_timeout_)
      reason = "planner_heartbeat_timeout";
    else if (!have_trajectory_ ||
             (current - last_trajectory_time_).seconds() > trajectory_valid_for_)
      reason = "trajectory_timeout";
    else if (!have_cmd_ || (current - last_cmd_time_).seconds() > command_timeout_)
      reason = "command_timeout";
    else if (!finiteCommand(latest_cmd_))
      reason = "invalid_command";

    geometry_msgs::msg::Twist output;
    if (reason == "active")
    {
      output = latest_cmd_;
      output.linear.x = std::clamp(output.linear.x, -max_vx_, max_vx_);
      output.linear.y = std::clamp(output.linear.y, -max_vy_, max_vy_);
      output.linear.z = 0.0;
      output.angular.x = 0.0;
      output.angular.y = 0.0;
      output.angular.z = std::clamp(output.angular.z, -max_vyaw_, max_vyaw_);
    }
    cmd_pub_->publish(output);
    if (reason != last_reason_)
    {
      last_reason_ = reason;
      std_msgs::msg::String status;
      status.data = reason;
      status_pub_->publish(status);
      RCLCPP_INFO(get_logger(), "Safety gate state: %s", reason.c_str());
    }
  }

  bool enabled_{false}, emergency_stop_{false};
  bool have_cmd_{false}, have_odom_{false}, have_trajectory_{false};
  bool have_planner_heartbeat_{false};
  double max_vx_, max_vy_, max_vyaw_, odom_timeout_, trajectory_timeout_, command_timeout_;
  double planner_heartbeat_timeout_, trajectory_valid_for_{0.0};
  double publish_rate_;
  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_trajectory_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_planner_heartbeat_time_{0, 0, RCL_ROS_TIME};
  std::string last_reason_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<scan_planner_msgs::msg::DataDisp>::SharedPtr planner_heartbeat_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::CmdVelSafetyGate>());
  rclcpp::shutdown();
  return 0;
}
