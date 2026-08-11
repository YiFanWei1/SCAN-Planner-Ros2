#include <chrono>
#include <cstddef>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <iomanip>
#include <sstream>
#include <string>

#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include "plan_manage/real_input_utils.h"

namespace scan_planner
{
class RealGo2InputAdapter : public rclcpp::Node
{
public:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;

  RealGo2InputAdapter() : Node("real_go2_input_adapter")
  {
    lidar_to_base_ << declare_parameter<double>("lidar_to_base_x", -0.15),
        declare_parameter<double>("lidar_to_base_y", 0.0),
        declare_parameter<double>("lidar_to_base_z", -0.21);
    sync_tolerance_ = declare_parameter<double>("sync_tolerance", 0.05);
    max_horizontal_speed_ = declare_parameter<double>("max_input_horizontal_speed", 5.0);
    max_vertical_speed_ = declare_parameter<double>("max_input_vertical_speed", 2.0);
    validation_window_ = declare_parameter<double>("odom_validation_window", 0.10);
    max_vertical_displacement_ =
        declare_parameter<double>("max_vertical_displacement_from_start", 2.0);
    path_update_rate_ = declare_parameter<double>("path_update_rate", 1.0);
    path_endpoint_threshold_ = declare_parameter<double>("path_endpoint_threshold", 0.30);
    path_geometry_threshold_ = declare_parameter<double>("path_geometry_threshold", 0.20);
    path_compare_horizon_ = declare_parameter<double>("path_compare_horizon", 5.0);
    world_frame_ = declare_parameter<std::string>("world_frame", "camera_init");
    if (sync_tolerance_ <= 0.0 || path_update_rate_ <= 0.0 || validation_window_ <= 0.0)
      throw std::invalid_argument("sync, validation window and path rate must be positive");

    body_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>("body_pose", rclcpp::SensorDataQoS());
    sensor_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>("sensor_pose", rclcpp::SensorDataQoS());
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", rclcpp::SensorDataQoS());
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "initial_path", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>("status", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "lidar_odom", rclcpp::SensorDataQoS(),
        std::bind(&RealGo2InputAdapter::odomCallback, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "global_path", rclcpp::QoS(10).reliable(),
        std::bind(&RealGo2InputAdapter::pathCallback, this, std::placeholders::_1));

    cloud_sync_sub_.subscribe(this, "cloud", rmw_qos_profile_sensor_data);
    odom_sync_sub_.subscribe(this, "lidar_odom", rmw_qos_profile_sensor_data);
    synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(100), cloud_sync_sub_, odom_sync_sub_);
    synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_tolerance_));
    synchronizer_->registerCallback(
        std::bind(&RealGo2InputAdapter::cloudOdomCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    path_timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                    std::bind(&RealGo2InputAdapter::pathTimer, this));
    last_path_publish_time_ = now() - rclcpp::Duration::from_seconds(10.0);
    publishStatus("waiting_for_inputs");
    RCLCPP_DEBUG(get_logger(), "Real Go2 input adapter ready; lidar->base=(%.3f, %.3f, %.3f)",
                 lidar_to_base_.x(), lidar_to_base_.y(), lidar_to_base_.z());
  }

private:
  bool validateOdom(const nav_msgs::msg::Odometry &msg, std::string &reason)
  {
    if (!finitePose(msg.pose.pose))
    {
      reason = "invalid lidar odometry pose";
      return false;
    }
    const rclcpp::Time stamp(msg.header.stamp);
    const Eigen::Vector3d position(msg.pose.pose.position.x, msg.pose.pose.position.y,
                                   msg.pose.pose.position.z);
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (last_input_stamp_ && stamp <= *last_input_stamp_)
    {
      reason = "non-monotonic lidar odometry timestamp";
      return false;
    }
    last_input_stamp_ = stamp;
    if (!initial_position_)
      initial_position_ = position;
    if (std::abs(position.z() - initial_position_->z()) > max_vertical_displacement_)
    {
      reason = "lidar odometry height displaced too far from startup";
      input_valid_ = false;
      return false;
    }
    if (last_valid_stamp_ && (stamp - *last_valid_stamp_).seconds() < validation_window_)
    {
      if (!input_valid_)
        reason = "waiting for a valid odometry recovery window";
      return input_valid_;
    }
    if (last_valid_stamp_ && (stamp - *last_valid_stamp_).seconds() >= validation_window_)
    {
      const double dt = (stamp - *last_valid_stamp_).seconds();
      const Eigen::Vector3d delta = position - last_valid_position_;
      const Eigen::Vector3d velocity = delta / dt;
      const double horizontal_speed = velocity.head<2>().norm();
      const double vertical_speed = std::abs(velocity.z());
      if (horizontal_speed > max_horizontal_speed_ || vertical_speed > max_vertical_speed_)
      {
        std::ostringstream detail;
        detail << std::fixed << std::setprecision(3)
               << "lidar odometry jump: dt=" << dt
               << "s delta=[" << delta.x() << ", " << delta.y() << ", " << delta.z() << "]m"
               << " horizontal_speed=" << horizontal_speed << "m/s (limit="
               << max_horizontal_speed_ << ") vertical_speed=" << vertical_speed
               << "m/s (limit=" << max_vertical_speed_ << ")";
        reason = detail.str();
        input_valid_ = false;
        return false;
      }
    }
    if (!last_valid_stamp_ || (stamp - *last_valid_stamp_).seconds() >= validation_window_)
    {
      last_valid_stamp_ = stamp;
      last_valid_position_ = position;
    }
    input_valid_ = true;
    return true;
  }

  nav_msgs::msg::Odometry makeSensorPose(const nav_msgs::msg::Odometry &lidar_odom) const
  {
    nav_msgs::msg::Odometry result = lidar_odom;
    result.header.frame_id = lidar_odom.header.frame_id.empty() ? world_frame_ : lidar_odom.header.frame_id;
    result.child_frame_id = "livox_lidar";
    Eigen::Quaterniond q(result.pose.pose.orientation.w, result.pose.pose.orientation.x,
                         result.pose.pose.orientation.y, result.pose.pose.orientation.z);
    q.normalize();
    result.pose.pose.orientation.x = q.x();
    result.pose.pose.orientation.y = q.y();
    result.pose.pose.orientation.z = q.z();
    result.pose.pose.orientation.w = q.w();
    return result;
  }

  nav_msgs::msg::Odometry makeBodyPose(const nav_msgs::msg::Odometry &lidar_odom) const
  {
    nav_msgs::msg::Odometry result = makeSensorPose(lidar_odom);
    result.pose.pose = composePose(result.pose.pose, lidar_to_base_);
    result.child_frame_id = "base_link";
    return result;
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    std::string reason;
    if (!validateOdom(*msg, reason))
    {
      publishStatus("invalid_odom: " + reason);
      have_valid_odom_ = false;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s", reason.c_str());
      return;
    }
    const auto body_pose = makeBodyPose(*msg);
    body_pose_pub_->publish(body_pose);
    current_body_xy_ = Eigen::Vector2d(body_pose.pose.pose.position.x,
                                       body_pose.pose.pose.position.y);
    have_valid_odom_ = true;
  }

  void cloudOdomCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud,
                         const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    if (!have_valid_odom_ || !finitePose(odom->pose.pose))
      return;
    auto sensor_pose = makeSensorPose(*odom);
    sensor_pose.header.stamp = cloud->header.stamp;
    sensor_pose_pub_->publish(sensor_pose);
    cloud_pub_->publish(*cloud);
    publishStatus("active");
  }

  static bool validPath(const nav_msgs::msg::Path &path)
  {
    if (path.poses.size() < 2)
      return false;
    for (const auto &pose : path.poses)
      if (!std::isfinite(pose.pose.position.x) ||
          !std::isfinite(pose.pose.position.y) ||
          !std::isfinite(pose.pose.position.z))
        return false;
    return true;
  }

  void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg)
  {
    if (!validPath(*msg))
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid global path");
      return;
    }
    std::lock_guard<std::mutex> lock(path_mutex_);
    pending_path_ = *msg;
  }

  bool materiallyChanged(const nav_msgs::msg::Path &candidate) const
  {
    if (!published_path_)
      return true;
    const auto &old_end = published_path_->poses.back().pose.position;
    const auto &new_end = candidate.poses.back().pose.position;
    if (std::hypot(new_end.x - old_end.x, new_end.y - old_end.y) >= path_endpoint_threshold_)
      return true;
    const auto old_forward = trimFromCurrentPosition(*published_path_);
    const auto new_forward = trimFromCurrentPosition(candidate);
    return pathGeometryRms(old_forward, new_forward, path_compare_horizon_, 0.20) >=
           path_geometry_threshold_;
  }

  nav_msgs::msg::Path trimFromCurrentPosition(const nav_msgs::msg::Path &path) const
  {
    if (!current_body_xy_ || path.poses.empty())
      return path;
    size_t nearest = 0;
    double nearest_squared = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
      const Eigen::Vector2d point(path.poses[i].pose.position.x,
                                  path.poses[i].pose.position.y);
      const double squared = (point - *current_body_xy_).squaredNorm();
      if (squared < nearest_squared)
      {
        nearest_squared = squared;
        nearest = i;
      }
    }
    nav_msgs::msg::Path result = path;
    result.poses.assign(path.poses.begin() + static_cast<std::ptrdiff_t>(nearest),
                        path.poses.end());
    return result;
  }

  void pathTimer()
  {
    if (!have_valid_odom_)
      return;
    if ((now() - last_path_publish_time_).seconds() < 1.0 / path_update_rate_)
      return;
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!pending_path_ || !materiallyChanged(*pending_path_))
      return;
    nav_msgs::msg::Path output = *pending_path_;
    // The real setup assumes map -> camera_init is identity. Normalize the
    // path frame here so RViz and SCAN do not require an otherwise redundant
    // TF edge when a navigation stack labels /plan as "map".
    output.header.frame_id = world_frame_;
    for (auto &pose : output.poses)
      pose.header.frame_id = world_frame_;
    path_pub_->publish(output);
    published_path_ = output;
    pending_path_.reset();
    last_path_publish_time_ = now();
    RCLCPP_DEBUG(get_logger(), "Forwarded global path with %zu points, length %.2f m",
                 output.poses.size(), pathLength(output));
  }

  void publishStatus(const std::string &text)
  {
    if (text == last_status_)
      return;
    last_status_ = text;
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  Eigen::Vector3d lidar_to_base_;
  double sync_tolerance_, max_horizontal_speed_, max_vertical_speed_, validation_window_;
  double max_vertical_displacement_;
  double path_update_rate_, path_endpoint_threshold_, path_geometry_threshold_;
  double path_compare_horizon_;
  std::string world_frame_, last_status_;
  bool have_valid_odom_{false}, input_valid_{true};
  std::mutex state_mutex_, path_mutex_;
  std::optional<rclcpp::Time> last_input_stamp_, last_valid_stamp_;
  Eigen::Vector3d last_valid_position_{Eigen::Vector3d::Zero()};
  std::optional<Eigen::Vector3d> initial_position_;
  std::optional<Eigen::Vector2d> current_body_xy_;
  std::optional<nav_msgs::msg::Path> pending_path_, published_path_;
  rclcpp::Time last_path_publish_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr body_pose_pub_, sensor_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sync_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sync_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  rclcpp::TimerBase::SharedPtr path_timer_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::RealGo2InputAdapter>());
  rclcpp::shutdown();
  return 0;
}
