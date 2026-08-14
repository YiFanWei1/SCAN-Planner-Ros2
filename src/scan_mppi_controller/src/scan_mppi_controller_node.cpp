#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <bspline_opt/uniform_bspline.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "scan_mppi_controller/mppi_core.hpp"

namespace scan_mppi_controller
{

class ScanMppiControllerNode : public rclcpp::Node
{
public:
  ScanMppiControllerNode() : Node("scan_mppi_controller")
  {
    MppiConfig config;
    controller_frequency_ = declare_parameter<double>("controller_frequency", 30.0);
    config.dt = declare_parameter<double>("model_dt", 0.05);
    config.time_steps = positiveSize("time_steps", 24);
    config.batch_size = positiveSize("batch_size", 800);
    config.min_batch_size = positiveSize("min_batch_size", 600);
    config.worker_threads = declare_parameter<int>("worker_threads", 8);
    config.max_solve_time_ms = declare_parameter<double>("max_solve_time_ms", 33.0);
    config.vx_min = declare_parameter<double>("vx_min", -0.20);
    config.vx_max = declare_parameter<double>("vx_max", 0.50);
    config.vy_min = declare_parameter<double>("vy_min", -0.35);
    config.vy_max = declare_parameter<double>("vy_max", 0.35);
    config.wz_min = declare_parameter<double>("wz_min", -0.80);
    config.wz_max = declare_parameter<double>("wz_max", 0.80);
    config.linear_accel_max = declare_parameter<double>("linear_accel_max", 0.50);
    config.angular_accel_max = declare_parameter<double>("angular_accel_max", 1.0);
    config.noise_vx = declare_parameter<double>("noise_vx", 0.20);
    config.noise_vy = declare_parameter<double>("noise_vy", 0.15);
    config.noise_wz = declare_parameter<double>("noise_wz", 0.35);
    config.noise_smoothing = declare_parameter<double>("noise_smoothing", 0.65);
    config.temperature = declare_parameter<double>("temperature", 1.0);
    config.front_rear_offset = declare_parameter<double>("front_rear_offset", 0.18);
    map_resolution_ = declare_parameter<double>("map_resolution", 0.05);
    clearance_index_radius_xy_ = declare_parameter<double>(
        "clearance_index_radius_xy", 2.0);
    clearance_index_half_height_ = declare_parameter<double>(
        "clearance_index_half_height", 1.5);
    config.collision_translation_step = declare_parameter<double>(
        "collision_translation_step", 0.5 * map_resolution_);
    config.collision_yaw_step = declare_parameter<double>(
        "collision_yaw_step", 5.0 * M_PI / 180.0);
    config.safety_distance = declare_parameter<double>("safety_distance", 0.20);
    config.critical_distance = declare_parameter<double>("critical_distance", 0.08);
    config.clearance_stride = positiveSize("clearance_stride", 3);
    config.weight_path = declare_parameter<double>("weight_path", 15.0);
    config.weight_heading = declare_parameter<double>("weight_heading", 3.0);
    config.weight_velocity = declare_parameter<double>("weight_velocity", 2.0);
    config.weight_control = declare_parameter<double>("weight_control", 0.20);
    config.weight_smooth = declare_parameter<double>("weight_smooth", 4.0);
    config.weight_clearance = declare_parameter<double>("weight_clearance", 40.0);
    config.weight_progress = declare_parameter<double>("weight_progress", 4.0);
    config.weight_terminal = declare_parameter<double>("weight_terminal", 20.0);
    config.critical_cost = declare_parameter<double>("critical_cost", 10000.0);
    config.random_seed = static_cast<std::uint32_t>(
        declare_parameter<int>("random_seed", 42));
    optimizer_ = std::make_unique<MppiOptimizer>(config);

    trajectory_frame_ = declare_parameter<std::string>("trajectory_frame", "world");
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.25);
    map_timeout_ = declare_parameter<double>("map_timeout", 0.25);
    trajectory_timeout_ = declare_parameter<double>("trajectory_timeout", 0.50);
    finish_distance_ = declare_parameter<double>("finish_distance", 0.15);
    projection_dt_ = declare_parameter<double>("projection_dt", 0.025);
    projection_forward_window_ = declare_parameter<double>(
        "projection_forward_window", 1.0);
    visualization_rate_ = declare_parameter<double>("visualization_rate", 10.0);
    visualization_max_candidates_ = positiveSize("visualization_max_candidates", 50);
    footprint_stride_ = positiveSize("footprint_stride", 3);
    footprint_radius_ = declare_parameter<double>("footprint_radius", 0.35);
    footprint_height_ = declare_parameter<double>("footprint_height", 0.36);
    odom_twist_in_body_frame_ = declare_parameter<bool>(
        "odom_twist_in_body_frame", true);

    if (controller_frequency_ <= 0.0 || map_resolution_ <= 0.0 ||
        clearance_index_radius_xy_ <= 0.0 || clearance_index_half_height_ <= 0.0 ||
        odom_timeout_ <= 0.0 || map_timeout_ <= 0.0 || trajectory_timeout_ < 0.0 ||
        visualization_rate_ < 0.0)
      throw std::invalid_argument("invalid controller timing or map parameters");

    // Keep the expensive controller and map rebuild callbacks from starving the
    // high-rate odometry callback.  A MultiThreadedExecutor alone is not enough:
    // callbacks in a node's default group are mutually exclusive.
    trajectory_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    odom_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    map_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    control_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions trajectory_options;
    trajectory_options.callback_group = trajectory_callback_group_;
    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = odom_callback_group_;
    rclcpp::SubscriptionOptions map_options;
    map_options.callback_group = map_callback_group_;

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ScanMppiControllerNode::bsplineCallback, this, std::placeholders::_1),
        trajectory_options);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ScanMppiControllerNode::odomCallback, this, std::placeholders::_1),
        odom_options);
    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "grid_map/occupancy_inflate", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&ScanMppiControllerNode::mapCallback, this, std::placeholders::_1),
        map_options);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_raw", 20);
    full_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "planning/bspline_path", rclcpp::QoS(1).reliable().transient_local());
    reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "planning/mppi/reference_path", 10);
    optimal_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "planning/mppi/optimal_trajectory", 10);
    candidate_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "planning/mppi/candidate_trajectories", 10);
    footprint_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "planning/mppi/optimal_footprints", 10);
    velocity_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "planning/mppi/velocity_markers", 10);
    status_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "planning/mppi/status_markers", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("planning/mppi/status", 10);

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / controller_frequency_)),
        std::bind(&ScanMppiControllerNode::controlCallback, this),
        control_callback_group_);
    last_visualization_time_ = now() - rclcpp::Duration::from_seconds(1.0);
    RCLCPP_INFO(get_logger(),
                "Standalone SCAN MPPI ready: %zu samples, %zu x %.3fs horizon",
                config.batch_size, config.time_steps, config.dt);
  }

private:
  struct TrajectoryData
  {
    scan_planner::UniformBspline position;
    scan_planner::UniformBspline velocity;
    double duration{0.0};
    std::int64_t id{0};
    rclcpp::Time received{0, 0, RCL_ROS_TIME};

    TrajectoryData(const scan_planner::UniformBspline &position_in,
                   const scan_planner::UniformBspline &velocity_in,
                   double duration_in, std::int64_t id_in, const rclcpp::Time &received_in)
        : position(position_in), velocity(velocity_in), duration(duration_in),
          id(id_in), received(received_in) {}
  };

  std::size_t positiveSize(const std::string &name, int default_value)
  {
    const int value = declare_parameter<int>(name, default_value);
    if (value <= 0)
      throw std::invalid_argument(name + " must be positive");
    return static_cast<std::size_t>(value);
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    try
    {
      Eigen::MatrixXd points(3, msg->pos_pts.size());
      for (std::size_t i = 0; i < msg->pos_pts.size(); ++i)
        points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
      Eigen::VectorXd knots(msg->knots.size());
      for (std::size_t i = 0; i < msg->knots.size(); ++i)
        knots(static_cast<Eigen::Index>(i)) = msg->knots[i];
      scan_planner::UniformBspline position(points, msg->order, 0.1);
      position.setKnot(knots);
      scan_planner::UniformBspline velocity = position.getDerivative();
      const double duration = position.getTimeSum();
      auto trajectory = std::make_shared<TrajectoryData>(
          position, velocity, duration, msg->traj_id, now());
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        trajectory_ = std::move(trajectory);
        projected_t_ = 0.0;
        projection_initialized_ = false;
      }
      {
        std::lock_guard<std::mutex> compute_lock(compute_mutex_);
        optimizer_->reset();
      }
    }
    catch (const std::exception &error)
    {
      RCLCPP_ERROR(get_logger(), "Failed to parse B-spline: %s", error.what());
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    odom_ = *msg;
    have_odom_ = true;
    last_odom_time_ = now();
  }

  void mapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    Eigen::Vector3d index_center = Eigen::Vector3d::Zero();
    bool have_index_center = false;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (have_odom_)
      {
        index_center << odom_.pose.pose.position.x, odom_.pose.pose.position.y,
            odom_.pose.pose.position.z;
        have_index_center = true;
      }
    }
    auto map = std::make_shared<VoxelMap>(map_resolution_);
    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
      for (; x != x.end(); ++x, ++y, ++z)
        map->insert(Eigen::Vector3d(*x, *y, *z));
    }
    catch (const std::exception &error)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Invalid inflated map cloud: %s", error.what());
      return;
    }
    if (have_index_center)
      map->finalize(index_center, clearance_index_radius_xy_,
                    clearance_index_half_height_);
    else
      map->finalize();
    std::lock_guard<std::mutex> lock(data_mutex_);
    map_ = std::move(map);
    map_frame_ = msg->header.frame_id;
    last_map_time_ = now();
  }

  double projectionTime(const TrajectoryData &trajectory,
                        const nav_msgs::msg::Odometry &odom)
  {
    const Eigen::Vector2d position(odom.pose.pose.position.x, odom.pose.pose.position.y);
    const double begin = projection_initialized_ ? projected_t_ : 0.0;
    const double end = projection_initialized_ ?
        std::min(trajectory.duration, projected_t_ + projection_forward_window_) :
        trajectory.duration;
    double best_t = begin;
    double best_distance = std::numeric_limits<double>::infinity();
    const double step = std::max(0.005, projection_dt_);
    for (double t = begin; t <= end + 1e-9; t += step)
    {
      const Eigen::Vector2d point = trajectory.position.evaluateDeBoorT(
          std::min(t, end)).head<2>();
      const double distance = (point - position).squaredNorm();
      if (distance < best_distance)
      {
        best_distance = distance;
        best_t = std::min(t, end);
      }
    }
    projected_t_ = std::max(begin, best_t);
    projection_initialized_ = true;
    return projected_t_;
  }

  std::vector<ReferencePoint> makeReference(const TrajectoryData &trajectory,
                                             double start_t) const
  {
    std::vector<ReferencePoint> reference;
    reference.reserve(optimizer_->config().time_steps + 1U);
    double fallback_yaw = 0.0;
    for (std::size_t i = 0; i <= optimizer_->config().time_steps; ++i)
    {
      const double t = std::min(trajectory.duration,
          start_t + static_cast<double>(i) * optimizer_->config().dt);
      ReferencePoint point;
      point.position = trajectory.position.evaluateDeBoorT(t);
      point.velocity = trajectory.velocity.evaluateDeBoorT(t);
      if (point.velocity.head<2>().squaredNorm() > 1e-8)
        fallback_yaw = std::atan2(point.velocity.y(), point.velocity.x());
      else if (!reference.empty())
      {
        const Eigen::Vector2d delta = point.position.head<2>() -
                                      reference.back().position.head<2>();
        if (delta.squaredNorm() > 1e-8)
          fallback_yaw = std::atan2(delta.y(), delta.x());
      }
      point.yaw = fallback_yaw;
      reference.push_back(point);
    }
    return reference;
  }

  void publishStop(const std::string &reason, const rclcpp::Time &stamp,
                   const nav_msgs::msg::Odometry *odom = nullptr,
                   const std::vector<ReferencePoint> *reference = nullptr,
                   const MppiResult *failure_result = nullptr)
  {
    previous_command_ = Control();
    optimizer_->reset();
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    publishStatus(reason);
    if (visualizationDue(stamp))
      publishFailureVisualization(stamp, reason, odom, reference,
                                  failure_result);
  }

  void controlCallback()
  {
    std::unique_lock<std::mutex> compute_lock(compute_mutex_, std::try_to_lock);
    if (!compute_lock.owns_lock())
      return;
    const rclcpp::Time stamp = now();
    std::shared_ptr<TrajectoryData> trajectory;
    std::shared_ptr<const VoxelMap> map;
    nav_msgs::msg::Odometry odom;
    rclcpp::Time last_odom(0, 0, RCL_ROS_TIME), last_map(0, 0, RCL_ROS_TIME);
    std::string map_frame;
    bool have_odom = false;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      trajectory = trajectory_;
      map = map_;
      odom = odom_;
      last_odom = last_odom_time_;
      last_map = last_map_time_;
      map_frame = map_frame_;
      have_odom = have_odom_;
    }
    if (!have_odom || (stamp - last_odom).seconds() > odom_timeout_)
    {
      publishStop("odometry_timeout", stamp);
      return;
    }
    if (!trajectory)
    {
      publishStop("waiting_for_trajectory", stamp, &odom);
      return;
    }
    if ((stamp - trajectory->received).seconds() >
        trajectory->duration + trajectory_timeout_)
    {
      publishStop("trajectory_timeout", stamp, &odom);
      return;
    }
    if (!map || map->size() == 0U)
    {
      publishStop("empty_map", stamp, &odom);
      return;
    }
    if ((stamp - last_map).seconds() > map_timeout_)
    {
      publishStop("map_timeout", stamp, &odom);
      return;
    }
    if (!map_frame.empty() && map_frame != trajectory_frame_)
    {
      publishStop("map_frame_mismatch", stamp, &odom);
      return;
    }

    const double projected_t = projectionTime(*trajectory, odom);
    const auto reference = makeReference(*trajectory, projected_t);
    const Eigen::Vector2d endpoint_error =
        trajectory->position.evaluateDeBoorT(trajectory->duration).head<2>() -
        Eigen::Vector2d(odom.pose.pose.position.x, odom.pose.pose.position.y);
    if (projected_t >= trajectory->duration - projection_dt_ &&
        endpoint_error.norm() <= finish_distance_)
    {
      publishStop("goal_reached", stamp, &odom, &reference);
      return;
    }

    State initial;
    initial.x = odom.pose.pose.position.x;
    initial.y = odom.pose.pose.position.y;
    initial.z = odom.pose.pose.position.z;
    initial.yaw = tf2::getYaw(odom.pose.pose.orientation);
    MppiResult result = optimizer_->optimize(
        initial, reference, *map, previous_command_);
    if (!result.valid)
    {
      publishStop(result.reason, stamp, &odom, &reference, &result);
      return;
    }
    previous_command_ = result.command;
    geometry_msgs::msg::Twist command;
    command.linear.x = result.command.vx;
    command.linear.y = result.command.vy;
    command.angular.z = result.command.wz;
    cmd_pub_->publish(command);
    publishStatus(result.reason);

    if (visualizationDue(stamp))
      publishVisualization(stamp, *trajectory, projected_t, odom, reference,
                           result, (stamp - last_map).seconds(), map->size());
  }

  bool visualizationDue(const rclcpp::Time &stamp)
  {
    if (visualization_rate_ <= 0.0)
      return false;
    if ((stamp - last_visualization_time_).seconds() < 1.0 / visualization_rate_)
      return false;
    last_visualization_time_ = stamp;
    return true;
  }

  geometry_msgs::msg::PoseStamped poseMessage(const Eigen::Vector3d &position,
                                               double yaw,
                                               const std_msgs::msg::Header &header) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = position.x();
    pose.pose.position.y = position.y();
    pose.pose.position.z = position.z();
    pose.pose.orientation.z = std::sin(0.5 * yaw);
    pose.pose.orientation.w = std::cos(0.5 * yaw);
    return pose;
  }

  nav_msgs::msg::Path statePath(const std::vector<State> &states,
                                const rclcpp::Time &stamp) const
  {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = trajectory_frame_;
    for (const auto &state : states)
      path.poses.push_back(poseMessage(Eigen::Vector3d(state.x, state.y, state.z),
                                       state.yaw, path.header));
    return path;
  }

  void publishFullPath(const TrajectoryData &trajectory, const rclcpp::Time &stamp)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = trajectory_frame_;
    for (double t = 0.0; t < trajectory.duration; t += 0.10)
    {
      const Eigen::Vector3d position = trajectory.position.evaluateDeBoorT(t);
      const Eigen::Vector3d velocity = trajectory.velocity.evaluateDeBoorT(t);
      const double yaw = velocity.head<2>().squaredNorm() > 1e-8 ?
          std::atan2(velocity.y(), velocity.x()) : 0.0;
      path.poses.push_back(poseMessage(position, yaw, path.header));
    }
    const Eigen::Vector3d end = trajectory.position.evaluateDeBoorT(trajectory.duration);
    path.poses.push_back(poseMessage(end, 0.0, path.header));
    full_path_pub_->publish(path);
  }

  visualization_msgs::msg::Marker baseMarker(
      const rclcpp::Time &stamp, const std::string &ns, int id, int type) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = trajectory_frame_;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  visualization_msgs::msg::Marker deleteAll(const rclcpp::Time &stamp) const
  {
    auto marker = baseMarker(stamp, "clear", 0, visualization_msgs::msg::Marker::LINE_STRIP);
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    return marker;
  }

  std::vector<std::size_t> selectCandidates(const MppiResult &result) const
  {
    std::vector<std::size_t> safe;
    std::vector<std::size_t> collision;
    for (std::size_t i = 0; i < result.candidates.size(); ++i)
      (result.candidates[i].collision ? collision : safe).push_back(i);
    std::sort(safe.begin(), safe.end(), [&](std::size_t lhs, std::size_t rhs) {
      return result.candidates[lhs].cost < result.candidates[rhs].cost;
    });
    std::set<std::size_t> selected;
    for (std::size_t i = 0; i < std::min<std::size_t>(20U, safe.size()); ++i)
      selected.insert(safe[i]);
    const std::size_t stride = std::max<std::size_t>(1U,
        result.candidates.size() / std::max<std::size_t>(1U, visualization_max_candidates_));
    for (std::size_t i = 0; i < result.candidates.size() &&
         selected.size() < std::min<std::size_t>(40U, visualization_max_candidates_); i += stride)
      selected.insert(i);
    for (std::size_t index : collision)
    {
      if (selected.size() >= visualization_max_candidates_)
        break;
      selected.insert(index);
    }
    return std::vector<std::size_t>(selected.begin(), selected.end());
  }

  void publishCandidates(const rclcpp::Time &stamp, const MppiResult &result)
  {
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(deleteAll(stamp));
    const auto selected = selectCandidates(result);
    double min_cost = std::numeric_limits<double>::infinity();
    double max_cost = -std::numeric_limits<double>::infinity();
    for (std::size_t index : selected)
      if (!result.candidates[index].collision && std::isfinite(result.candidates[index].cost))
      {
        min_cost = std::min(min_cost, result.candidates[index].cost);
        max_cost = std::max(max_cost, result.candidates[index].cost);
      }
    int marker_id = 1;
    for (std::size_t index : selected)
    {
      const Candidate &candidate = result.candidates[index];
      auto marker = baseMarker(stamp, "mppi_candidates", marker_id++,
                               visualization_msgs::msg::Marker::LINE_STRIP);
      marker.scale.x = 0.012;
      marker.color.a = 0.42F;
      if (candidate.collision)
      {
        marker.color.r = 1.0F;
        marker.color.g = 0.05F;
      }
      else
      {
        const double ratio = max_cost > min_cost ?
            std::clamp((candidate.cost - min_cost) / (max_cost - min_cost), 0.0, 1.0) : 0.0;
        marker.color.r = static_cast<float>(ratio);
        marker.color.g = static_cast<float>(1.0 - 0.45 * ratio);
      }
      for (const auto &state : candidate.states)
      {
        geometry_msgs::msg::Point point;
        point.x = state.x;
        point.y = state.y;
        point.z = state.z;
        marker.points.push_back(point);
      }
      array.markers.push_back(std::move(marker));
    }
    candidate_pub_->publish(array);
  }

  void publishFootprints(const rclcpp::Time &stamp, const Candidate &optimal)
  {
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(deleteAll(stamp));
    int id = 1;
    for (std::size_t i = 0; i < optimal.states.size(); i += footprint_stride_)
    {
      const State &state = optimal.states[i];
      const Eigen::Vector3d heading(std::cos(state.yaw), std::sin(state.yaw), 0.0);
      for (int sign : {-1, 1})
      {
        auto marker = baseMarker(stamp, "mppi_footprints", id++,
                                 visualization_msgs::msg::Marker::CYLINDER);
        const Eigen::Vector3d center = Eigen::Vector3d(state.x, state.y, state.z) +
            static_cast<double>(sign) * optimizer_->config().front_rear_offset * heading;
        marker.pose.position.x = center.x();
        marker.pose.position.y = center.y();
        marker.pose.position.z = center.z();
        marker.scale.x = marker.scale.y = 2.0 * footprint_radius_;
        marker.scale.z = footprint_height_;
        marker.color.a = 0.12F;
        if (optimal.min_clearance < optimizer_->config().critical_distance)
          marker.color.r = 1.0F;
        else if (optimal.min_clearance < optimizer_->config().safety_distance)
        {
          marker.color.r = 1.0F;
          marker.color.g = 0.75F;
        }
        else
          marker.color.g = 1.0F;
        array.markers.push_back(std::move(marker));
      }
    }
    footprint_pub_->publish(array);
  }

  visualization_msgs::msg::Marker velocityArrow(
      const rclcpp::Time &stamp, int id, const Eigen::Vector3d &origin,
      const Eigen::Vector2d &velocity, float red, float green, float blue) const
  {
    auto marker = baseMarker(stamp, "mppi_velocity", id,
                             visualization_msgs::msg::Marker::ARROW);
    marker.scale.x = 0.035;
    marker.scale.y = 0.075;
    marker.scale.z = 0.10;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0F;
    geometry_msgs::msg::Point start;
    start.x = origin.x(); start.y = origin.y(); start.z = origin.z() + 0.15;
    geometry_msgs::msg::Point end = start;
    end.x += velocity.x(); end.y += velocity.y();
    marker.points = {start, end};
    return marker;
  }

  void publishVelocity(const rclcpp::Time &stamp, const nav_msgs::msg::Odometry &odom,
                       const std::vector<ReferencePoint> &reference,
                       const Control &command)
  {
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(deleteAll(stamp));
    const double yaw = tf2::getYaw(odom.pose.pose.orientation);
    const double c = std::cos(yaw), s = std::sin(yaw);
    const Eigen::Vector3d origin(odom.pose.pose.position.x, odom.pose.pose.position.y,
                                 odom.pose.pose.position.z);
    const Eigen::Vector2d command_world(c * command.vx - s * command.vy,
                                        s * command.vx + c * command.vy);
    Eigen::Vector2d measured(odom.twist.twist.linear.x, odom.twist.twist.linear.y);
    if (odom_twist_in_body_frame_)
      measured = Eigen::Vector2d(c * measured.x() - s * measured.y(),
                                 s * measured.x() + c * measured.y());
    Eigen::Vector2d reference_velocity = Eigen::Vector2d::Zero();
    if (!reference.empty())
      reference_velocity = reference.front().velocity.head<2>();
    array.markers.push_back(velocityArrow(stamp, 1, origin, command_world, 0.1F, 1.0F, 0.1F));
    array.markers.push_back(velocityArrow(stamp, 2, origin, measured, 0.1F, 0.4F, 1.0F));
    array.markers.push_back(velocityArrow(stamp, 3, origin, reference_velocity, 0.0F, 1.0F, 1.0F));

    auto arc = baseMarker(stamp, "mppi_velocity", 4,
                          visualization_msgs::msg::Marker::LINE_STRIP);
    arc.scale.x = 0.025;
    arc.color.r = 0.75F; arc.color.b = 1.0F; arc.color.a = 1.0F;
    const double radius = 0.30;
    const double angle = std::clamp(command.wz, -1.0, 1.0) * M_PI_2;
    for (int i = 0; i <= 20; ++i)
    {
      const double a = yaw + angle * static_cast<double>(i) / 20.0;
      geometry_msgs::msg::Point point;
      point.x = origin.x() + radius * std::cos(a);
      point.y = origin.y() + radius * std::sin(a);
      point.z = origin.z() + 0.22;
      arc.points.push_back(point);
    }
    array.markers.push_back(std::move(arc));
    velocity_pub_->publish(array);
  }

  void publishStatusMarker(const rclcpp::Time &stamp,
                           const nav_msgs::msg::Odometry &odom,
                           const MppiResult &result, double map_age,
                           std::size_t map_points)
  {
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(deleteAll(stamp));
    auto text = baseMarker(stamp, "mppi_status", 1,
                           visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    text.pose.position = odom.pose.pose.position;
    text.pose.position.z += 0.65;
    text.scale.z = 0.12;
    text.color.r = text.color.g = text.color.b = text.color.a = 1.0F;
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2)
           << result.reason << "  cmd [" << result.command.vx << ", "
           << result.command.vy << ", " << result.command.wz << "]\n"
           << "clearance=";
    if (std::isfinite(result.optimal.min_clearance))
      stream << result.optimal.min_clearance << "m";
    else
      stream << ">" << optimizer_->config().safety_distance << "m";
    stream << " safe=" << result.safe_candidates << "/" << result.active_batch_size
           << " solve=" << result.solve_time_ms << "ms\n"
           << "inflated_cloud points=" << map_points << " age=" << map_age << "s";
    text.text = stream.str();
    array.markers.push_back(std::move(text));
    status_marker_pub_->publish(array);
  }

  void publishVisualization(const rclcpp::Time &stamp,
                            const TrajectoryData &trajectory, double projected_t,
                            const nav_msgs::msg::Odometry &odom,
                            const std::vector<ReferencePoint> &reference,
                            const MppiResult &result, double map_age,
                            std::size_t map_points)
  {
    publishFullPath(trajectory, stamp);
    nav_msgs::msg::Path reference_path;
    reference_path.header.stamp = stamp;
    reference_path.header.frame_id = trajectory_frame_;
    for (const auto &point : reference)
      reference_path.poses.push_back(poseMessage(point.position, point.yaw,
                                                  reference_path.header));
    reference_path_pub_->publish(reference_path);
    optimal_path_pub_->publish(statePath(result.optimal.states, stamp));
    publishCandidates(stamp, result);
    publishFootprints(stamp, result.optimal);
    publishVelocity(stamp, odom, reference, result.command);
    publishStatusMarker(stamp, odom, result, map_age, map_points);
    (void)projected_t;
  }

  void publishFailureVisualization(const rclcpp::Time &stamp,
                                   const std::string &reason,
                                   const nav_msgs::msg::Odometry *odom,
                                   const std::vector<ReferencePoint> *reference,
                                   const MppiResult *failure_result)
  {
    visualization_msgs::msg::MarkerArray clear;
    clear.markers.push_back(deleteAll(stamp));
    candidate_pub_->publish(clear);
    footprint_pub_->publish(clear);
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = stamp;
    empty_path.header.frame_id = trajectory_frame_;
    optimal_path_pub_->publish(empty_path);
    if (!odom)
      return;
    MppiResult result;
    result.reason = reason;
    if (failure_result)
    {
      result.safe_candidates = failure_result->safe_candidates;
      result.active_batch_size = failure_result->active_batch_size;
      result.solve_time_ms = failure_result->solve_time_ms;
      result.optimal.min_clearance = failure_result->optimal.min_clearance;
    }
    if (reference)
      publishVelocity(stamp, *odom, *reference, Control());
    std::size_t map_points = 0U;
    double map_age = 0.0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      map_points = map_ ? map_->size() : 0U;
      if (map_)
        map_age = std::max(0.0, (stamp - last_map_time_).seconds());
    }
    publishStatusMarker(stamp, *odom, result, map_age, map_points);
  }

  void publishStatus(const std::string &reason)
  {
    if (reason == last_status_)
      return;
    last_status_ = reason;
    std_msgs::msg::String message;
    message.data = reason;
    status_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "MPPI state: %s", reason.c_str());
  }

  std::mutex data_mutex_;
  std::mutex compute_mutex_;
  std::shared_ptr<TrajectoryData> trajectory_;
  std::shared_ptr<const VoxelMap> map_;
  nav_msgs::msg::Odometry odom_;
  bool have_odom_{false};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_map_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_visualization_time_{0, 0, RCL_ROS_TIME};
  std::string map_frame_;
  std::string trajectory_frame_;
  std::string last_status_;
  double projected_t_{0.0};
  bool projection_initialized_{false};
  Control previous_command_;
  std::unique_ptr<MppiOptimizer> optimizer_;

  double controller_frequency_, map_resolution_, clearance_index_radius_xy_;
  double clearance_index_half_height_, odom_timeout_, map_timeout_;
  double trajectory_timeout_, finish_distance_, projection_dt_;
  double projection_forward_window_, visualization_rate_;
  double footprint_radius_, footprint_height_;
  std::size_t visualization_max_candidates_, footprint_stride_;
  bool odom_twist_in_body_frame_;

  rclcpp::CallbackGroup::SharedPtr trajectory_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
  rclcpp::CallbackGroup::SharedPtr map_callback_group_;
  rclcpp::CallbackGroup::SharedPtr control_callback_group_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr full_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimal_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr footprint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr velocity_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr status_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace scan_mppi_controller

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<scan_mppi_controller::ScanMppiControllerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4U);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
