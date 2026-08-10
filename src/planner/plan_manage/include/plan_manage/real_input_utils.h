#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>

namespace scan_planner
{
inline bool finitePose(const geometry_msgs::msg::Pose &pose)
{
  const auto &p = pose.position;
  const auto &q = pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w) && (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 1e-12;
}

inline geometry_msgs::msg::Pose composePose(
    const geometry_msgs::msg::Pose &parent_child,
    const Eigen::Vector3d &child_offset)
{
  Eigen::Quaterniond rotation(parent_child.orientation.w, parent_child.orientation.x,
                              parent_child.orientation.y, parent_child.orientation.z);
  rotation.normalize();
  const Eigen::Vector3d origin(parent_child.position.x, parent_child.position.y,
                               parent_child.position.z);
  const Eigen::Vector3d result = origin + rotation * child_offset;
  geometry_msgs::msg::Pose pose = parent_child;
  pose.position.x = result.x();
  pose.position.y = result.y();
  pose.position.z = result.z();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

inline double pathLength(const nav_msgs::msg::Path &path)
{
  double length = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i)
  {
    const auto &a = path.poses[i - 1].pose.position;
    const auto &b = path.poses[i].pose.position;
    length += std::hypot(b.x - a.x, b.y - a.y);
  }
  return length;
}

inline std::vector<Eigen::Vector2d> samplePathXY(
    const nav_msgs::msg::Path &path, double horizon, double spacing)
{
  std::vector<Eigen::Vector2d> result;
  if (path.poses.empty() || spacing <= 0.0 || horizon <= 0.0)
    return result;
  result.emplace_back(path.poses.front().pose.position.x, path.poses.front().pose.position.y);
  double next_distance = spacing;
  double traversed = 0.0;
  for (size_t i = 1; i < path.poses.size() && next_distance <= horizon; ++i)
  {
    Eigen::Vector2d a(path.poses[i - 1].pose.position.x, path.poses[i - 1].pose.position.y);
    Eigen::Vector2d b(path.poses[i].pose.position.x, path.poses[i].pose.position.y);
    const double segment = (b - a).norm();
    if (segment < 1e-9)
      continue;
    while (next_distance <= traversed + segment && next_distance <= horizon)
    {
      result.push_back(a + (b - a) * ((next_distance - traversed) / segment));
      next_distance += spacing;
    }
    traversed += segment;
  }
  return result;
}

inline double pathGeometryRms(
    const nav_msgs::msg::Path &a, const nav_msgs::msg::Path &b,
    double horizon, double spacing)
{
  const auto sa = samplePathXY(a, horizon, spacing);
  const auto sb = samplePathXY(b, horizon, spacing);
  const size_t count = std::min(sa.size(), sb.size());
  if (count < 2)
    return std::numeric_limits<double>::infinity();
  double squared_sum = 0.0;
  for (size_t i = 0; i < count; ++i)
    squared_sum += (sa[i] - sb[i]).squaredNorm();
  return std::sqrt(squared_sum / static_cast<double>(count));
}
}  // namespace scan_planner
