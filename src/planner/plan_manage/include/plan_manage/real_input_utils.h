#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
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

struct PathProjection3D
{
  size_t segment_index{0};
  double segment_ratio{0.0};
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
  double weighted_distance{std::numeric_limits<double>::infinity()};
  double xy_distance{std::numeric_limits<double>::infinity()};
  double z_distance{std::numeric_limits<double>::infinity()};
};

inline std::optional<PathProjection3D> projectPointOntoPath3D(
    const nav_msgs::msg::Path &path, const Eigen::Vector3d &query,
    double vertical_weight = 1.0)
{
  if (path.poses.size() < 2 || !query.allFinite() ||
      !std::isfinite(vertical_weight) || vertical_weight <= 0.0)
    return std::nullopt;

  PathProjection3D best;
  for (size_t i = 0; i + 1 < path.poses.size(); ++i)
  {
    const auto &pa = path.poses[i].pose.position;
    const auto &pb = path.poses[i + 1].pose.position;
    const Eigen::Vector3d a(pa.x, pa.y, pa.z);
    const Eigen::Vector3d b(pb.x, pb.y, pb.z);
    if (!a.allFinite() || !b.allFinite())
      continue;

    Eigen::Vector3d scaled_a = a;
    Eigen::Vector3d scaled_b = b;
    Eigen::Vector3d scaled_query = query;
    scaled_a.z() *= vertical_weight;
    scaled_b.z() *= vertical_weight;
    scaled_query.z() *= vertical_weight;
    const Eigen::Vector3d segment = scaled_b - scaled_a;
    const double squared_length = segment.squaredNorm();
    double ratio = 0.0;
    if (squared_length > 1e-12)
      ratio = std::clamp(
          (scaled_query - scaled_a).dot(segment) / squared_length, 0.0, 1.0);

    const Eigen::Vector3d point = a + ratio * (b - a);
    const Eigen::Vector2d xy_error = point.head<2>() - query.head<2>();
    const double z_error = std::abs(point.z() - query.z());
    const double weighted_distance =
        std::sqrt(xy_error.squaredNorm() +
                  vertical_weight * vertical_weight * z_error * z_error);
    // Preserve the earliest path occurrence when two floors/branches are
    // numerically tied. A later occurrence must be decisively closer in 3-D.
    if (weighted_distance + 1e-9 < best.weighted_distance)
    {
      best.segment_index = i;
      best.segment_ratio = ratio;
      best.point = point;
      best.weighted_distance = weighted_distance;
      best.xy_distance = xy_error.norm();
      best.z_distance = z_error;
    }
  }
  if (!std::isfinite(best.weighted_distance))
    return std::nullopt;
  return best;
}

inline nav_msgs::msg::Path trimPathFromProjection(
    const nav_msgs::msg::Path &path, const PathProjection3D &projection)
{
  nav_msgs::msg::Path result;
  result.header = path.header;
  if (path.poses.size() < 2 || projection.segment_index + 1 >= path.poses.size())
    return result;

  geometry_msgs::msg::PoseStamped projected = path.poses[projection.segment_index];
  projected.pose.position.x = projection.point.x();
  projected.pose.position.y = projection.point.y();
  projected.pose.position.z = projection.point.z();
  result.poses.push_back(projected);

  size_t next = projection.segment_index + 1;
  const auto &next_position = path.poses[next].pose.position;
  const Eigen::Vector3d next_point(next_position.x, next_position.y, next_position.z);
  if ((next_point - projection.point).norm() < 1e-6)
    ++next;
  result.poses.insert(result.poses.end(), path.poses.begin() +
                                          static_cast<std::ptrdiff_t>(next),
                      path.poses.end());
  return result;
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

inline std::vector<Eigen::Vector3d> samplePathXYZ(
    const nav_msgs::msg::Path &path, double horizon, double spacing)
{
  std::vector<Eigen::Vector3d> result;
  if (path.poses.empty() || spacing <= 0.0 || horizon <= 0.0)
    return result;
  const auto &first = path.poses.front().pose.position;
  result.emplace_back(first.x, first.y, first.z);
  double next_distance = spacing;
  double traversed = 0.0;
  for (size_t i = 1; i < path.poses.size() && next_distance <= horizon; ++i)
  {
    const auto &pa = path.poses[i - 1].pose.position;
    const auto &pb = path.poses[i].pose.position;
    const Eigen::Vector3d a(pa.x, pa.y, pa.z);
    const Eigen::Vector3d b(pb.x, pb.y, pb.z);
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

inline double pathGeometryRms3D(
    const nav_msgs::msg::Path &a, const nav_msgs::msg::Path &b,
    double horizon, double spacing, double vertical_weight = 1.0)
{
  if (!std::isfinite(vertical_weight) || vertical_weight <= 0.0)
    return std::numeric_limits<double>::infinity();
  const auto sa = samplePathXYZ(a, horizon, spacing);
  const auto sb = samplePathXYZ(b, horizon, spacing);
  const size_t count = std::min(sa.size(), sb.size());
  if (count < 2)
    return std::numeric_limits<double>::infinity();
  double squared_sum = 0.0;
  for (size_t i = 0; i < count; ++i)
  {
    Eigen::Vector3d delta = sa[i] - sb[i];
    delta.z() *= vertical_weight;
    squared_sum += delta.squaredNorm();
  }
  return std::sqrt(squared_sum / static_cast<double>(count));
}
}  // namespace scan_planner
