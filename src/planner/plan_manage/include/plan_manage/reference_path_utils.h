#ifndef PLAN_MANAGE_REFERENCE_PATH_UTILS_H
#define PLAN_MANAGE_REFERENCE_PATH_UTILS_H

#include <cmath>
#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <nav_msgs/msg/path.hpp>

namespace scan_planner
{

inline bool transformBodyVelocityToWorld(
    const Eigen::Vector3d &body_velocity,
    const Eigen::Quaterniond &world_from_body,
    Eigen::Vector3d &world_velocity)
{
  if (!body_velocity.allFinite() || !world_from_body.coeffs().allFinite())
    return false;

  const double quaternion_norm = world_from_body.norm();
  if (!std::isfinite(quaternion_norm) || quaternion_norm < 1e-9)
    return false;

  world_velocity = world_from_body.normalized() * body_velocity;
  return world_velocity.allFinite();
}

inline bool alignStartZToReference(
    Eigen::Vector3d &start,
    const Eigen::Vector3d &reference,
    double max_correction,
    double *applied_correction = nullptr)
{
  if (applied_correction) *applied_correction = 0.0;
  if (!start.allFinite() || !reference.allFinite() ||
      !std::isfinite(max_correction) || max_correction < 0.0)
    return false;

  const double correction = reference.z() - start.z();
  if (std::abs(correction) > max_correction)
    return false;

  start.z() = reference.z();
  if (applied_correction) *applied_correction = correction;
  return true;
}

inline bool estimateLocalReferenceTangent(
    const std::vector<Eigen::Vector3d> &path,
    const Eigen::Vector3d &query,
    double half_window,
    Eigen::Vector3d &tangent)
{
  tangent.setZero();
  if (path.size() < 2 || !query.allFinite() ||
      !std::isfinite(half_window) || half_window <= 0.0)
    return false;

  std::vector<double> cumulative(path.size(), 0.0);
  double best_distance = std::numeric_limits<double>::infinity();
  double query_arc = 0.0;
  bool found_segment = false;
  for (std::size_t i = 0; i + 1 < path.size(); ++i)
  {
    if (!path[i].allFinite() || !path[i + 1].allFinite())
      return false;

    const Eigen::Vector3d segment = path[i + 1] - path[i];
    const double segment_length = segment.norm();
    cumulative[i + 1] = cumulative[i] + segment_length;
    if (segment_length < 1e-9)
      continue;

    const double u = std::clamp(
        (query - path[i]).dot(segment) / (segment_length * segment_length),
        0.0, 1.0);
    const double distance = (query - (path[i] + u * segment)).norm();
    if (distance < best_distance)
    {
      best_distance = distance;
      query_arc = cumulative[i] + u * segment_length;
      found_segment = true;
    }
  }
  if (!found_segment || cumulative.back() < 1e-9)
    return false;

  const auto interpolate_at_arc = [&](double arc) -> Eigen::Vector3d {
    arc = std::clamp(arc, 0.0, cumulative.back());
    auto upper = std::upper_bound(cumulative.begin(), cumulative.end(), arc);
    std::size_t index = upper == cumulative.begin()
                            ? 0
                            : static_cast<std::size_t>(upper - cumulative.begin() - 1);
    index = std::min(index, path.size() - 2);
    while (index + 1 < cumulative.size() &&
           cumulative[index + 1] - cumulative[index] < 1e-9)
      ++index;
    if (index + 1 >= path.size())
      return path.back();
    const double length = cumulative[index + 1] - cumulative[index];
    const double u = length > 1e-9 ? (arc - cumulative[index]) / length : 0.0;
    return (path[index] +
            std::clamp(u, 0.0, 1.0) * (path[index + 1] - path[index])).eval();
  };

  const Eigen::Vector3d before = interpolate_at_arc(query_arc - half_window);
  const Eigen::Vector3d after = interpolate_at_arc(query_arc + half_window);
  tangent = after - before;
  const double tangent_norm = tangent.norm();
  if (!tangent.allFinite() || tangent_norm < 1e-9)
  {
    tangent.setZero();
    return false;
  }
  tangent /= tangent_norm;
  return true;
}

inline bool projectVelocityOntoReferenceTangent(
    const Eigen::Vector3d &raw_velocity,
    const Eigen::Vector3d &tangent,
    double max_speed,
    Eigen::Vector3d &projected_velocity,
    double *raw_along_speed = nullptr,
    double *used_along_speed = nullptr,
    double *removed_lateral_speed = nullptr)
{
  projected_velocity.setZero();
  if (!raw_velocity.allFinite() || !tangent.allFinite() ||
      !std::isfinite(max_speed) || max_speed < 0.0)
    return false;

  const double tangent_norm = tangent.norm();
  if (tangent_norm < 1e-9)
    return false;
  const Eigen::Vector3d unit_tangent = tangent / tangent_norm;
  const double raw_along = raw_velocity.dot(unit_tangent);
  const double used_along = std::clamp(raw_along, 0.0, max_speed);
  const Eigen::Vector3d lateral = raw_velocity - raw_along * unit_tangent;
  projected_velocity = used_along * unit_tangent;

  if (raw_along_speed) *raw_along_speed = raw_along;
  if (used_along_speed) *used_along_speed = used_along;
  if (removed_lateral_speed) *removed_lateral_speed = lateral.norm();
  return projected_velocity.allFinite();
}

inline bool prepareReferenceWaypoints(
    const nav_msgs::msg::Path &path,
    double body_height,
    double min_distance,
    std::vector<Eigen::Vector3d> &waypoints,
    std::string *error = nullptr)
{
  waypoints.clear();
  if (path.poses.empty())
  {
    if (error) *error = "reference path is empty";
    return false;
  }
  if (!std::isfinite(body_height) || !std::isfinite(min_distance) || min_distance < 0.0)
  {
    if (error) *error = "body height and minimum distance must be finite and non-negative";
    return false;
  }

  waypoints.reserve(path.poses.size());
  Eigen::Vector3d final_waypoint = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_waypoint = Eigen::Vector3d::Zero();
  bool first = true;

  for (const auto &pose_stamped : path.poses)
  {
    const auto &position = pose_stamped.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z))
    {
      waypoints.clear();
      if (error) *error = "reference path contains a non-finite coordinate";
      return false;
    }

    Eigen::Vector3d waypoint(position.x, position.y, position.z + body_height);
    final_waypoint = waypoint;
    if (first || (waypoint - last_waypoint).norm() >= min_distance)
    {
      waypoints.push_back(waypoint);
      last_waypoint = waypoint;
      first = false;
    }
  }

  if ((waypoints.back() - final_waypoint).norm() > 1e-6)
    waypoints.push_back(final_waypoint);

  if (waypoints.size() < 2)
  {
    waypoints.clear();
    if (error) *error = "reference path requires at least two distinct points";
    return false;
  }
  return true;
}

}  // namespace scan_planner

#endif  // PLAN_MANAGE_REFERENCE_PATH_UTILS_H
