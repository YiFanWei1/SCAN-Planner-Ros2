#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace scan_planner
{

inline double translationHeadingScale(double yaw_error)
{
  if (!std::isfinite(yaw_error))
    return 0.0;
  return std::clamp(std::cos(yaw_error), 0.0, 1.0);
}

inline Eigen::Vector2d calculateTrackingVelocity(
    const Eigen::Vector2d &path_tangent,
    double lookahead_speed,
    const Eigen::Vector2d &projection_error,
    double yaw_error,
    double position_gain,
    double max_speed)
{
  if (!path_tangent.allFinite() || !projection_error.allFinite() ||
      !std::isfinite(lookahead_speed) || !std::isfinite(position_gain) ||
      !std::isfinite(max_speed) || max_speed <= 0.0)
    return Eigen::Vector2d::Zero();

  const double tangent_norm = path_tangent.norm();
  if (tangent_norm < 1e-9)
    return Eigen::Vector2d::Zero();

  const Eigen::Vector2d tangent = path_tangent / tangent_norm;
  Eigen::Vector2d velocity = translationHeadingScale(yaw_error) *
      (std::max(0.0, lookahead_speed) * tangent +
       std::max(0.0, position_gain) * projection_error);
  const double velocity_norm = velocity.norm();
  if (velocity_norm > max_speed)
    velocity *= max_speed / velocity_norm;
  return velocity;
}

}  // namespace scan_planner
