#ifndef PLAN_MANAGE_VELOCITY_COMMAND_UTILS_H
#define PLAN_MANAGE_VELOCITY_COMMAND_UTILS_H

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace scan_planner
{

inline Eigen::Vector2d applyMinimumPlanarSpeed(const Eigen::Vector2d &velocity,
                                               double command_deadband,
                                               double minimum_speed)
{
  if (!velocity.allFinite())
    return Eigen::Vector2d::Zero();

  const double speed = velocity.norm();
  const double deadband = std::max(0.0, command_deadband);
  const double minimum = std::max(0.0, minimum_speed);
  if (speed <= deadband)
    return Eigen::Vector2d::Zero();
  if (speed >= minimum || speed < 1e-9)
    return velocity;
  return velocity * (minimum / speed);
}

inline double applyMinimumAngularSpeed(double yaw_rate,
                                       double command_deadband,
                                       double minimum_speed)
{
  if (!std::isfinite(yaw_rate))
    return 0.0;

  const double magnitude = std::abs(yaw_rate);
  if (magnitude <= std::max(0.0, command_deadband))
    return 0.0;
  if (magnitude >= std::max(0.0, minimum_speed))
    return yaw_rate;
  return std::copysign(std::max(0.0, minimum_speed), yaw_rate);
}

}  // namespace scan_planner

#endif  // PLAN_MANAGE_VELOCITY_COMMAND_UTILS_H
