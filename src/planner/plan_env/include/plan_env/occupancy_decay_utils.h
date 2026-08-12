#ifndef PLAN_ENV_OCCUPANCY_DECAY_UTILS_H
#define PLAN_ENV_OCCUPANCY_DECAY_UTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace plan_env
{

// Test the point against the horizontal plane through the body center whose
// normal is the projection of the body +X axis. Using the body frame keeps
// this policy independent of a tilted or backwards-facing lidar mounting.
inline bool pointInBodyFrontHalfPlane(const Eigen::Vector3d &point_world,
                                      const Eigen::Vector3d &body_position_world,
                                      const Eigen::Quaterniond &body_orientation_world)
{
  if (!point_world.allFinite() || !body_position_world.allFinite() ||
      !body_orientation_world.coeffs().allFinite() || body_orientation_world.norm() < 1e-6)
    return false;

  const Eigen::Quaterniond normalized_orientation = body_orientation_world.normalized();
  Eigen::Vector3d forward_world = normalized_orientation * Eigen::Vector3d::UnitX();
  forward_world.z() = 0.0;
  const double horizontal_norm = forward_world.head<2>().norm();
  if (!std::isfinite(horizontal_norm) || horizontal_norm < 1e-6)
    return false;

  forward_world /= horizontal_norm;
  const Eigen::Vector3d displacement = point_world - body_position_world;
  return displacement.head<2>().dot(forward_world.head<2>()) > 0.0;
}

}  // namespace plan_env

#endif  // PLAN_ENV_OCCUPANCY_DECAY_UTILS_H
