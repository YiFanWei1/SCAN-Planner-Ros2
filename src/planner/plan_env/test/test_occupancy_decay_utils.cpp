#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "plan_env/occupancy_decay_utils.h"

namespace
{

TEST(OccupancyDecayUtils, IdentityBodyUsesPositiveWorldXAsFront)
{
  const Eigen::Vector3d body_position(1.0, 2.0, 0.5);
  const Eigen::Quaterniond body_orientation = Eigen::Quaterniond::Identity();

  EXPECT_TRUE(plan_env::pointInBodyFrontHalfPlane(
      Eigen::Vector3d(2.0, 2.0, -5.0), body_position, body_orientation));
  EXPECT_FALSE(plan_env::pointInBodyFrontHalfPlane(
      Eigen::Vector3d(0.0, 2.0, 8.0), body_position, body_orientation));
  EXPECT_FALSE(plan_env::pointInBodyFrontHalfPlane(
      Eigen::Vector3d(1.0, 3.0, 0.5), body_position, body_orientation));
}

TEST(OccupancyDecayUtils, YawRotatesFrontHalfPlane)
{
  constexpr double kHalfPi = 1.5707963267948966;
  const Eigen::Quaterniond body_orientation(
      Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitZ()));

  EXPECT_TRUE(plan_env::pointInBodyFrontHalfPlane(
      Eigen::Vector3d(0.0, 2.0, 0.0), Eigen::Vector3d::Zero(), body_orientation));
  EXPECT_FALSE(plan_env::pointInBodyFrontHalfPlane(
      Eigen::Vector3d(0.0, -2.0, 0.0), Eigen::Vector3d::Zero(), body_orientation));
}

TEST(OccupancyDecayUtils, PitchDoesNotTiltTheFrontBoundaryVertically)
{
  constexpr double kThirtyDegrees = 0.5235987755982988;
  const Eigen::Quaterniond body_orientation(
      Eigen::AngleAxisd(kThirtyDegrees, Eigen::Vector3d::UnitY()));

  EXPECT_TRUE(plan_env::pointInBodyFrontHalfPlane(
      Eigen::Vector3d(2.0, 0.0, -10.0), Eigen::Vector3d::Zero(), body_orientation));
  EXPECT_FALSE(plan_env::pointInBodyFrontHalfPlane(
      Eigen::Vector3d(-2.0, 0.0, 10.0), Eigen::Vector3d::Zero(), body_orientation));
}

TEST(OccupancyDecayUtils, RejectsInvalidOrientation)
{
  const Eigen::Quaterniond invalid_orientation(0.0, 0.0, 0.0, 0.0);
  EXPECT_FALSE(plan_env::pointInBodyFrontHalfPlane(
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero(), invalid_orientation));
}

}  // namespace
