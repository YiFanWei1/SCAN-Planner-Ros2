#include <gtest/gtest.h>

#include <Eigen/Core>

#include "plan_manage/velocity_command_utils.h"

namespace
{

TEST(VelocityCommandUtils, PreservesExactStopAndNumericalDeadband)
{
  EXPECT_TRUE(scan_planner::applyMinimumPlanarSpeed(
      Eigen::Vector2d::Zero(), 0.02, 0.25).isZero());
  EXPECT_TRUE(scan_planner::applyMinimumPlanarSpeed(
      Eigen::Vector2d(0.01, 0.0), 0.02, 0.25).isZero());
  EXPECT_DOUBLE_EQ(scan_planner::applyMinimumAngularSpeed(0.0, 0.03, 0.35), 0.0);
  EXPECT_DOUBLE_EQ(scan_planner::applyMinimumAngularSpeed(-0.02, 0.03, 0.35), 0.0);
}

TEST(VelocityCommandUtils, RaisesPlanarSpeedAndPreservesDirection)
{
  const Eigen::Vector2d input(0.06, 0.08);
  const Eigen::Vector2d output =
      scan_planner::applyMinimumPlanarSpeed(input, 0.02, 0.25);
  EXPECT_NEAR(output.norm(), 0.25, 1e-12);
  EXPECT_NEAR(output.x() / output.y(), input.x() / input.y(), 1e-12);
}

TEST(VelocityCommandUtils, LeavesExecutablePlanarSpeedUnchanged)
{
  const Eigen::Vector2d input(-0.30, 0.10);
  EXPECT_TRUE(scan_planner::applyMinimumPlanarSpeed(input, 0.02, 0.25)
                  .isApprox(input));
}

TEST(VelocityCommandUtils, RaisesYawRateAndPreservesSign)
{
  EXPECT_DOUBLE_EQ(scan_planner::applyMinimumAngularSpeed(0.10, 0.03, 0.35), 0.35);
  EXPECT_DOUBLE_EQ(scan_planner::applyMinimumAngularSpeed(-0.10, 0.03, 0.35), -0.35);
  EXPECT_DOUBLE_EQ(scan_planner::applyMinimumAngularSpeed(0.60, 0.03, 0.35), 0.60);
}

}  // namespace
