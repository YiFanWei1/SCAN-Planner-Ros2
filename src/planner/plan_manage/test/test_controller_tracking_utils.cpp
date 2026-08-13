#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include <plan_manage/controller_tracking_utils.h>

namespace
{

TEST(ControllerTrackingUtils, LookaheadFeedforwardBreaksZeroSpeedStart)
{
  const Eigen::Vector2d command = scan_planner::calculateTrackingVelocity(
      Eigen::Vector2d::UnitX(), 0.35, Eigen::Vector2d::Zero(),
      0.0, 0.8, 0.5);

  EXPECT_NEAR(command.x(), 0.35, 1e-9);
  EXPECT_NEAR(command.y(), 0.0, 1e-9);
}

TEST(ControllerTrackingUtils, SuppressesTranslationUntilRobotTurnsAround)
{
  const Eigen::Vector2d command = scan_planner::calculateTrackingVelocity(
      Eigen::Vector2d::UnitX(), 0.35, Eigen::Vector2d(0.1, 0.0),
      M_PI, 0.8, 0.5);

  EXPECT_TRUE(command.isZero(1e-12));
}

TEST(ControllerTrackingUtils, RestoresTranslationSmoothlyAsHeadingAligns)
{
  const Eigen::Vector2d aligned = scan_planner::calculateTrackingVelocity(
      Eigen::Vector2d::UnitX(), 0.30, Eigen::Vector2d::Zero(),
      0.0, 0.8, 0.5);
  const Eigen::Vector2d sixty_degrees = scan_planner::calculateTrackingVelocity(
      Eigen::Vector2d::UnitX(), 0.30, Eigen::Vector2d::Zero(),
      M_PI / 3.0, 0.8, 0.5);

  EXPECT_NEAR(sixty_degrees.norm(), 0.5 * aligned.norm(), 1e-9);
}

TEST(ControllerTrackingUtils, PreservesSpeedLimitAndRejectsInvalidInput)
{
  const Eigen::Vector2d limited = scan_planner::calculateTrackingVelocity(
      Eigen::Vector2d::UnitX(), 0.8, Eigen::Vector2d(1.0, 0.0),
      0.0, 0.8, 0.5);
  EXPECT_NEAR(limited.norm(), 0.5, 1e-9);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Eigen::Vector2d invalid = scan_planner::calculateTrackingVelocity(
      Eigen::Vector2d(nan, 0.0), 0.3, Eigen::Vector2d::Zero(),
      0.0, 0.8, 0.5);
  EXPECT_TRUE(invalid.isZero(1e-12));
}

}  // namespace
