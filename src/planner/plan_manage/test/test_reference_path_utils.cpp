#include <limits>

#include <gtest/gtest.h>

#include <plan_manage/reference_path_utils.h>

namespace
{

geometry_msgs::msg::PoseStamped pose(double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped result;
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.position.z = z;
  result.pose.orientation.w = 1.0;
  return result;
}

TEST(ReferencePathUtils, RejectsEmptyAndSinglePointPaths)
{
  std::vector<Eigen::Vector3d> waypoints;
  std::string error;
  nav_msgs::msg::Path path;
  EXPECT_FALSE(scan_planner::prepareReferenceWaypoints(path, 0.4, 0.5, waypoints, &error));
  EXPECT_TRUE(waypoints.empty());

  path.poses.push_back(pose(0.0, 0.0, 0.0));
  EXPECT_FALSE(scan_planner::prepareReferenceWaypoints(path, 0.4, 0.5, waypoints, &error));
  EXPECT_TRUE(waypoints.empty());
}

TEST(ReferencePathUtils, DownsamplesInThreeDimensionsAndPreservesFinalPoint)
{
  nav_msgs::msg::Path path;
  path.poses = {
      pose(0.0, 0.0, 0.1),
      pose(0.0, 0.0, 0.1),
      pose(0.2, 0.0, 0.1),
      pose(0.4, 0.0, 0.1),
      pose(0.6, 0.0, 0.1),
      pose(0.7, 0.0, 0.1),
  };

  std::vector<Eigen::Vector3d> waypoints;
  ASSERT_TRUE(scan_planner::prepareReferenceWaypoints(path, 0.4, 0.5, waypoints));
  ASSERT_EQ(waypoints.size(), 3u);
  EXPECT_NEAR(waypoints.front().z(), 0.5, 1e-9);
  EXPECT_NEAR(waypoints[1].x(), 0.6, 1e-9);
  EXPECT_NEAR(waypoints.back().x(), 0.7, 1e-9);
  EXPECT_NEAR(waypoints.back().z(), 0.5, 1e-9);

  path.poses = {pose(1.0, 2.0, 0.0), pose(1.0, 2.0, 0.6)};
  ASSERT_TRUE(scan_planner::prepareReferenceWaypoints(path, 0.4, 0.5, waypoints));
  ASSERT_EQ(waypoints.size(), 2u);
  EXPECT_NEAR(waypoints.back().z(), 1.0, 1e-9);
}

TEST(ReferencePathUtils, RejectsNonFiniteCoordinates)
{
  nav_msgs::msg::Path path;
  path.poses = {
      pose(0.0, 0.0, 0.0),
      pose(std::numeric_limits<double>::quiet_NaN(), 1.0, 0.0),
  };

  std::vector<Eigen::Vector3d> waypoints;
  EXPECT_FALSE(scan_planner::prepareReferenceWaypoints(path, 0.4, 0.5, waypoints));
  EXPECT_TRUE(waypoints.empty());
}

TEST(ReferencePathUtils, AlignsOnlyStartHeightWithinGuard)
{
  Eigen::Vector3d start(1.0, 2.0, -0.30);
  const Eigen::Vector3d reference(9.0, 8.0, 0.10);
  double correction = 0.0;

  ASSERT_TRUE(scan_planner::alignStartZToReference(start, reference, 0.60, &correction));
  EXPECT_DOUBLE_EQ(start.x(), 1.0);
  EXPECT_DOUBLE_EQ(start.y(), 2.0);
  EXPECT_DOUBLE_EQ(start.z(), 0.10);
  EXPECT_DOUBLE_EQ(correction, 0.40);
}

TEST(ReferencePathUtils, RejectsUnsafeOrNonFiniteStartHeightCorrection)
{
  Eigen::Vector3d start(1.0, 2.0, -0.30);
  double correction = 123.0;
  EXPECT_FALSE(scan_planner::alignStartZToReference(
      start, Eigen::Vector3d(1.0, 2.0, 1.0), 0.60, &correction));
  EXPECT_DOUBLE_EQ(start.z(), -0.30);
  EXPECT_DOUBLE_EQ(correction, 0.0);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(scan_planner::alignStartZToReference(
      start, Eigen::Vector3d(1.0, 2.0, nan), 0.60));
  EXPECT_DOUBLE_EQ(start.z(), -0.30);
}

TEST(ReferencePathUtils, RotatesBodyVelocityIntoWorldFrame)
{
  const Eigen::Quaterniond yaw_90(
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  Eigen::Vector3d world_velocity = Eigen::Vector3d::Zero();

  ASSERT_TRUE(scan_planner::transformBodyVelocityToWorld(
      Eigen::Vector3d(1.0, 0.0, 0.25), yaw_90, world_velocity));
  EXPECT_NEAR(world_velocity.x(), 0.0, 1e-9);
  EXPECT_NEAR(world_velocity.y(), 1.0, 1e-9);
  EXPECT_NEAR(world_velocity.z(), 0.25, 1e-9);
}

TEST(ReferencePathUtils, RotatesVelocityWithFullThreeDimensionalAttitude)
{
  const Eigen::Quaterniond pitch_30(
      Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitY()));
  Eigen::Vector3d world_velocity = Eigen::Vector3d::Zero();

  ASSERT_TRUE(scan_planner::transformBodyVelocityToWorld(
      Eigen::Vector3d(1.0, 0.0, 0.0), pitch_30, world_velocity));
  EXPECT_NEAR(world_velocity.x(), std::sqrt(3.0) / 2.0, 1e-9);
  EXPECT_NEAR(world_velocity.y(), 0.0, 1e-9);
  EXPECT_NEAR(world_velocity.z(), -0.5, 1e-9);
}

TEST(ReferencePathUtils, RejectsInvalidVelocityTransformInputs)
{
  Eigen::Vector3d world_velocity = Eigen::Vector3d::Zero();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(scan_planner::transformBodyVelocityToWorld(
      Eigen::Vector3d(nan, 0.0, 0.0), Eigen::Quaterniond::Identity(), world_velocity));
  EXPECT_FALSE(scan_planner::transformBodyVelocityToWorld(
      Eigen::Vector3d::Zero(), Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0), world_velocity));
}

}  // namespace
