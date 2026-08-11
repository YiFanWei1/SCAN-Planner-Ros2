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

}  // namespace
