#include <gtest/gtest.h>

#include <cmath>
#include <Eigen/Geometry>

#include "plan_manage/real_input_utils.h"

TEST(RealInputUtils, LidarOffsetRotatesWithPose)
{
  geometry_msgs::msg::Pose lidar;
  lidar.position.x = 1.0;
  lidar.position.y = 2.0;
  lidar.position.z = 0.5;
  const Eigen::Quaterniond yaw(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  lidar.orientation.x = yaw.x();
  lidar.orientation.y = yaw.y();
  lidar.orientation.z = yaw.z();
  lidar.orientation.w = yaw.w();
  const auto base = scan_planner::composePose(lidar, Eigen::Vector3d(-0.15, 0.0, -0.21));
  EXPECT_NEAR(base.position.x, 1.0, 1e-9);
  EXPECT_NEAR(base.position.y, 1.85, 1e-9);
  EXPECT_NEAR(base.position.z, 0.29, 1e-9);
}

TEST(RealInputUtils, PathGeometryDetectsLateralChange)
{
  nav_msgs::msg::Path a;
  nav_msgs::msg::Path b;
  for (int i = 0; i <= 20; ++i)
  {
    geometry_msgs::msg::PoseStamped pa;
    pa.pose.orientation.w = 1.0;
    pa.pose.position.x = i * 0.2;
    auto pb = pa;
    pb.pose.position.y = 0.3;
    a.poses.push_back(pa);
    b.poses.push_back(pb);
  }
  EXPECT_NEAR(scan_planner::pathGeometryRms(a, a, 4.0, 0.2), 0.0, 1e-9);
  EXPECT_GT(scan_planner::pathGeometryRms(a, b, 4.0, 0.2), 0.29);
}

namespace
{
geometry_msgs::msg::PoseStamped pathPose(double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped result;
  result.pose.orientation.w = 1.0;
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.position.z = z;
  return result;
}
}  // namespace

TEST(RealInputUtils, ThreeDimensionalProjectionSelectsCurrentFloor)
{
  nav_msgs::msg::Path path;
  path.poses = {
      pathPose(0.0, 0.0, 0.0), pathPose(2.0, 0.0, 0.0),
      pathPose(3.0, 0.0, -3.0), pathPose(2.0, 0.0, -3.0),
      pathPose(0.0, 0.0, -3.0)};

  // Pure XY has two exact matches. Height must keep the projection on the
  // upper floor rather than selecting the lower occurrence of the same XY.
  const auto projection = scan_planner::projectPointOntoPath3D(
      path, Eigen::Vector3d(1.0, 0.0, 0.05));
  ASSERT_TRUE(projection.has_value());
  EXPECT_EQ(projection->segment_index, 0U);
  EXPECT_NEAR(projection->point.x(), 1.0, 1e-9);
  EXPECT_NEAR(projection->point.z(), 0.0, 1e-9);
  EXPECT_NEAR(projection->z_distance, 0.05, 1e-9);

  const auto lower_projection = scan_planner::projectPointOntoPath3D(
      path, Eigen::Vector3d(1.0, 0.0, -2.95));
  ASSERT_TRUE(lower_projection.has_value());
  EXPECT_EQ(lower_projection->segment_index, 3U);
  EXPECT_NEAR(lower_projection->point.x(), 1.0, 1e-9);
  EXPECT_NEAR(lower_projection->point.z(), -3.0, 1e-9);
  EXPECT_NEAR(lower_projection->z_distance, 0.05, 1e-9);
}

TEST(RealInputUtils, TrimmedPathStartsAtProjectionAndContainsOnlyForwardRoute)
{
  nav_msgs::msg::Path path;
  path.poses = {
      pathPose(0.0, 0.0, 0.0), pathPose(1.0, 0.0, 0.0),
      pathPose(2.0, 0.0, 0.0), pathPose(3.0, 0.0, -1.0)};
  const auto projection = scan_planner::projectPointOntoPath3D(
      path, Eigen::Vector3d(1.4, 0.1, 0.0));
  ASSERT_TRUE(projection.has_value());
  const auto trimmed = scan_planner::trimPathFromProjection(path, *projection);
  ASSERT_EQ(trimmed.poses.size(), 3U);
  EXPECT_NEAR(trimmed.poses.front().pose.position.x, 1.4, 1e-9);
  EXPECT_NEAR(trimmed.poses.front().pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(trimmed.poses[1].pose.position.x, 2.0, 1e-9);
  EXPECT_NEAR(trimmed.poses.back().pose.position.x, 3.0, 1e-9);
}

TEST(RealInputUtils, ThreeDimensionalGeometryDetectsFloorChange)
{
  nav_msgs::msg::Path upper;
  nav_msgs::msg::Path lower;
  for (int i = 0; i <= 20; ++i)
  {
    upper.poses.push_back(pathPose(i * 0.2, 0.0, 0.0));
    lower.poses.push_back(pathPose(i * 0.2, 0.0, -3.0));
  }
  EXPECT_NEAR(scan_planner::pathGeometryRms3D(upper, upper, 4.0, 0.2), 0.0, 1e-9);
  EXPECT_GT(scan_planner::pathGeometryRms3D(upper, lower, 4.0, 0.2), 2.99);
}
