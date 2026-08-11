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
