#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "scan_mppi_controller/mppi_core.hpp"
#include "scan_mppi_controller/voxel_map.hpp"

using scan_mppi_controller::Candidate;
using scan_mppi_controller::Control;
using scan_mppi_controller::MppiConfig;
using scan_mppi_controller::MppiOptimizer;
using scan_mppi_controller::ReferencePoint;
using scan_mppi_controller::State;
using scan_mppi_controller::VoxelMap;

namespace
{

std::vector<ReferencePoint> straightReference(std::size_t count, double step,
                                               double z = 0.4)
{
  std::vector<ReferencePoint> result;
  for (std::size_t i = 0; i < count; ++i)
  {
    ReferencePoint point;
    point.position = Eigen::Vector3d(step * static_cast<double>(i), 0.0, z);
    point.velocity = Eigen::Vector3d(0.3, 0.0, 0.0);
    result.push_back(point);
  }
  return result;
}

}  // namespace

TEST(MotionModel, IntegratesBodyFrameHolonomicVelocity)
{
  State initial;
  initial.yaw = M_PI_2;
  const State next = scan_mppi_controller::integrate(
      initial, Control{1.0, 0.5, 0.2}, 1.0);
  EXPECT_NEAR(next.x, -0.5, 1e-9);
  EXPECT_NEAR(next.y, 1.0, 1e-9);
  EXPECT_NEAR(next.yaw, M_PI_2 + 0.2, 1e-9);
}

TEST(ControlLimits, EnforcesAccelerationAndAbsoluteBounds)
{
  MppiConfig config;
  config.dt = 0.1;
  config.linear_accel_max = 0.5;
  config.angular_accel_max = 1.0;
  const Control clamped = scan_mppi_controller::clampControl(
      Control{4.0, -4.0, 4.0}, Control{}, config);
  EXPECT_DOUBLE_EQ(clamped.vx, 0.05);
  EXPECT_DOUBLE_EQ(clamped.vy, -0.05);
  EXPECT_DOUBLE_EQ(clamped.wz, 0.1);
}

TEST(VoxelMap, PreservesThreeDimensionalSeparation)
{
  VoxelMap map(0.05);
  map.insert(Eigen::Vector3d(1.0, 2.0, 1.0));
  EXPECT_TRUE(map.occupied(Eigen::Vector3d(1.01, 2.01, 1.01)));
  EXPECT_FALSE(map.occupied(Eigen::Vector3d(1.01, 2.01, 0.40)));
  EXPECT_FALSE(std::isfinite(
      map.nearestDistance(Eigen::Vector3d(1.0, 2.0, 0.40), 0.2)));
}

TEST(ReferenceMatching, IsMonotonicAndUsesLocalForwardWindow)
{
  const auto reference = straightReference(20, 0.1);
  State state;
  state.x = 0.76;
  EXPECT_EQ(scan_mppi_controller::matchReferenceMonotonic(state, reference, 4), 8U);
  state.x = 0.1;
  EXPECT_EQ(scan_mppi_controller::matchReferenceMonotonic(state, reference, 8), 8U);
}

TEST(CollisionChecking, DetectsThinObstacleBetweenPredictionSamples)
{
  VoxelMap map(0.05);
  map.insert(Eigen::Vector3d(0.5, 0.0, 0.4));
  MppiConfig config;
  config.front_rear_offset = 0.0;
  config.collision_translation_step = 0.025;
  State from;
  from.z = 0.4;
  State to = from;
  to.x = 1.0;
  EXPECT_TRUE(scan_mppi_controller::sweptCollision(from, to, map, config));
}

TEST(CollisionChecking, DoesNotProjectDifferentHeightObstacleIntoPath)
{
  VoxelMap map(0.05);
  map.insert(Eigen::Vector3d(0.5, 0.0, 1.4));
  MppiConfig config;
  config.front_rear_offset = 0.0;
  State from;
  from.z = 0.4;
  State to = from;
  to.x = 1.0;
  EXPECT_FALSE(scan_mppi_controller::sweptCollision(from, to, map, config));
}

TEST(MppiOptimizer, GroundBelowBodyDoesNotCausePersistentStop)
{
  MppiConfig config;
  config.time_steps = 6;
  config.batch_size = 40;
  config.min_batch_size = 40;
  config.front_rear_offset = 0.0;
  VoxelMap map(0.05);
  for (int x = -5; x <= 20; ++x)
    for (int y = -5; y <= 5; ++y)
      map.insert(Eigen::Vector3d(0.05 * x, 0.05 * y, 0.0));
  map.finalize();
  MppiOptimizer optimizer(config);
  State initial;
  initial.z = 0.4;
  const auto result = optimizer.optimize(
      initial, straightReference(8, 0.05), map, Control{});
  EXPECT_TRUE(result.valid);
  EXPECT_GT(result.safe_candidates, 0U);
}

TEST(MppiOptimizer, ReturnsStopWhenAllCandidatesAreBlocked)
{
  MppiConfig config;
  config.time_steps = 6;
  config.batch_size = 20;
  config.min_batch_size = 20;
  config.front_rear_offset = 0.0;
  config.random_seed = 3;
  VoxelMap map(0.05);
  map.insert(Eigen::Vector3d(0.0, 0.0, 0.4));
  MppiOptimizer optimizer(config);
  State initial;
  initial.z = 0.4;
  const auto result = optimizer.optimize(
      initial, straightReference(8, 0.05), map, Control{});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "no_safe_trajectory");
  EXPECT_DOUBLE_EQ(result.command.vx, 0.0);
}

TEST(MppiOptimizer, TracksAFreeReferenceWithFiniteCommand)
{
  MppiConfig config;
  config.time_steps = 8;
  config.batch_size = 80;
  config.min_batch_size = 80;
  config.front_rear_offset = 0.0;
  config.random_seed = 9;
  VoxelMap map(0.05);
  map.insert(Eigen::Vector3d(3.0, 3.0, 3.0));
  MppiOptimizer optimizer(config);
  State initial;
  initial.z = 0.4;
  const auto result = optimizer.optimize(
      initial, straightReference(10, 0.05), map, Control{});
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(std::isfinite(result.command.vx));
  EXPECT_GT(result.safe_candidates, 0U);
  EXPECT_EQ(result.optimal.states.size(), config.time_steps + 1U);
}

TEST(MppiOptimizer, BiasesAwayFromAnInflatedWallBesideTheReference)
{
  MppiConfig config;
  config.time_steps = 16;
  config.batch_size = 300;
  config.min_batch_size = 300;
  config.clearance_stride = 1;
  config.weight_clearance = 400.0;
  config.weight_path = 5.0;
  config.random_seed = 17;
  VoxelMap map(0.05);
  for (int x = -10; x <= 30; ++x)
    map.insert(Eigen::Vector3d(0.05 * x, 0.15, 0.4));
  map.finalize();
  MppiOptimizer optimizer(config);
  State initial;
  initial.z = 0.4;
  const auto result = optimizer.optimize(
      initial, straightReference(18, 0.025), map, Control{});
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.command.vy, 0.0);
}

TEST(MppiPerformance, DefaultHorizonUsesBoundedCpuTime)
{
  MppiConfig config;
  config.batch_size = 800;
  config.min_batch_size = 600;
  config.time_steps = 24;
  config.max_solve_time_ms = 0.001;
  config.random_seed = 11;
  VoxelMap map(0.05);
  for (int x = -20; x <= 40; ++x)
    for (int z = 0; z <= 20; ++z)
    {
      map.insert(Eigen::Vector3d(0.05 * x, -1.0, 0.05 * z));
      map.insert(Eigen::Vector3d(0.05 * x, 1.0, 0.05 * z));
    }
  map.finalize();
  MppiOptimizer optimizer(config);
  State initial;
  initial.z = 0.4;
  auto result = optimizer.optimize(
      initial, straightReference(25, 0.02), map, Control{});
  ASSERT_TRUE(result.valid);
  std::cout << "Default 800x24 MPPI solve time: " << result.solve_time_ms << " ms\n";
  EXPECT_LT(result.solve_time_ms, 250.0);
  for (int i = 0; i < 4; ++i)
    result = optimizer.optimize(initial, straightReference(25, 0.02), map, Control{});
  EXPECT_EQ(result.active_batch_size, 600U);
}
