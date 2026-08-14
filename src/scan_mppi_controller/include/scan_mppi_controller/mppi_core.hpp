#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "scan_mppi_controller/voxel_map.hpp"

namespace scan_mppi_controller
{

struct State
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  std::size_t reference_index{0};
};

struct Control
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct ReferencePoint
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

struct MppiConfig
{
  double dt{0.05};
  std::size_t time_steps{24};
  std::size_t batch_size{800};
  std::size_t min_batch_size{600};
  int worker_threads{8};
  double max_solve_time_ms{33.0};
  double vx_min{-0.20};
  double vx_max{0.50};
  double vy_min{-0.35};
  double vy_max{0.35};
  double wz_min{-0.80};
  double wz_max{0.80};
  double linear_accel_max{0.50};
  double angular_accel_max{1.0};
  double noise_vx{0.20};
  double noise_vy{0.15};
  double noise_wz{0.35};
  double noise_smoothing{0.65};
  double temperature{1.0};
  double front_rear_offset{0.18};
  double collision_translation_step{0.025};
  double collision_yaw_step{0.0872664626};
  double safety_distance{0.20};
  double critical_distance{0.08};
  std::size_t clearance_stride{3};
  double weight_path{15.0};
  double weight_heading{3.0};
  double weight_velocity{2.0};
  double weight_control{0.20};
  double weight_smooth{4.0};
  double weight_clearance{40.0};
  double weight_progress{4.0};
  double weight_terminal{20.0};
  double critical_cost{10000.0};
  std::uint32_t random_seed{42};
};

struct Candidate
{
  std::vector<State> states;
  std::vector<Control> controls;
  std::vector<Control> noise;
  double cost{std::numeric_limits<double>::infinity()};
  double min_clearance{std::numeric_limits<double>::infinity()};
  bool collision{false};
};

struct MppiResult
{
  bool valid{false};
  Control command;
  Candidate optimal;
  std::vector<Candidate> candidates;
  std::size_t safe_candidates{0};
  std::size_t active_batch_size{0};
  double solve_time_ms{0.0};
  std::string reason;
};

double normalizeAngle(double angle);
State integrate(const State &state, const Control &control, double dt);
Control clampControl(const Control &desired, const Control &previous,
                     const MppiConfig &config);
std::size_t matchReferenceMonotonic(const State &state,
                                    const std::vector<ReferencePoint> &reference,
                                    std::size_t begin_index);
bool sweptCollision(const State &from, const State &to, const VoxelMap &map,
                    const MppiConfig &config);

class MppiOptimizer
{
public:
  explicit MppiOptimizer(MppiConfig config = MppiConfig());

  MppiResult optimize(const State &initial,
                      const std::vector<ReferencePoint> &reference,
                      const VoxelMap &map, const Control &previous_command);
  void reset();
  const MppiConfig &config() const { return config_; }

private:
  using ClearanceCache = std::unordered_map<VoxelKey, double, VoxelKeyHash>;

  Candidate evaluate(const State &initial,
                     const std::vector<ReferencePoint> &reference,
                     const VoxelMap &map,
                     std::vector<Control> controls,
                     std::vector<Control> noise,
                     const Control &previous_command,
                     ClearanceCache &clearance_cache) const;
  std::vector<Control> makePerturbedControls(std::vector<Control> &noise,
                                              const Control &previous_command);
  void enforceSequenceLimits(std::vector<Control> &controls,
                             const Control &previous_command) const;
  double bodyClearance(const State &state, const VoxelMap &map,
                       ClearanceCache &clearance_cache) const;

  MppiConfig config_;
  std::vector<Control> nominal_controls_;
  std::mt19937 random_;
  std::size_t active_batch_size_;
  int fast_cycle_count_{0};
};

}  // namespace scan_mppi_controller
