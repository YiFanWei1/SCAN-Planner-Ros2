#include "scan_mppi_controller/mppi_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <stdexcept>

#ifdef SCAN_MPPI_HAS_OPENMP
#include <omp.h>
#endif

namespace scan_mppi_controller
{

double normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

State integrate(const State &state, const Control &control, double dt)
{
  State next = state;
  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);
  next.x += (c * control.vx - s * control.vy) * dt;
  next.y += (s * control.vx + c * control.vy) * dt;
  next.yaw = normalizeAngle(state.yaw + control.wz * dt);
  return next;
}

Control clampControl(const Control &desired, const Control &previous,
                     const MppiConfig &config)
{
  Control result;
  const double linear_step = std::max(0.0, config.linear_accel_max * config.dt);
  const double angular_step = std::max(0.0, config.angular_accel_max * config.dt);
  result.vx = std::clamp(desired.vx, previous.vx - linear_step,
                         previous.vx + linear_step);
  result.vy = std::clamp(desired.vy, previous.vy - linear_step,
                         previous.vy + linear_step);
  result.wz = std::clamp(desired.wz, previous.wz - angular_step,
                         previous.wz + angular_step);
  result.vx = std::clamp(result.vx, config.vx_min, config.vx_max);
  result.vy = std::clamp(result.vy, config.vy_min, config.vy_max);
  result.wz = std::clamp(result.wz, config.wz_min, config.wz_max);
  return result;
}

std::size_t matchReferenceMonotonic(
    const State &state, const std::vector<ReferencePoint> &reference,
    std::size_t begin_index)
{
  if (reference.empty())
    return 0U;
  begin_index = std::min(begin_index, reference.size() - 1U);
  const std::size_t end = std::min(reference.size() - 1U, begin_index + 6U);
  std::size_t best = begin_index;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = begin_index; i <= end; ++i)
  {
    const double distance =
        (reference[i].position.head<2>() - Eigen::Vector2d(state.x, state.y))
            .squaredNorm();
    if (distance < best_distance)
    {
      best_distance = distance;
      best = i;
    }
  }
  return best;
}

namespace
{

bool poseCollision(const State &state, const VoxelMap &map,
                   const MppiConfig &config)
{
  const Eigen::Vector3d center(state.x, state.y, state.z);
  const Eigen::Vector3d offset(config.front_rear_offset * std::cos(state.yaw),
                               config.front_rear_offset * std::sin(state.yaw), 0.0);
  return map.occupied(center + offset) || map.occupied(center - offset);
}

double squared(double value) { return value * value; }

bool sweptCollisionImpl(const State &from, const State &to,
                        const VoxelMap &map, const MppiConfig &config,
                        bool include_start)
{
  const double translation = std::hypot(to.x - from.x, to.y - from.y);
  const double yaw_delta = std::abs(normalizeAngle(to.yaw - from.yaw));
  const std::size_t translation_steps = static_cast<std::size_t>(std::ceil(
      translation / std::max(1e-6, config.collision_translation_step)));
  const std::size_t rotation_steps = static_cast<std::size_t>(std::ceil(
      yaw_delta / std::max(1e-6, config.collision_yaw_step)));
  const std::size_t steps = std::max<std::size_t>(
      1U, std::max(translation_steps, rotation_steps));
  const double signed_yaw_delta = normalizeAngle(to.yaw - from.yaw);
  const std::size_t first_sample = include_start ? 0U : 1U;
  for (std::size_t i = first_sample; i <= steps; ++i)
  {
    const double alpha = static_cast<double>(i) / static_cast<double>(steps);
    State sample;
    sample.x = from.x + alpha * (to.x - from.x);
    sample.y = from.y + alpha * (to.y - from.y);
    sample.z = from.z + alpha * (to.z - from.z);
    sample.yaw = normalizeAngle(from.yaw + alpha * signed_yaw_delta);
    if (poseCollision(sample, map, config))
      return true;
  }
  return false;
}

}  // namespace

bool sweptCollision(const State &from, const State &to, const VoxelMap &map,
                    const MppiConfig &config)
{
  return sweptCollisionImpl(from, to, map, config, true);
}

MppiOptimizer::MppiOptimizer(MppiConfig config)
    : config_(std::move(config)), random_(config_.random_seed),
      active_batch_size_(config_.batch_size)
{
  if (config_.dt <= 0.0 || config_.time_steps == 0U || config_.batch_size == 0U ||
      config_.min_batch_size == 0U || config_.min_batch_size > config_.batch_size ||
      config_.temperature <= 0.0 || config_.worker_threads <= 0 ||
      config_.clearance_stride == 0U)
    throw std::invalid_argument("invalid MPPI configuration");
#ifdef SCAN_MPPI_HAS_OPENMP
  omp_set_dynamic(0);
  omp_set_num_threads(config_.worker_threads);
#endif
  nominal_controls_.assign(config_.time_steps, Control());
}

void MppiOptimizer::reset()
{
  nominal_controls_.assign(config_.time_steps, Control());
  fast_cycle_count_ = 0;
}

void MppiOptimizer::enforceSequenceLimits(std::vector<Control> &controls,
                                          const Control &previous_command) const
{
  Control previous = previous_command;
  for (auto &control : controls)
  {
    control = clampControl(control, previous, config_);
    previous = control;
  }
}

std::vector<Control> MppiOptimizer::makePerturbedControls(
    std::vector<Control> &noise, const Control &previous_command)
{
  std::normal_distribution<double> normal(0.0, 1.0);
  noise.assign(config_.time_steps, Control());
  Control filtered;
  for (std::size_t i = 0; i < config_.time_steps; ++i)
  {
    const Control raw{config_.noise_vx * normal(random_),
                      config_.noise_vy * normal(random_),
                      config_.noise_wz * normal(random_)};
    filtered.vx = config_.noise_smoothing * filtered.vx +
                  (1.0 - config_.noise_smoothing) * raw.vx;
    filtered.vy = config_.noise_smoothing * filtered.vy +
                  (1.0 - config_.noise_smoothing) * raw.vy;
    filtered.wz = config_.noise_smoothing * filtered.wz +
                  (1.0 - config_.noise_smoothing) * raw.wz;
    noise[i] = filtered;
  }

  std::vector<Control> controls = nominal_controls_;
  for (std::size_t i = 0; i < controls.size(); ++i)
  {
    controls[i].vx += noise[i].vx;
    controls[i].vy += noise[i].vy;
    controls[i].wz += noise[i].wz;
  }
  enforceSequenceLimits(controls, previous_command);
  for (std::size_t i = 0; i < controls.size(); ++i)
  {
    noise[i].vx = controls[i].vx - nominal_controls_[i].vx;
    noise[i].vy = controls[i].vy - nominal_controls_[i].vy;
    noise[i].wz = controls[i].wz - nominal_controls_[i].wz;
  }
  return controls;
}

double MppiOptimizer::bodyClearance(
    const State &state, const VoxelMap &map,
    ClearanceCache &clearance_cache) const
{
  const Eigen::Vector3d center(state.x, state.y, state.z);
  const Eigen::Vector3d offset(config_.front_rear_offset * std::cos(state.yaw),
                               config_.front_rear_offset * std::sin(state.yaw), 0.0);
  const double quantization_margin = 0.5 * std::sqrt(3.0) * map.resolution();
  const double search_distance =
      config_.safety_distance + map.resolution() + quantization_margin;
  const auto cached_distance = [&](const Eigen::Vector3d &query) {
    const VoxelKey key = map.pointToKey(query);
    const auto found = clearance_cache.find(key);
    if (found != clearance_cache.end())
      return found->second;
    const double center_distance = map.nearestDistance(
        map.keyToCenter(key), search_distance);
    const double conservative_distance = std::isfinite(center_distance) ?
        std::max(0.0, center_distance - quantization_margin) :
        std::numeric_limits<double>::infinity();
    clearance_cache.emplace(key, conservative_distance);
    return conservative_distance;
  };
  return std::min(cached_distance(center + offset),
                  cached_distance(center - offset));
}

Candidate MppiOptimizer::evaluate(
    const State &initial, const std::vector<ReferencePoint> &reference,
    const VoxelMap &map, std::vector<Control> controls,
    std::vector<Control> noise, const Control &previous_command,
    ClearanceCache &clearance_cache) const
{
  Candidate candidate;
  candidate.controls = std::move(controls);
  candidate.noise = std::move(noise);
  candidate.states.reserve(candidate.controls.size() + 1U);
  State current = initial;
  current.reference_index = 0U;
  if (!reference.empty())
    current.z = reference.front().position.z();
  candidate.states.push_back(current);
  candidate.cost = 0.0;
  Control previous = previous_command;

  if (poseCollision(current, map, config_))
  {
    candidate.collision = true;
    candidate.cost = std::numeric_limits<double>::infinity();
    return candidate;
  }

  for (std::size_t i = 0; i < candidate.controls.size(); ++i)
  {
    State next = integrate(current, candidate.controls[i], config_.dt);
    next.reference_index = matchReferenceMonotonic(
        next, reference, current.reference_index);
    const ReferencePoint &target = reference[next.reference_index];
    next.z = target.position.z();

    if (sweptCollisionImpl(current, next, map, config_, false))
    {
      candidate.collision = true;
      candidate.cost = std::numeric_limits<double>::infinity();
      candidate.states.push_back(next);
      return candidate;
    }

    const Eigen::Vector2d error =
        Eigen::Vector2d(next.x, next.y) - target.position.head<2>();
    candidate.cost += config_.weight_path * error.squaredNorm();
    candidate.cost += config_.weight_heading *
                      squared(normalizeAngle(next.yaw - target.yaw));

    const double c = std::cos(next.yaw);
    const double s = std::sin(next.yaw);
    const Eigen::Vector2d world_velocity(
        c * candidate.controls[i].vx - s * candidate.controls[i].vy,
        s * candidate.controls[i].vx + c * candidate.controls[i].vy);
    candidate.cost += config_.weight_velocity *
        (world_velocity - target.velocity.head<2>()).squaredNorm();
    candidate.cost += config_.weight_control *
        (squared(candidate.controls[i].vx) + squared(candidate.controls[i].vy) +
         0.25 * squared(candidate.controls[i].wz));
    candidate.cost += config_.weight_smooth *
        (squared(candidate.controls[i].vx - previous.vx) +
         squared(candidate.controls[i].vy - previous.vy) +
         0.25 * squared(candidate.controls[i].wz - previous.wz));

    if (i % config_.clearance_stride == 0U ||
        i + 1U == candidate.controls.size())
    {
      const double clearance = bodyClearance(next, map, clearance_cache);
      candidate.min_clearance = std::min(candidate.min_clearance, clearance);
      if (std::isfinite(clearance) && clearance < config_.safety_distance)
        candidate.cost += config_.weight_clearance *
                          squared(config_.safety_distance - clearance);
      if (std::isfinite(clearance) && clearance < config_.critical_distance)
        candidate.cost += config_.critical_cost *
                          squared(config_.critical_distance - clearance + 1.0);
    }

    const double progress = reference.size() <= 1U ? 0.0 :
        static_cast<double>(next.reference_index) /
        static_cast<double>(reference.size() - 1U);
    candidate.cost -= config_.weight_progress * progress;
    previous = candidate.controls[i];
    current = next;
    candidate.states.push_back(current);
  }

  const ReferencePoint &terminal = reference.back();
  candidate.cost += config_.weight_terminal *
      (Eigen::Vector2d(current.x, current.y) - terminal.position.head<2>())
          .squaredNorm();
  return candidate;
}

MppiResult MppiOptimizer::optimize(
    const State &initial, const std::vector<ReferencePoint> &reference,
    const VoxelMap &map, const Control &previous_command)
{
  const auto started = std::chrono::steady_clock::now();
  MppiResult result;
  result.active_batch_size = active_batch_size_;
  if (reference.size() < 2U)
  {
    result.reason = "reference_too_short";
    return result;
  }
  if (map.size() == 0U)
  {
    result.reason = "empty_map";
    return result;
  }
  if (nominal_controls_.size() != config_.time_steps)
    reset();
  enforceSequenceLimits(nominal_controls_, previous_command);

  std::vector<std::vector<Control>> controls(active_batch_size_);
  std::vector<std::vector<Control>> noises(active_batch_size_);
  for (std::size_t i = 0; i < active_batch_size_; ++i)
    controls[i] = makePerturbedControls(noises[i], previous_command);

  result.candidates.resize(active_batch_size_);
#ifdef SCAN_MPPI_HAS_OPENMP
#pragma omp parallel
  {
    ClearanceCache clearance_cache;
    clearance_cache.reserve(config_.time_steps * 64U);
#pragma omp for schedule(static)
    for (std::int64_t i = 0; i < static_cast<std::int64_t>(active_batch_size_); ++i)
      result.candidates[static_cast<std::size_t>(i)] = evaluate(
          initial, reference, map,
          std::move(controls[static_cast<std::size_t>(i)]),
          std::move(noises[static_cast<std::size_t>(i)]), previous_command,
          clearance_cache);
  }
#else
  ClearanceCache clearance_cache;
  clearance_cache.reserve(config_.time_steps * 64U);
  for (std::int64_t i = 0; i < static_cast<std::int64_t>(active_batch_size_); ++i)
    result.candidates[static_cast<std::size_t>(i)] = evaluate(
        initial, reference, map,
        std::move(controls[static_cast<std::size_t>(i)]),
        std::move(noises[static_cast<std::size_t>(i)]), previous_command,
        clearance_cache);
#endif

  double minimum_cost = std::numeric_limits<double>::infinity();
  std::size_t best_index = 0U;
  for (std::size_t i = 0; i < result.candidates.size(); ++i)
  {
    const auto &candidate = result.candidates[i];
    if (!candidate.collision && std::isfinite(candidate.cost))
    {
      ++result.safe_candidates;
      if (candidate.cost < minimum_cost)
      {
        minimum_cost = candidate.cost;
        best_index = i;
      }
    }
  }
  if (result.safe_candidates == 0U)
  {
    reset();
    result.reason = "no_safe_trajectory";
  }
  else
  {
    std::vector<double> weights(result.candidates.size(), 0.0);
    double weight_sum = 0.0;
    for (std::size_t i = 0; i < result.candidates.size(); ++i)
    {
      if (result.candidates[i].collision || !std::isfinite(result.candidates[i].cost))
        continue;
      weights[i] = std::exp(-(result.candidates[i].cost - minimum_cost) /
                            config_.temperature);
      weight_sum += weights[i];
    }
    if (weight_sum > 1e-12)
      for (std::size_t t = 0; t < nominal_controls_.size(); ++t)
      {
        Control delta;
        for (std::size_t i = 0; i < result.candidates.size(); ++i)
        {
          delta.vx += weights[i] * result.candidates[i].noise[t].vx;
          delta.vy += weights[i] * result.candidates[i].noise[t].vy;
          delta.wz += weights[i] * result.candidates[i].noise[t].wz;
        }
        nominal_controls_[t].vx += delta.vx / weight_sum;
        nominal_controls_[t].vy += delta.vy / weight_sum;
        nominal_controls_[t].wz += delta.wz / weight_sum;
      }
    enforceSequenceLimits(nominal_controls_, previous_command);
    const std::vector<Control> zero_noise(config_.time_steps);
    ClearanceCache optimal_clearance_cache;
    Candidate updated = evaluate(initial, reference, map, nominal_controls_,
                                 zero_noise, previous_command,
                                 optimal_clearance_cache);
    if (updated.collision || !std::isfinite(updated.cost))
    {
      updated = result.candidates[best_index];
      nominal_controls_ = updated.controls;
    }
    result.valid = true;
    result.reason = "tracking";
    result.optimal = std::move(updated);
    result.command = nominal_controls_.front();
    if (nominal_controls_.size() > 1U)
    {
      for (std::size_t i = 1; i < nominal_controls_.size(); ++i)
        nominal_controls_[i - 1U] = nominal_controls_[i];
      nominal_controls_.back() = nominal_controls_[nominal_controls_.size() - 2U];
    }
  }

  result.solve_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  if (result.solve_time_ms > config_.max_solve_time_ms &&
      active_batch_size_ > config_.min_batch_size)
  {
    active_batch_size_ = std::max(config_.min_batch_size, active_batch_size_ - 50U);
    fast_cycle_count_ = 0;
  }
  else if (result.solve_time_ms < 0.8 * config_.max_solve_time_ms &&
           active_batch_size_ < config_.batch_size)
  {
    if (++fast_cycle_count_ >= 10)
    {
      active_batch_size_ = std::min(config_.batch_size, active_batch_size_ + 25U);
      fast_cycle_count_ = 0;
    }
  }
  else
  {
    fast_cycle_count_ = 0;
  }
  return result;
}

}  // namespace scan_mppi_controller
