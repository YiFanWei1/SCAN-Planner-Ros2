#include "scan_mppi_controller/voxel_map.hpp"

#include <algorithm>
#include <stdexcept>

namespace scan_mppi_controller
{

std::size_t VoxelKeyHash::operator()(const VoxelKey &key) const noexcept
{
  std::size_t seed = std::hash<int>{}(key.x);
  seed ^= std::hash<int>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  seed ^= std::hash<int>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  return seed;
}

VoxelMap::VoxelMap(double resolution) : resolution_(resolution)
{
  if (!std::isfinite(resolution_) || resolution_ <= 0.0)
    throw std::invalid_argument("voxel resolution must be positive");
}

VoxelKey VoxelMap::pointToKey(const Eigen::Vector3d &point) const
{
  return {static_cast<int>(std::floor(point.x() / resolution_)),
          static_cast<int>(std::floor(point.y() / resolution_)),
          static_cast<int>(std::floor(point.z() / resolution_))};
}

Eigen::Vector3d VoxelMap::keyToCenter(const VoxelKey &key) const
{
  return resolution_ *
         Eigen::Vector3d(key.x + 0.5, key.y + 0.5, key.z + 0.5);
}

void VoxelMap::insert(const Eigen::Vector3d &point)
{
  if (point.allFinite())
  {
    const VoxelKey key = pointToKey(point);
    if (occupied_.insert(key).second)
      points_.push_back(keyToCenter(key));
    finalized_ = false;
  }
}

int VoxelMap::buildKdTree(std::vector<Eigen::Vector3d> &points,
                          std::size_t begin, std::size_t end, int depth)
{
  if (begin >= end)
    return -1;
  const int axis = depth % 3;
  const std::size_t middle = begin + (end - begin) / 2U;
  std::nth_element(points.begin() + static_cast<std::ptrdiff_t>(begin),
                   points.begin() + static_cast<std::ptrdiff_t>(middle),
                   points.begin() + static_cast<std::ptrdiff_t>(end),
                   [axis](const Eigen::Vector3d &lhs, const Eigen::Vector3d &rhs) {
                     return lhs(axis) < rhs(axis);
                   });
  const int index = static_cast<int>(kd_nodes_.size());
  KdNode node;
  node.point = points[middle];
  node.axis = axis;
  node.minimum = node.maximum = node.point;
  kd_nodes_.push_back(node);
  const int left = buildKdTree(points, begin, middle, depth + 1);
  const int right = buildKdTree(points, middle + 1U, end, depth + 1);
  KdNode &stored = kd_nodes_[static_cast<std::size_t>(index)];
  stored.left = left;
  stored.right = right;
  for (const int child_index : {left, right})
    if (child_index >= 0)
    {
      const KdNode &child = kd_nodes_[static_cast<std::size_t>(child_index)];
      stored.minimum = stored.minimum.cwiseMin(child.minimum);
      stored.maximum = stored.maximum.cwiseMax(child.maximum);
    }
  return index;
}

void VoxelMap::finalize()
{
  kd_nodes_.clear();
  std::vector<Eigen::Vector3d> working = points_;
  kd_nodes_.reserve(working.size());
  kd_root_ = buildKdTree(working, 0U, working.size(), 0);
  finalized_ = true;
}

void VoxelMap::finalize(const Eigen::Vector3d &center, double radius_xy,
                        double half_height)
{
  if (!center.allFinite() || !std::isfinite(radius_xy) || radius_xy <= 0.0 ||
      !std::isfinite(half_height) || half_height <= 0.0)
    throw std::invalid_argument("invalid local clearance index bounds");

  kd_nodes_.clear();
  std::vector<Eigen::Vector3d> working;
  working.reserve(points_.size() / 4U);
  const double radius_squared = radius_xy * radius_xy;
  for (const auto &point : points_)
  {
    const Eigen::Vector2d delta_xy = point.head<2>() - center.head<2>();
    if (delta_xy.squaredNorm() <= radius_squared &&
        std::abs(point.z() - center.z()) <= half_height)
      working.push_back(point);
  }
  kd_nodes_.reserve(working.size());
  kd_root_ = buildKdTree(working, 0U, working.size(), 0);
  finalized_ = true;
}

bool VoxelMap::occupied(const Eigen::Vector3d &point) const
{
  return point.allFinite() && occupied_.count(pointToKey(point)) != 0U;
}

double VoxelMap::nearestDistance(const Eigen::Vector3d &point,
                                 double max_distance) const
{
  if (!point.allFinite() || occupied_.empty() || !std::isfinite(max_distance) ||
      max_distance < 0.0)
    return std::numeric_limits<double>::infinity();

  double best_squared = max_distance * max_distance;
  if (finalized_ && kd_root_ >= 0)
    nearestKd(kd_root_, point, best_squared);
  else
    for (const auto &occupied_point : points_)
      best_squared = std::min(best_squared, (occupied_point - point).squaredNorm());
  return best_squared < max_distance * max_distance ? std::sqrt(best_squared) :
      std::numeric_limits<double>::infinity();
}

void VoxelMap::nearestKd(int node_index, const Eigen::Vector3d &query,
                         double &best_squared) const
{
  if (node_index < 0)
    return;
  const KdNode &node = kd_nodes_[static_cast<std::size_t>(node_index)];
  double box_distance_squared = 0.0;
  for (int axis = 0; axis < 3; ++axis)
  {
    if (query(axis) < node.minimum(axis))
    {
      const double delta_to_box = node.minimum(axis) - query(axis);
      box_distance_squared += delta_to_box * delta_to_box;
    }
    else if (query(axis) > node.maximum(axis))
    {
      const double delta_to_box = query(axis) - node.maximum(axis);
      box_distance_squared += delta_to_box * delta_to_box;
    }
  }
  if (box_distance_squared > best_squared)
    return;
  best_squared = std::min(best_squared, (node.point - query).squaredNorm());
  const double delta = query(node.axis) - node.point(node.axis);
  const int near_child = delta < 0.0 ? node.left : node.right;
  const int far_child = delta < 0.0 ? node.right : node.left;
  nearestKd(near_child, query, best_squared);
  if (delta * delta <= best_squared)
    nearestKd(far_child, query, best_squared);
}

}  // namespace scan_mppi_controller
