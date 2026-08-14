#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

namespace scan_mppi_controller
{

struct VoxelKey
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelKey &other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey &key) const noexcept;
};

class VoxelMap
{
public:
  explicit VoxelMap(double resolution = 0.05);

  void insert(const Eigen::Vector3d &point);
  void finalize();
  void finalize(const Eigen::Vector3d &center, double radius_xy,
                double half_height);
  bool occupied(const Eigen::Vector3d &point) const;
  double nearestDistance(const Eigen::Vector3d &point, double max_distance) const;
  std::size_t size() const { return occupied_.size(); }
  double resolution() const { return resolution_; }

  VoxelKey pointToKey(const Eigen::Vector3d &point) const;
  Eigen::Vector3d keyToCenter(const VoxelKey &key) const;

private:
  struct KdNode
  {
    Eigen::Vector3d point{Eigen::Vector3d::Zero()};
    int axis{0};
    int left{-1};
    int right{-1};
    Eigen::Vector3d minimum{Eigen::Vector3d::Zero()};
    Eigen::Vector3d maximum{Eigen::Vector3d::Zero()};
  };

  int buildKdTree(std::vector<Eigen::Vector3d> &points, std::size_t begin,
                  std::size_t end, int depth);
  void nearestKd(int node_index, const Eigen::Vector3d &query,
                 double &best_squared) const;

  double resolution_;
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied_;
  std::vector<Eigen::Vector3d> points_;
  std::vector<KdNode> kd_nodes_;
  int kd_root_{-1};
  bool finalized_{false};
};

}  // namespace scan_mppi_controller
