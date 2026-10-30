// SPDX-License-Identifier: BSD-3-Clause
#ifndef OKVIS_LIDAR_POINT_CLOUD_MAP_HPP
#define OKVIS_LIDAR_POINT_CLOUD_MAP_HPP

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

namespace okvis {

/// Voxel-sampled LiDAR endpoints in local submap frames. World snapshots use
/// the latest anchor poses and are voxel-sampled again across submap overlaps.
/// Callers provide synchronization.
class LidarPointCloudMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Points = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
  struct SubmapSnapshot {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Isometry3d pose;
    std::shared_ptr<const Points> points;
  };
  using Snapshot = std::vector<SubmapSnapshot, Eigen::aligned_allocator<SubmapSnapshot>>;
  explicit LidarPointCloudMap(double voxelSize);

  void addPoints(uint64_t id, const Eigen::Isometry3d& initialPose, const Points& points);
  /// Estimator poses take precedence over older, queued sensor packets.
  void updatePose(uint64_t id, const Eigen::Isometry3d& pose, bool fromEstimator = false, uint64_t estimatorSequence = 0);
  bool hasSubmap(uint64_t id) const { return submaps_.count(id) != 0; }
  Points pointCloud() const;
  Snapshot snapshot() const;
  /// Assemble outside the owner's lock. maxPoints limits sampled input work;
  /// zero retains all points for complete PCD export.
  static Points assemble(const Snapshot& snapshot, double voxelSize, size_t maxPoints = 0);
  uint64_t revision() const { return revision_; }

 private:
  using Key = std::array<int64_t, 3>;
  struct Hash { size_t operator()(const Key& key) const; };
  using Voxels = std::unordered_map<Key, Eigen::Vector3d, Hash>;
  struct Submap {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    bool estimatorPose = false;
    uint64_t estimatorSequence = 0;
    mutable std::shared_ptr<const Points> cachedPoints;
    Voxels voxels;
  };
  Key key(const Eigen::Vector3d& point) const;
  bool insert(Voxels& voxels, const Eigen::Vector3d& point) const;
  double voxelSize_;
  uint64_t revision_ = 0;
  std::map<uint64_t, Submap> submaps_;
};

} // namespace okvis
#endif
