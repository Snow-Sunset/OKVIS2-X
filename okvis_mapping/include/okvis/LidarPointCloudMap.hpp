// SPDX-License-Identifier: BSD-3-Clause
#ifndef OKVIS_LIDAR_POINT_CLOUD_MAP_HPP
#define OKVIS_LIDAR_POINT_CLOUD_MAP_HPP

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
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
  struct ArchiveOptions {
    std::string directory;
    size_t maxResidentSubmaps = 32;
    // Sequential disk write size, independent of RAM submap retention.
    size_t chunkPointLimit = 262144;
  };
  struct ArchiveChunk {
    std::string filename;
    uint64_t pointCount = 0;
  };
  struct SubmapSnapshot {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Isometry3d pose;
    std::shared_ptr<const Points> points;
    std::shared_ptr<const ArchiveChunk> archive;
  };
  using Snapshot = std::vector<SubmapSnapshot, Eigen::aligned_allocator<SubmapSnapshot>>;
  explicit LidarPointCloudMap(double voxelSize);
  /// RAM-only map retaining up to maxResidentSubmaps populated submaps,
  /// including the active one. Entire old submaps are removed; no byte cap.
  LidarPointCloudMap(double voxelSize, size_t maxResidentSubmaps);
  /// The configured directory contains a unique, persistent session directory.
  /// Successful immutable chunk writes precede releasing resident point data.
  LidarPointCloudMap(double voxelSize, const ArchiveOptions& archive);

  /// Keep a coarse, bounded full-trajectory preview for RViz while the
  /// high-resolution realtime map uses a sliding local window. Preview points
  /// stay in submap-local coordinates so estimator pose corrections remain
  /// applicable. The voxel size increases automatically if maxPoints is hit.
  void enableGlobalPreview(double voxelSize, size_t maxPoints);

  void addPoints(uint64_t id, const Eigen::Isometry3d& initialPose, const Points& points);
  /// Estimator poses take precedence over older, queued sensor packets.
  void updatePose(uint64_t id, const Eigen::Isometry3d& pose, bool fromEstimator = false, uint64_t estimatorSequence = 0);
  /// Keep only voxels inside a world-frame axis-aligned local window. This is
  /// the realtime mode equivalent of FAST-LIVO2 map sliding.
  void setLocalWindow(const Eigen::Vector3d& center, double halfExtent);
  bool hasSubmap(uint64_t id) const {
    return submaps_.count(id) != 0 || previewSubmaps_.count(id) != 0;
  }
  Points pointCloud() const;
  Snapshot snapshot() const;
  /// Snapshot used for visualization/export. Returns the bounded global
  /// preview when enabled, otherwise the ordinary map snapshot.
  Snapshot visualizationSnapshot() const;
  void finalizeSubmap(uint64_t id);
  void flush();
  size_t residentBytes() const;
  size_t residentSubmapCount() const;
  uint64_t archivedPointCount() const;
  const std::string& archiveDirectory() const { return archiveDirectory_; }
  /// Assemble outside the owner's lock. maxPoints limits sampled input work;
  /// zero retains all points for complete PCD export.
  static Points assemble(const Snapshot& snapshot, double voxelSize, size_t maxPoints = 0);
  /// Transform every retained point to world coordinates, without display
  /// sampling, voxel filtering, deduplication or a point-count limit.
  static Points assembleUnfiltered(const Snapshot& snapshot);
  /// Complete globally voxel-deduplicated binary PCD export. External sorted
  /// runs keep RAM bounded; an atomic rename publishes the completed result.
  /// Returns false for an empty map. I/O failures throw without deleting data.
  static bool savePcd(const Snapshot& snapshot, double voxelSize,
                      const std::string& filename, size_t workspaceBytes = 8 * 1024 * 1024);
  uint64_t revision() const { return revision_; }
  uint64_t visualizationRevision() const {
    return globalPreviewEnabled_ ? previewRevision_ : revision_;
  }
  double visualizationVoxelSize() const {
    return globalPreviewEnabled_ ? globalPreviewVoxelSize_ : voxelSize_;
  }

 private:
  using Key = std::array<int64_t, 3>;
  struct Hash { size_t operator()(const Key& key) const; };
  using Voxels = std::unordered_map<Key, Eigen::Vector3d, Hash>;
  struct Submap {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    bool estimatorPose = false;
    uint64_t estimatorSequence = 0;
    uint64_t lastPointUpdate = 0;
    mutable std::shared_ptr<const Points> cachedPoints;
    Voxels voxels;
    std::vector<std::shared_ptr<const ArchiveChunk>> chunks;
  };
  Key key(const Eigen::Vector3d& point) const;
  Key key(const Eigen::Vector3d& point, double voxelSize) const;
  bool insideLocalWindow(const Eigen::Vector3d& world) const;
  bool insert(Voxels& voxels, const Eigen::Vector3d& point) const;
  bool insert(Voxels& voxels, const Eigen::Vector3d& point, double voxelSize) const;
  Snapshot snapshot(const std::map<uint64_t, Submap>& submaps) const;
  void addPreviewPoints(uint64_t id, const Submap& poseSource, const Points& points);
  void enforcePreviewLimit();
  void archiveSubmap(uint64_t id, Submap& submap);
  void writeSubmapManifest(uint64_t id, const Submap& submap) const;
  void writeManifest() const;
  void enforceSubmapLimit(uint64_t activeId);
  double voxelSize_;
  uint64_t revision_ = 0;
  std::map<uint64_t, Submap> submaps_;
  std::string archiveDirectory_;
  size_t maxResidentSubmaps_ = 0; ///< zero only for the unbounded constructor
  uint64_t pointUpdateClock_ = 0;
  size_t chunkPointLimit_ = 0;
  uint64_t nextChunk_ = 0;
  bool localWindowEnabled_ = false;
  Eigen::Vector3d localWindowCenter_ = Eigen::Vector3d::Zero();
  double localWindowHalfExtent_ = 0.0;
  Eigen::Vector3d localWindowSweepCenter_ = Eigen::Vector3d::Zero();
  double localWindowSweepHalfExtent_ = 0.0;
  bool localWindowSweepInitialized_ = false;
  bool globalPreviewEnabled_ = false;
  double globalPreviewVoxelSize_ = 0.0;
  size_t globalPreviewMaxPoints_ = 0;
  size_t globalPreviewPointCount_ = 0;
  uint64_t previewRevision_ = 0;
  std::map<uint64_t, Submap> previewSubmaps_;
};

} // namespace okvis
#endif
