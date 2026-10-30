// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/LidarPointCloudMap.hpp>
#include <cmath>
#include <stdexcept>

namespace okvis {

LidarPointCloudMap::LidarPointCloudMap(double voxelSize) : voxelSize_(voxelSize) {
  if (!(voxelSize > 0.0) || !std::isfinite(voxelSize)) {
    throw std::invalid_argument("pointcloud_map_voxel_size must be finite and positive");
  }
}

size_t LidarPointCloudMap::Hash::operator()(const Key& key) const {
  size_t seed = 0;
  for (const auto value : key) {
    seed ^= std::hash<int64_t>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }
  return seed;
}

LidarPointCloudMap::Key LidarPointCloudMap::key(const Eigen::Vector3d& point) const {
  return {{int64_t(std::floor(point.x() / voxelSize_)),
           int64_t(std::floor(point.y() / voxelSize_)),
           int64_t(std::floor(point.z() / voxelSize_))}};
}

bool LidarPointCloudMap::insert(Voxels& voxels, const Eigen::Vector3d& point) const {
  if (!point.allFinite()) return false;
  const auto index = key(point);
  const auto result = voxels.emplace(index, point);
  if (result.second) return true;
  const Eigen::Vector3d center = voxelSize_ * Eigen::Vector3d(
      double(index[0]) + 0.5, double(index[1]) + 0.5, double(index[2]) + 0.5);
  // Keep a measured point closest to the cell center, rather than an average
  // that could smear two surfaces. Equal-distance ties retain the older point.
  if ((point - center).squaredNorm() < (result.first->second - center).squaredNorm()) {
    result.first->second = point;
    return true;
  }
  return false;
}

void LidarPointCloudMap::updatePose(uint64_t id, const Eigen::Isometry3d& pose,
                                   bool fromEstimator, uint64_t estimatorSequence) {
  if (!pose.matrix().allFinite()) return;
  auto& submap = submaps_[id];
  if (submap.estimatorPose && !fromEstimator) return;
  if (fromEstimator && estimatorSequence < submap.estimatorSequence) return;
  if (fromEstimator) submap.estimatorSequence = estimatorSequence;
  submap.estimatorPose = submap.estimatorPose || fromEstimator;
  if ((submap.pose.matrix() - pose.matrix()).cwiseAbs().maxCoeff() > 1e-12) {
    submap.pose = pose;
    if (!submap.voxels.empty()) ++revision_;
  }
}

void LidarPointCloudMap::addPoints(uint64_t id, const Eigen::Isometry3d& initialPose,
                                  const Points& points) {
  if (!initialPose.matrix().allFinite()) return;
  updatePose(id, initialPose);
  auto& submap = submaps_.at(id);
  bool changed = false;
  for (const auto& point : points) changed = insert(submap.voxels, point) || changed;
  if (changed) {
    submap.cachedPoints.reset();
    ++revision_;
  }
}

LidarPointCloudMap::Snapshot LidarPointCloudMap::snapshot() const {
  Snapshot result;
  result.reserve(submaps_.size());
  for (const auto& entry : submaps_) {
    const auto& submap = entry.second;
    if (submap.voxels.empty()) continue;
    if (!submap.cachedPoints) {
      auto points = std::make_shared<Points>();
      points->reserve(submap.voxels.size());
      for (const auto& voxel : submap.voxels) points->push_back(voxel.second);
      submap.cachedPoints = std::move(points);
    }
    result.push_back({submap.pose, submap.cachedPoints});
  }
  return result;
}

LidarPointCloudMap::Points LidarPointCloudMap::assemble(const Snapshot& snapshot,
    double voxelSize, size_t maxPoints) {
  LidarPointCloudMap sampler(voxelSize);
  size_t total = 0;
  for (const auto& submap : snapshot) total += submap.points->size();
  const size_t stride = maxPoints && total > maxPoints ? 1 + (total - 1) / maxPoints : 1;
  Voxels world;
  world.reserve(stride == 1 ? total : 1 + (total - 1) / stride);
  size_t offset = 0, next = 0;
  for (const auto& submap : snapshot) {
    const size_t end = offset + submap.points->size();
    while (next < end) {
      sampler.insert(world, submap.pose * submap.points->at(next - offset));
      next += stride;
    }
    offset = end;
  }
  Points points;
  points.reserve(world.size());
  for (const auto& voxel : world) points.push_back(voxel.second);
  return points;
}

LidarPointCloudMap::Points LidarPointCloudMap::pointCloud() const {
  return assemble(snapshot(), voxelSize_);
}

} // namespace okvis
