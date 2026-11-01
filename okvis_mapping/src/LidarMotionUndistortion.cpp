/**
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense 
 * Depth or LiDAR, and GNSS
 *
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich 
 * and ETH Zurich
 *
 * SPDX-License-Identifier: BSD-3-Clause, see LICENESE file for details
 */

#include <okvis/LidarMotionUndistortion.hpp>
#include <okvis/SubmappingUtils.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace okvis{
namespace {
struct SpatialCandidate {
  uint64_t priority;
  size_t index;
  bool initial = false;
  bool matched = false;
};

bool candidatePriorityLess(const SpatialCandidate& a, const SpatialCandidate& b) {
  return a.priority < b.priority || (a.priority == b.priority && a.index < b.index);
}

std::vector<SpatialCandidate> spatialCandidates(const Point3Cloud<float>& points,
                                               size_t maximum, double resolution) {
  const auto mix = [](uint64_t value) {
    value = (value ^ (value >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    value = (value ^ (value >> 27)) * UINT64_C(0x94d049bb133111eb);
    return value ^ (value >> 31);
  };
  std::vector<SpatialCandidate> candidates;
  candidates.reserve(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    const Eigen::Array3d index = (points[i].cast<double>().array() / resolution).floor();
    if (!index.isFinite().all()
        || (index < double(std::numeric_limits<int>::min())).any()
        || (index > double(std::numeric_limits<int>::max())).any()) continue;
    const Voxel voxel = index.cast<int>().matrix();
    uint64_t priority = mix(uint64_t(uint32_t(voxel.x())) + UINT64_C(0x9e3779b97f4a7c15));
    priority = mix(priority ^ uint64_t(uint32_t(voxel.y())));
    priority = mix(priority ^ uint64_t(uint32_t(voxel.z())));
    candidates.push_back({priority, i});
  }
  if (maximum && candidates.size() > maximum) {
    std::nth_element(candidates.begin(), candidates.begin() + maximum,
                     candidates.end(), candidatePriorityLess);
    candidates.resize(maximum);
  }
  return candidates;
}
}

LidarMotionUndistortion::LidarMotionUndistortion(const State& initialState, const kinematics::Transformation& T_WS_live,
                                                 const kinematics::Transformation& T_SL,
                                                 const LidarMeasurementDeque& lidarScan,
                                                 const ImuMeasurementDeque& imuMeasurementDeque)
                                                 : initialState_(initialState),
                                                 T_WS_live_(T_WS_live), T_SW_live_(T_WS_live.inverse()), T_SL_(T_SL),
                                                 lidarScan_(&lidarScan), imuMeasurementDeque_(&imuMeasurementDeque){};

LidarMotionUndistortion::LidarMotionUndistortion(const State& initialState,
    const kinematics::Transformation& T_WS_live, const kinematics::Transformation& T_SL,
    LidarMeasurementDeque&& lidarScan, const ImuMeasurementDeque& imuMeasurements)
    : LidarMotionUndistortion(initialState, T_WS_live, T_SL, lidarScan, imuMeasurements) {
  ownedLidarScan_ = std::make_shared<const LidarMeasurementDeque>(std::move(lidarScan));
  lidarScan_ = ownedLidarScan_.get();
}

LidarMotionUndistortion::LidarMotionUndistortion(const State& initialState,
    const kinematics::Transformation& T_WS_live, const kinematics::Transformation& T_SL,
    const LidarMeasurementDeque& lidarScan, ImuMeasurementDeque&& imuMeasurements)
    : LidarMotionUndistortion(initialState, T_WS_live, T_SL, lidarScan, imuMeasurements) {
  ownedImuMeasurements_ = std::make_shared<const ImuMeasurementDeque>(std::move(imuMeasurements));
  imuMeasurementDeque_ = ownedImuMeasurements_.get();
}

LidarMotionUndistortion::LidarMotionUndistortion(const State& initialState,
    const kinematics::Transformation& T_WS_live, const kinematics::Transformation& T_SL,
    LidarMeasurementDeque&& lidarScan, ImuMeasurementDeque&& imuMeasurements)
    : LidarMotionUndistortion(initialState, T_WS_live, T_SL,
                             std::move(lidarScan), imuMeasurements) {
  ownedImuMeasurements_ = std::make_shared<const ImuMeasurementDeque>(std::move(imuMeasurements));
  imuMeasurementDeque_ = ownedImuMeasurements_.get();
}

bool LidarMotionUndistortion::deskew() {
  lidarScanPostProcessed_.clear();
  if (initialState_.id.value() == 0 || lidarScan_->empty() || imuMeasurementDeque_->empty()) {
    return false;
  }
  if (imuMeasurementDeque_->front().timeStamp > initialState_.timestamp
      || imuMeasurementDeque_->back().timeStamp < lidarScan_->back().timeStamp
      || !initialState_.T_WS.T().allFinite() || !T_SW_live_.T().allFinite()
      || !T_SL_.T().allFinite()) {
    return false;
  }
  // The batched propagator needs ordered, nonzero IMU intervals to interpolate
  // safely. Reject unsupported input instead of constructing partial factors.
  for (size_t i = 1; i < imuMeasurementDeque_->size(); ++i) {
    if ((*imuMeasurementDeque_)[i].timeStamp <= (*imuMeasurementDeque_)[i - 1].timeStamp)
      return false;
  }

  // do the propagation...
  kinematics::Transformation T_WS_0 = initialState_.T_WS;
  SpeedAndBias speedAndBias_0;
  speedAndBias_0.head<3>() = initialState_.v_W;
  speedAndBias_0.segment<3>(3) = initialState_.b_g;
  speedAndBias_0.tail<3>() = initialState_.b_a;
  if (!speedAndBias_0.allFinite()) return false;
  kinematics::Transformation T_WS_1;
  SpeedAndBias speedAndBias_1;

  Eigen::Vector3d omega_S;
  // This pass never changes the anchor biases. Their correction Jacobians do
  // not contribute to its output and need not be propagated for every point time.
  BatchedLidarPropagator propagator(initialState_.timestamp, *imuMeasurementDeque_, false);
  lidarScanPostProcessed_.reserve(lidarScan_->size());

  kinematics::Transformation T_SliveL = T_SW_live_ * initialState_.T_WS * T_SL_;
  // All points at the initial timestamp share its known pose and require no
  // propagation. Later groups reuse one successfully propagated pose per time.
  Time previousTime = initialState_.timestamp;
  Time previousInputTime = lidarScan_->front().timeStamp;
  for (const auto& measurement : *lidarScan_) {
    if (measurement.timeStamp < previousInputTime) {
      lidarScanPostProcessed_.clear();
      return false;
    }
    previousInputTime = measurement.timeStamp;
    if (measurement.timeStamp < initialState_.timestamp) continue;

    if (previousTime != measurement.timeStamp) {
      if (!propagator.appendTo(*imuMeasurementDeque_, T_WS_0, speedAndBias_0,
                               measurement.timeStamp)
          || !propagator.getState(T_WS_0, speedAndBias_0, T_WS_1, speedAndBias_1, omega_S)
          || !T_WS_1.T().allFinite() || !speedAndBias_1.allFinite()) {
        lidarScanPostProcessed_.clear();
        return false;
      }
      previousTime = measurement.timeStamp;
      T_SliveL = T_SW_live_ * T_WS_1 * T_SL_;
    }

    lidarScanPostProcessed_.push_back(
        (T_SliveL.C() * measurement.measurement.rayMeasurement + T_SliveL.r()).cast<float>());
  }
  return !lidarScanPostProcessed_.empty();
}

size_t LidarMotionUndistortion::filterPlanes(PlaneMapPtr map,
    const kinematics::Transformation& T_WM, double sensorSigma) {
  if (!map) {
    lidarScanPostProcessed_.clear();
    return 0;
  }
  const auto T_MS = T_WM.inverse() * T_WS_live_;
  size_t writeIndex = 0;
  for (const auto& point : lidarScanPostProcessed_) {
    const Eigen::Vector3d p = T_MS.C() * point.cast<double>() + T_MS.r();
    if (map->hasMatch(p, sensorSigma)) {
      lidarScanPostProcessed_[writeIndex++] = point;
    }
  }
  lidarScanPostProcessed_.resize(writeIndex);
  return writeIndex;
}

bool LidarMotionUndistortion::voxelDownsample(double voxel_grid_resolution) {
  if (!std::isfinite(voxel_grid_resolution) || voxel_grid_resolution <= 0.0) {
    throw std::invalid_argument("LiDAR candidate voxel resolution must be finite and positive");
  }
  std::unordered_set<Voxel, VoxelHash> allocatedVoxels;
  allocatedVoxels.reserve(lidarScanPostProcessed_.size());
  size_t writeIndex = 0;
  for (const auto& point : lidarScanPostProcessed_) {
    // Compute in double before conversion: tiny resolutions, nonfinite points,
    // and large coordinates must not reach an out-of-range integer cast.
    const Eigen::Array3d index = (point.cast<double>().array() / voxel_grid_resolution).floor();
    if (!index.isFinite().all()
        || (index < double(std::numeric_limits<int>::min())).any()
        || (index > double(std::numeric_limits<int>::max())).any()) continue;
    const Voxel voxel = index.cast<int>().matrix();
    if (allocatedVoxels.insert(voxel).second) {
      lidarScanPostProcessed_[writeIndex++] = point;
    }
  }
  lidarScanPostProcessed_.resize(writeIndex);
  return writeIndex != 0;
}

void LidarMotionUndistortion::selectPlaneCandidates(size_t maxCandidates,
    double voxelResolution) {
  if (!std::isfinite(voxelResolution) || voxelResolution <= 0.0) {
    throw std::invalid_argument("LiDAR candidate voxel resolution must be finite and positive");
  }
  if (!maxCandidates || lidarScanPostProcessed_.size() <= maxCandidates) return;

  // A fixed hash spreads a prefix across occupied spatial voxels instead of
  // keeping only the beginning of a scan. No RNG state or input-order shuffle
  // is involved, so enlarged budgets select nested sets from the same cloud.
  auto candidates = spatialCandidates(lidarScanPostProcessed_, maxCandidates, voxelResolution);
  // Sort only the bounded selection by source index. This permits safe in-place
  // compaction and retains the original order for downstream processing.
  std::sort(candidates.begin(), candidates.end(),
      [](const SpatialCandidate& lhs, const SpatialCandidate& rhs) { return lhs.index < rhs.index; });
  size_t writeIndex = 0;
  for (const auto& candidate : candidates)
    lidarScanPostProcessed_[writeIndex++] = lidarScanPostProcessed_[candidate.index];
  lidarScanPostProcessed_.resize(writeIndex);
}

size_t LidarMotionUndistortion::filterPlaneCandidates(PlaneMapPtr map,
    const kinematics::Transformation& T_WM, double sensorSigma,
    size_t initialCandidates, size_t maxCandidates, size_t minMatches,
    double voxelResolution, PlaneFilterStatistics* statistics) {
  if (!std::isfinite(voxelResolution) || voxelResolution <= 0.0)
    throw std::invalid_argument("LiDAR candidate voxel resolution must be finite and positive");
  PlaneFilterStatistics stats;
  if (!map || lidarScanPostProcessed_.empty()) {
    lidarScanPostProcessed_.clear();
    if (statistics) *statistics = stats;
    return 0;
  }
  const size_t fullSize = lidarScanPostProcessed_.size();
  const size_t maximum = maxCandidates ? std::min(maxCandidates, fullSize) : fullSize;
  const size_t initial = initialCandidates ? std::min(initialCandidates, maximum) : maximum;
  if (maximum == fullSize && initial == fullSize) {
    // Unlimited budgets and small clouds need no priority computation or sort.
    stats.initial = stats.maximum = stats.queried = fullSize;
    const size_t matches = filterPlanes(map, T_WM, sensorSigma);
    if (statistics) *statistics = stats;
    return matches;
  }
  auto candidates = spatialCandidates(lidarScanPostProcessed_, maxCandidates, voxelResolution);
  stats.maximum = candidates.size();
  stats.initial = initialCandidates ? std::min(initialCandidates, candidates.size())
                                    : candidates.size();
  if (stats.initial < candidates.size()) {
    std::nth_element(candidates.begin(), candidates.begin() + stats.initial,
                     candidates.end(), candidatePriorityLess);
  }
  for (size_t i = 0; i < stats.initial; ++i) candidates[i].initial = true;
  std::sort(candidates.begin(), candidates.end(),
      [](const SpatialCandidate& a, const SpatialCandidate& b) { return a.index < b.index; });
  const auto T_MS = T_WM.inverse() * T_WS_live_;
  const Eigen::Matrix3d rotation = T_MS.C();
  const Eigen::Vector3d translation = T_MS.r();
  size_t matches = 0;
  const auto query = [&](SpatialCandidate& candidate) {
    const Eigen::Vector3d point = rotation * lidarScanPostProcessed_[candidate.index].cast<double>()
                               + translation;
    candidate.matched = map->hasMatch(point, sensorSigma);
    ++stats.queried;
    matches += size_t(candidate.matched);
  };
  for (auto& candidate : candidates) if (candidate.initial) query(candidate);
  if (matches < minMatches && stats.initial < stats.maximum) {
    stats.expanded = true;
    // Previously the first pool was queried a second time after expanding it.
    for (auto& candidate : candidates) if (!candidate.initial) query(candidate);
  }
  size_t write = 0;
  for (const auto& candidate : candidates)
    if (candidate.matched) lidarScanPostProcessed_[write++] = lidarScanPostProcessed_[candidate.index];
  lidarScanPostProcessed_.resize(write);
  if (statistics) *statistics = stats;
  return write;
}

bool LidarMotionUndistortion::downsample(size_t num_output_points, double voxel_grid_resolution)
{
  if(lidarScanPostProcessed_.empty()){
    return false;
  }
  okvis::VoxelDownSampleInplace(lidarScanPostProcessed_, voxel_grid_resolution);
  randomDownsample(num_output_points);
  return true;
}

void LidarMotionUndistortion::randomDownsample(size_t num_output_points) {
  if(lidarScanPostProcessed_.size() > num_output_points) {
    // Partial Fisher-Yates is a uniform sample without replacement. Work is
    // proportional to retained factors rather than every matched candidate.
    thread_local std::mt19937 generator{std::random_device{}()};
    for (size_t i = 0; i < num_output_points; ++i) {
      std::uniform_int_distribution<size_t> distribution(i, lidarScanPostProcessed_.size() - 1);
      std::swap(lidarScanPostProcessed_[i], lidarScanPostProcessed_[distribution(generator)]);
    }
    lidarScanPostProcessed_.resize(num_output_points);
  }

  return;
}

size_t LidarMotionUndistortion::filterObserved(const SupereightMapType* map, const okvis::kinematics::Transformation& T_WM)
{
  if(lidarScanPostProcessed_.size() == 0)
  {
    return 0;
  }

  Eigen::Matrix<float, 3, 4> T_MS_live = T_WM.inverse().T3x4().cast<float>() * T_WS_live_.T().cast<float>();
  const Eigen::Matrix3f R_MS = T_MS_live.leftCols<3>();
  const Eigen::Vector3f t_MS = T_MS_live.col(3);

  size_t write_idx = 0;
  
  for (size_t i = 0; i < lidarScanPostProcessed_.size(); ++i) {
    auto& lidarMeasurement = lidarScanPostProcessed_[i];
    const Eigen::Vector3f pt = R_MS * lidarMeasurement + t_MS;
    if (map->interpField<se::Safe::On>(pt)) {
      lidarScanPostProcessed_[write_idx++] = lidarMeasurement;
    }
  }

  lidarScanPostProcessed_.resize(write_idx);
  return write_idx;
}

}
