// SPDX-License-Identifier: BSD-3-Clause
#ifndef INCLUDE_OKVIS_TRACKINGFRONTEND_HPP_
#define INCLUDE_OKVIS_TRACKINGFRONTEND_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <okvis/Measurements.hpp>
#include <okvis/MultiFrame.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/ViInterface.hpp>

namespace okvis {

/// A copied, world-frame landmark. No backend parameter blocks are shared.
struct TrackingMapPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LandmarkId id;
  Eigen::Vector4d point = Eigen::Vector4d::Zero();
  std::vector<std::array<uint8_t, 48>> descriptors;
};

/// Published atomically by the backend, then immutable for its entire lifetime.
/// The publisher bounds the point count and distance and copies calibrated
/// extrinsics; readers never access the backend's live graph.
struct TrackingSnapshot {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t version = 0;
  State state{};
  bool initialized = false;
  cameras::NCameraSystem cameras;
  AlignedVector<TrackingMapPoint> points;
};

/// Single-consumer BRISK tracker. Only this frame's six-DoF pose is optimized.
/// Features must have been detected/described before track() is called.
class TrackingFrontend {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct Result {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    State state{};
    TrackingState tracking{};
    size_t matches = 0;
    size_t inliers = 0;
    /// Final inlier-region area divided by current detected-region area.
    /// Uses only enabled cameras with an image, independently of map size.
    double keyframeOverlap = 0.0;
    uint64_t feedbackVersion = 0;
    bool valid = false;
    bool requestKeyframe = true;
  };

  explicit TrackingFrontend(const ViParameters& params);

  /// Repropagate the newest eligible backend anchor to timestamp. An older
  /// version, a regressing anchor time, or a future initialized anchor is never
  /// installed. A newer uninitialized snapshot explicitly invalidates the map;
  /// it needs no usable State and blocks older snapshots until a newer valid
  /// snapshot arrives. Output is untouched on failure or incomplete IMU coverage.
  bool predict(const Time& timestamp,
               std::shared_ptr<const TrackingSnapshot> feedback,
               const ImuMeasurementDeque& imu, State* output);

  Result track(const MultiFramePtr& frame,
               std::shared_ptr<const TrackingSnapshot> feedback,
               const ImuMeasurementDeque& imu);

  /// Call only after a proposed keyframe has actually entered the backend queue.
  void markKeyframeAccepted(const State& state);

 private:
  bool selectFeedback(const Time& timestamp,
                      std::shared_ptr<const TrackingSnapshot> feedback);
  bool wantsKeyframe(const State& state, double overlap) const;

  ViParameters params_;
  std::shared_ptr<const TrackingSnapshot> feedback_;
  State acceptedKeyframe_{};
  Time lastTrackedTime_;
  bool haveAcceptedKeyframe_ = false;
  bool haveTrackedFrame_ = false;
};

}  // namespace okvis
#endif  // INCLUDE_OKVIS_TRACKINGFRONTEND_HPP_
