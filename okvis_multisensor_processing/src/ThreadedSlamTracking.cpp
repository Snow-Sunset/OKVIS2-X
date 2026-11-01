// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/ThreadedSlam.hpp>
#include <okvis/ceres/ImuError.hpp>

#include <chrono>
#include <cmath>
#include <cstring>
#include <glog/logging.h>

namespace okvis {
namespace {
constexpr double kTrackingImuOverlap = 0.02;
constexpr size_t kBriskBytes = 48;
}

bool ThreadedSlam::independentTracking() const {
  return parameters_.frontend.independent_tracking && !blocking_.load();
}

void ThreadedSlam::startBackend() {
  if (backendStarted_) return;
  // Lazy start allows dataset applications to select blocking mode after construction.
  backendStarted_ = true;
  try {
    backendThread_ = std::thread(&ThreadedSlam::backendLoop, this);
  } catch (...) {
    backendStarted_ = false;
    throw;
  }
  LOG(INFO) << "Independent BRISK tracking enabled: immutable local map, one admitted backend "
               "frame at a time; busy candidates remain unadmitted";
}

void ThreadedSlam::stopBackend() {
  if (!backendStarted_) return;
  {
    std::lock_guard<std::mutex> lock(backendQueueMutex_);
    backendStop_ = true;
  }
  backendCondition_.notify_all();
  if (backendThread_.joinable()) backendThread_.join();
  // Keep backendStarted_ true: mode cannot be changed after graph ownership transfer.
}

void ThreadedSlam::backendLoop() {
  try {
    for (;;) {
      {
        std::lock_guard<std::mutex> lock(backendQueueMutex_);
        if (backendStop_) break;
        backendBusy_ = true;
      }
      const bool progressed = processBackendFrame();
      std::unique_lock<std::mutex> lock(backendQueueMutex_);
      if (!progressed && backendDraining_ && activeBackendFrame_.frame) {
        // Do not discard accepted work and report drainage success at EOF.
        throw Exception("Cannot drain admitted backend frame: incomplete IMU/startup coverage");
      }
      backendBusy_ = false;
      backendCondition_.notify_all();
      if (!progressed) {
        // Also poll completed optional loop proposals while cameras are idle.
        backendCondition_.wait_for(lock, std::chrono::milliseconds(2));
      }
    }
  } catch (...) {
    std::lock_guard<std::mutex> lock(backendQueueMutex_);
    backendException_ = std::current_exception();
    backendBusy_ = false;
    backendCondition_.notify_all();
  }
}

void ThreadedSlam::consumeBackendFrame() {
  if (backendStarted_) {
    std::lock_guard<std::mutex> lock(backendQueueMutex_);
    activeBackendFrame_ = BackendFrame{};
  } else {
    std::vector<CameraMeasurement> measurements;
    cameraMeasurementsReceived_.PopBlocking(&measurements);
  }
}

MultiFramePtr ThreadedSlam::copyTrackingFrame(const MultiFramePtr& frame) const {
  auto copy = std::make_shared<MultiFrame>(frame->cameraSystem(), frame->timestamp(), 0);
  for (size_t im = 0; im < frame->numFrames(); ++im) {
    // Images are immutable; keypoints, descriptors and association storage are independent.
    copy->setImage(im, frame->image(im));
    copy->setDepthImage(im, frame->depthImage(im));
    std::vector<cv::KeyPoint> keys;
    cv::Mat descriptors(int(frame->numKeypoints(im)), int(kBriskBytes), CV_8UC1);
    for (size_t k = 0; k < frame->numKeypoints(im); ++k) {
      cv::KeyPoint key;
      if (!frame->getCvKeypoint(im, k, key) || !frame->keypointDescriptor(im, k))
        throw Exception("Cannot admit frame with incomplete BRISK features");
      keys.push_back(key);
      std::memcpy(descriptors.ptr(int(k)), frame->keypointDescriptor(im, k), kBriskBytes);
    }
    copy->resetKeypoints(im, keys); // Clears landmark IDs: backend associates original observations.
    copy->resetDescriptors(im, descriptors);
    copy->computeBackProjections(im);
  }
  frontend_.prepareFrameForBackend(copy);
  return copy;
}

bool ThreadedSlam::processTrackingFrame() {
  if (shutdown_) return false;
  startBackend();
  {
    std::lock_guard<std::mutex> lock(backendQueueMutex_);
    if (backendException_) std::rethrow_exception(backendException_);
    if (backendStop_ || backendDraining_) return false;
  }
  if (!pendingTrackingFrame_) {
    std::vector<CameraMeasurement> measurements;
    // Pop once, never peek then later pop a possibly replaced raw queue entry.
    if (!cameraMeasurementsReceived_.PopNonBlocking(&measurements)) return false;
    const Time timestamp = measurements.at(0).timeStamp;
    if (haveTrackingImageTime_ && timestamp <= lastTrackingImageTime_) {
      rawImageDrops_.fetch_add(1);
      LOG(ERROR) << "Rejected non-monotonic camera timestamp " << timestamp;
      return true;
    }
    lastTrackingImageTime_ = timestamp;
    haveTrackingImageTime_ = true;
    pendingTrackingFrame_ = std::make_shared<MultiFrame>(
        parameters_.nCameraSystem, measurements.at(0).timeStamp, ++trackingFrameSequence_);
    for (size_t im = 0; im < measurements.size(); ++im) {
      pendingTrackingFrame_->setImage(im, measurements[im].measurement.image);
      pendingTrackingFrame_->setDepthImage(im, measurements[im].measurement.depthImage);
    }
  }
  const auto frame = pendingTrackingFrame_;
  const auto processingStart = std::chrono::steady_clock::now();
  std::shared_ptr<const TrackingSnapshot> snapshot;
  {
    std::lock_guard<std::mutex> lock(trackingSnapshotMutex_);
    snapshot = trackingSnapshot_;
  }
  ImuMeasurementDeque imu;
  if (parameters_.imu.use) {
    std::lock_guard<std::mutex> lock(trackingImuMutex_);
    if (trackingImuMeasurements_.empty()
        || trackingImuMeasurements_.back().timeStamp
            < frame->timestamp() + Duration(kTrackingImuOverlap)) return false;
    if ((!snapshot || !snapshot->initialized)
        && trackingImuMeasurements_.front().timeStamp
            >= frame->timestamp() - Duration(kTrackingImuOverlap)) {
      LOG(WARNING) << "Tracking startup: image precedes available IMU, dropping "
                   << frame->timestamp();
      pendingTrackingFrame_.reset();
      return true;
    }
    // Retain the feedback anchor and its bracketing sample, regardless of backend lag.
    // Before the first feedback arrives its anchor is not known yet. Retain the
    // bootstrap interval too, so delayed initialisation can always be replayed.
    if (snapshot && snapshot->initialized) {
      while (trackingImuMeasurements_.size() > 2
             && trackingImuMeasurements_[1].timeStamp < snapshot->state.timestamp) {
        trackingImuMeasurements_.pop_front();
      }
    }
    for (const auto& sample : trackingImuMeasurements_) {
      imu.push_back(sample);
      if (sample.timeStamp >= frame->timestamp() + Duration(kTrackingImuOverlap)) break;
    }
  }

  State prediction{};
  if (!trackingFrontend_->predict(frame->timestamp(), snapshot, imu, &prediction)) {
    prediction.T_WS = kinematics::Transformation();
    if (parameters_.imu.use) {
      if (!ceres::ImuError::initPose(imu, prediction.T_WS)) {
        pendingTrackingFrame_.reset();
        LOG(WARNING) << "Tracking startup: unable to initialise gravity";
        return true;
      }
    } else {
      kinematics::Transformation T_WC;
      T_WC.set(T_WC.r(), T_WC.q() * Eigen::Quaterniond(-std::sqrt(2.), std::sqrt(2.), 0, 0));
      prediction.T_WS = T_WC * parameters_.nCameraSystem.T_SC(0)->inverse();
    }
  }
  // Backend frames already contain features and never extract. Camera-aware
  // BRISK initialisation is protected by the corresponding per-camera mutex.
  const auto detect = [&](size_t im) {
    if (!frame->image(im).empty()) {
      const auto& cameras = snapshot && snapshot->initialized ? snapshot->cameras
                                                            : parameters_.nCameraSystem;
      frontend_.detectAndDescribe(im, frame, prediction.T_WS * (*cameras.T_SC(im)), nullptr);
    }
  };
  std::vector<std::thread> detectionThreads;
  for (size_t im = 1; im < frame->numFrames(); ++im) {
    if (parameters_.frontend.parallelise_detection) detectionThreads.emplace_back(detect, im);
    else detect(im);
  }
  detect(0);
  for (auto& thread : detectionThreads) thread.join();

  auto result = trackingFrontend_->track(frame, snapshot, imu);
  trackingFrames_.fetch_add(1);
  if (!result.valid) trackingLostFrames_.fetch_add(1);
  if (result.valid && frontendStateCallback_)
    frontendStateCallback_(result.state, result.tracking, result.feedbackVersion);

  // A sparse visual keyframe cadence must not leave a completed LiDAR scan
  // waiting while the backend is idle. Submit an ordinary (non-forced-keyframe)
  // image update only when it can start immediately; never add a LiDAR-only backlog.
  bool lidarUpdate = false;
  bool frameAdmitted = false;
  if (!result.requestKeyframe && result.valid && snapshot && snapshot->initialized
      && parameters_.lidar && useAlignmentFactors_) {
    const auto input = lidarMeasurementsReceived_.statistics();
    // A scan wholly before the propagation anchor cannot add live factors;
    // leave it to normal backend drainage rather than scheduling useless work.
    if (input.hasFront && input.frontScanEnd >= snapshot->state.timestamp
        && input.frontScanEnd < frame->timestamp()) {
      std::lock_guard<std::mutex> lock(backendQueueMutex_);
      lidarUpdate = !backendBusy_ && backendFrames_.empty() && !activeBackendFrame_.frame;
    }
  }
  if ((result.requestKeyframe || lidarUpdate) && frame->numKeypoints() >= 15) {
    // Admit fresh candidates only when the backend can take one immediately.
    // Keeping two already accepted old images behind an active solve magnified
    // state lag by the replay rate. Busy candidates remain unadmitted; a later
    // tracked image gets a new opportunity without dropping any accepted job.
    bool capacity;
    {
      std::lock_guard<std::mutex> lock(backendQueueMutex_);
      if (backendException_) std::rethrow_exception(backendException_);
      capacity = !backendBusy_ && backendFrames_.empty() && !activeBackendFrame_.frame
                 && !backendStop_ && !backendDraining_;
    }
    if (capacity) {
      auto backendFrame = copyTrackingFrame(frame);
      bool admitted = false;
      {
        std::lock_guard<std::mutex> lock(backendQueueMutex_);
        if (backendException_) std::rethrow_exception(backendException_);
        if (!backendStop_ && !backendDraining_
            && !backendBusy_ && backendFrames_.empty() && !activeBackendFrame_.frame) {
          backendFrames_.push_back({std::move(backendFrame)});
          admitted = true;
        }
      }
      if (admitted) {
        frameAdmitted = true;
        backendAcceptedFrames_.fetch_add(1);
        if (result.valid && result.requestKeyframe)
          trackingFrontend_->markKeyframeAccepted(result.state);
        if (lidarUpdate) lidarDrivenBackendFrames_.fetch_add(1);
        backendCondition_.notify_all();
      } else if (result.requestKeyframe) backendRejectedFrames_.fetch_add(1);
    } else if (result.requestKeyframe) {
      backendRejectedFrames_.fetch_add(1);
    }
  }
  // Retaining one image by shared ownership costs no feature copy. It never
  // enters the online FIFO; shutdown may retry it after the active job finishes.
  if (frame->numKeypoints() >= 15)
    finalTrackingFrame_ = frameAdmitted ? MultiFramePtr{} : frame;
  const double durationMs = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - processingStart).count();
  trackingDurationsMs_[trackingTimingIndex_] = durationMs;
  trackingTimingIndex_ = (trackingTimingIndex_ + 1) % trackingDurationsMs_.size();
  trackingTimingCount_ = std::min(trackingTimingCount_ + 1, trackingDurationsMs_.size());
  const auto latencyPercentiles = [&]() {
    auto sorted = trackingDurationsMs_;
    std::sort(sorted.begin(), sorted.begin() + trackingTimingCount_);
    const auto percentile = [&](double p) {
      return sorted[size_t(std::ceil(p * double(trackingTimingCount_))) - 1];
    };
    return " frontend p95 ms=" + std::to_string(percentile(.95))
        + " p99 ms=" + std::to_string(percentile(.99));
  };
  LOG_EVERY_N(INFO, 100) << "Tracking frames=" << trackingFrames_.load()
      << " lost=" << trackingLostFrames_.load() << " matches=" << result.matches
      << " inliers=" << result.inliers << " feedback=" << result.feedbackVersion
      << " keyframe overlap=" << result.keyframeOverlap
      << " backend accepted=" << backendAcceptedFrames_.load()
      << " lidar-driven updates=" << lidarDrivenBackendFrames_.load()
      << " candidate rejected=" << backendRejectedFrames_.load()
      << " raw image drops=" << rawImageDrops_.load()
      << " frontend ms=" << durationMs << latencyPercentiles()
      << " feedback lag s=" << (snapshot && snapshot->initialized
             ? (frame->timestamp() - snapshot->state.timestamp).toSec() : -1.0);
  pendingTrackingFrame_.reset();
  return true;
}

void ThreadedSlam::publishTrackingSnapshot() {
  if (!independentTracking()) return;
  auto snapshot = std::make_shared<TrackingSnapshot>();
  snapshot->version = ++trackingSnapshotVersion_;
  snapshot->cameras = parameters_.nCameraSystem;
  snapshot->initialized = frontend_.isInitialized() && estimator_.numFrames() > 0;
  if (snapshot->initialized) {
    const StateId id = estimator_.currentStateId();
    auto& state = snapshot->state;
    state.id = id;
    state.timestamp = estimator_.timestamp(id);
    state.T_WS = estimator_.pose(id);
    const SpeedAndBias sb = estimator_.speedAndBias(id);
    state.v_W = sb.head<3>();
    state.b_g = sb.segment<3>(3);
    state.b_a = sb.tail<3>();
    state.omega_S.setZero();
    state.isKeyframe = estimator_.isKeyframe(id);
    state.isOnlineExtrinsics = parameters_.camera.online_calibration.do_extrinsics;
    for (size_t im = 0; im < snapshot->cameras.numCameras(); ++im) {
      const kinematics::Transformation extrinsics = estimator_.extrinsics(id, uchar(im));
      snapshot->cameras.setExtrinsics(im, extrinsics);
      state.extrinsics[im] = extrinsics;
    }

    // Copy only the live local observation neighbourhood, never scan historical maps.
    std::set<StateId> neighbours;
    estimator_.getObservedIds(id, neighbours);
    std::vector<StateId> frames{id};
    const size_t maxFrames = size_t(std::max(1, parameters_.estimator.num_keyframes
        + parameters_.estimator.num_loop_closure_frames + parameters_.estimator.num_imu_frames));
    for (auto it = neighbours.rbegin(); it != neighbours.rend() && frames.size() < maxFrames; ++it)
      if (*it != id) frames.push_back(*it);
    std::map<LandmarkId, size_t> pointIndices;
    const double radius2 = parameters_.frontend.tracking_map_radius
                         * parameters_.frontend.tracking_map_radius;
    const size_t maxPoints = size_t(parameters_.frontend.tracking_max_landmarks);
    for (StateId frameId : frames) {
      const auto frame = estimator_.multiFrame(frameId);
      if (!frame) continue;
      for (size_t im = 0; im < frame->numFrames(); ++im) {
        for (size_t k = 0; k < frame->numKeypoints(im); ++k) {
          const LandmarkId landmarkId(frame->landmarkId(im, k));
          const auto* descriptor = frame->keypointDescriptor(im, k);
          if (!landmarkId.isInitialised() || !descriptor
              || !estimator_.isLandmarkAdded(landmarkId)) continue;
          auto existing = pointIndices.find(landmarkId);
          if (existing == pointIndices.end()) {
            if (snapshot->points.size() >= maxPoints) continue;
            MapPoint2 point;
            if (!estimator_.getLandmark(landmarkId, point) || !point.isInitialised
                || !point.point.allFinite() || std::abs(point.point[3]) < 1.e-12) continue;
            const Eigen::Vector3d p = point.point.head<3>() / point.point[3];
            if ((p - state.T_WS.r()).squaredNorm() > radius2) continue;
            TrackingMapPoint copy;
            copy.id = landmarkId;
            copy.point = point.point;
            const size_t index = snapshot->points.size();
            snapshot->points.push_back(std::move(copy));
            existing = pointIndices.emplace(landmarkId, index).first;
          }
          auto& descriptors = snapshot->points[existing->second].descriptors;
          if (descriptors.size() >= 3) continue;
          std::array<uint8_t, kBriskBytes> copy;
          std::memcpy(copy.data(), descriptor, kBriskBytes);
          descriptors.push_back(copy);
        }
      }
    }
  }
  // State, calibrated cameras and map coordinates become visible together.
  std::lock_guard<std::mutex> lock(trackingSnapshotMutex_);
  if (!snapshot->initialized && trackingSnapshot_)
    snapshot->state.timestamp = trackingSnapshot_->state.timestamp;
  trackingSnapshot_ = std::move(snapshot);
}

}  // namespace okvis
