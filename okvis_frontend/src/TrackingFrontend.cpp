// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/TrackingFrontend.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <set>
#include <utility>

#include <ceres/ceres.h>
#include <opencv2/imgproc.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/EucmCamera.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/ceres/ImuError.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/ReprojectionError.hpp>

namespace okvis {
namespace {
constexpr double kGridCellPixels = 24.0;
constexpr double kSearchRadiusPixels = 30.0;
constexpr size_t kMaxKeypointsPerCell = 64;
constexpr size_t kMaxCandidatesPerProjection = 128;
constexpr size_t kMaxDescriptorsPerPoint = 3;
constexpr size_t kMinimumInliers = 8;
constexpr double kCoverageRadius = 0.09;

State emptyState(const Time& timestamp) {
  State state{};
  state.T_WS.setIdentity();
  state.T_GW.setIdentity();
  state.v_W.setZero();
  state.b_g.setZero();
  state.b_a.setZero();
  state.omega_S.setZero();
  state.timestamp = timestamp;
  return state;
}

bool finiteState(const State& state) {
  return state.T_WS.T().allFinite() && state.v_W.allFinite()
      && state.b_g.allFinite() && state.b_a.allFinite()
      && std::abs(state.T_WS.q().squaredNorm() - 1.0) < 1.0e-3;
}

unsigned hamming(const uint8_t* first, const uint8_t* second) {
  // memcpy avoids imposing SIMD/word alignment on copied descriptor arrays.
  unsigned distance = 0;
  for (size_t i = 0; i < 48; i += sizeof(uint64_t)) {
    uint64_t a, b;
    std::memcpy(&a, first + i, sizeof(a));
    std::memcpy(&b, second + i, sizeof(b));
    distance += unsigned(__builtin_popcountll(a ^ b));
  }
  return distance;
}

struct Observation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  size_t camera = 0;
  size_t keypoint = 0;
  size_t point = 0;
  Eigen::Vector2d pixel = Eigen::Vector2d::Zero();
  double information = 1.0;
  unsigned distance = 0;
};

struct ImageCoverage {
  size_t activeCameras = 0;
  double overlap = 0.0;
};

ImageCoverage measureImageCoverage(const MultiFrame& frame,
                                  const cameras::NCameraSystem& cameras,
                                  const AlignedVector<Observation>& observations,
                                  const std::vector<bool>& inliers) {
  std::vector<std::vector<bool>> matched(frame.numFrames());
  for (size_t camera = 0; camera < frame.numFrames(); ++camera)
    matched[camera].resize(frame.numKeypoints(camera), false);
  for (size_t i = 0; i < observations.size(); ++i)
    if (inliers[i]) matched[observations[i].camera][observations[i].keypoint] = true;

  ImageCoverage result;
  size_t detectedArea = 0, matchedArea = 0;
  for (size_t camera = 0; camera < frame.numFrames(); ++camera) {
    if (!cameras.cameraType(camera).isUsed || frame.image(camera).empty()) continue;
    const cv::Size imageSize = frame.image(camera).size();
    const int rows = imageSize.height / 10, cols = imageSize.width / 10;
    if (rows <= 0 || cols <= 0) continue;
    ++result.activeCameras;
    cv::Mat detections = cv::Mat::zeros(rows, cols, CV_8UC1);
    cv::Mat matches = cv::Mat::zeros(rows, cols, CV_8UC1);
    const int radius = int(double(std::min(rows, cols)) * kCoverageRadius);
    for (size_t keypoint = 0; keypoint < frame.numKeypoints(camera); ++keypoint) {
      cv::KeyPoint key;
      if (!frame.getCvKeypoint(camera, keypoint, key)
          || !std::isfinite(key.pt.x) || !std::isfinite(key.pt.y)
          || !std::isfinite(key.size) || key.size <= 0.0f
          || key.pt.x < 0 || key.pt.y < 0
          || key.pt.x >= imageSize.width || key.pt.y >= imageSize.height) continue;
      cv::circle(detections, key.pt * 0.1, radius, cv::Scalar(255), cv::FILLED);
      if (matched[camera][keypoint])
        cv::circle(matches, key.pt * 0.1, radius, cv::Scalar(255), cv::FILLED);
    }
    // Use the same 1/10-resolution circles as the legacy frontend. Inlier
    // circles are a subset of detection circles, so their IoU is this ratio.
    detectedArea += size_t(cv::countNonZero(detections));
    matchedArea += size_t(cv::countNonZero(matches));
  }
  result.overlap = detectedArea ? double(matchedArea) / double(detectedArea) : 0.0;
  return result;
}

bool project(const cameras::NCameraSystem& cameras,
             const kinematics::Transformation& T_WS, size_t camera,
             const Eigen::Vector4d& point, Eigen::Vector2d* pixel) {
  const Eigen::Vector4d p_C = (*cameras.T_SC(camera)).inverse() * T_WS.inverse() * point;
  if (!p_C.allFinite() || std::abs(p_C.w()) < 1.0e-12) return false;
  return cameras.cameraGeometry(camera)->project(p_C.head<3>() / p_C.w(), pixel)
             == cameras::ProjectionStatus::Successful && pixel->allFinite();
}

template<class Camera>
std::shared_ptr<ceres::ReprojectionError2dBase> makeError(
    const cameras::NCameraSystem& cameras, const Observation& observation) {
  auto camera = std::dynamic_pointer_cast<const Camera>(
      cameras.cameraGeometry(observation.camera));
  if (!camera) return nullptr;
  return std::make_shared<ceres::ReprojectionError<Camera>>(
      camera, observation.camera, observation.pixel,
      observation.information * Eigen::Matrix2d::Identity());
}

std::shared_ptr<ceres::ReprojectionError2dBase> makeError(
    const cameras::NCameraSystem& cameras, const Observation& observation) {
  switch (cameras.distortionType(observation.camera)) {
    case cameras::NCameraSystem::RadialTangential:
      return makeError<cameras::PinholeCamera<cameras::RadialTangentialDistortion>>(
          cameras, observation);
    case cameras::NCameraSystem::Equidistant:
      return makeError<cameras::PinholeCamera<cameras::EquidistantDistortion>>(
          cameras, observation);
    case cameras::NCameraSystem::RadialTangential8:
      return makeError<cameras::PinholeCamera<cameras::RadialTangentialDistortion8>>(
          cameras, observation);
    case cameras::NCameraSystem::NoDistortion:
      return makeError<cameras::EucmCamera>(cameras, observation);
    default:
      return nullptr;
  }
}

AlignedVector<Observation> findMatches(const MultiFrame& frame,
                                      const TrackingSnapshot& snapshot,
                                      const State& predicted, double threshold) {
  AlignedVector<Observation> proposals;
  for (size_t camera = 0; camera < frame.numFrames(); ++camera) {
    if (!snapshot.cameras.cameraType(camera).isUsed || frame.image(camera).empty()
        || frame.numKeypoints(camera) == 0)
      continue;
    const auto geometry = snapshot.cameras.cameraGeometry(camera);
    std::map<std::pair<int, int>, std::vector<size_t>> grid;
    for (size_t keypoint = 0; keypoint < frame.numKeypoints(camera); ++keypoint) {
      Eigen::Vector2d pixel;
      double size = 0.0;
      if (!frame.getKeypoint(camera, keypoint, pixel) || !pixel.allFinite()
          || !frame.getKeypointSize(camera, keypoint, size) || !std::isfinite(size)
          || size <= 0.0 || pixel.x() < 0 || pixel.y() < 0
          || pixel.x() >= geometry->imageWidth() || pixel.y() >= geometry->imageHeight())
        continue;
      const std::pair<int, int> cell(int(pixel.x() / kGridCellPixels),
                                     int(pixel.y() / kGridCellPixels));
      auto& bucket = grid[cell];
      if (bucket.size() < kMaxKeypointsPerCell) bucket.push_back(keypoint);
    }
    // detectAndDescribe guarantees complete 48-byte BRISK descriptor rows.
    if (!frame.keypointDescriptor(camera, 0)) continue;
    for (size_t pointIndex = 0; pointIndex < snapshot.points.size(); ++pointIndex) {
      const auto& point = snapshot.points[pointIndex];
      if (!point.id.isInitialised() || point.descriptors.empty() || !point.point.allFinite()
          || std::abs(point.point.w()) < 1.0e-12) continue;
      Eigen::Vector2d projection;
      if (!project(snapshot.cameras, predicted.T_WS, camera, point.point, &projection)) continue;
      const int left = int(std::floor((projection.x() - kSearchRadiusPixels) / kGridCellPixels));
      const int right = int(std::floor((projection.x() + kSearchRadiusPixels) / kGridCellPixels));
      const int top = int(std::floor((projection.y() - kSearchRadiusPixels) / kGridCellPixels));
      const int bottom = int(std::floor((projection.y() + kSearchRadiusPixels) / kGridCellPixels));
      unsigned best = 385, second = 385;
      size_t bestIndex = 0, examined = 0;
      for (int y = top; y <= bottom && examined < kMaxCandidatesPerProjection; ++y) {
        for (int x = left; x <= right && examined < kMaxCandidatesPerProjection; ++x) {
          const auto cell = grid.find(std::make_pair(x, y));
          if (cell == grid.end()) continue;
          for (const size_t keypoint : cell->second) {
            if (++examined > kMaxCandidatesPerProjection) break;
            Eigen::Vector2d pixel;
            frame.getKeypoint(camera, keypoint, pixel);
            if ((pixel - projection).squaredNorm() > kSearchRadiusPixels * kSearchRadiusPixels)
              continue;
            const auto* descriptor = frame.keypointDescriptor(camera, keypoint);
            unsigned distance = 385;
            for (size_t d = 0; d < std::min(kMaxDescriptorsPerPoint, point.descriptors.size()); ++d)
              distance = std::min(distance, hamming(descriptor, point.descriptors[d].data()));
            if (distance < best) {
              second = best;
              best = distance;
              bestIndex = keypoint;
            } else if (distance < second) {
              second = distance;
            }
          }
        }
      }
      if (double(best) >= threshold || double(best) >= 0.8 * double(second)) continue;
      Observation match;
      match.camera = camera;
      match.keypoint = bestIndex;
      match.point = pointIndex;
      match.distance = best;
      frame.getKeypoint(camera, bestIndex, match.pixel);
      double size = 1.0;
      frame.getKeypointSize(camera, bestIndex, size);
      match.information = 64.0 / (size * size);
      proposals.push_back(match);
    }
  }
  std::stable_sort(proposals.begin(), proposals.end(), [](const Observation& a, const Observation& b) {
    return a.distance < b.distance;
  });
  AlignedVector<Observation> matches;
  std::set<std::pair<size_t, size_t>> usedKeypoints;
  std::set<std::pair<size_t, uint64_t>> usedLandmarks;
  for (const auto& proposal : proposals) {
    const auto keypoint = std::make_pair(proposal.camera, proposal.keypoint);
    const auto landmark = std::make_pair(proposal.camera, snapshot.points[proposal.point].id.value());
    if (usedKeypoints.count(keypoint) || usedLandmarks.count(landmark)) continue;
    usedKeypoints.insert(keypoint);
    usedLandmarks.insert(landmark);
    matches.push_back(proposal);
  }
  return matches;
}
}  // namespace

TrackingFrontend::TrackingFrontend(const ViParameters& params) : params_(params) {}

bool TrackingFrontend::selectFeedback(const Time& timestamp,
                                     std::shared_ptr<const TrackingSnapshot> feedback) {
  if (feedback && (!feedback_ || feedback->version > feedback_->version)) {
    if (!feedback->initialized) {
      // A reset is a control message, not a state estimate. In particular, its
      // Eigen state members may never have been initialized. Keep its version
      // as a watermark so a delayed valid snapshot cannot resurrect the old map.
      feedback_ = std::move(feedback);
      haveAcceptedKeyframe_ = false;
      acceptedKeyframe_ = emptyState(Time(0));
    } else if (feedback->state.timestamp <= timestamp && finiteState(feedback->state)
               && (!feedback_ || !feedback_->initialized
                   || feedback->state.timestamp >= feedback_->state.timestamp)) {
      feedback_ = std::move(feedback);
    }
  }
  return feedback_ && feedback_->initialized && feedback_->state.timestamp <= timestamp;
}

bool TrackingFrontend::predict(const Time& timestamp,
                              std::shared_ptr<const TrackingSnapshot> feedback,
                              const ImuMeasurementDeque& imu, State* output) {
  if (!output) return false;
  // Even an out-of-order image must observe a backend reset immediately.
  const bool haveFeedback = selectFeedback(timestamp, std::move(feedback));
  if (!haveFeedback || (haveTrackedFrame_ && timestamp < lastTrackedTime_)) return false;
  State state = feedback_->state;
  const Time start = state.timestamp;
  if (start < timestamp) {
    const double dt = (timestamp - start).toSec();
    if (params_.imu.use) {
      if (imu.size() < 2 || imu.front().timeStamp > start || imu.back().timeStamp < timestamp)
        return false;
      // The preintegrator requires strictly ordered samples and finite readings.
      for (size_t i = 0; i < imu.size(); ++i) {
        if (!imu[i].measurement.gyroscopes.allFinite()
            || !imu[i].measurement.accelerometers.allFinite()
            || (i && imu[i].timeStamp <= imu[i - 1].timeStamp)) return false;
      }
      SpeedAndBias speedAndBias;
      speedAndBias << state.v_W, state.b_g, state.b_a;
      if (ceres::ImuError::propagation(imu, params_.imu, state.T_WS, speedAndBias,
                                      start, timestamp) <= 0) return false;
      state.v_W = speedAndBias.head<3>();
      state.b_g = speedAndBias.segment<3>(3);
      state.b_a = speedAndBias.tail<3>();
      for (auto measurement = imu.rbegin(); measurement != imu.rend(); ++measurement) {
        if (measurement->timeStamp <= timestamp) {
          state.omega_S = measurement->measurement.gyroscopes - state.b_g;
          break;
        }
      }
    } else {
      state.T_WS.set(state.T_WS.r() + state.v_W * dt,
                     state.T_WS.q() * kinematics::deltaQ(state.omega_S * dt));
    }
  }
  if (!finiteState(state)) return false;
  state.timestamp = timestamp;
  state.isKeyframe = false;
  state.stateChanged = true;
  state.previousImuMeasurements.clear();
  *output = std::move(state);
  return true;
}

void TrackingFrontend::markKeyframeAccepted(const State& state) {
  if (!feedback_ || !feedback_->initialized) return;
  if (!haveAcceptedKeyframe_ || state.timestamp >= acceptedKeyframe_.timestamp) {
    acceptedKeyframe_ = state;
    acceptedKeyframe_.previousImuMeasurements.clear();
    haveAcceptedKeyframe_ = true;
  }
}

bool TrackingFrontend::wantsKeyframe(const State& state, double overlap) const {
  if (!haveAcceptedKeyframe_) return true;
  const auto relative = acceptedKeyframe_.T_WS.inverse() * state.T_WS;
  const double angle = Eigen::AngleAxisd(relative.q()).angle();
  return overlap < params_.frontend.keyframe_overlap
      || relative.r().norm() > params_.frontend.tracking_keyframe_translation
      || angle > params_.frontend.tracking_keyframe_rotation
      || (state.timestamp - acceptedKeyframe_.timestamp).toSec()
             >= params_.frontend.tracking_max_keyframe_interval;
}

TrackingFrontend::Result TrackingFrontend::track(
    const MultiFramePtr& frame, std::shared_ptr<const TrackingSnapshot> feedback,
    const ImuMeasurementDeque& imu) {
  Result result;
  result.state = emptyState(frame ? frame->timestamp() : Time(0));
  result.tracking.trackingQuality = TrackingQuality::Lost;
  if (!frame) return result;
  result.tracking.id = StateId(frame->id());
  result.state.id = StateId(frame->id());
  const bool havePrediction = predict(frame->timestamp(), std::move(feedback), imu, &result.state);
  result.feedbackVersion = feedback_ ? feedback_->version : 0;
  if (!havePrediction) return result;
  result.state.id = StateId(frame->id());
  result.tracking.currentKeyframeId = haveAcceptedKeyframe_ ? acceptedKeyframe_.id : feedback_->state.id;
  lastTrackedTime_ = frame->timestamp();
  haveTrackedFrame_ = true;
  if (!feedback_->initialized || feedback_->points.empty()
      || feedback_->cameras.numCameras() != frame->numFrames()) return result;

  const auto observations = findMatches(*frame, *feedback_, result.state,
      params_.frontend.matching_threshold);
  result.matches = observations.size();
  if (observations.size() < kMinimumInliers) return result;

  ceres::PoseManifold poseManifold;
  ::ceres::CauchyLoss loss(3.0);
  ceres::PoseParameterBlock pose(result.state.T_WS, frame->id());
  std::vector<std::shared_ptr<ceres::PoseParameterBlock>> extrinsics;
  std::map<size_t, std::shared_ptr<ceres::HomogeneousPointParameterBlock>> landmarks;
  std::vector<std::shared_ptr<ceres::ReprojectionError2dBase>> errors;
  ::ceres::Problem::Options options;
  options.manifold_ownership = ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  options.loss_function_ownership = ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  options.cost_function_ownership = ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  ::ceres::Problem problem(options);
  problem.AddParameterBlock(pose.parameters(), 7, &poseManifold);
  for (size_t camera = 0; camera < frame->numFrames(); ++camera) {
    auto extrinsic = std::make_shared<ceres::PoseParameterBlock>(
        *feedback_->cameras.T_SC(camera), camera + 1);
    problem.AddParameterBlock(extrinsic->parameters(), 7, &poseManifold);
    problem.SetParameterBlockConstant(extrinsic->parameters());
    extrinsics.push_back(std::move(extrinsic));
  }
  for (const auto& observation : observations) {
    auto error = makeError(feedback_->cameras, observation);
    if (!error) return result;
    auto& landmark = landmarks[observation.point];
    if (!landmark) {
      landmark = std::make_shared<ceres::HomogeneousPointParameterBlock>(
          feedback_->points[observation.point].point,
          feedback_->points[observation.point].id.value());
      problem.AddParameterBlock(landmark->parameters(), 4);
      problem.SetParameterBlockConstant(landmark->parameters());
    }
    problem.AddResidualBlock(error.get(), &loss, pose.parameters(), landmark->parameters(),
                             extrinsics[observation.camera]->parameters());
    errors.push_back(std::move(error));
  }
  ::ceres::Solver::Options solverOptions;
  solverOptions.num_threads = 1;
  solverOptions.linear_solver_type = ::ceres::DENSE_QR;
  solverOptions.max_num_iterations = 4;
  solverOptions.max_solver_time_in_seconds = 0.005;
  solverOptions.minimizer_progress_to_stdout = false;
  ::ceres::Solver::Summary summary;
  ::ceres::Solve(solverOptions, &problem, &summary);
  if (!summary.IsSolutionUsable() || !pose.estimate().T().allFinite()) return result;

  std::vector<bool> inliers(observations.size(), false);
  std::set<std::pair<size_t, int>> occupiedCells;
  for (size_t i = 0; i < observations.size(); ++i) {
    const auto& observation = observations[i];
    Eigen::Vector2d pixel;
    if (!project(feedback_->cameras, pose.estimate(), observation.camera,
                 feedback_->points[observation.point].point, &pixel)
        || (pixel - observation.pixel).squaredNorm() * observation.information > 9.0) continue;
    inliers[i] = true;
    ++result.inliers;
    const auto geometry = feedback_->cameras.cameraGeometry(observation.camera);
    const int x = std::min(3, int(4.0 * observation.pixel.x() / geometry->imageWidth()));
    const int y = std::min(3, int(4.0 * observation.pixel.y() / geometry->imageHeight()));
    occupiedCells.insert(std::make_pair(observation.camera, 4 * y + x));
  }
  if (result.inliers < kMinimumInliers || result.inliers * 2 < observations.size()
      || occupiedCells.size() < 3) return result;

  // Velocity and biases remain the IMU-propagated feedback values. This solve
  // contains no historical poses, inertial factors or mutable map variables.
  result.state.T_WS = pose.estimate();
  result.valid = true;
  const auto imageCoverage = measureImageCoverage(*frame, feedback_->cameras, observations, inliers);
  result.keyframeOverlap = imageCoverage.overlap;
  const double coverage = imageCoverage.activeCameras
      ? double(occupiedCells.size()) / (16.0 * imageCoverage.activeCameras) : 0.0;
  result.tracking.trackingQuality = result.inliers >= 20 && coverage >= 0.3
      ? TrackingQuality::Good : TrackingQuality::Marginal;
  result.requestKeyframe = wantsKeyframe(result.state, result.keyframeOverlap);
  for (size_t i = 0; i < observations.size(); ++i) {
    if (!inliers[i]) continue;
    const auto& observation = observations[i];
    const auto& point = feedback_->points[observation.point];
    frame->setLandmarkId(observation.camera, observation.keypoint, point.id.value());
    frame->setLandmark(observation.camera, observation.keypoint, point.point, true);
  }
  return result;
}

}  // namespace okvis
