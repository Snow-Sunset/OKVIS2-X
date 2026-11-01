// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/TrackingFrontend.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/EucmCamera.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

namespace okvis {
namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

State stateAt(double timestamp, double position = 0.0, double velocity = 0.0) {
  State state{};
  state.timestamp = Time(timestamp);
  state.id = StateId(1);
  state.T_WS.set(Eigen::Vector3d(position, 0, 0), Eigen::Quaterniond::Identity());
  state.T_GW.setIdentity();
  state.v_W = Eigen::Vector3d(velocity, 0, 0);
  state.b_g = Eigen::Vector3d(.02, -.01, .03);
  state.b_a = Eigen::Vector3d(.1, -.05, .03);
  state.omega_S.setZero();
  return state;
}

ViParameters parameters(const cameras::NCameraSystem& cameras) {
  ViParameters params{};
  params.nCameraSystem = cameras;
  params.imu.use = true;
  params.imu.g = 9.81;
  params.imu.g_max = 100;
  params.imu.a_max = 100;
  params.imu.sigma_g_c = 1.0e-3;
  params.imu.sigma_a_c = 1.0e-2;
  params.frontend.matching_threshold = 60;
  params.frontend.keyframe_overlap = .6;
  params.frontend.tracking_max_keyframe_interval = .25;
  params.frontend.tracking_keyframe_translation = .5;
  params.frontend.tracking_keyframe_rotation = .15;
  return params;
}

ImuMeasurementDeque measurements(const State& state, double start, double end) {
  ImuMeasurementDeque imu;
  const size_t steps = size_t(std::lround((end - start) * 200));
  for (size_t i = 0; i <= steps; ++i) {
    const double time = i == steps ? end : start + double(i) / 200;
    imu.emplace_back(Time(time), ImuSensorReadings(state.b_g,
        state.b_a + Eigen::Vector3d(0, 0, 9.81)));
  }
  return imu;
}

cameras::NCameraSystem cameraSystem(std::shared_ptr<const cameras::CameraBase> camera,
                                    cameras::NCameraSystem::DistortionType distortion) {
  cameras::NCameraSystem cameras;
  cameras::NCameraSystem::CameraType type;
  type.isColour = false;
  cameras.addCamera(std::make_shared<const kinematics::Transformation>(
      kinematics::Transformation::Identity()), std::move(camera), distortion, false, type);
  return cameras;
}

std::shared_ptr<TrackingSnapshot> snapshot(const cameras::NCameraSystem& cameras,
                                         uint64_t version, const State& state) {
  auto feedback = std::make_shared<TrackingSnapshot>();
  feedback->version = version;
  feedback->state = state;
  feedback->initialized = true;
  feedback->cameras = cameras;
  std::mt19937 random(71);
  for (int i = 0; i < 48; ++i) {
    TrackingMapPoint point;
    point.id = LandmarkId(uint64_t(i + 1));
    point.point = Eigen::Vector4d((i % 8 - 3.5) * .4, (i / 8 - 2.5) * .4,
                                 3.5 + (i % 5) * .13, 1);
    point.descriptors.resize(1);
    for (auto& byte : point.descriptors.front()) byte = uint8_t(random());
    feedback->points.push_back(std::move(point));
  }
  return feedback;
}

MultiFramePtr frame(const TrackingSnapshot& feedback, double timestamp,
                    uint64_t id, const kinematics::Transformation& T_WS,
                    bool invalid = false) {
  auto output = std::make_shared<MultiFrame>(feedback.cameras, Time(timestamp), id);
  for (size_t camera = 0; camera < feedback.cameras.numCameras(); ++camera) {
    const auto geometry = feedback.cameras.cameraGeometry(camera);
    output->setImage(camera, cv::Mat::zeros(int(geometry->imageHeight()),
                                           int(geometry->imageWidth()), CV_8UC1));
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors(int(feedback.points.size()), 48, CV_8UC1);
    for (size_t i = 0; i < feedback.points.size(); ++i) {
      const auto& point = feedback.points[i];
      const Eigen::Vector4d p_C = (*feedback.cameras.T_SC(camera)).inverse()
          * T_WS.inverse() * point.point;
      Eigen::Vector2d pixel;
      check(feedback.cameras.cameraGeometry(camera)->project(p_C.head<3>() / p_C.w(), &pixel)
                == cameras::ProjectionStatus::Successful, "Fixture point is outside the camera");
      if (invalid) pixel.x() = std::numeric_limits<double>::quiet_NaN();
      keypoints.emplace_back(float(pixel.x()), float(pixel.y()), 12.0f);
      for (size_t d = 0; d < 48; ++d)
        descriptors.at<uchar>(int(i), int(d)) = point.descriptors.front()[d];
    }
    output->resetKeypoints(camera, keypoints);
    output->resetDescriptors(camera, descriptors);
    if (!invalid) output->computeBackProjections(camera);
  }
  return output;
}

void feedbackChecks(const cameras::NCameraSystem& cameras) {
  TrackingFrontend tracker(parameters(cameras));
  auto initial = snapshot(cameras, 1, stateAt(10.0, 10.0, 1.0));
  const auto imu = measurements(initial->state, 10.0, 12.0);
  State predicted{};
  check(tracker.predict(Time(11.0), initial, imu, &predicted), "Initial propagation failed");
  check(predicted.timestamp == Time(11.0)
        && (predicted.T_WS.r() - Eigen::Vector3d(11, 0, 0)).norm() < 1.0e-6
        && (predicted.v_W - Eigen::Vector3d(1, 0, 0)).norm() < 1.0e-6,
        "Current prediction used the old backend pose without IMU propagation");
  auto corrected = snapshot(cameras, 2, stateAt(10.5, 20.0, 2.0));
  check(tracker.predict(Time(11.0), corrected, imu, &predicted)
        && std::abs(predicted.T_WS.r().x() - 21.0) < 1.0e-6,
        "New feedback did not repropagate from its own timestamp and velocity");
  check(predicted.b_g.isApprox(corrected->state.b_g)
        && predicted.b_a.isApprox(corrected->state.b_a), "Prediction lost backend IMU biases");
  auto olderVersion = snapshot(cameras, 1, stateAt(10.7, 100.0));
  check(tracker.predict(Time(11.1), olderVersion, imu, &predicted)
        && std::abs(predicted.T_WS.r().x() - 21.2) < 1.0e-6,
        "Old feedback version replaced the accepted anchor");
  auto olderTime = snapshot(cameras, 3, stateAt(10.4, 100.0));
  check(tracker.predict(Time(11.1), olderTime, imu, &predicted)
        && std::abs(predicted.T_WS.r().x() - 21.2) < 1.0e-6,
        "A higher version with regressing anchor time was accepted");
  auto future = snapshot(cameras, 4, stateAt(11.5, 100.0));
  check(tracker.predict(Time(11.2), future, imu, &predicted)
        && std::abs(predicted.T_WS.r().x() - 21.4) < 1.0e-6,
        "Future feedback replaced the current-time prediction");
  State sentinel = stateAt(0.0, 42.0);
  check(!tracker.predict(Time(12.1), corrected, imu, &sentinel)
        && sentinel.T_WS.r().x() == 42.0 && sentinel.timestamp == Time(0.0),
        "Incomplete IMU coverage produced a purported current state");
}

void geometricChecks(const cameras::NCameraSystem& cameras) {
  auto params = parameters(cameras);
  params.imu.use = false;
  TrackingFrontend tracker(params);
  auto feedback = snapshot(cameras, 7, stateAt(10.0));
  const kinematics::Transformation actual(Eigen::Vector3d(.025, -.015, .01),
      Eigen::Quaterniond(Eigen::AngleAxisd(.008, Eigen::Vector3d::UnitY())));
  auto query = frame(*feedback, 10.1, 2, actual);
  auto result = tracker.track(query, feedback, {});
  check(result.valid && result.matches >= 40 && result.inliers >= 40
        && result.feedbackVersion == 7 && result.state.timestamp == query->timestamp()
        && result.state.id == StateId(2), "Fixed local map tracking failed a valid current frame");
  check((result.state.T_WS.r() - actual.r()).norm() < .01
        && Eigen::AngleAxisd(result.state.T_WS.q().inverse() * actual.q()).angle() < .01,
        "Pose-only refinement did not recover the current frame pose");
  check(query->landmarkId(0, 0) == 1 && feedback->state.T_WS.r().norm() == 0.0
        && feedback->points.front().id == LandmarkId(1),
        "Tracker modified feedback geometry or failed to associate its own frame");
  check(result.state.v_W.isApprox(feedback->state.v_W)
        && result.state.b_g.isApprox(feedback->state.b_g)
        && result.state.b_a.isApprox(feedback->state.b_a),
        "Pose-only refinement unexpectedly changed velocity or IMU bias");
  tracker.markKeyframeAccepted(result.state);
  auto next = tracker.track(frame(*feedback, 10.15, 3, actual), feedback, {});
  check(next.valid && !next.requestKeyframe, "Keyframe admission was not remembered");
  auto timed = tracker.track(frame(*feedback, 10.4, 4, actual), feedback, {});
  check(timed.valid && timed.requestKeyframe, "Sensor-time keyframe interval was ignored");
  auto invalid = tracker.track(frame(*feedback, 10.5, 5, actual, true), feedback, {});
  check(!invalid.valid && invalid.requestKeyframe && invalid.inliers == 0
        && invalid.tracking.trackingQuality == TrackingQuality::Lost,
        "Invalid observations were published as successful visual tracking");
  auto regressing = tracker.track(frame(*feedback, 10.3, 6, actual), feedback, {});
  check(!regressing.valid, "Out-of-order image timestamp was accepted");
  TrackingFrontend bootstrap(params);
  check(!bootstrap.track(frame(*feedback, 10.1, 2, actual), nullptr, {}).valid,
        "An absent backend map unexpectedly initialized tracking");
}

void invalidationChecks(const cameras::NCameraSystem& cameras) {
  auto params = parameters(cameras);
  params.imu.use = false;
  TrackingFrontend tracker(params);
  auto active = snapshot(cameras, 10, stateAt(10.0));
  const auto pose = kinematics::Transformation::Identity();
  const auto tracked = tracker.track(frame(*active, 10.1, 2, pose), active, {});
  check(tracked.valid, "Reset fixture could not track its initial valid snapshot");
  tracker.markKeyframeAccepted(tracked.state);

  auto reset = std::make_shared<TrackingSnapshot>();
  reset->version = 11;
  reset->state.timestamp = active->state.timestamp;
  // A failed initialization publishes no usable state; poison its vectors to
  // ensure invalidation does not depend on the finite-state predicate.
  reset->state.v_W.setConstant(std::numeric_limits<double>::quiet_NaN());
  reset->state.b_g.setConstant(std::numeric_limits<double>::quiet_NaN());
  reset->state.b_a.setConstant(std::numeric_limits<double>::quiet_NaN());
  const auto invalidated = tracker.track(frame(*active, 10.12, 3, pose), reset, {});
  check(!invalidated.valid && invalidated.requestKeyframe && invalidated.feedbackVersion == 11
        && invalidated.matches == 0 && invalidated.inliers == 0,
        "A newer uninitialized snapshot failed to invalidate the old map");
  State predicted = stateAt(0, 42);
  check(!tracker.predict(Time(10.13), active, {}, &predicted)
        && predicted.timestamp == Time(0) && predicted.T_WS.r().x() == 42,
        "Old valid feedback resurrected a map after explicit invalidation");
  const auto delayed = tracker.track(frame(*active, 10.14, 4, pose), active, {});
  check(!delayed.valid && delayed.feedbackVersion == 11 && delayed.matches == 0,
        "Tracking fell back to old geometry after reset");
  auto equalVersion = snapshot(cameras, 11, stateAt(10.05));
  check(!tracker.predict(Time(10.14), equalVersion, {}, &predicted),
        "Equal-version valid feedback bypassed the reset watermark");
  const auto absent = tracker.track(frame(*active, 10.15, 5, pose), nullptr, {});
  check(!absent.valid && absent.feedbackVersion == 11,
        "An absent update discarded the reset watermark");
  tracker.markKeyframeAccepted(stateAt(10.15)); // Bootstrap queue admission has no tracked pose yet.

  auto restarted = snapshot(cameras, 12, stateAt(10.15));
  const auto recovered = tracker.track(frame(*restarted, 10.16, 6, pose), restarted, {});
  check(recovered.valid && recovered.feedbackVersion == 12 && recovered.requestKeyframe,
        "New valid feedback did not restart tracking with a cleared keyframe baseline");
}

void coverageKeyframeChecks(const cameras::NCameraSystem& cameras) {
  auto params = parameters(cameras);
  params.imu.use = false;
  TrackingFrontend tracker(params);
  const auto identity = kinematics::Transformation::Identity();
  auto base = snapshot(cameras, 1, stateAt(10.0));
  const auto first = tracker.track(frame(*base, 10.1, 2, identity), base, {});
  check(first.valid && first.keyframeOverlap == 1.0,
        "All matched detections did not produce full image-region overlap");
  tracker.markKeyframeAccepted(first.state);

  auto expanded = std::make_shared<TrackingSnapshot>(*base);
  expanded->version = 2;
  for (size_t i = 0; i < 256; ++i) {
    TrackingMapPoint point;
    point.id = LandmarkId(100 + i);
    point.point = Eigen::Vector4d((double(i % 16) - 7.5) * .1,
                                 (double(i / 16) - 7.5) * .1, 4.0, 1.0);
    point.descriptors.resize(1);
    // No current detection has this descriptor. The extra points are visible
    // map content, not failed observations of the current image.
    point.descriptors.front().fill(0);
    Eigen::Vector2d pixel;
    check(cameras.cameraGeometry(0)->project(point.point.head<3>(), &pixel)
              == cameras::ProjectionStatus::Successful,
          "Extra-map fixture contains a nonvisible landmark");
    expanded->points.push_back(std::move(point));
  }
  const auto largerMap = tracker.track(frame(*base, 10.15, 3, identity), expanded, {});
  check(largerMap.valid && largerMap.inliers == base->points.size()
        && largerMap.keyframeOverlap == 1.0 && !largerMap.requestKeyframe,
        "Extra visible map landmarks incorrectly reduced current-image overlap");

  // A current frame with detections throughout the image but surviving tracks
  // only in the middle must still request a keyframe on low region coverage.
  TrackingFrontend weakTracker(params);
  weakTracker.markKeyframeAccepted(first.state); // No accepted map yet: ignored.
  const auto seed = weakTracker.track(frame(*base, 10.1, 2, identity), base, {});
  check(seed.valid, "Low-coverage fixture failed to seed its active map");
  weakTracker.markKeyframeAccepted(seed.state);
  auto sparse = frame(*base, 10.15, 3, identity);
  cv::Mat sparseDescriptors = cv::Mat::zeros(int(base->points.size()), 48, CV_8UC1);
  for (size_t i = 0; i < base->points.size(); ++i) {
    const size_t row = i / 8, column = i % 8;
    if (row < 2 || row > 3 || column < 2 || column > 5) continue;
    for (size_t d = 0; d < 48; ++d)
      sparseDescriptors.at<uchar>(int(i), int(d)) = base->points[i].descriptors.front()[d];
  }
  sparse->resetDescriptors(0, sparseDescriptors);
  const auto weak = weakTracker.track(sparse, base, {});
  check(weak.valid && weak.inliers == 8 && weak.keyframeOverlap < params.frontend.keyframe_overlap
        && weak.requestKeyframe,
        "Low image-region coverage did not request a keyframe despite valid surviving tracks");

  auto moved = std::make_shared<TrackingSnapshot>(*base);
  moved->version = 3;
  moved->state = stateAt(10.18, .6);
  const auto translated = tracker.track(frame(*base, 10.19, 4, moved->state.T_WS), moved, {});
  check(translated.valid && translated.keyframeOverlap == 1.0 && translated.requestKeyframe,
        "Full region overlap suppressed the translation keyframe threshold");
  auto turned = std::make_shared<TrackingSnapshot>(*base);
  turned->version = 4;
  turned->state = stateAt(10.20);
  turned->state.T_WS.set(Eigen::Vector3d::Zero(),
      Eigen::Quaterniond(Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitZ())));
  const auto rotated = tracker.track(frame(*base, 10.21, 5, turned->state.T_WS), turned, {});
  check(rotated.valid && rotated.keyframeOverlap == 1.0 && rotated.requestKeyframe,
        "Full region overlap suppressed the rotation keyframe threshold");
  auto later = std::make_shared<TrackingSnapshot>(*base);
  later->version = 5;
  later->state = stateAt(10.3);
  const auto timed = tracker.track(frame(*base, 10.5, 6, identity), later, {});
  check(timed.valid && timed.keyframeOverlap == 1.0 && timed.requestKeyframe,
        "Full region overlap suppressed the sensor-time keyframe threshold");
}

void inactiveCameraCoverageChecks(const cameras::NCameraSystem& source) {
  auto cameras = source;
  cameras::NCameraSystem::CameraType type;
  type.isColour = false;
  type.isUsed = false;
  cameras.addCamera(source.T_SC(0), source.cameraGeometry(0), source.distortionType(0), false, type);
  type.isUsed = true;
  cameras.addCamera(source.T_SC(0), source.cameraGeometry(0), source.distortionType(0), false, type);
  auto params = parameters(cameras);
  params.imu.use = false;
  TrackingFrontend tracker(params);
  auto feedback = snapshot(cameras, 1, stateAt(10.0));
  const auto identity = kinematics::Transformation::Identity();
  const auto query = [&](double timestamp, uint64_t id) {
    auto output = frame(*feedback, timestamp, id, identity);
    // Camera 1 has an image and unmatched detections but is configured unused.
    output->resetDescriptors(1, cv::Mat::zeros(int(feedback->points.size()), 48, CV_8UC1));
    // Camera 2 has matching features but no current image and must not be used.
    output->setImage(2, cv::Mat());
    return output;
  };
  const auto first = tracker.track(query(10.1, 2), feedback, {});
  check(first.valid && first.inliers == feedback->points.size() && first.keyframeOverlap == 1.0,
        "Disabled or image-less cameras participated in matching/overlap");
  tracker.markKeyframeAccepted(first.state);
  const auto next = tracker.track(query(10.15, 3), feedback, {});
  check(next.valid && next.keyframeOverlap == 1.0 && !next.requestKeyframe,
        "Disabled or image-less cameras incorrectly requested a new keyframe");
}

void run() {
  using RadTan = cameras::PinholeCamera<cameras::RadialTangentialDistortion>;
  using Equidistant = cameras::PinholeCamera<cameras::EquidistantDistortion>;
  using RadTan8 = cameras::PinholeCamera<cameras::RadialTangentialDistortion8>;
  const auto cameras = cameraSystem(RadTan::createTestObject(), cameras::NCameraSystem::RadialTangential);
  feedbackChecks(cameras);
  invalidationChecks(cameras);
  coverageKeyframeChecks(cameras);
  inactiveCameraCoverageChecks(cameras);
  geometricChecks(cameras);
  geometricChecks(cameraSystem(Equidistant::createTestObject(), cameras::NCameraSystem::Equidistant));
  geometricChecks(cameraSystem(RadTan8::createTestObject(), cameras::NCameraSystem::RadialTangential8));
  geometricChecks(cameraSystem(cameras::EucmCamera::createTestObject(), cameras::NCameraSystem::NoDistortion));
}
}  // namespace
}  // namespace okvis

int main() {
  try {
    okvis::run();
    std::cout << "Tracking feedback, reset, IMU repropagation, camera geometry and invalid-observation checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
