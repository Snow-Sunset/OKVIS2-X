// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/Frontend.hpp>
#include <okvis/TemporalFeatureTracker.hpp>
#include <okvis/TemporalMatching.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>

namespace {
using Camera = okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion>;
void check(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }

okvis::cameras::NCameraSystem cameraSystem() {
  okvis::cameras::NCameraSystem cameras;
  auto camera = Camera::createTestObject();
  camera->initialiseCameraAwarenessMaps();
  okvis::cameras::NCameraSystem::CameraType type;
  type.isColour = false;
  cameras.addCamera(std::make_shared<const okvis::kinematics::Transformation>(
      okvis::kinematics::Transformation::Identity()), camera,
      okvis::cameras::NCameraSystem::RadialTangential, false, type);
  return cameras;
}

okvis::ViParameters parameters(const okvis::cameras::NCameraSystem& cameras) {
  okvis::ViParameters parameters{};
  parameters.nCameraSystem = cameras;
  parameters.imu.use = false;
  parameters.imu.g0.setZero(); parameters.imu.a0.setZero();
  parameters.imu.sigma_bg = .01; parameters.imu.sigma_ba = .01;
  parameters.frontend.matching_threshold = 60;
  parameters.frontend.num_matching_threads = 1;
  parameters.frontend.keyframe_overlap = .6;
  parameters.frontend.max_num_keypoints = 200;
  parameters.estimator.realtime_num_threads = 1;
  return parameters;
}

cv::Mat texture(cv::Size size) {
  cv::Mat image(size, CV_8UC1);
  cv::RNG random(789);
  random.fill(image, cv::RNG::UNIFORM, 0, 255);
  cv::GaussianBlur(image, image, cv::Size(3, 3), .7);
  return image;
}

cv::Mat translated(const cv::Mat& image) {
  cv::Mat result;
  const cv::Mat transform = (cv::Mat_<double>(2, 3) << 1, 0, 3, 0, 1, -2);
  cv::warpAffine(image, result, transform, image.size());
  return result;
}

void imageTrackingChecks(const okvis::cameras::NCameraSystem& cameras, const std::string& resources) {
  auto params = parameters(cameras);
  okvis::TemporalFeatureTracker tracker(params.frontend);
  const auto image = texture(cv::Size(640, 480));
  check(tracker.propose(image, okvis::Time(1.0)).empty(), "First image unexpectedly had tracks");
  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(image, corners, 100, .01, 25);
  std::vector<cv::KeyPoint> keys;
  for (auto point : corners) keys.emplace_back(point, 12.f);
  const auto first = tracker.commit(image, okvis::Time(1.0), keys);
  const auto shifted = translated(image);
  auto tracked = tracker.propose(shifted, okvis::Time(1.05));
  check(tracked.size() >= keys.size() * .8, "LK lost most features under a small translation");
  for (const auto& key : tracked)
    check(cv::norm(key.pt - keys.at(key.class_id).pt - cv::Point2f(3, -2)) < .15,
          "LK accepted an inaccurate temporal correspondence");
  // Emulate an extractor dropping and reordering some tracked points.
  tracked.resize(tracked.size() / 2);
  std::reverse(tracked.begin(), tracked.end());
  const auto second = tracker.commit(shifted, okvis::Time(1.05), tracked);
  for (size_t i = 0; i < tracked.size(); ++i)
    check(second[i].id == first.at(tracked[i].class_id).id && second[i].length == 2,
          "Descriptor filtering/reordering changed track identity");
  check(tracker.propose(shifted, okvis::Time(2.0)).empty(), "Long image gap retained stale tracks");
  keys.resize(1); keys[0].class_id = -1;
  check(tracker.commit(shifted, okvis::Time(2.0), keys)[0].id != first[0].id,
        "Restarted track reused a terminated identity");

  // Exercise real camera-aware BRISK extraction, circular masking and post-extractor metadata.
  okvis::Frontend frontend(1, resources);
  frontend.configureTemporalTracking(params.frontend);
  frontend.setBriskDetectionMaximumKeypoints(200);
  frontend.setBriskDetectionThreshold(20);
  frontend.setBriskDetectionAbsoluteThreshold(30);
  const auto geometry = cameras.cameraGeometry(0);
  const auto pixels = texture(cv::Size(geometry->imageWidth(), geometry->imageHeight()));
  std::map<uint64_t, cv::Point2f> oldTracks;
  for (int index = 0; index < 2; ++index) {
    auto frame = std::make_shared<okvis::MultiFrame>(cameras, okvis::Time(3 + index * .05), index + 1);
    frame->setImage(0, index ? translated(pixels) : pixels);
    frontend.detectAndDescribe(0, frame, okvis::kinematics::Transformation::Identity(), nullptr);
    check(frame->numKeypoints(0) >= 20 && frame->numKeypoints(0) <= 200, "Invalid replenished feature count");
    size_t continuing = 0;
    for (size_t k = 0; k < frame->numKeypoints(0); ++k) {
      const auto track = frame->featureTrack(0, k);
      cv::KeyPoint key; frame->getCvKeypoint(0, k, key);
      check(track.id && track.length && frame->keypointDescriptor(0, k), "Incomplete tracked feature output");
      if (!index) oldTracks.emplace(track.id, key.pt);
      else if (track.length == 2) {
        check(oldTracks.count(track.id) && cv::norm(key.pt - oldTracks.at(track.id)
            - cv::Point2f(3, -2)) < .2, "BRISK extraction misaligned LK track metadata");
        ++continuing;
      }
      for (size_t j = 0; j < k; ++j) {
        cv::KeyPoint other; frame->getCvKeypoint(0, j, other);
        check(cv::norm(key.pt - other.pt) >= params.frontend.track_min_distance - 1.5,
              "Circular mask allowed duplicate nearby features");
      }
    }
    if (index) check(continuing >= oldTracks.size() / 2, "Replenishment replaced healthy long tracks");
  }
}

okvis::MultiFramePtr geometryFrame(const okvis::cameras::NCameraSystem& cameras,
    double timestamp, double translation, uint32_t length, bool outliers, bool reverse = false) {
  auto frame = std::make_shared<okvis::MultiFrame>(cameras, okvis::Time(timestamp));
  const auto geometry = cameras.cameraGeometry(0);
  frame->setImage(0, cv::Mat::zeros(geometry->imageHeight(), geometry->imageWidth(), CV_8UC1));
  std::vector<cv::KeyPoint> keys;
  for (int j = 0; j < 48; ++j) {
    const int i = reverse ? 47 - j : j;
    Eigen::Vector3d point((i % 8 - 3.5) * .4 - translation, (i / 8 - 2.5) * .4,
                           3.5 + (i % 5) * .13);
    Eigen::Vector2d pixel;
    check(geometry->project(point, &pixel) == okvis::cameras::ProjectionStatus::Successful,
          "Invalid geometric fixture projection");
    if (outliers && i >= 40) pixel += Eigen::Vector2d(50 + i * 2, 40 - i * 3);
    keys.emplace_back(float(pixel.x()), float(pixel.y()), 12.f);
  }
  frame->resetKeypoints(0, keys);
  frame->resetDescriptors(0, cv::Mat(48, 48, CV_8UC1, cv::Scalar(0)));
  frame->computeBackProjections(0);
  for (size_t j = 0; j < keys.size(); ++j)
    frame->setFeatureTrack(0, j, okvis::FeatureTrack{100 + (reverse ? 47 - j : j), length});
  return frame;
}

void geometryChecks(const okvis::cameras::NCameraSystem& cameras, const std::string& resources) {
  const auto params = parameters(cameras);
  auto a = geometryFrame(cameras, 1, 0, 1, false);
  auto b = geometryFrame(cameras, 1.1, .3, 3, true, true);
  const auto pairs = okvis::temporalTrackPairs(*a, *b, 0, 3);
  check(pairs.size() == 48 && pairs[0].previous == 47 && pairs[0].current == 0,
        "Temporal pairing depended on descriptor or row order");
  bool rotationOnly;
  const auto good = okvis::temporalRansac(*a, *b, 0, pairs, 2.5 / 400, &rotationOnly);
  size_t staticInliers = 0, dynamicInliers = 0;
  for (size_t i = 0; i < pairs.size(); ++i)
    if (good[i]) (pairs[i].previous < 40 ? staticInliers : dynamicInliers)++;
  // Epipolar consensus can retain a few coincidental matches. The backend test
  // below additionally requires every injected outlier to fail triangulation/reprojection.
  if (!(staticInliers >= 36 && dynamicInliers <= 3 && !rotationOnly))
    throw std::runtime_error("Temporal RANSAC: static=" + std::to_string(staticInliers)
        + ", dynamic=" + std::to_string(dynamicInliers) + ", rotation=" + std::to_string(rotationOnly));
  const std::vector<okvis::TemporalPair> few(pairs.begin(), pairs.begin() + 5);
  const auto insufficient = okvis::temporalRansac(*a, *b, 0, few, 2.5 / 400, &rotationOnly);
  check(std::count(insufficient.begin(), insufficient.end(), true) == 0,
        "Insufficient temporal geometry bypassed validation");
  const auto stationary = geometryFrame(cameras, 1.1, 0, 3, false);
  const auto stationaryPairs = okvis::temporalTrackPairs(*a, *stationary, 0, 3);
  const auto stationaryInliers = okvis::temporalRansac(*a, *stationary, 0, stationaryPairs,
      2.5 / 400, &rotationOnly);
  check(rotationOnly && std::count(stationaryInliers.begin(), stationaryInliers.end(), true) >= 40,
        "Pure rotation/stationary geometry was treated as a valid translation");

  okvis::ViSlamBackend backend;
  backend.addCamera(params.camera); backend.addImu(params.imu);
  okvis::Frontend frontend(1, resources);
  frontend.configureTemporalTracking(params.frontend);
  std::set<uint64_t> mappedTracks;
  for (int index = 0; index < 3; ++index) {
    const double translation = index * .15;
    auto frame = geometryFrame(cameras, 2 + index * .05, translation, index + 1,
                                index == 2, index == 2);
    check(backend.addStates(frame, {}, true), "Cannot add temporal test state");
    backend.setPose(okvis::StateId(frame->id()), okvis::kinematics::Transformation(
        Eigen::Vector3d(translation, 0, 0), Eigen::Quaterniond::Identity()));
    bool keyframe = true;
    check(frontend.dataAssociationAndInitialization(backend, params, frame, false, &keyframe),
          "Healthy immature/low-parallax tracks reset monocular initialization");
    backend.setKeyframe(okvis::StateId(frame->id()), keyframe);
    if (index == 1)
      for (size_t k = 0; k < 48; ++k)
        check(!frame->landmarkId(0, k), "Reserved young tracks created premature landmarks");
    if (index == 2) {
      size_t mapped = 0;
      for (size_t k = 0; k < 48; ++k) {
        if (frame->landmarkId(0, k)) {
          ++mapped;
          mappedTracks.insert(frame->featureTrack(0, k).id);
          check(frame->featureTrack(0, k).association == okvis::FeatureAssociation::MotionStereo,
                "Triangulated observation was mislabeled as a map match");
        }
        if (frame->featureTrack(0, k).id >= 140)
          check(!frame->landmarkId(0, k), "Temporal outlier entered the optimization graph");
      }
      if (mapped < 30 || !frontend.isInitialized())
        throw std::runtime_error("Track-ID triangulation: mapped=" + std::to_string(mapped)
            + ", initialized=" + std::to_string(frontend.isInitialized()));
    }
  }

  // Backend admission skips image frames and descriptors may change. Track identity
  // must still recover live map observations from previous admitted frames.
  auto mature = geometryFrame(cameras, 2.3, .45, 7, true, false);
  mature->resetDescriptors(0, cv::Mat(48, 48, CV_8UC1, cv::Scalar(255)));
  check(backend.addStates(mature, {}, false), "Cannot admit delayed mature track frame");
  backend.setPose(okvis::StateId(mature->id()), okvis::kinematics::Transformation(
      Eigen::Vector3d(.45, 0, 0), Eigen::Quaterniond::Identity()));
  bool keyframe = false;
  check(frontend.dataAssociationAndInitialization(backend, params, mature, false, &keyframe),
        "Backend lost mature tracks after skipped image frames");
  size_t linked = 0;
  for (size_t k = 0; k < mature->numKeypoints(0); ++k) if (mature->landmarkId(0, k)) {
    // Previously unmapped tracks may triangulate for the first time on this frame.
    if (!mappedTracks.count(mature->featureTrack(0, k).id)) continue;
    ++linked;
    check(mature->featureTrack(0, k).association == okvis::FeatureAssociation::TemporalTrack,
          "Backend continuous track association was mislabeled");
  }
  check(linked >= 30, "Backend mature track association depended on BRISK descriptors");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("Expected vocabulary resource directory");
    const auto cameras = cameraSystem();
    imageTrackingChecks(cameras, argv[1]);
    geometryChecks(cameras, argv[1]);
    std::cout << "LK, mask, feature lifetime, pre-insertion RANSAC and temporal map initialization passed\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
