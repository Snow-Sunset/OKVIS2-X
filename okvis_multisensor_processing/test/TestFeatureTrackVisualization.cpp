// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/ViVisualizer.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
void run(const char* previewPath) {
  using namespace okvis;
  using Camera = cameras::PinholeCamera<cameras::RadialTangentialDistortion>;
  auto camera = Camera::createTestObject();
  ViParameters parameters{};
  cameras::NCameraSystem::CameraType cameraType;
  cameraType.isColour = false;
  parameters.nCameraSystem.addCamera(std::make_shared<const kinematics::Transformation>(
      kinematics::Transformation::Identity()), camera, cameras::NCameraSystem::RadialTangential,
      false, cameraType);
  auto data = std::make_shared<ViVisualizer::VisualizationData>();
  data->frontendTracking = true;
  data->feedbackVersion = 19;
  data->T_WS.setIdentity();
  data->T_SCi.push_back(kinematics::Transformation::Identity());
  data->currentFrames = std::make_shared<MultiFrame>(parameters.nCameraSystem, Time(12.345), 321);
  auto& frame = data->currentFrames;
  frame->setImage(0, cv::Mat(camera->imageHeight(), camera->imageWidth(), CV_8UC1, cv::Scalar(45)));
  const FeatureAssociation associations[] = {FeatureAssociation::TemporalTrack, FeatureAssociation::BriskMap,
      FeatureAssociation::MotionStereo, FeatureAssociation::Stereo, FeatureAssociation::LoopClosure,
      FeatureAssociation::None, FeatureAssociation::None, FeatureAssociation::None, FeatureAssociation::Unknown};
  const uint32_t ages[] = {18, 7, 3, 1, 5, 2, 1, 8, 12};
  std::vector<cv::KeyPoint> keys;
  for (size_t i = 0; i < 9; ++i) {
    Observation observation;
    observation.cameraIdx = 0; observation.keypointIdx = i; observation.frameId = frame->id();
    observation.keypointMeasurement = Eigen::Vector2d(90 + (i % 3) * 230, 85 + (i / 3) * 135);
    observation.keypointSize = 12;
    observation.track = FeatureTrack{100 + i, ages[i], associations[i]};
    observation.landmarkId = associations[i] == FeatureAssociation::None ? 0 : 200 + i;
    observation.isInitialized = observation.landmarkId != 0;
    observation.landmark_W.setZero();
    if (observation.landmarkId) {
      Eigen::Vector3d ray;
      check(camera->backProject(observation.keypointMeasurement + Eigen::Vector2d(40, 30), &ray),
          "Invalid projection fixture");
      observation.landmark_W.head<3>() = ray * (4.0 / ray.z());
      observation.landmark_W.w() = 1;
    }
    keys.emplace_back(float(observation.keypointMeasurement.x()), float(observation.keypointMeasurement.y()), 12.f);
    data->observations.push_back(observation);
  }
  frame->resetKeypoints(0, keys);
  ViVisualizer visualizer(parameters);
  const auto compact = visualizer.drawMatches(data, 0);
  check(compact.type() == CV_8UC3 && compact.cols == camera->imageWidth()
      && compact.rows == camera->imageHeight(), "Age overlay changed image dimensions or added a footer");
  check(cv::countNonZero(frame->image(0) != 45) == 0, "Visualization modified source image");
  const auto youngTrackColor = compact.at<cv::Vec3b>(85, 550);  // age 3, red
  const auto mediumTrackColor = compact.at<cv::Vec3b>(85, 320); // age 7, orange
  const auto longTrackColor = compact.at<cv::Vec3b>(85, 90);    // age 18, green
  check(youngTrackColor == cv::Vec3b(40, 40, 255)
      && mediumTrackColor == cv::Vec3b(0, 165, 255)
      && longTrackColor == cv::Vec3b(0, 255, 0),
      "Feature point colors do not encode their tracked-frame ranges");
  check(compact.at<cv::Vec3b>(115, 130) == cv::Vec3b(45, 45, 45),
      "Age-only overlay still drew a projected map-point marker");
  // Graph mutations after enqueue must not change the diagnostic being rendered.
  for (size_t i = 0; i < keys.size(); ++i) {
    frame->setFeatureTrack(0, i, FeatureTrack{9999, 999});
    frame->setLandmarkId(0, i, 9999, FeatureAssociation::LoopClosure);
  }
  const auto afterMutation = visualizer.drawMatches(data, 0);
  check(cv::norm(compact, afterMutation, cv::NORM_INF) == 0,
      "Visualizer read mutable graph/track metadata instead of its observation snapshot");
  frame->setLandmarkId(0, 0, 0);
  check(frame->featureTrack(0, 0).association == FeatureAssociation::None
      && frame->featureTrack(0, 0).id == 9999 && frame->featureTrack(0, 0).length == 999,
      "Outlier removal retained association provenance or destroyed image-track lifetime");
  for (auto& observation : data->observations) {
    observation.track.id = 123456789;
    observation.track.association = FeatureAssociation::LoopClosure;
    observation.landmarkId = 987654321;
    observation.landmark_W.setZero();
  }
  check(cv::norm(compact, visualizer.drawMatches(data, 0), cv::NORM_INF) == 0,
      "Track/map IDs or association source affected age-only visualization");
  ++data->observations.front().track.length;
  check(cv::norm(compact, visualizer.drawMatches(data, 0), cv::NORM_INF) > 0,
      "Changed track age was not reflected in its label");
  --data->observations.front().track.length;
  // Malformed measurements must not send NaNs or extreme coordinates into OpenCV drawing.
  data->observations.front().keypointMeasurement.x() = std::numeric_limits<double>::quiet_NaN();
  check(!visualizer.drawMatches(data, 0).empty(), "Invalid keypoint crashed diagnostics");
  data->observations.front().keypointMeasurement.x() = 90;
  if (previewPath) check(cv::imwrite(previewPath, compact), "Cannot save diagnostic preview");
  frame->setImage(0, cv::Mat());
  check(visualizer.drawMatches(data, 0).empty(), "Image-less camera generated a bogus visualization");
}
}
int main(int argc, char** argv) {
  try {
    run(argc > 1 ? argv[1] : nullptr);
    std::cout << "Age-only overlay, immutable snapshots and cleared-outlier diagnostics passed\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
