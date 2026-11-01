// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/Frontend.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include "../src/PlaceRecognitionDb.hpp"
#include <chrono>
#include <iostream>
#include <random>
#include <stdexcept>

namespace okvis {
struct FrontendPlaceRecognitionTest {
  static void check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
  }
  static MultiFramePtr frame(const cameras::NCameraSystem& cameras, uint64_t id) {
    auto frame = std::make_shared<MultiFrame>(cameras, Time(10, 0), id);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors(32, 48, CV_8UC1);
    std::mt19937 random(1234);
    for (int i = 0; i < 32; ++i) {
      const Eigen::Vector3d point((i % 8 - 3.5) * .15, (i / 8 - 1.5) * .15, 3.0 + (i % 5) * .07);
      Eigen::Vector2d pixel;
      frame->geometry(0)->project(point, &pixel);
      keypoints.emplace_back(float(pixel.x()), float(pixel.y()), 12.0f);
      for (int k = 0; k < 48; ++k) descriptors.at<uchar>(i, k) = uchar(random());
    }
    frame->resetKeypoints(0, keypoints);
    frame->resetDescriptors(0, descriptors);
    frame->computeBackProjections(0);
    for (int i = 0; i < 32; ++i) {
      frame->setLandmarkId(0, i, uint64_t(i + 1));
      frame->setLandmark(0, i, Eigen::Vector4d((i % 8 - 3.5) * .15,
          (i / 8 - 1.5) * .15, 3.0 + (i % 5) * .07, 1.0), true);
    }
    return frame;
  }
  static void wait(Frontend& frontend, uint64_t count) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (frontend.placeRecognitionStatistics().processed < count) {
      check(std::chrono::steady_clock::now() < deadline, "Place recognition worker did not finish");
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  static void run(const std::string& vocabulary) {
    using Camera = cameras::PinholeCamera<cameras::RadialTangentialDistortion>;
    cameras::NCameraSystem cameras;
    cameras::NCameraSystem::CameraType type;
    type.isColour = false;
    cameras.addCamera(std::make_shared<const kinematics::Transformation>(kinematics::Transformation::Identity()),
        Camera::createTestObject(), cameras::NCameraSystem::RadialTangential, false, type);
    ViParameters params{};
    params.nCameraSystem = cameras;
    params.estimator.do_loop_closures = true;
    params.estimator.p_dbow = .55;
    params.estimator.realtime_num_threads = 1;
    params.estimator.realtime_max_iterations = 5;
    {
      Frontend sparse(1, vocabulary);
      std::vector<uchar> zero(48, 0), distinct(48, 255);
      std::mt19937 random(91);
      while (sparse.dBow_->vocabulary.transform(zero) == sparse.dBow_->vocabulary.transform(distinct))
        for (auto& byte : distinct) byte = uchar(random());
      for (uint64_t i = 0; i < 30; ++i) {
        sparse.dBow_->database.add(std::vector<std::vector<uchar>>{zero});
        sparse.dBow_->poseIds.push_back(100 + i);
      }
      sparse.dBow_->database.add(std::vector<std::vector<uchar>>{distinct});
      sparse.dBow_->poseIds.push_back(999);
      std::vector<std::pair<StateId, double>> candidates;
      sparse.getFilteredDBoWResult(sparse.dBow_, {distinct}, candidates);
      check(candidates.size() == 1 && candidates[0].first == StateId(999),
            "Sparse database entry ID was interpreted as a query-result index");
    }
    Frontend frontend(1, vocabulary);
    auto reference = frame(cameras, 10);
    auto saved = Frontend::snapshotPlaceRecognitionFrame(*reference, true);
    reference->setLandmark(0, 0, Eigen::Vector4d(999, 999, 999, 1), true);
    Eigen::Vector4d point;
    bool initialized;
    saved->getLandmark(0, 0, point, initialized);
    check(initialized && point.x() < 1.0 && saved->image(0).empty(),
          "Worker snapshot shares live geometry or retains raw images");
    check(frontend.enqueuePlaceRecognition(params, Frontend::snapshotPlaceRecognitionFrame(*saved, false), {saved}),
          "First keyframe was not queued");
    wait(frontend, 1);
    check(frontend.takePlaceRecognitionResults().empty(), "Keyframe retrieved itself");
    auto query = frame(cameras, 100);
    check(frontend.enqueuePlaceRecognition(params, Frontend::snapshotPlaceRecognitionFrame(*query, false), {}),
          "Revisit keyframe was not queued");
    wait(frontend, 2);
    const auto results = frontend.takePlaceRecognitionResults();
    check(results.size() == 1 && results[0].candidateId == StateId(10)
        && results[0].queryId == StateId(100) && results[0].information.allFinite()
        && results[0].relativePose.T().isApprox(Eigen::Matrix4d::Identity(), 1e-5),
        "Background DBoW/RANSAC/refinement failed a geometrically valid revisit");
    auto invalid = frame(cameras, 150);
    std::vector<cv::KeyPoint> inconsistent;
    std::mt19937 random(98);
    for (int i = 0; i < 32; ++i)
      inconsistent.emplace_back(float(100 + random() % 400), float(100 + random() % 200), 12.0f);
    invalid->resetKeypoints(0, inconsistent);
    invalid->computeBackProjections(0);
    check(frontend.enqueuePlaceRecognition(params, invalid, {}), "Invalid revisit was not queued");
    wait(frontend, 3);
    check(frontend.takePlaceRecognitionResults().empty(),
          "Identical descriptors bypassed background geometric verification");
    frontend.excludePlaceRecognitionFrame(StateId(10));
    auto blocked = Frontend::snapshotPlaceRecognitionFrame(*query, false);
    blocked->setId(160);
    check(frontend.enqueuePlaceRecognition(params, blocked, {}), "Excluded-frame query was not queued");
    wait(frontend, 4);
    check(frontend.takePlaceRecognitionResults().empty(), "Worker repeatedly proposed an ineligible reference");
    auto restored = Frontend::snapshotPlaceRecognitionFrame(*query, false);
    restored->setId(170);
    check(frontend.enqueuePlaceRecognition(params, restored, {saved}), "Refreshed-reference query was not queued");
    wait(frontend, 5);
    check(frontend.takePlaceRecognitionResults().size() == 1, "Completed reference did not become eligible again");
    for (uint64_t i = 0; i < 100; ++i) {
      auto next = Frontend::snapshotPlaceRecognitionFrame(*query, false);
      next->setId(200 + i);
      frontend.enqueuePlaceRecognition(params, next, {});
      check(frontend.placeRecognitionStatistics().pending <= 4, "Optional query backlog is not bounded");
    }
    frontend.finishPlaceRecognition();
    auto stats = frontend.placeRecognitionStatistics();
    check(!stats.active && stats.pending == 0 && stats.processed == stats.submitted
        && stats.submitted + stats.skipped == 105, "Shutdown lost accepted jobs or queue accounting");
    frontend.clear();
    check(frontend.placeRecognitionStatistics().submitted == 0, "Clear retained stale worker state");
    check(frontend.enqueuePlaceRecognition(params, Frontend::snapshotPlaceRecognitionFrame(*query, false), {}),
          "Worker did not restart after clear");
    frontend.finishPlaceRecognition();
    check(frontend.takePlaceRecognitionResults().empty(), "Clear retained old database entries or results");
  }
};
} // namespace okvis
int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("Expected vocabulary directory");
    okvis::FrontendPlaceRecognitionTest::run(argv[1]);
    std::cout << "Async place recognition snapshot, revisit, queue and lifecycle checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
