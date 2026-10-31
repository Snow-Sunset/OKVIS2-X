// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/Frame.hpp>
#include <okvis/MultiFrame.hpp>
#include <okvis/ViSlamBackend.hpp>
#include <okvis/ceres/TwoPoseExtrinsicsGraphError.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/NoDistortion.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <unistd.h>

namespace {
void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
using Camera = okvis::cameras::PinholeCamera<okvis::cameras::NoDistortion>;
okvis::cameras::NCameraSystem cameraSystem() {
  okvis::cameras::NCameraSystem system;
  auto extrinsics = std::make_shared<const okvis::kinematics::Transformation>(
      okvis::kinematics::Transformation::Identity());
  okvis::cameras::NCameraSystem::CameraType type;
  type.isColour = false;
  system.addCamera(extrinsics, Camera::createTestObject(),
      okvis::cameras::NCameraSystem::NoDistortion, false, type);
  return system;
}
okvis::ImuParameters imuParameters() {
  okvis::ImuParameters imu{};
  imu.use = false;
  imu.g0.setZero(); imu.a0.setZero();
  imu.sigma_bg = 0.01; imu.sigma_ba = 0.01;
  return imu;
}
std::vector<cv::KeyPoint> keypoints() {
  return {cv::KeyPoint(100.f, 110.f, 5.f), cv::KeyPoint(150.f, 120.f, 7.f),
      cv::KeyPoint(200.f, 130.f, 9.f), cv::KeyPoint(250.f, 140.f, 11.f)};
}
void testPayload(const std::filesystem::path& directory) {
  okvis::Frame frame;
  frame.setGeometry(Camera::createTestObject());
  cv::Mat original(240, 340, CV_8UC1);
  cv::randu(original, 0, 255);
  // Verify that non-contiguous images round-trip correctly too.
  const cv::Mat image = original(cv::Rect(7, 5, 320, 220)).clone();
  frame.setImage(original(cv::Rect(7, 5, 320, 220)));
  cv::Mat depth(24, 32, CV_32FC1, cv::Scalar(4.25));
  frame.setDepthImage(depth.clone());
  frame.resetKeypoints(keypoints());
  frame.computeBackProjections();
  cv::Mat descriptors(4, 48, CV_8UC1);
  cv::randu(descriptors, 0, 255);
  frame.resetDescriptors(descriptors.clone());
  frame.setLandmarkId(1, 123);
  frame.setLandmark(1, Eigen::Vector4d(1, 2, 3, 1), true);
  Eigen::Vector3d ray;
  frame.getBackProjection(1, ray);
  const auto file = (directory / "frame.payload").string();
  const size_t before = frame.residentPayloadBytes();
  check(before > 70000, "Payload accounting missing image or feature memory");
  check(frame.archivePayload(file), "Frame archive failed");
  check(!frame.payloadResident() && frame.residentPayloadBytes() == 0, "Archive retained payload memory");
  check(frame.numKeypoints() == 4 && !frame.payloadResident(), "Feature count unnecessarily loaded payload");
  check(cv::norm(frame.image(), image, cv::NORM_INF) == 0, "Image archive mismatch");
  check(cv::norm(frame.depthImage(), depth, cv::NORM_INF) == 0, "Depth archive mismatch");
  check(std::memcmp(frame.keypointDescriptor(0), descriptors.data, 4 * 48) == 0, "Descriptor mismatch");
  cv::KeyPoint point;
  frame.getCvKeypoint(2, point);
  check(point.pt == keypoints()[2].pt && point.size == keypoints()[2].size, "Keypoint mismatch");
  Eigen::Vector3d loadedRay;
  frame.getBackProjection(1, loadedRay);
  check((ray - loadedRay).norm() == 0, "Backprojection mismatch");
  Eigen::Vector4d landmark;
  bool initialised;
  frame.getLandmark(1, landmark, initialised);
  check(initialised && (landmark - Eigen::Vector4d(1, 2, 3, 1)).norm() == 0 && frame.landmarkId(1) == 123,
      "Landmark archive mismatch");
  frame.setLandmarkId(1, 456);
  frame.archivePayload(file);
  check(frame.landmarkId(1) == 456, "Dirty landmark mutation was lost on page-out");
  okvis::Frame copy(frame);
  frame.archivePayload(file);
  check(copy.landmarkId(1) == 456 && copy.numKeypoints() == 4, "Frame copy lost feature payload");

  // A failed archive must retain the only in-memory copy.
  frame.ensurePayload();
  bool failed = false;
  try { frame.archivePayload((directory / "missing" / "bad.frame").string()); }
  catch (const std::exception&) { failed = true; }
  check(failed && frame.payloadResident() && frame.landmarkId(1) == 456, "Failed disk write discarded frame data");

  // Failed loading must not turn a truncated file into an empty/mis-associated loop candidate.
  frame.archivePayload(file);
  std::filesystem::resize_file(file, 24);
  failed = false;
  try { frame.image(); } catch (const std::exception&) { failed = true; }
  check(failed && !frame.payloadResident(), "Truncated historical payload was silently accepted");
}

void testPayloadIntegrity(const std::filesystem::path& directory) {
  class ClassifiedFrame : public okvis::Frame {
   public:
    void classify() {
      classifications_ = cv::Mat(4, 19, CV_32FC1, cv::Scalar(0.25));
      isClassified_ = true;
    }
    void setClassificationBusy(bool busy) { classificationInProgress_ = busy; }
  } frame;
  frame.setImage(cv::Mat(20, 30, CV_8UC1, cv::Scalar(7)));
  frame.resetKeypoints(keypoints());
  frame.resetDescriptors(cv::Mat(4, 48, CV_8UC1, cv::Scalar(11)));
  frame.classify();
  const auto filename = (directory / "integrity.frame").string();
  check(frame.archivePayload(filename), "Cannot archive integrity fixture");
  std::ifstream input(filename, std::ios::binary);
  const std::vector<char> original((std::istreambuf_iterator<char>(input)),
                                   std::istreambuf_iterator<char>());
  input.close();
  const auto write = [&](const std::vector<char>& bytes) {
    std::ofstream out(filename, std::ios::binary | std::ios::trunc);
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out.write(bytes.data(), bytes.size());
    out.close();
  };
  const auto reject = [&](const std::vector<char>& bytes) {
    write(bytes);
    bool failed = false;
    try { frame.ensurePayload(); } catch (const std::exception&) { failed = true; }
    check(failed && !frame.payloadResident(), "Corrupt archive became a resident loop candidate");
  };
  auto corrupted = original;
  // Leave payload lengths unchanged to exercise semantic, rather than truncation, checks.
  const size_t descriptorTypeOffset = sizeof(uint64_t) + 3 * sizeof(int) + 600 +
      3 * sizeof(int) + 2 * sizeof(int);
  const int signedBytes = CV_8SC1;
  std::memcpy(corrupted.data() + descriptorTypeOffset, &signedBytes, sizeof(int));
  reject(corrupted);
  corrupted = original;
  const size_t classificationHeaderOffset = descriptorTypeOffset + sizeof(int) + 4 * 48;
  const int classificationShape[] = {2, 38}; // Same bytes, wrong per-keypoint layout.
  std::memcpy(corrupted.data() + classificationHeaderOffset, classificationShape,
      sizeof(classificationShape));
  reject(corrupted);
  corrupted = original;
  corrupted[corrupted.size() - sizeof(uint64_t) - 1] = 0; // Lost classification status.
  reject(corrupted);
  corrupted = original;
  corrupted[corrupted.size() - sizeof(uint64_t) - 2] = 2; // Invalid landmark boolean.
  reject(corrupted);
  corrupted = original;
  corrupted.push_back('x');
  reject(corrupted);
  corrupted = original;
  const int dimensions[] = {1 << 30, 1 << 30, CV_64FC(16)};
  std::memcpy(corrupted.data() + sizeof(uint64_t), dimensions, sizeof(dimensions));
  reject(corrupted);
  // All failed attempts must leave the frame reloadable once the original file is restored.
  write(original);
  cv::Mat classification;
  check(frame.getClassification(1, classification) && classification.at<float>(0, 0) == 0.25f &&
      frame.image().at<uchar>(0, 0) == 7 && frame.keypointDescriptor(0)[0] == 11,
      "Failed archive reads damaged the original classified payload");
  frame.setClassificationBusy(true);
  check(!frame.releaseImages() && !frame.image().empty(), "Released images used by a CNN worker");
  frame.setClassificationBusy(false);
  check(frame.releaseImages() && frame.image().empty() && frame.imageSize() == cv::Size(30, 20) &&
      frame.getClassification(1, classification) && classification.at<float>(0, 0) == 0.25f,
      "Cold image release lost dimensions or classification results");
}

void testBackendCache(const std::filesystem::path& directory) {
  okvis::ViSlamBackend backend;
  const auto system = cameraSystem();
  backend.addCamera(okvis::CameraParameters{});
  backend.addImu(imuParameters());
  backend.configureFrameArchive(directory.string(), 0);
  for (int i = 1; i <= 10; ++i) {
    auto frame = std::make_shared<okvis::MultiFrame>(system, okvis::Time(i, 0));
    frame->setImage(0, cv::Mat(240, 320, CV_8UC1, cv::Scalar(i)));
    frame->resetKeypoints(0, keypoints());
    frame->computeBackProjections(0);
    frame->resetDescriptors(0, cv::Mat(4, 48, CV_8UC1, cv::Scalar(i)));
    check(backend.addStates(frame, {}, true), "Cannot add test keyframe");
    frame.reset();
    std::set<okvis::StateId> affected;
    backend.applyStrategy(1, 0, 1, affected, false);
  }
  const auto stats = backend.frameArchiveStatistics();
  check(stats.frameCount == 10 && stats.coldResidentPayloadBytes == 0,
      "Cold frame cache exceeded zero-byte budget or deleted long-term frames");
  check(stats.residentFrameCount <= 2, "Historical images remain resident");
  {
    const auto candidate = backend.multiFrame(okvis::StateId(1));
    const auto* descriptor = candidate->keypointDescriptor(0, 0);
    const cv::Mat& image = candidate->image(0);
    backend.trimFramePayloadCache();
    check(candidate->residentPayloadBytes() > 0 && descriptor[0] == 1 && image.at<uchar>(0, 0) == 1,
        "LRU evicted a borrowed loop candidate/descriptor reference");
    check(candidate->numKeypoints(0) == 4, "Historical loop candidate features lost");
  }
  backend.trimFramePayloadCache();
  check(backend.frameArchiveStatistics().coldResidentPayloadBytes == 0,
      "Unpinned historical candidate was not paged out");
  auto candidate = backend.multiFrame(okvis::StateId(1));
  check(candidate->keypointDescriptor(0, 0)[0] == 1, "Old loop candidate cannot be reloaded repeatedly");
  candidate->setLandmarkId(0, 0, 987);
  candidate.reset(); backend.trimFramePayloadCache();
  check(backend.multiFrame(okvis::StateId(1))->landmarkId(0, 0) == 987,
      "Historical graph landmark updates lost after archive rewrite");
}

void testInMemoryHistory(const std::filesystem::path& directory) {
  const auto previousDirectory = std::filesystem::current_path();
  struct RestoreDirectory {
    std::filesystem::path previous;
    ~RestoreDirectory() { std::filesystem::current_path(previous); }
  } restore{previousDirectory};
  std::filesystem::create_directories(directory);
  std::filesystem::current_path(directory);
  okvis::ViSlamBackend backend;
  const auto system = cameraSystem();
  backend.addCamera(okvis::CameraParameters{});
  backend.addImu(imuParameters());
  backend.configureInMemoryHistory();
  okvis::MultiFramePtr pinned;
  for (int i = 1; i <= 10; ++i) {
    auto frame = std::make_shared<okvis::MultiFrame>(system, okvis::Time(i, 0));
    frame->setImage(0, cv::Mat(240, 320, CV_8UC1, cv::Scalar(i)));
    frame->setDepthImage(0, cv::Mat(240, 320, CV_32FC1, cv::Scalar(4.25)));
    frame->resetKeypoints(0, keypoints());
    frame->computeBackProjections(0);
    frame->resetDescriptors(0, cv::Mat(4, 48, CV_8UC1, cv::Scalar(i)));
    check(backend.addStates(frame, {}, true), "Cannot add RAM-only test frame");
    if (i == 1) pinned = frame;
    frame.reset();
    std::set<okvis::StateId> affected;
    backend.applyStrategy(1, 0, 1, affected, false);
  }
  check(!pinned->image(0).empty(), "RAM-only history released a borrowed image");
  pinned.reset();
  backend.trimFramePayloadCache();
  auto old = backend.multiFrame(okvis::StateId(1));
  check(old->image(0).empty() && old->depthImage(0).empty(), "Cold images retained in RAM");
  check(old->imageSize(0) == cv::Size(320, 240) && old->numKeypoints(0) == 4 &&
      old->keypointDescriptor(0, 0)[0] == 1 && old->residentPayloadBytes() > 0,
      "RAM-only history discarded dimensions or loop-closure features");
  Eigen::Vector3d ray;
  check(old->getBackProjection(0, 1, ray), "Cold frame lost cached backprojection");
  auto newest = backend.multiFrame(okvis::StateId(10));
  check(!newest->image(0).empty(), "Active frame image released prematurely");
  check(backend.frameArchiveStatistics().frameCount == 10, "RAM-only history deleted old frames");

  // Overlap must depend on feature positions and original dimensions, even after image release.
  auto a = std::make_shared<okvis::MultiFrame>(system, okvis::Time(11, 0));
  auto b = std::make_shared<okvis::MultiFrame>(system, okvis::Time(12, 0));
  for (auto frame : {a, b}) {
    frame->setImage(0, cv::Mat(240, 320, CV_8UC1, cv::Scalar(1)));
    frame->resetKeypoints(0, keypoints());
    for (size_t k = 0; k < 4; ++k) frame->setLandmarkId(0, k, 100 + k);
  }
  const double before = backend.overlapFraction(a, b);
  check(before > 0.99, "Invalid overlap test fixture");
  check(a->releaseImages() && b->releaseImages(), "Cannot release cold test images");
  check(backend.overlapFraction(a, b) == before, "Cold image release changed feature overlap");
  check(std::filesystem::is_empty(directory), "RAM-only history created files/directories");
}

class Graph : public okvis::ViGraphEstimator {
 public:
  std::weak_ptr<::ceres::CostFunction> firstFactor() const {
    return states_.at(okvis::StateId(1)).submapReferenceLinks.front().errorTerm;
  }
  bool linksEmpty() const {
    for (const auto& state : states_) {
      if (!state.second.submapReferenceLinks.empty() || !state.second.submapLinks.empty() ||
          !state.second.mapResIds.empty()) return false;
    }
    return true;
  }
};
void testFactorLifetime() {
  Graph graph;
  graph.addCamera(okvis::CameraParameters{});
  graph.addImu(imuParameters());
  graph.addStatesInitialise(okvis::Time(1, 0), {}, cameraSystem());
  graph.addStatesPropagate(okvis::Time(2, 0), {}, false);
  graph.addStatesPropagate(okvis::Time(3, 0), {}, true);
  auto plane = std::make_shared<okvis::PlaneMap>(okvis::PlaneMap::Config{});
  okvis::PlaneMap::Points support;
  for (int x=1; x<=4; ++x) for (int y=1; y<=4; ++y) support.emplace_back(x*.1, y*.1, .2);
  plane->addPoints(support);
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points{{.2f, .2f, .2f}};
  check(graph.addSubmapAlignmentConstraints(nullptr, 1, 2, points, {.05f}, true, "Cauchy", plane),
      "Cannot add point-plane factor");
  auto lifetime = graph.firstFactor();
  check(!lifetime.expired(), "Missing point-plane factor");
  graph.eliminateStateByImuMerge(okvis::StateId(2), okvis::StateId(1));
  check(lifetime.expired() && graph.linksEmpty(), "Eliminated laser factor retained at surviving anchor");
  graph.addSubmapAlignmentConstraints(nullptr, 1, 3, points, {.05f}, true, "Cauchy", plane);
  lifetime = graph.firstFactor();
  graph.freezePosesUntil(okvis::StateId(3), true);
  check(lifetime.expired() && graph.linksEmpty(), "Frozen/removed pose retains laser factor ownership");
}
void testTwoPoseArchive(const std::filesystem::path& directory, bool calibrate) {
  const auto geometry = std::static_pointer_cast<const Camera>(Camera::createTestObject());
  auto a = std::make_shared<okvis::ceres::PoseParameterBlock>(
      okvis::kinematics::Transformation::Identity(), 1, okvis::Time(1, 0));
  auto b = std::make_shared<okvis::ceres::PoseParameterBlock>(
      okvis::kinematics::Transformation(Eigen::Vector3d(.25, 0, 0), Eigen::Quaterniond::Identity()),
      2, okvis::Time(2, 0));
  auto extrinsic = std::make_shared<okvis::ceres::PoseParameterBlock>(
      okvis::kinematics::Transformation::Identity(), 10, okvis::Time(1, 0));
  const auto makeEdge = [&]() -> std::shared_ptr<okvis::ceres::TwoPoseGraphError> {
    if (calibrate) return std::make_shared<okvis::ceres::TwoPoseExtrinsicsGraphError>(
        okvis::StateId(1), okvis::StateId(2), 1);
    return std::make_shared<okvis::ceres::TwoPoseStandardGraphError>(
        okvis::StateId(1), okvis::StateId(2), 1);
  };
  auto baseline = makeEdge(), paged = makeEdge();
  ::ceres::CauchyLoss loss(1.0);
  for (int k = 0; k < 30; ++k) {
    const Eigen::Vector4d hp(-.7 + .25 * (k % 6), -.5 + .2 * ((k/6) % 5), 3.0 + .1 * (k%7), 1);
    auto point = std::make_shared<okvis::ceres::HomogeneousPointParameterBlock>(hp, 100 + k, true);
    for (int side = 0; side < 2; ++side) {
      const auto pose = side ? b : a;
      const Eigen::Vector4d pc = pose->estimate().inverse() * hp;
      Eigen::Vector2d pixel;
      geometry->project(pc.head<3>(), &pixel);
      auto reprojection = std::make_shared<okvis::ceres::ReprojectionError<Camera>>(
          geometry, 0, pixel, Eigen::Matrix2d::Identity());
      for (const auto& edge : {baseline, paged}) {
        edge->addObservation(okvis::KeypointIdentifier(side + 1, 0, k), reprojection, &loss,
            pose, point, extrinsic, k % 4 == 0);
      }
    }
  }
  baseline->compute(); paged->compute();
  auto* cost = dynamic_cast<::ceres::CostFunction*>(paged.get());
  check(cost != nullptr, "Missing Ceres interface on visual graph edge");
  b->setEstimate(okvis::kinematics::Transformation(Eigen::Vector3d(.28, -.01, .02), Eigen::Quaterniond::Identity()));
  const double* parameters[] = {a->parameters(), b->parameters(), extrinsic->parameters()};
  std::vector<double> before(cost->num_residuals()), after(cost->num_residuals());
  check(cost->Evaluate(parameters, before.data(), nullptr), "Cannot evaluate visual graph edge");
  std::vector<okvis::ceres::TwoPoseGraphError::Observation> original;
  paged->getObservations(original);
  const auto filename = (directory / (calibrate ? "calibrated.two_pose" : "standard.two_pose")).string();
  check(paged->residentObservationBytes() > 0, "Missing visual recovery payload accounting");
  check(paged->archiveObservations(filename), "Cannot archive computed visual graph edge");
  check(!paged->observationsResident() && paged->residentObservationBytes() == 0,
      "Visual recovery observations remain resident");
  check(cost->Evaluate(parameters, after.data(), nullptr), "Archived factor cannot evaluate without images/observations");
  check(before == after && !paged->observationsResident(), "Archive changed graph residual or Evaluate loaded history");
  // Loading a truncated recovery file must fail without inventing observations.
  std::filesystem::copy_file(filename, filename + ".backup", std::filesystem::copy_options::overwrite_existing);
  std::filesystem::resize_file(filename, 32);
  bool failed = false;
  try { std::vector<okvis::ceres::TwoPoseGraphError::Observation> invalid; paged->getObservations(invalid); }
  catch (const std::exception&) { failed = true; }
  check(failed && !paged->observationsResident(), "Truncated visual recovery archive was silently accepted");
  std::filesystem::copy_file(filename + ".backup", filename, std::filesystem::copy_options::overwrite_existing);
  std::vector<okvis::ceres::TwoPoseGraphError::Observation> recovered;
  paged->getObservations(recovered);
  failed = false;
  try { paged->archiveObservations((directory / "missing" / "failure.two_pose").string()); }
  catch (const std::exception&) { failed = true; }
  check(failed && paged->observationsResident(), "Failed visual recovery archive discarded observations");
  check(recovered.size() == original.size(), "Recovered observation count mismatch");
  std::map<uint64_t, std::shared_ptr<okvis::ceres::HomogeneousPointParameterBlock>> pointIdentities;
  for (size_t i = 0; i < original.size(); ++i) {
    const auto& x = original[i]; const auto& y = recovered[i];
    check(x.keypointIdentifier == y.keypointIdentifier && x.pose == y.pose && x.extrinsics == y.extrinsics &&
        x.lossFunction == y.lossFunction && x.isMarginalised == y.isMarginalised && x.isDuplication == y.isDuplication &&
        (x.reprojectionError->measurement() - y.reprojectionError->measurement()).norm() == 0 &&
        (x.reprojectionError->information() - y.reprojectionError->information()).norm() == 0 &&
        x.hPoint->id() == y.hPoint->id() && (x.hPoint->estimate() - y.hPoint->estimate()).norm() == 0,
        "Visual recovery payload/parameter identity mismatch");
    const auto inserted = pointIdentities.emplace(y.hPoint->id(), y.hPoint);
    check(inserted.first->second == y.hPoint, "Observations of one landmark lost shared parameter identity");
  }
  original.clear(); recovered.clear(); pointIdentities.clear();
  paged->archiveObservations(filename);
  // Simulate a loop-correction of the reference pose before reactivating old observations.
  a->setEstimate(okvis::kinematics::Transformation(Eigen::Vector3d(2, -1, .3),
      Eigen::Quaterniond(Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitZ()))));
  std::vector<okvis::ceres::TwoPoseGraphError::Observation> baselineObs, pagedObs;
  std::vector<okvis::KeypointIdentifier> baselineDuplicates, pagedDuplicates;
  baseline->convertToReprojectionErrors(baselineObs, baselineDuplicates);
  paged->convertToReprojectionErrors(pagedObs, pagedDuplicates);
  check(baselineObs.size() == pagedObs.size() && baselineDuplicates == pagedDuplicates,
      "Archived edge reactivation lost original or duplicate observations");
  for (size_t i = 0; i < baselineObs.size(); ++i) {
    check(baselineObs[i].keypointIdentifier == pagedObs[i].keypointIdentifier &&
        (baselineObs[i].hPoint->estimate() - pagedObs[i].hPoint->estimate()).norm() < 1e-12,
        "Archived edge reactivation ignored corrected anchor pose");
  }
  std::vector<okvis::ceres::TwoPoseGraphError::Observation> empty;
  paged->getObservations(empty);
  check(empty.empty(), "Consumed edge reloaded stale archived observations");
}

}
int main() {
  const auto directory = std::filesystem::temp_directory_path() /
      ("okvis_frame_archive_test_" + std::to_string(::getpid()));
  try {
    std::filesystem::create_directories(directory / "frames");
    testPayload(directory); testPayloadIntegrity(directory);
    testBackendCache(directory / "frames"); testInMemoryHistory(directory / "ram-only");
    testFactorLifetime();
    testTwoPoseArchive(directory, false); testTwoPoseArchive(directory, true);
    std::filesystem::remove_all(directory);
    std::cout << "RAM-only cold image release, feature overlap, no disk output, frame archive roundtrip, write/load failures, bounded cold cache, borrowed candidate pin, factor lifecycle and standard/calibrated visual edge reactivation passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::filesystem::remove_all(directory);
    std::cerr << error.what() << '\n';
    return 1;
  }
}
