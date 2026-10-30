// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/SubmappingInterface.hpp>
#include <cmath>
#include <future>
#include <condition_variable>
#include <okvis/LidarMotionUndistortion.hpp>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

okvis::SupereightFrames packet(uint64_t id, float offset = 0.0f) {
  okvis::SupereightFrames frame(id);
  frame.frameData_.emplace(id, okvis::SiFrameData(id));
  for (int x = 0; x < 4; ++x) {
    for (int y = 0; y < 4; ++y) {
      frame.vecRayMeasurements.emplace_back(Eigen::Isometry3f::Identity(),
          Eigen::Vector3f(offset + 0.1f + 0.2f * x, 0.1f + 0.2f * y, 2.2f));
    }
  }
  return frame;
}

void run(bool mapToMap) {
  okvis::ViParameters parameters{};
  parameters.lidar.emplace();
  parameters.lidar->T_SL = okvis::kinematics::Transformation::Identity();
  parameters.lidar->elevation_resolution_angle = 1.0f;
  parameters.lidar->azimuth_resolution_angle = 1.0f;
  se::SubMapConfig config;
  config.lidarPlaneOnly = true;
  config.pointCloudMapPublishInterval = 0.0;
  config.usePointToPlaneFactors = true;
  // Even an accidental mesh request must not cause extraction or file writes.
  config.write_mesh_output = true;
  config.useMap2LiveFactors = true;
  config.useMap2MapFactors = mapToMap;
  config.submapMinFrames = 0;
  config.submapKfThreshold = 1;
  config.numSubmapFactors = 8;
  config.far_plane = 30.0f;
  config.near_plane = 0.1f;
  okvis::SupereightMapType::Config mapConfig;
  okvis::SupereightMapType::DataConfigType dataConfig;
  okvis::SubmappingInterface mapping(mapConfig, dataConfig, config, parameters);
  check(mapping.planeOnlyMapping() && !mapping.meshEnabled(), "Plane-only mode not selected");
  check(!mapping.objectMap_, "Object/mesh map allocated in plane-only mode");
  int meshCallbacks = 0, fieldCallbacks = 0, announcements = 0, constraints = 0;
  int cloudCallbacks = 0;
  okvis::LidarPointCloudMap::Points publishedCloud;
  mapping.setPointCloudMapCallback([&](const auto&, const auto& cloud) {
    ++cloudCallbacks;
    publishedCloud = cloud;
  });
  okvis::PlaneMapPtr completed;
  mapping.setSubmapCallback([&](auto, auto, auto) { ++meshCallbacks; });
  mapping.setFieldSliceCallback([&](auto&, auto&) { ++fieldCallbacks; });
  mapping.setAlignCallback([&](const okvis::SupereightMapType* a,
      const okvis::SupereightMapType* b, const uint64_t& idA, const uint64_t& idB,
      auto& points, auto errors, okvis::PlaneMapPtr planeA, okvis::PlaneMapPtr planeB) {
    check(!a && !b, "Occupancy map allocated/published in plane-only mode");
    check(bool(planeB) && planeB->numPlanes() > 0, "Completed plane map not handed off");
    if (idA == 0) {
      ++announcements;
      if (!completed) {
        check(idB == 10, "Wrong initial reference map announced");
        completed = planeB;
      }
    } else {
      if (!points.empty()) ++constraints;
      check(planeA && points.size() == errors.size(), "Invalid map-to-map constraint");
    }
    return true;
  });
  auto first = packet(10);
  first.vecRayMeasurements.emplace_back(Eigen::Isometry3f::Identity(),
      Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN()));
  first.vecRayMeasurements.emplace_back(Eigen::Isometry3f::Identity(), Eigen::Vector3f(100, 0, 0));
  first.vecRayMeasurements.emplace_back(Eigen::Isometry3f::Identity(), Eigen::Vector3f(0, 0, 0));
  mapping.processSupereightFrame(first);
  check(first.vecRayMeasurements.size() == 16, "Invalid rays were retained");
  check(mapping.lidarPointCloudMap().size() == 16 && publishedCloud.size() == 16,
        "First scan was not added/published as a point cloud map");
  check(mapping.seSubmapLookup_.size() == 1 && !mapping.seSubmapLookup_.at(10).map,
        "First submap allocated occupancy storage");
  check(mapping.finishedIntegrating() && mapping.seSubmapLookup_.size() == 1,
        "Single-submap shutdown created an invalid reference");
  auto second = packet(20);
  mapping.processSupereightFrame(second);
  check(announcements == 1 && completed, "No plane map reference announced");
  check(mapping.seSubmapLookup_.size() == 2, "Bootstrap submap transition failed");
  const auto savedBounds = completed->bounds();
  check(mapping.lidarPointCloudMap().size() == 16, "Overlapping scans duplicated the cloud map");
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points;
  for (const auto& ray : second.vecRayMeasurements) points.push_back(ray.second);
  check(mapping.findMostOverlappingSubmap(points, 20, 20) == 10,
        "Plane bounds did not select overlapping submap");
  mapping.seSubmapLookup_.at(20).T_WK.translation() = Eigen::Vector3f(20, 0, 0);
  check(mapping.findMostOverlappingSubmap(points, 20, 20) == std::numeric_limits<uint64_t>::max(),
        "Submap bounds ignored anchor pose correction");
  mapping.seSubmapLookup_.at(20).T_WK = Eigen::Isometry3f::Identity();
  uint64_t id = 10;
  auto overlapping = packet(30);
  check(std::abs(mapping.evaluateLidarOverlap(id, overlapping) - 1.0) < 1e-12,
        "Plane overlap evaluation failed");
  auto invalid = packet(30, 100.0f);
  check(std::isfinite(mapping.evaluateLidarOverlap(id, invalid)), "Out-of-range overlap returned NaN");
  // Two successive low-overlap batches must create a new submap without any
  // occupancy field, preserving map-to-map handoff when requested.
  auto third = packet(30, 3.0f);
  mapping.processSupereightFrame(third);
  auto fourth = packet(40, 6.0f);
  mapping.processSupereightFrame(fourth);
  check(mapping.seSubmapLookup_.size() == 3, "Low-overlap transition failed");
  check(mapToMap ? constraints > 0 : constraints == 0, "Map-to-map enable flag ignored");
  check((completed->bounds().min() - savedBounds.min()).norm() == 0.0
        && (completed->bounds().max() - savedBounds.max()).norm() == 0.0,
        "Completed reference plane map was mutated");
  for (const auto& submap : mapping.seSubmapLookup_) {
    check(!submap.second.map, "Occupancy storage was allocated");
  }
  check(mapping.finishedIntegrating(), "Final mapping queue did not drain");
  check(cloudCallbacks > 0 && publishedCloud.size() == 48,
        "Unmatched points did not enter the accumulated cloud map");
  auto updated = std::make_shared<okvis::AlignedMap<okvis::StateId, okvis::State>>();
  okvis::State corrected{};
  corrected.id = okvis::StateId(10);
  corrected.isKeyframe = true;
  corrected.T_WS = okvis::kinematics::Transformation(Eigen::Vector3d(20, 0, 0), Eigen::Quaterniond::Identity());
  updated->emplace(corrected.id, corrected);
  okvis::TrackingState tracking{};
  mapping.setT_BS(okvis::kinematics::Transformation::Identity());
  mapping.stateUpdateCallback(corrected, tracking, updated);
  const auto correctedCloud = mapping.lidarPointCloudMap();
  check(correctedCloud.size() == 64, "Pose-only correction did not relocate the point cloud");
  okvis::SupereightFrames stale(40);
  stale.frameData_.emplace(10, okvis::SiFrameData(10));
  mapping.processSupereightFrame(stale);
  check(mapping.lidarPointCloudMap().size() == 64, "Queued old pose undid estimator correction");
  const auto directory = std::filesystem::temp_directory_path()
      / ("okvis_lidar_cloud_test_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  const auto filename = directory / "map.pcd";
  check(mapping.saveLidarPointCloudMap(filename.string()), "Point cloud export failed");
  pcl::PointCloud<pcl::PointXYZ> loaded;
  check(pcl::io::loadPCDFile(filename.string(), loaded) == 0 && loaded.size() == 64,
        "Saved PCD does not match map snapshot");
  bool movedPoint = false;
  for (const auto& point : loaded) movedPoint |= point.x > 20.0f;
  check(movedPoint, "PCD did not use corrected anchor poses");
  std::filesystem::remove_all(directory);
  mapping.saveSubmap(999); // Must return before any lookup or filesystem work.
  mapping.saveAllSubmapMeshes();
  mapping.setFinished();
  check(mapping.seMeshLookup_.empty(), "Mesh extracted at submap transition/shutdown");
  check(meshCallbacks == 0 && fieldCallbacks == 0, "Mesh/occupancy visualization callback invoked");
  okvis::SubmappingInterface empty(mapConfig, dataConfig, config, parameters);
  check(empty.finishedIntegrating(), "No-input shutdown would wait forever");
  empty.setFinished(); // No sensor packets is also a supported shutdown path.
  check(!empty.saveLidarPointCloudMap(filename.string()), "Empty map wrote a PCD file");
  config.buildLidarPointCloudMap = false;
  okvis::SubmappingInterface disabled(mapConfig, dataConfig, config, parameters);
  int disabledCallbacks = 0;
  disabled.setPointCloudMapCallback([&](const auto&, const auto&) { ++disabledCallbacks; });
  auto disabledPacket = packet(10);
  disabled.processSupereightFrame(disabledPacket);
  disabled.setFinished();
  check(disabled.lidarPointCloudMap().empty() && disabledCallbacks == 0,
        "Disabled accumulated map stored or published points");
  config.buildLidarPointCloudMap = true;
  config.pointCloudMapPublishInterval = 1e12;
  okvis::SubmappingInterface throttled(mapConfig, dataConfig, config, parameters);
  int throttledCallbacks = 0;
  size_t throttledPoints = 0;
  throttled.setPointCloudMapCallback([&](const auto&, const auto& cloud) {
    ++throttledCallbacks;
    throttledPoints = cloud.size();
  });
  auto firstThrottled = packet(10);
  throttled.processSupereightFrame(firstThrottled);
  auto nextThrottled = packet(10, 3.0f);
  throttled.processSupereightFrame(nextThrottled);
  check(throttledCallbacks == 1 && throttledPoints == 16,
        "First publication or interval throttling failed");
  throttled.setFinished();
  check(throttledCallbacks == 2 && throttledPoints == 32,
        "Final snapshot did not bypass interval throttling");
}
void runAsyncPublication() {
  okvis::ViParameters parameters{};
  parameters.lidar.emplace();
  parameters.lidar->T_SL = okvis::kinematics::Transformation::Identity();
  parameters.lidar->elevation_resolution_angle = 1.0f;
  parameters.lidar->azimuth_resolution_angle = 1.0f;
  se::SubMapConfig config;
  config.useMap2LiveFactors = true;
  config.pointCloudMapPublishInterval = 0.0;
  config.far_plane = 30.0f;
  config.near_plane = 0.1f;
  okvis::SupereightMapType::Config mapConfig;
  okvis::SupereightMapType::DataConfigType dataConfig;
  okvis::SubmappingInterface mapping(mapConfig, dataConfig, config, parameters);
  auto frame = packet(10);
  mapping.processSupereightFrame(frame);
  std::mutex mutex;
  std::condition_variable cv;
  bool entered = false, release = false;
  mapping.setPointCloudMapCallback([&](const auto&, const auto&) {
    std::unique_lock lock(mutex);
    entered = true;
    cv.notify_all();
    cv.wait(lock, [&] { return release; });
  });
  mapping.start();
  auto request = std::async(std::launch::async, [&] { mapping.publishLidarPointCloudMap(); });
  bool callbackEntered;
  {
    std::unique_lock lock(mutex);
    callbackEntered = cv.wait_for(lock, std::chrono::seconds(2), [&] { return entered; });
  }
  const bool requestReturned = request.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
  auto states = std::make_shared<okvis::AlignedMap<okvis::StateId, okvis::State>>();
  okvis::State state{};
  state.id = okvis::StateId(10);
  state.isKeyframe = true;
  state.T_WS = okvis::kinematics::Transformation::Identity();
  states->emplace(state.id, state);
  okvis::TrackingState tracking{};
  auto update = std::async(std::launch::async, [&] { mapping.stateUpdateCallback(state, tracking, states); });
  const bool updateReturned = update.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
  {
    std::lock_guard lock(mutex);
    release = true;
  }
  cv.notify_all();
  request.get();
  update.get();
  mapping.setFinished();
  {
    auto directDestruction = std::make_unique<okvis::SubmappingInterface>(mapConfig, dataConfig, config, parameters);
    directDestruction->start();
  }
  check(callbackEntered && requestReturned && updateReturned,
        "Slow map visualization blocked a live publication request or estimator state callback");
}

void runCandidateSampling() {
  okvis::State state{};
  const auto identity = okvis::kinematics::Transformation::Identity();
  okvis::LidarMotionUndistortion scan(state, identity, identity, {}, {});
  auto& points = scan.deskewedDownsampledPointCloud();
  points = {{0.01f, 0.01f, 0.2f}, {0.02f, 0.01f, 0.2f}, {-0.01f, 0.01f, 0.2f},
            Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN())};
  scan.voxelDownsample(0.1);
  check(points.size() == 2, "Candidate voxel sampling lost negative cells or retained duplicate/invalid points");
  bool invalid = false;
  try { scan.voxelDownsample(0.0); } catch (const std::invalid_argument&) { invalid = true; }
  check(invalid, "Invalid candidate sampling resolution accepted");
  points.clear();
  check(!scan.voxelDownsample(0.1), "Empty candidate cloud sampling reported success");
}

} // namespace

int main() {
  try {
    se::SubMapConfig config;
    check(config.lidarPlaneOnly, "LiDAR plane-only default disabled");
    config.usePointToPlaneFactors = false;
    bool rejected = false;
    try { config.validateLidarMapping(); } catch (const std::invalid_argument&) { rejected = true; }
    check(rejected, "Occupancy residual permitted in plane-only mode");
    run(true);
    run(false);
    runAsyncPublication();
    runCandidateSampling();
    std::cout << "Plane-only allocation, overlap, handoff, output and shutdown checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
