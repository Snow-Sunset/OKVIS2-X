// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/SubmappingInterface.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <future>
#include <condition_variable>
#include <okvis/LidarMotionUndistortion.hpp>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <set>
#include <utility>

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
  check(publishedCloud.size() == 32,
        "RViz publication merged retained points from overlapping submaps");
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
  if (cloudCallbacks == 0 || publishedCloud.size() != 64)
    throw std::runtime_error("Unfiltered accumulated publication expected 64 retained points, got " +
        std::to_string(publishedCloud.size()));
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

  // Occupied cells cover all octants, including negative voxel coordinates.
  for (int x = -4; x < 4; ++x)
    for (int y = -4; y < 4; ++y)
      for (int z = -4; z < 4; ++z)
        points.emplace_back(float(x) + 0.25f, float(y) + 0.25f, float(z) + 0.25f);
  check(scan.voxelDownsample(0.5) && points.size() == 512,
        "Spatial candidate sampling lost distinct positive/negative cells");
  const auto allCandidates = points;
  scan.selectPlaneCandidates(0, 0.5);
  check(points.size() == allCandidates.size(), "Zero plane candidate budget must be unlimited");
  for (size_t i = 0; i < points.size(); ++i)
    check((points[i] - allCandidates[i]).squaredNorm() == 0.0f,
          "Unlimited plane candidate selection changed point order");

  scan.selectPlaneCandidates(128, 0.5);
  const auto smallSelection = points;
  check(smallSelection.size() == 128, "Plane candidate budget was not enforced");
  std::array<bool, 8> octants{};
  for (const auto& point : smallSelection) {
    const size_t octant = (point.x() >= 0.0f ? 4 : 0)
        + (point.y() >= 0.0f ? 2 : 0) + (point.z() >= 0.0f ? 1 : 0);
    octants[octant] = true;
  }
  check(std::all_of(octants.begin(), octants.end(), [](bool covered) { return covered; }),
        "Bounded plane candidates failed to cover spatial octants");
  points = allCandidates;
  scan.selectPlaneCandidates(128, 0.5);
  check(points.size() == smallSelection.size(), "Repeated candidate selection changed its size");
  for (size_t i = 0; i < points.size(); ++i)
    check((points[i] - smallSelection[i]).squaredNorm() == 0.0f,
          "Plane candidate selection is not deterministic");
  points = allCandidates;
  scan.selectPlaneCandidates(256, 0.5);
  check(points.size() == 256, "Expanded plane candidate budget was not enforced");
  std::set<std::array<float, 3>> largerSelection;
  for (const auto& point : points) largerSelection.insert({point.x(), point.y(), point.z()});
  for (const auto& point : smallSelection)
    check(largerSelection.count({point.x(), point.y(), point.z()}) == 1,
          "Expanding plane candidate budget discarded an earlier candidate");

  for (const double resolution : {0.0, -0.5, std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected = false;
    try { scan.selectPlaneCandidates(128, resolution); }
    catch (const std::invalid_argument&) { rejected = true; }
    check(rejected, "Invalid plane candidate resolution accepted");
  }
  points = allCandidates;
  scan.selectPlaneCandidates(allCandidates.size() + 1, 0.5);
  check(points.size() == allCandidates.size(), "Candidate budget larger than scan changed its size");
  points.clear();
  scan.selectPlaneCandidates(128, 0.5);
  check(points.empty(), "Empty plane candidate pool did not stay empty");
}

okvis::State stationaryMotionState() {
  okvis::State state{};
  state.id = okvis::StateId(1);
  state.timestamp = okvis::Time(10, 0);
  state.T_WS = okvis::kinematics::Transformation::Identity();
  state.v_W.setZero();
  state.b_g.setZero();
  state.b_a.setZero();
  state.omega_S.setZero();
  return state;
}

okvis::LidarMeasurementDeque motionRays() {
  okvis::LidarMeasurementDeque rays;
  const std::array<uint32_t, 5> times = {0, 0, 50000000, 50000000, 100000000};
  for (size_t i = 0; i < times.size(); ++i) {
    okvis::LidarMeasurement ray;
    ray.timeStamp = okvis::Time(10, times[i]);
    ray.measurement.rayMeasurement = Eigen::Vector3d(1.0 + double(i), -0.5, 2.0);
    rays.push_back(ray);
  }
  return rays;
}

okvis::ImuMeasurementDeque stationaryMotionImu() {
  okvis::ImuMeasurementDeque imu;
  for (const uint32_t nsec : {0u, 100000000u, 200000000u}) {
    okvis::ImuMeasurement measurement;
    measurement.timeStamp = okvis::Time(10, nsec);
    measurement.measurement.gyroscopes.setZero();
    measurement.measurement.accelerometers = Eigen::Vector3d(0, 0, 9.81);
    imu.push_back(measurement);
  }
  return imu;
}

void checkStationaryDeskew(okvis::LidarMotionUndistortion& scan,
    const okvis::LidarMeasurementDeque& expectedRays, const Eigen::Vector3d& offset) {
  check(scan.deskew(), "Supported stationary LiDAR scan failed deskew");
  const auto& points = scan.deskewedDownsampledPointCloud();
  check(points.size() == expectedRays.size(), "Deskew lost or duplicated a timestamp group");
  for (size_t i = 0; i < points.size(); ++i)
    check((points[i].cast<double>() - expectedRays[i].measurement.rayMeasurement - offset).norm() < 1e-5,
          "Deskew used an incorrect pose for stationary/duplicate-timestamp points");
}

void runMotionUndistortion() {
  const auto state = stationaryMotionState();
  const auto identity = okvis::kinematics::Transformation::Identity();
  auto rays = motionRays();
  auto imu = stationaryMotionImu();
  okvis::LidarMotionUndistortion scan(state, identity, identity, rays, imu);
  checkStationaryDeskew(scan, rays, Eigen::Vector3d::Zero());
  checkStationaryDeskew(scan, rays, Eigen::Vector3d::Zero());

  // At the initial time no IMU propagation is needed, even for multiple rays.
  auto initialRays = rays;
  initialRays.resize(2);
  auto singleImu = imu;
  singleImu.resize(1);
  const okvis::kinematics::Transformation live(Eigen::Vector3d(1, 2, 3), Eigen::Quaterniond::Identity());
  const okvis::kinematics::Transformation extrinsics(Eigen::Vector3d(0.25, -0.5, 1), Eigen::Quaterniond::Identity());
  okvis::LidarMotionUndistortion initialGroup(state, live, extrinsics, initialRays, singleImu);
  checkStationaryDeskew(initialGroup, initialRays, extrinsics.r() - live.r());

  const auto expectUnsupported = [&](const okvis::LidarMeasurementDeque& inputRays,
      const okvis::ImuMeasurementDeque& inputImu) {
    okvis::LidarMotionUndistortion unsupported(state, identity, identity, inputRays, inputImu);
    unsupported.deskewedDownsampledPointCloud().push_back(Eigen::Vector3f::Ones());
    check(!unsupported.deskew() && unsupported.deskewedDownsampledPointCloud().empty(),
          "Unsupported deskew returned success or left stale/partial output");
  };
  expectUnsupported(rays, {});
  auto shortImu = imu;
  shortImu.resize(1);
  expectUnsupported(rays, shortImu);
  auto lateImu = imu;
  lateImu.pop_front();
  expectUnsupported(rays, lateImu);
  auto unorderedImu = imu;
  std::swap(unorderedImu[1], unorderedImu[2]);
  expectUnsupported(rays, unorderedImu);
  auto duplicateImu = imu;
  duplicateImu.insert(duplicateImu.begin() + 1, duplicateImu.front());
  expectUnsupported(rays, duplicateImu);
  auto unorderedRays = rays;
  std::swap(unorderedRays[2], unorderedRays[4]);
  expectUnsupported(unorderedRays, imu);

  // Borrowed input can become unavailable between calls; successful old output
  // must disappear rather than being reused as this frame's LiDAR factors.
  imu.clear();
  check(!scan.deskew() && scan.deskewedDownsampledPointCloud().empty(),
        "Failed repeated deskew retained output from an earlier successful call");

  const auto expected = motionRays();
  const auto stableImu = stationaryMotionImu();
  okvis::LidarMotionUndistortion owned(state, identity, identity, motionRays(), stationaryMotionImu());
  checkStationaryDeskew(owned, expected, Eigen::Vector3d::Zero());
  okvis::LidarMotionUndistortion ownedRays(state, identity, identity, motionRays(), stableImu);
  checkStationaryDeskew(ownedRays, expected, Eigen::Vector3d::Zero());
  okvis::LidarMotionUndistortion ownedImu(state, identity, identity, expected, stationaryMotionImu());
  checkStationaryDeskew(ownedImu, expected, Eigen::Vector3d::Zero());
  auto transferred = [&] {
    okvis::LidarMotionUndistortion original(state, identity, identity, motionRays(), stationaryMotionImu());
    auto copied = original;
    return okvis::LidarMotionUndistortion(std::move(copied));
  }();
  checkStationaryDeskew(transferred, expected, Eigen::Vector3d::Zero());
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
    runMotionUndistortion();
    std::cout << "Plane-only allocation, overlap, handoff, candidate/deskew, output and shutdown checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
