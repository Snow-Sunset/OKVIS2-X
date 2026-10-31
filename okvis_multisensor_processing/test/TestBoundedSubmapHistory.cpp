// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/SubmappingInterface.hpp>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace {
void check(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

okvis::SupereightFrames packet(uint64_t id, double anchorX) {
  okvis::SupereightFrames frame(id);
  const okvis::kinematics::Transformation anchor(
      Eigen::Vector3d(anchorX, 0, 0), Eigen::Quaterniond::Identity());
  frame.frameData_.emplace(id, okvis::SiFrameData(id));
  frame.frameData_.at(id).T_WS = anchor;
  const Eigen::Isometry3f lidarPose(anchor.T().cast<float>());
  // 512 independent, non-collinear planar cells. Completing each cell releases
  // its support samples; the fitted historical parameters still exceed the
  // deliberately small cache after enough submaps have been completed.
  for (int x = 0; x < 32; ++x) for (int y = 0; y < 16; ++y) {
    for (int sample = 0; sample < 8; ++sample) {
      frame.vecRayMeasurements.emplace_back(lidarPose, Eigen::Vector3f(
          x + 0.1f + 0.12f * (sample % 4), y + 0.1f + 0.2f * (sample / 4), 2.2f));
    }
  }
  return frame;
}

void run(const std::filesystem::path& directory) {
  okvis::ViParameters parameters{};
  parameters.lidar.emplace();
  parameters.lidar->T_SL = okvis::kinematics::Transformation::Identity();
  parameters.lidar->elevation_resolution_angle = 1.0f;
  parameters.lidar->azimuth_resolution_angle = 1.0f;
  se::SubMapConfig config;
  config.memoryBounded = true;
  config.memoryArchiveDirectory = directory.string();
  config.lidarPlaneOnly = true;
  config.usePointToPlaneFactors = true;
  config.useMap2LiveFactors = true;
  config.useMap2MapFactors = false;
  config.planeCacheMemoryMb = 1;
  config.pointCloudMemoryMb = 2;
  config.pointCloudPersistentChunks = true;
  config.planeActiveMaxCells = 512;
  config.planeWindowHalfExtent = 50.0;
  config.planeMapConfig.minPoints = 6;
  config.planeMapConfig.maxPoints = 8;
  config.voxelGridResolution = 0.05f;
  config.pointCloudMapVoxelSize = 0.05;
  config.submapMinFrames = 0;
  config.submapKfThreshold = 1;
  config.near_plane = 0.1f;
  config.far_plane = 100.0f;
  okvis::SupereightMapType::Config mapConfig;
  okvis::SupereightMapType::DataConfigType dataConfig;
  okvis::SubmappingInterface mapping(mapConfig, dataConfig, config, parameters);
  std::weak_ptr<const okvis::PlaneMap> firstReference;
  size_t oneMapBytes = 0;
  size_t announcements = 0;
  constexpr uint64_t firstId = 10;
  constexpr size_t numberOfMaps = 32;
  constexpr size_t pointsPerMap = 512 * 8;
  mapping.setAlignCallback([&](const okvis::SupereightMapType* a,
      const okvis::SupereightMapType* b, const uint64_t& idA, const uint64_t& idB,
      auto&, auto, okvis::PlaneMapPtr, okvis::PlaneMapPtr planeB) {
    check(!a && !b && idA == 0 && planeB && planeB->finalized(),
          "Completed plane history was not announced without occupancy storage");
    check(planeB->numPlanes() == 512, "A completed submap lost fitted plane cells");
    ++announcements;
    if (idB == firstId) {
      firstReference = planeB;
      oneMapBytes = planeB->residentBytes();
    }
    return true;
  });
  for (size_t index = 0; index < numberOfMaps; ++index) {
    auto frame = packet(firstId + index, double(index) * 100.0);
    mapping.processSupereightFrame(frame);
    check(frame.vecRayMeasurements.size() == pointsPerMap,
          "Spatial sampling unexpectedly dropped distinct test endpoints");
    check(mapping.seSubmapLookup_.size() == index + 1,
          "Active cell cap did not rotate to the next submap");
  }
  check(announcements == numberOfMaps - 1 && oneMapBytes > 0,
        "Completed submap history announcements are incomplete");
  check(oneMapBytes * (numberOfMaps - 1) > config.planeCacheMemoryMb * 1024 * 1024,
        "Test history did not exceed the configured plane cache");
  check(firstReference.expired(), "The oldest plane map was not actually evicted from RAM");
  check(mapping.planeResidentBytes() <= config.planeCacheMemoryMb * 1024 * 1024 + oneMapBytes,
        "Plane history cache exceeded its budget plus the active submap");
  const auto session = std::filesystem::path(mapping.memoryArchiveDirectory());
  check(std::filesystem::exists(session / "planes" / (std::to_string(firstId) + ".planes")),
        "The evicted plane map has no persistent archive");

  // Simulate the corrected anchor of a revisit. Public association must reload
  // the old, demonstrably evicted map and transform live points into its frame.
  uint64_t oldId = firstId;
  uint64_t liveId = firstId + numberOfMaps - 1;
  mapping.seSubmapLookup_.at(liveId).T_WK = Eigen::Isometry3f::Identity();
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> candidates, observed;
  for (int x = 0; x < 32; ++x) for (int y = 0; y < 16; ++y)
    candidates.emplace_back(x + 0.28f, y + 0.2f, 2.22f);
  check(mapping.determineObservedPoints(oldId, liveId, candidates, observed) == candidates.size()
        && observed.size() == candidates.size(), "Cold plane reload failed to restore point-to-plane association");
  check(mapping.findMostOverlappingSubmap(candidates, liveId, liveId) == firstId,
        "An archived historical submap was not selected after the anchor correction");

  // Do not invoke saveLidarPointCloudMap: shutdown itself must preserve the last
  // active cloud tail and plane parameters when final PCD export is disabled.
  const size_t beforeFlush = mapping.cloudResidentBytes();
  check(beforeFlush > 100000, "Test has no resident cloud tail to flush at shutdown");
  mapping.setFinished();
  check(mapping.cloudResidentBytes() < beforeFlush,
        "Shutdown without PCD export retained an unarchived cloud tail");
  check(std::filesystem::exists(session / "planes" / (std::to_string(liveId) + ".planes")),
        "Shutdown without PCD export lost the last active plane map");
  size_t manifests = 0;
  uint64_t archivedPoints = 0;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(session / "cloud")) {
    if (entry.path().filename() != "manifest.txt") continue;
    ++manifests;
    std::ifstream input(entry.path());
    std::string line;
    while (std::getline(input, line)) {
      std::istringstream fields(line);
      std::string type, chunkFile;
      uint64_t id = 0, count = 0;
      if ((fields >> type) && type == "chunk") {
        check(bool(fields >> id >> chunkFile >> count), "Incomplete final cloud manifest");
        check(std::filesystem::exists(entry.path().parent_path() / chunkFile),
              "Final cloud manifest references a missing chunk");
        archivedPoints += count;
      }
    }
  }
  check(manifests == 1 && archivedPoints == numberOfMaps * pointsPerMap,
        "Shutdown without PCD export did not persist the complete sampled point map");
}

void runDelayedInput(const std::filesystem::path& directory) {
  okvis::ViParameters parameters{};
  parameters.lidar.emplace();
  parameters.lidar->T_SL = okvis::kinematics::Transformation::Identity();
  parameters.lidar->elevation_resolution_angle = 1.0f;
  parameters.lidar->azimuth_resolution_angle = 1.0f;
  se::SubMapConfig config;
  config.memoryBounded = true;
  config.memoryArchiveDirectory = (directory / "delayed").string();
  config.lidarSpoolEnabled = true;
  config.useMap2LiveFactors = true;
  config.useMap2MapFactors = false;
  config.pointCloudMemoryMb = 2;
  config.planeCacheMemoryMb = 1;
  config.lidarQueueMaxPoints = 64;
  config.voxelGridResolution = 0.005f;
  config.pointCloudMapVoxelSize = 0.005;
  config.sensorMeasurementDownsampling = 1;
  config.near_plane = 0.1f;
  config.far_plane = 20.0f;
  okvis::SupereightMapType::Config mapConfig;
  okvis::SupereightMapType::DataConfigType dataConfig;
  std::mutex inputMutex;
  std::condition_variable inputCv;
  bool releaseInput = false;
  std::atomic<size_t> processedStates{0};
  okvis::SubmappingInterface mapping(mapConfig, dataConfig, config, parameters);
  mapping.setT_BS(okvis::kinematics::Transformation::Identity());
  // Hold the worker after both trajectory updates so the test deterministically
  // creates a disk backlog larger than one final-drain batch.
  struct ReleaseWorker {
    std::mutex& mutex;
    std::condition_variable& cv;
    bool& release;
    ~ReleaseWorker() {
      { std::lock_guard lock(mutex); release = true; }
      cv.notify_all();
    }
  } releaseWorker{inputMutex, inputCv, releaseInput};
  mapping.setAnchoredTrajectoryUpdateCallback([&](const auto&, auto) {
    if (++processedStates == 2) {
      std::unique_lock lock(inputMutex);
      inputCv.wait(lock, [&] { return releaseInput; });
    }
  });
  for (uint64_t id = 1; id <= 2; ++id) {
    okvis::State state{};
    state.id = okvis::StateId(id);
    state.timestamp = okvis::Time(uint32_t(id), 0);
    state.T_WS = okvis::kinematics::Transformation::Identity();
    state.T_GW = okvis::kinematics::Transformation::Identity();
    state.v_W.setZero();
    state.b_g.setZero();
    state.b_a.setZero();
    state.omega_S.setZero();
    state.isKeyframe = true;
    state.stateChanged = true;
    okvis::TrackingState tracking{};
    tracking.id = state.id;
    tracking.currentKeyframeId = state.id;
    tracking.isKeyframe = true;
    tracking.trackingQuality = okvis::TrackingQuality::Good;
    const auto states = std::make_shared<okvis::AlignedMap<okvis::StateId, okvis::State>>();
    states->emplace(state.id, state);
    check(mapping.stateUpdateCallback(state, tracking, states), "Delayed-input state update was rejected");
  }
  mapping.start();
  const auto stateDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (processedStates.load() < 2 && std::chrono::steady_clock::now() < stateDeadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  check(processedStates.load() == 2, "Input worker did not consume both pose updates");
  // No additional state updates arrive. The delayed cloud must bootstrap the
  // first map from trajectory history and drain many bounded read-cache batches.
  constexpr size_t numberOfRays = 10000;
  okvis::LidarMeasurementBatch delayed;
  delayed.reserve(numberOfRays);
  for (size_t i = 0; i < numberOfRays; ++i) {
    okvis::LidarMeasurement measurement;
    measurement.timeStamp = okvis::Time(1, 500000000);
    measurement.measurement.rayMeasurement = Eigen::Vector3d(
        0.1 + 0.02 * double(i % 100), 0.1 + 0.02 * double(i / 100), 2.2);
    measurement.measurement.intensity = 0;
    delayed.push_back(measurement);
  }
  check(mapping.addLidarMeasurements(delayed), "Delayed LiDAR batch was rejected");
  const Eigen::Vector3d futureRay(9, 0, 2);
  check(mapping.addLidarMeasurement(okvis::Time(3, 0), futureRay), "Future raw ray was rejected");
  { std::lock_guard lock(inputMutex); releaseInput = true; }
  inputCv.notify_all();
  bool finished = false;
  const auto finishDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < finishDeadline) {
    if (mapping.finishedIntegrating()) { finished = true; break; }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  check(finished, "Delayed LiDAR backlog prevented normal completion");
  mapping.setFinished(); // Final PCD export remains disabled.
  const auto cloud = mapping.lidarPointCloudMap();
  check(cloud.size() == numberOfRays, "Normal shutdown lost posed delayed rays or inserted the unposed future tail");
  for (const auto& point : cloud) check(point.x() < 3.0, "Future ray entered the map without a valid pose");
  const auto spool = std::filesystem::path(mapping.memoryArchiveDirectory()) / "lidar-spool.bin";
  // The internal spool format is fixed-width timestamp + double XYZ + metadata.
  // Check that processed records and the unposed final record remain durable.
  constexpr size_t recordBytes = 2 * sizeof(uint32_t) + 3 * sizeof(double) + sizeof(int32_t) + sizeof(uint32_t);
  check(std::filesystem::file_size(spool) == (numberOfRays + 1) * recordBytes,
        "Normal shutdown did not retain all raw input spool records");
  std::ifstream input(spool, std::ios::binary);
  input.seekg(-std::streamoff(recordBytes), std::ios::end);
  uint32_t sec = 0, nsec = 0;
  double xyz[3];
  input.read(reinterpret_cast<char*>(&sec), sizeof(sec));
  input.read(reinterpret_cast<char*>(&nsec), sizeof(nsec));
  input.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
  check(bool(input) && sec == 3 && nsec == 0
      && (Eigen::Vector3d(xyz[0], xyz[1], xyz[2]) - futureRay).norm() == 0.0,
      "The unposed final ray was not preserved in the input archive");
}

void runExpiredInput(const std::filesystem::path& directory) {
  okvis::ViParameters parameters{};
  parameters.lidar.emplace();
  parameters.lidar->T_SL = okvis::kinematics::Transformation::Identity();
  parameters.lidar->elevation_resolution_angle = 1.0f;
  parameters.lidar->azimuth_resolution_angle = 1.0f;
  se::SubMapConfig config;
  config.memoryBounded = true;
  config.memoryArchiveDirectory = (directory / "expired").string();
  config.lidarSpoolEnabled = false;
  config.useMap2LiveFactors = true;
  config.useMap2MapFactors = false;
  config.lidarQueueMaxPoints = 256;
  config.sensorMeasurementDownsampling = 1;
  config.voxelGridResolution = 0.01f;
  config.pointCloudMapVoxelSize = 0.01;
  config.near_plane = 0.1f;
  config.far_plane = 20.0f;
  okvis::SupereightMapType::Config mapConfig;
  okvis::SupereightMapType::DataConfigType dataConfig;
  okvis::SubmappingInterface mapping(mapConfig, dataConfig, config, parameters);
  mapping.setT_BS(okvis::kinematics::Transformation::Identity());
  // Populate and actually prune the rolling IMU window before pose updates are
  // consumed. The interval beginning at state t=1 is now permanently expired.
  for (uint32_t tick = 10; tick <= 110; ++tick) {
    mapping.realtimePredict(okvis::Time(tick / 10, (tick % 10) * 100000000),
                            Eigen::Vector3d(0, 0, 9.81), Eigen::Vector3d::Zero());
  }
  const uint32_t times[] = {1, 2, 10, 11};
  for (uint64_t index = 0; index < 4; ++index) {
    okvis::State state{};
    state.id = okvis::StateId(index + 1);
    state.timestamp = okvis::Time(times[index], 0);
    state.T_WS = okvis::kinematics::Transformation::Identity();
    state.T_GW = okvis::kinematics::Transformation::Identity();
    state.v_W.setZero();
    state.b_g.setZero();
    state.b_a.setZero();
    state.omega_S.setZero();
    state.isKeyframe = true;
    state.stateChanged = true;
    okvis::TrackingState tracking{};
    tracking.id = state.id;
    tracking.currentKeyframeId = state.id;
    tracking.isKeyframe = true;
    tracking.trackingQuality = okvis::TrackingQuality::Good;
    auto states = std::make_shared<okvis::AlignedMap<okvis::StateId, okvis::State>>();
    states->emplace(state.id, state);
    check(mapping.stateUpdateCallback(state, tracking, states), "Expired-input state update was rejected");
  }
  okvis::LidarMeasurementBatch input;
  for (size_t i = 0; i < 128; ++i) {
    okvis::LidarMeasurement measurement;
    measurement.timeStamp = okvis::Time(1, 500000000);
    measurement.measurement.rayMeasurement = Eigen::Vector3d(8, 0, 2);
    measurement.measurement.intensity = 0;
    input.push_back(measurement);
  }
  okvis::LidarMeasurement current;
  current.timeStamp = okvis::Time(10, 0); // exact estimated state needs no old IMU samples
  current.measurement.rayMeasurement = Eigen::Vector3d(1, 0, 2);
  current.measurement.intensity = 0;
  input.push_back(current);
  current.timeStamp = okvis::Time(12, 0); // future input must remain unposed
  current.measurement.rayMeasurement = Eigen::Vector3d(9, 0, 2);
  input.push_back(current);
  check(mapping.addLidarMeasurements(input), "Expired/current/future input batch was rejected");
  mapping.start();
  bool finished = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    if (mapping.finishedIntegrating()) { finished = true; break; }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  check(finished, "Expired IMU prefix blocked current LiDAR input or final shutdown");
  mapping.setFinished();
  const auto cloud = mapping.lidarPointCloudMap();
  check(cloud.size() == 1 && (cloud.front() - Eigen::Vector3f(1, 0, 2)).norm() < 1e-5f,
        "Expired/future input was inserted, or the supported current point was lost");
}
} // namespace

int main() {
  const auto directory = std::filesystem::temp_directory_path() / ("okvis-history-test-" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    std::filesystem::create_directory(directory);
    run(directory);
    runDelayedInput(directory);
    runExpiredInput(directory);
    std::filesystem::remove_all(directory);
    std::cout << "Plane cache eviction, cold revisit matching and final history flush checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::filesystem::remove_all(directory);
    std::cerr << error.what() << '\n';
    return 1;
  }
}
