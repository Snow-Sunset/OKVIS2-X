// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/SubmappingInterface.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <initializer_list>
#include <set>
#include <stdexcept>

namespace {
void check(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
constexpr size_t kPointsPerPacket = 64 * 64;
constexpr double kVoxelSize = 0.1;

okvis::ViParameters parameters() {
  okvis::ViParameters result{};
  result.lidar.emplace();
  result.lidar->T_SL = okvis::kinematics::Transformation::Identity();
  result.lidar->elevation_resolution_angle = 1.0f;
  result.lidar->azimuth_resolution_angle = 1.0f;
  return result;
}

se::SubMapConfig configuration(const std::filesystem::path& directory, bool global) {
  se::SubMapConfig config;
  config.pointCloudMapGlobal = global;
  config.memoryArchiveDirectory = (directory / "unexpected-history").string();
  config.maxSubmaps = 1;
  config.planeActiveMaxCells = 16;
  config.pointCloudWindowHalfExtent = 50.0;
  config.pointCloudMapVoxelSize = kVoxelSize;
  config.pointCloudMapPublishInterval = 0.0;
  config.pointCloudPersistentChunks = false;
  config.lidarSpoolEnabled = false;
  config.lidarPlaneOnly = true;
  config.usePointToPlaneFactors = true;
  config.useMap2LiveFactors = true;
  config.useMap2MapFactors = false;
  config.write_mesh_output = false;
  config.voxelGridResolution = 0.05f;
  config.submapMinFrames = 0;
  config.submapKfThreshold = 1;
  config.near_plane = 0.1f;
  config.far_plane = 30.0f;
  return config;
}

okvis::SupereightFrames packet(uint64_t id, double anchorX) {
  okvis::SupereightFrames frame(id);
  const okvis::kinematics::Transformation anchor(
      Eigen::Vector3d(anchorX, 0, 0), Eigen::Quaterniond::Identity());
  frame.frameData_.emplace(id, okvis::SiFrameData(id));
  frame.frameData_.at(id).T_WS = anchor;
  const Eigen::Isometry3f sensorPose(anchor.T().cast<float>());
  for (int x = 0; x < 64; ++x) for (int y = 0; y < 64; ++y)
    frame.vecRayMeasurements.emplace_back(sensorPose,
        Eigen::Vector3f(0.03f + 0.2f * x, 0.03f + 0.2f * y, 2.23f));
  return frame;
}

using VoxelKeys = std::set<std::array<int64_t, 3>>;
template<class Points> VoxelKeys keys(const Points& points) {
  VoxelKeys result;
  for (const auto& point : points) {
    result.insert({int64_t(std::floor(double(point.x()) / kVoxelSize)),
                   int64_t(std::floor(double(point.y()) / kVoxelSize)),
                   int64_t(std::floor(double(point.z()) / kVoxelSize))});
  }
  return result;
}

VoxelKeys keys(const pcl::PointCloud<pcl::PointXYZ>& points) {
  VoxelKeys result;
  for (const auto& point : points)
    result.insert({int64_t(std::floor(double(point.x) / kVoxelSize)),
                   int64_t(std::floor(double(point.y) / kVoxelSize)),
                   int64_t(std::floor(double(point.z) / kVoxelSize))});
  return result;
}

void correctPoses(okvis::SubmappingInterface& mapping,
                  const std::initializer_list<uint64_t>& ids, double x) {
  auto states = std::make_shared<okvis::AlignedMap<okvis::StateId, okvis::State>>();
  for (const uint64_t id : ids) {
    okvis::State state{};
    state.id = okvis::StateId(id);
    state.isKeyframe = true;
    state.T_WS = okvis::kinematics::Transformation(
        Eigen::Vector3d(x, 0, 0), Eigen::Quaterniond::Identity());
    states->emplace(state.id, state);
  }
  const okvis::TrackingState tracking{};
  mapping.stateUpdateCallback(states->rbegin()->second, tracking, states);
}

void globalChecks(const std::filesystem::path& directory) {
  const auto params = parameters();
  const auto config = configuration(directory, true);
  okvis::SupereightMapType::Config mapConfig;
  okvis::SupereightMapType::DataConfigType dataConfig;
  okvis::SubmappingInterface mapping(mapConfig, dataConfig, config, params);
  mapping.setT_BS(okvis::kinematics::Transformation::Identity());
  okvis::LidarPointCloudMap::Points published;
  size_t publications = 0;
  mapping.setPointCloudMapCallback([&](const auto&, const auto& cloud) {
    ++publications;
    published = cloud;
  });
  for (size_t index = 0; index < 3; ++index) {
    auto input = packet(10 + index, 200.0 * index);
    mapping.processSupereightFrame(input);
    check(input.vecRayMeasurements.size() == kPointsPerPacket,
          "Distinct global test endpoints were lost before insertion");
    check(published.size() == (index + 1) * kPointsPerPacket,
          "Global visualization lost history outside its old local window/budget");
  }
  check(mapping.cloudResidentSubmapCount() > config.maxSubmaps,
        "Global fixture did not exceed the local submap count limit");
  check(mapping.seSubmapLookup_.size() == 3, "Global fixture did not create separate anchors");
  auto revisit = packet(13, 0.0);
  mapping.processSupereightFrame(revisit);
  check(published.size() == 3 * kPointsPerPacket && keys(published).size() == published.size(),
        "Revisiting world voxels duplicated global visualization points");

  // Move both contributors to the revisited region. Rebuilding global world
  // voxels must remove their former positions and merge their corrected ones.
  correctPoses(mapping, {10, 13}, 70.0);
  mapping.publishLidarPointCloudMap(true);
  check(published.size() == 3 * kPointsPerPacket, "Loop correction lost globally retained points");
  bool correctedRegion = false;
  for (const auto& point : published) {
    check(point.x() > 69.0, "Loop correction left a ghost at the old world position");
    correctedRegion = correctedRegion || (point.x() > 70.0 && point.x() < 83.0);
  }
  check(correctedRegion, "Loop correction did not publish the corrected historical region");
  const auto corrected = keys(published);
  okvis::SupereightFrames stale(13);
  stale.frameData_.emplace(10, okvis::SiFrameData(10));
  stale.frameData_.emplace(13, okvis::SiFrameData(13));
  mapping.processSupereightFrame(stale);
  mapping.publishLidarPointCloudMap(true);
  check(keys(published) == corrected, "Queued sensor poses undid the optimized global anchors");
  check(keys(mapping.lidarPointCloudMap()) == corrected,
        "Global query and RViz publication disagree");
  mapping.setFinished();
  check(publications > 0 && keys(published) == corrected,
        "Final publication lost corrected global history");
  check(mapping.memoryArchiveDirectory().empty() && std::filesystem::is_empty(directory),
        "RAM global visualization unexpectedly wrote history files/directories");

  // This single explicit export is the only authorized file creation by the
  // mapping component; normal processing and shutdown above must be RAM-only.
  const auto filename = directory / "global.pcd";
  check(mapping.saveLidarPointCloudMap(filename.string()), "Explicit global PCD export failed");
  pcl::PointCloud<pcl::PointXYZ> loaded;
  check(pcl::io::loadPCDFile(filename.string(), loaded) == 0 &&
        loaded.size() == published.size() && keys(loaded) == corrected,
        "Explicit PCD does not contain the complete corrected global voxel map");
  std::filesystem::remove(filename);
}

void localChecks(const std::filesystem::path& directory) {
  const auto params = parameters();
  const auto config = configuration(directory, false);
  okvis::SupereightMapType::Config mapConfig;
  okvis::SupereightMapType::DataConfigType dataConfig;
  okvis::SubmappingInterface mapping(mapConfig, dataConfig, config, params);
  okvis::LidarPointCloudMap::Points published;
  mapping.setPointCloudMapCallback([&](const auto&, const auto& cloud) { published = cloud; });
  auto first = packet(10, 0.0);
  mapping.processSupereightFrame(first);
  auto distant = packet(11, 200.0);
  mapping.processSupereightFrame(distant);
  check(published.size() == kPointsPerPacket, "Opt-in local map retained outside-window history");
  for (const auto& point : published)
    check(point.x() > 199.0, "Opt-in local map still displayed a distant historical region");
  check(mapping.cloudResidentSubmapCount() <= config.maxSubmaps,
        "Opt-in local point cloud exceeded its submap count limit");
  mapping.setFinished();
  check(mapping.memoryArchiveDirectory().empty() && std::filesystem::is_empty(directory),
        "Local RAM comparison unexpectedly wrote history files/directories");
}

void poseBeforeScan(const std::filesystem::path& directory) {
  const auto params = parameters();
  const auto config = configuration(directory, true);
  okvis::SupereightMapType::Config mapConfig;
  okvis::SupereightMapType::DataConfigType dataConfig;
  okvis::SubmappingInterface mapping(mapConfig, dataConfig, config, params);
  mapping.setT_BS(okvis::kinematics::Transformation::Identity());
  mapping.setPointCloudMapCallback([](const auto&, const auto&) {});
  correctPoses(mapping, {20}, 30.0);
  mapping.publishLidarPointCloudMap(true); // Apply correction before any geometry exists.
  auto first = packet(20, 0.0);
  mapping.processSupereightFrame(first);
  const auto cloud = mapping.lidarPointCloudMap();
  check(cloud.size() == kPointsPerPacket, "Pre-scan correction lost incoming geometry");
  for (const auto& point : cloud)
    check(point.x() > 29.0 && point.x() < 43.0,
          "Optimized anchor arriving before the first scan was overwritten");
  mapping.setFinished();
  check(std::filesystem::is_empty(directory), "Pre-scan global pose test wrote history files");
}
} // namespace

int main() {
  const auto previous = std::filesystem::current_path();
  const auto directory = std::filesystem::temp_directory_path() /
      ("okvis_global_voxel_test_" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    std::filesystem::create_directory(directory);
    std::filesystem::current_path(directory);
    globalChecks(directory);
    localChecks(directory);
    poseBeforeScan(directory);
    std::filesystem::current_path(previous);
    std::filesystem::remove_all(directory);
    std::cout << "Global voxel history, fusion, loop correction, RAM-only lifecycle, export and local-mode checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::filesystem::current_path(previous);
    std::filesystem::remove_all(directory);
    std::cerr << error.what() << '\n';
    return 1;
  }
}
