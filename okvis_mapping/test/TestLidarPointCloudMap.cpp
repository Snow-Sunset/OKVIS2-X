// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/LidarPointCloudMap.hpp>
#include <cmath>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unistd.h>

namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
bool contains(const okvis::LidarPointCloudMap::Points& points, const Eigen::Vector3d& expected) {
  for (const auto& point : points) if ((point - expected).norm() < 1e-10) return true;
  return false;
}
okvis::LidarPointCloudMap::Points readPcd(const std::string& filename) {
  std::ifstream input(filename, std::ios::binary);
  std::string line;
  size_t count = 0;
  while (std::getline(input, line)) {
    if (line.find("POINTS ") == 0) count = std::stoull(line.substr(7));
    if (line == "DATA binary") break;
  }
  okvis::LidarPointCloudMap::Points points;
  for (size_t i = 0; i < count; ++i) {
    float xyz[3];
    input.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
    check(bool(input), "Truncated binary PCD output");
    points.emplace_back(xyz[0], xyz[1], xyz[2]);
  }
  return points;
}
void archiveChecks() {
  namespace fs = std::filesystem;
  const fs::path directory = fs::temp_directory_path() / ("okvis-cloud-test-" + std::to_string(::getpid()));
  fs::remove_all(directory);
  fs::create_directories(directory);
  struct Cleanup { fs::path path; ~Cleanup() { std::error_code error; fs::remove_all(path, error); } } cleanup{directory};
  okvis::LidarPointCloudMap::ArchiveOptions options;
  options.directory = directory.string();
  options.maxResidentSubmaps = 2;
  options.chunkPointLimit = 8;
  okvis::LidarPointCloudMap map(0.1, options);
  const auto identity = Eigen::Isometry3d::Identity();
  okvis::LidarPointCloudMap::Points input;
  for (int i = 0; i < 120; ++i) input.emplace_back(i * 0.25 + 0.03, 0.03, 0.03);
  map.addPoints(1, identity, input);
  check(map.residentSubmapCount() <= options.maxResidentSubmaps, "Archived map exceeded resident submap limit");
  check(map.archivedPointCount() == input.size(), "Chunk cap did not archive a large active submap");
  const auto oldSnapshot = map.snapshot();
  check(!oldSnapshot.empty() && !oldSnapshot.front().points && oldSnapshot.front().archive,
        "Archived snapshot eagerly materialized historical points");
  check(okvis::LidarPointCloudMap::assemble(oldSnapshot, 0.1, 11).size() <= 11,
        "Archived display ignored point budget");
  check(map.pointCloud().size() == input.size(), "Archive reload lost sampled points");
  auto corrected = identity;
  corrected.translation() = Eigen::Vector3d(0, 5, 0);
  const auto revision = map.revision();
  map.updatePose(1, corrected, true, 10);
  check(map.revision() > revision, "Archive-only pose correction did not change revision");
  const auto cloud = map.pointCloud();
  check(cloud.size() == input.size(), "Pose correction changed archived point count");
  for (const auto& point : cloud) check(std::abs(point.y() - 5.03) < 1e-6, "Archive ignored corrected anchor pose");
  for (const auto& point : okvis::LidarPointCloudMap::assemble(oldSnapshot, 0.1)) {
    check(std::abs(point.y() - 0.03) < 1e-6, "Pose update mutated an earlier archive snapshot");
  }
  // Multiple submaps and multiple sort runs must still produce one representative
  // per world voxel. The workspace is intentionally tiny to exercise merging.
  auto overlapping = input;
  overlapping.front().x() = 0.049;
  map.addPoints(2, corrected, overlapping);
  map.flush();
  const std::string output = (directory / "full.pcd").string();
  check(okvis::LidarPointCloudMap::savePcd(map.snapshot(), 0.1, output, 4096), "Streaming PCD export failed");
  const auto exported = readPcd(output);
  check(exported.size() == input.size(), "External PCD merge did not deduplicate overlapping chunks");
  for (const auto& point : exported) check(std::abs(point.y() - 5.03) < 1e-5, "Export ignored anchor correction");
  bool preferredPoint = false;
  for (const auto& point : exported) if (std::abs(point.x() - 0.049) < 1e-6) preferredPoint = true;
  check(preferredPoint, "External merge did not keep the point nearest the world voxel center");
  check(fs::exists(fs::path(map.archiveDirectory()) / "manifest.txt"), "Persistent archive manifest missing");
  // A disk failure is reported and leaves the only resident copy intact.
  okvis::LidarPointCloudMap failed(0.1, options);
  failed.addPoints(1, identity, {{0.03, 0.03, 0.03}, {1.03, 0.03, 0.03}});
  const fs::path failedDirectory = failed.archiveDirectory();
  fs::rename(failedDirectory, failedDirectory.string() + ".moved");
  { std::ofstream obstruction(failedDirectory); obstruction << "not a directory"; }
  bool threw = false;
  try { failed.finalizeSubmap(1); } catch (const std::exception&) { threw = true; }
  check(threw, "Archive disk failure was silently ignored");
  check(failed.pointCloud().size() == 2 && failed.archivedPointCount() == 0,
        "Archive failure discarded the resident point data");
  okvis::LidarPointCloudMap manySubmaps(0.1, options);
  for (uint64_t id = 0; id < 40; ++id) {
    manySubmaps.addPoints(id, identity, {{double(id) * 10 + 0.03, 0.03, 0.03},
        {double(id) * 10 + 1.03, 0.03, 0.03}, {double(id) * 10 + 2.03, 0.03, 0.03}});
    const auto snapshot = manySubmaps.snapshot();
    check(manySubmaps.residentSubmapCount() <= options.maxResidentSubmaps,
          "A growing number of partial submaps exceeded the resident submap limit");
    check(okvis::LidarPointCloudMap::assemble(snapshot, 0.1).size() == (id + 1) * 3,
          "Submap eviction lost archived points");
  }
  // Unfiltered display must traverse both immutable chunks and resident tails,
  // applying the snapshot's pose without mutating older snapshots.
  okvis::LidarPointCloudMap mixed(0.1, options);
  okvis::LidarPointCloudMap::Points mixedInput(input.begin(), input.begin() + 9);
  mixed.addPoints(1, identity, mixedInput);
  const auto mixedSnapshot = mixed.snapshot();
  bool hasChunk = false, hasResident = false;
  for (const auto& entry : mixedSnapshot) {
    hasChunk = hasChunk || bool(entry.archive);
    hasResident = hasResident || bool(entry.points);
  }
  check(hasChunk && hasResident, "Mixed display fixture needs chunk and RAM payloads");
  mixed.updatePose(1, corrected, true);
  mixed.addPoints(1, identity, {input[9]});
  const auto mixedCloud = okvis::LidarPointCloudMap::assembleUnfiltered(mixed.snapshot());
  check(mixedCloud.size() == 10, "Unfiltered display lost chunk or resident tail points");
  for (const auto& point : mixedCloud)
    check(std::abs(point.y() - 5.03) < 1e-6, "Unfiltered chunk/RAM display ignored anchor pose");
  const auto oldMixedCloud = okvis::LidarPointCloudMap::assembleUnfiltered(mixedSnapshot);
  check(oldMixedCloud.size() == 9, "Unfiltered display mutated an earlier mixed snapshot");
  for (const auto& point : oldMixedCloud)
    check(std::abs(point.y() - 0.03) < 1e-6, "Old mixed display snapshot inherited a new pose");
  // Corruption is surfaced instead of quietly returning an incomplete map.
  const auto corrupt = map.snapshot();
  fs::resize_file(corrupt.front().archive->filename, 4);
  threw = false;
  try { okvis::LidarPointCloudMap::assemble(corrupt, 0.1, 1); } catch (const std::exception&) { threw = true; }
  check(threw, "Truncated archive silently lost points");
  threw = false;
  try { okvis::LidarPointCloudMap::assembleUnfiltered(corrupt); } catch (const std::exception&) { threw = true; }
  check(threw, "Unfiltered display silently accepted a truncated chunk");
  fs::remove(corrupt.front().archive->filename);
  threw = false;
  try { okvis::LidarPointCloudMap::assembleUnfiltered(corrupt); } catch (const std::exception&) { threw = true; }
  check(threw, "Unfiltered display silently accepted a missing chunk");
}
void unfilteredDisplayChecks() {
  const auto identity = Eigen::Isometry3d::Identity();
  okvis::LidarPointCloudMap map(0.1);
  const Eigen::Vector3d localPoint(0.03, 0.04, 0.05);
  map.addPoints(1, identity, {localPoint});
  map.addPoints(2, identity, {localPoint});
  const auto oldSnapshot = map.snapshot();
  const auto overlapping = okvis::LidarPointCloudMap::assembleUnfiltered(oldSnapshot);
  check(overlapping.size() == 2 && contains(overlapping, localPoint),
        "Unfiltered display deduplicated overlapping submap points");
  auto corrected = identity;
  corrected.linear() = Eigen::AngleAxisd(0.5 * std::acos(-1.0), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  corrected.translation() = Eigen::Vector3d(10, 2, -1);
  map.updatePose(2, corrected, true);
  const auto transformed = okvis::LidarPointCloudMap::assembleUnfiltered(map.snapshot());
  check(transformed.size() == 2 && contains(transformed, localPoint) &&
      contains(transformed, corrected * localPoint), "Unfiltered display ignored submap rotation/translation");
  const auto old = okvis::LidarPointCloudMap::assembleUnfiltered(oldSnapshot);
  check(old.size() == 2 && contains(old, localPoint) && !contains(old, corrected * localPoint),
        "Unfiltered old snapshot changed after pose correction");

  // More than the previous 200,000-point preview cap must reach display intact.
  auto many = std::make_shared<okvis::LidarPointCloudMap::Points>();
  constexpr size_t count = 200001;
  many->reserve(count);
  for (size_t i = 0; i < count; ++i) many->emplace_back(double(i) * 0.25, 0.03, 0.04);
  const okvis::LidarPointCloudMap::Snapshot largeSnapshot{{corrected, many, {}}};
  const auto large = okvis::LidarPointCloudMap::assembleUnfiltered(largeSnapshot);
  check(large.size() == count && (large.front() - corrected * many->front()).norm() < 1e-10 &&
      (large.back() - corrected * many->back()).norm() < 1e-10,
      "Unfiltered display truncated or strided a map above 200,000 points");
}
void submapRetentionChecks() {
  const auto identity = Eigen::Isometry3d::Identity();
  okvis::LidarPointCloudMap map(0.1, 2);
  okvis::LidarPointCloudMap::Points large;
  for (int i = 0; i < 20001; ++i) large.emplace_back(i * 0.25, 0.03, 0.03);
  map.addPoints(10, identity, large);
  const auto snapshot = map.snapshot();
  check(map.pointCloud().size() == large.size(), "Submap size caused partial point eviction");
  map.addPoints(20, identity, {{0.03, 10.03, 0.03}});
  // Retention follows the most recent point insertion, not numeric ID order
  // or background pose corrections.
  map.addPoints(10, identity, {{0.03, 20.03, 0.03}});
  auto corrected = identity;
  corrected.translation().z() = 1.0;
  map.updatePose(20, corrected, true);
  map.addPoints(30, identity, {{0.03, 30.03, 0.03}});
  check(map.residentSubmapCount() == 2 && map.hasSubmap(10) && map.hasSubmap(30)
      && !map.hasSubmap(20), "Count retention evicted the wrong submap");
  check(map.pointCloud().size() == large.size() + 2,
        "Count retention truncated a retained submap");
  check(okvis::LidarPointCloudMap::assembleUnfiltered(snapshot).size() == large.size(),
        "Submap eviction mutated an outstanding snapshot");
  check(map.archiveDirectory().empty(), "RAM submap retention enabled persistence");
  const auto revision = map.revision();
  map.addPoints(5, identity, {{0.03, 40.03, 0.03}});
  check(map.residentSubmapCount() == 2 && map.hasSubmap(5) && map.hasSubmap(30)
      && !map.hasSubmap(10) && map.revision() > revision,
        "Old-ID active submap was evicted or display revision was not updated");
  bool rejected = false;
  try { okvis::LidarPointCloudMap invalid(0.1, 0); }
  catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, "Zero resident submap limit was accepted");
}
void run() {
  const auto identity = Eigen::Isometry3d::Identity();
  okvis::LidarPointCloudMap map(1.0);
  check(map.pointCloud().empty(), "Empty map contains points");
  map.addPoints(1, identity, {{0.1, 0.1, 0.1}, {0.5, 0.5, 0.5}, {-0.1, 0.1, 0.1},
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())});
  auto cloud = map.pointCloud();
  check(cloud.size() == 2 && contains(cloud, {0.5, 0.5, 0.5}) && contains(cloud, {-0.1, 0.1, 0.1}),
        "Voxel sampling, negative cells, or invalid point filtering failed");
  const auto revision = map.revision();
  map.addPoints(1, identity, {{0.5, 0.5, 0.5}});
  check(map.revision() == revision, "Unchanged cloud was marked dirty");
  map.addPoints(2, identity, {{0.5, 0.5, 0.5}});
  check(map.pointCloud().size() == 2, "Overlapping submaps were not deduplicated");
  auto corrected = identity;
  corrected.linear() = Eigen::AngleAxisd(0.5 * std::acos(-1.0), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  corrected.translation() = Eigen::Vector3d(10, 0, 0);
  map.updatePose(2, corrected, true);
  cloud = map.pointCloud();
  check(cloud.size() == 3 && contains(cloud, corrected * Eigen::Vector3d(0.5, 0.5, 0.5)),
        "Anchor rotation/translation correction failed");
  const auto correctedRevision = map.revision();
  map.addPoints(2, identity, {{0.5, 0.5, 0.5}});
  check(map.revision() == correctedRevision, "Stale queued scan overwrote corrected pose");
  okvis::LidarPointCloudMap beforeScan(0.1);
  beforeScan.updatePose(3, corrected, true);
  beforeScan.addPoints(3, identity, {{0.2, 0.3, 0.4}});
  check(contains(beforeScan.pointCloud(), corrected * Eigen::Vector3d(0.2, 0.3, 0.4)),
        "Pose correction arriving before first scan was lost");
  auto shifted = corrected;
  shifted.translation().x() += 5.0;
  beforeScan.updatePose(3, shifted, true);
  check(contains(beforeScan.pointCloud(), shifted * Eigen::Vector3d(0.2, 0.3, 0.4)),
        "Pose-only update requires another scan");
  const auto immutable = beforeScan.snapshot();
  const auto repeated = beforeScan.snapshot();
  check(immutable.at(0).points == repeated.at(0).points, "Unchanged snapshot did not reuse cached points");
  beforeScan.addPoints(3, identity, {{2.2, 0.3, 0.4}});
  check(immutable.at(0).points->size() == 1 && beforeScan.snapshot().at(0).points->size() == 2,
        "Mutable active map changed an existing immutable snapshot");
  check(okvis::LidarPointCloudMap::assemble(beforeScan.snapshot(), 0.1, 1).size() <= 1,
        "Visualization point budget ignored");
  check(beforeScan.pointCloud().size() == 2, "Display point budget affected full map export");
  beforeScan.updatePose(3, shifted, true, 10);
  beforeScan.updatePose(3, identity, true, 9);
  check(contains(beforeScan.pointCloud(), shifted * Eigen::Vector3d(0.2, 0.3, 0.4)),
        "An older pending estimator update undid a newer pose");

  // Realtime mode must not create cloud chunks and must evict voxels outside
  // the local FAST-LIVO-style window instead of growing with the trajectory.
  okvis::LidarPointCloudMap local(0.1, 2);
  local.addPoints(1, identity, {{0.1, 0.1, 0.1}, {100.1, 0.1, 0.1}});
  local.setLocalWindow(Eigen::Vector3d::Zero(), 2.0);
  check(local.archiveDirectory().empty(), "RAM-only map unexpectedly created an archive directory");
  check(local.pointCloud().size() == 1 && contains(local.pointCloud(), {0.1, 0.1, 0.1}),
        "Local voxel window did not evict distant points");

  bool invalid = false;
  try { okvis::LidarPointCloudMap bad(0.0); } catch (const std::invalid_argument&) { invalid = true; }
  check(invalid, "Invalid map resolution accepted");
  unfilteredDisplayChecks();
  submapRetentionChecks();
  archiveChecks();
}
} // namespace

int main() {
  try {
    run();
    std::cout << "LiDAR cloud sampling, overlap and pose correction checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
