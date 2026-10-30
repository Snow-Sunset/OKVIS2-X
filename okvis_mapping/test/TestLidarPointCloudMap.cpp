// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/LidarPointCloudMap.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
bool contains(const okvis::LidarPointCloudMap::Points& points, const Eigen::Vector3d& expected) {
  for (const auto& point : points) if ((point - expected).norm() < 1e-10) return true;
  return false;
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
  bool invalid = false;
  try { okvis::LidarPointCloudMap bad(0.0); } catch (const std::invalid_argument&) { invalid = true; }
  check(invalid, "Invalid map resolution accepted");
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
