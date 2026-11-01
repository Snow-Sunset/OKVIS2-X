// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/PlaneMap.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }

bool samePlane(const okvis::PlaneMap::Plane& a, const okvis::PlaneMap::Plane& b) {
  return (a.normal.array() == b.normal.array()).all()
      && (a.center.array() == b.center.array()).all()
      && a.variance == b.variance && a.radius == b.radius && a.valid == b.valid;
}

void checkExistenceQueries(const okvis::PlaneMap& map) {
  okvis::PlaneMap::Points probes;
  for (double x : {-2.0, -0.01, 0.0, 0.2, 0.625, 0.99, 1.01, 10.2, 31.2, 64.0})
    for (double z : {-0.3, 0.0, 0.1, 0.12, 0.4, 1.0})
      probes.emplace_back(x, 0.2, z);
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int axis = 0; axis < 3; ++axis) {
    for (double value : {nan, infinity, -infinity, std::numeric_limits<double>::max()}) {
      Eigen::Vector3d point(0.2, 0.2, 0.12);
      point[axis] = value;
      probes.push_back(point);
    }
  }
  const double sigmas[] = {-1.0, -0.0, 0.0, 0.03, 0.1, 1.0,
      std::numeric_limits<double>::denorm_min(),
      std::numeric_limits<double>::max(), nan, infinity, -infinity};
  for (const auto& point : probes) for (double sensorSigma : sigmas) {
    okvis::PlaneMap::Plane selected;
    selected.normal.setConstant(11.0);
    selected.center.setConstant(12.0);
    selected.variance = 13.0;
    selected.radius = 14.0;
    double sigma = -15.0;
    const auto sentinel = selected;
    const bool matched = map.match(point, sensorSigma, selected, sigma);
    check(map.hasMatch(point, sensorSigma) == matched, "Existence and best-plane acceptance differ");
    auto after = sentinel;
    double afterSigma = -15.0;
    check(map.match(point, sensorSigma, after, afterSigma) == matched
          && samePlane(selected, after) && sigma == afterSigma,
          "Existence query changed subsequent best-plane output");
    if (!matched)
      check(samePlane(selected, sentinel) && sigma == -15.0,
            "Rejected match overwrote output parameters");
  }
}

void checkMatchBoundaries() {
  okvis::PlaneMap::Config config;
  config.maxMatchDistance = 0.25;
  okvis::PlaneMap map(config);
  okvis::PlaneMap::Points points;
  // Binary fractions give exact center (1/4,1/4,1/4), normal +/-Z and radius 3/8.
  for (int i = 0; i < 8; ++i)
    points.emplace_back(i % 2 ? 0.375 : 0.125, (i / 2) % 2 ? 0.375 : 0.125, 0.25);
  map.addPoints(points);
  okvis::PlaneMap::Plane plane;
  double sigma = 0.0;
  check(map.match({0.25, 0.25, 0.25}, 1.0, plane, sigma), "Boundary fixture has no plane");
  check(plane.radius == 0.375 && plane.variance == 0.0,
        "Boundary fixture does not have exact planar support");
  const double infinity = std::numeric_limits<double>::infinity();
  const auto boundary = [&](const Eigen::Vector3d& point, bool expected) {
    okvis::PlaneMap::Plane selected;
    double residualSigma = 0.0;
    check(map.match(point, 1.0, selected, residualSigma) == expected
          && map.hasMatch(point, 1.0) == expected, "Plane gate boundary changed");
  };
  boundary({0.25, 0.25, std::nextafter(0.5, 0.0)}, true);
  boundary({0.25, 0.25, 0.5}, true);
  boundary({0.25, 0.25, std::nextafter(0.5, infinity)}, false);
  boundary({std::nextafter(0.625, 0.0), 0.25, 0.25}, true);
  boundary({0.625, 0.25, 0.25}, true);
  boundary({std::nextafter(0.625, infinity), 0.25, 0.25}, false);
  check(!map.hasMatch({0.25, 0.25, 0.25}, std::numeric_limits<double>::max()),
        "Overflowed sensor variance was accepted by existence query");
  checkExistenceQueries(map);

  // Both planes pass the gates. The second candidate in Z must still win the
  // best-likelihood query, even though hasMatch() can stop at the first one.
  config.maxMatchDistance = 1.0;
  config.sigmaGate = 10.0;
  okvis::PlaneMap competing(config);
  okvis::PlaneMap::Points twoPlanes = points;
  for (const auto& point : points) twoPlanes.emplace_back(point + Eigen::Vector3d(0, 0, 1));
  competing.addPoints(twoPlanes);
  check(competing.hasMatch({0.25, 0.25, 1.0}, 0.2), "Competing planes did not match");
  check(competing.match({0.25, 0.25, 1.0}, 0.2, plane, sigma)
        && plane.center.z() == 1.25 && sigma == std::sqrt(0.2 * 0.2 + 0.03 * 0.03),
        "Existence fast path changed best-plane selection or residual sigma");
  checkExistenceQueries(competing);
}

void run(const std::filesystem::path& directory) {
  checkMatchBoundaries();
  okvis::PlaneMap::Config config;
  config.maxPoints = 8;
  config.maxCells = 32;
  config.freezeStableCells = true;
  okvis::PlaneMap map(config);
  map.setWindow(Eigen::Vector3d::Zero(), 100.0);
  okvis::PlaneMap::Points points;
  for (int cell = 0; cell < 64; ++cell) for (int i = 0; i < 8; ++i)
    points.emplace_back(cell + 0.1 + 0.15 * (i % 4), 0.1 + 0.2 * (i / 4), 0.1);
  map.addPoints(points);
  check(map.numCells() == 32 && map.numPlanes() == 32, "Active plane cell cap ignored");
  check(map.residentBytes() < 20000, "Stable plane samples not released");
  okvis::PlaneMap::Plane plane;
  double sigma = 0;
  check(map.match({0.2, 0.2, 0.12}, 0.1, plane, sigma), "Stable plane no longer matches");
  checkExistenceQueries(map);
  map.finalize();
  checkExistenceQueries(map);
  const auto filename = (directory / "plane.bin").string();
  map.save(filename);
  const auto loaded = okvis::PlaneMap::load(filename, config);
  check(loaded->numPlanes() == map.numPlanes() && loaded->finalized(), "Plane archive lost parameters");
  check((loaded->bounds().min() - map.bounds().min()).norm() == 0.0
      && (loaded->bounds().max() - map.bounds().max()).norm() == 0.0, "Plane archive changed bounds");
  okvis::PlaneMap::Plane restored;
  double restoredSigma = 0;
  check(loaded->match({0.2, 0.2, 0.12}, 0.1, restored, restoredSigma), "Archived plane cannot be matched");
  check((restored.center - plane.center).norm() == 0.0 && restoredSigma == sigma,
        "Archived point-plane residual changed");
  checkExistenceQueries(*loaded);
  bool rejected = false;
  try { loaded->addPoints(points); } catch (const std::logic_error&) { rejected = true; }
  check(rejected, "Completed archived planes were mutated");
  std::filesystem::resize_file(filename, 20);
  rejected = false;
  try { okvis::PlaneMap::load(filename, config); } catch (const std::exception&) { rejected = true; }
  check(rejected, "Truncated plane archive accepted");
  okvis::PlaneMap sliding(config);
  sliding.setWindow({0, 0, 0}, 1.0);
  sliding.addPoints(points);
  const auto oldCount = sliding.numCells();
  sliding.setWindow({10, 0, 0}, 1.0);
  sliding.addPoints(points);
  check(oldCount > 0 && sliding.numCells() <= 4, "Distant local plane cells not evicted");
  check(!sliding.match({0.2, 0.2, 0.12}, 0.1, plane, sigma), "Old window planes remained resident");
  checkExistenceQueries(sliding);

  okvis::PlaneMap::Config movingConfig = config;
  movingConfig.maxCells = 0;
  movingConfig.freezeStableCells = false;
  okvis::PlaneMap thresholded(movingConfig);
  thresholded.setWindow(Eigen::Vector3d::Zero(), 5.0);
  okvis::PlaneMap::Points trailing;
  for (int i = 0; i < 8; ++i)
    trailing.emplace_back(-4.8 + 0.2 * (i % 2), 0.1 + 0.2 * ((i / 2) % 2), 0.1);
  thresholded.addPoints(trailing);
  const auto beforeSmallMove = thresholded.numCells();
  thresholded.setWindow({0.5, 0, 0}, 5.0); // below max(voxelSize, 10% of extent)
  thresholded.addPoints({Eigen::Vector3d(5.25, 0.1, 0.1)});
  const auto afterSmallMove = thresholded.numCells();
  check(afterSmallMove > beforeSmallMove,
        "Small window move did not update the active filtering centre");
  thresholded.setWindow({2.0, 0, 0}, 5.0); // accumulated motion triggers one full prune
  check(thresholded.numCells() < afterSmallMove,
        "Accumulated window motion did not prune distant plane cells");
  checkExistenceQueries(thresholded);

  okvis::PlaneMap::Config saturatedConfig = movingConfig;
  saturatedConfig.maxCells = 2;
  okvis::PlaneMap saturated(saturatedConfig);
  saturated.setWindow(Eigen::Vector3d::Zero(), 50.0);
  saturated.addPoints({Eigen::Vector3d(-49.8, 0.1, 0.1), Eigen::Vector3d(0.1, 0.1, 0.1)});
  check(saturated.numCells() == 2, "Plane map did not reach the configured cell cap");
  saturated.setWindow({2.0, 0, 0}, 50.0); // below the normal 5 m prune threshold
  saturated.addPoints({Eigen::Vector3d(51.2, 0.1, 0.1)});
  check(saturated.numCells() == 2 && saturated.bounds().min().x() >= 0.0
          && saturated.bounds().max().x() > 51.0,
        "Stale full-window cells starved a newly visible leading-edge cell");
}
}
int main() {
  auto directory = std::filesystem::temp_directory_path() / ("okvis-plane-test-" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    std::filesystem::create_directory(directory);
    run(directory);
    std::filesystem::remove_all(directory);
    std::cout << "Bounded plane samples, window, archive and residual checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::filesystem::remove_all(directory);
    std::cerr << error.what() << '\n';
    return 1;
  }
}
