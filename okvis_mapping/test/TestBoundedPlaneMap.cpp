// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/PlaneMap.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
void run(const std::filesystem::path& directory) {
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
  map.finalize();
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
