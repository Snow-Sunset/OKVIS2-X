// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/LidarMapColour.hpp>
#include <okvis/LidarPointCloudMap.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unistd.h>

namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void run() {
  namespace fs = std::filesystem;
  const fs::path directory = fs::temp_directory_path()
      / ("okvis-colored-map-test-" + std::to_string(::getpid()));
  fs::create_directories(directory);
  struct Cleanup { fs::path path; ~Cleanup() { std::error_code e; fs::remove_all(path, e); } } cleanup{directory};
  okvis::LidarHeightColour colour(-2, 2);
  check(colour(-3) == 0x0000ff && colour(3) == 0xff0000
      && colour(-1) == 0x00ffff && colour(0) == 0x00ff00 && colour(1) == 0xffff00,
      "Height palette endpoints, clipping or channel packing changed");
  check(colour(std::numeric_limits<double>::quiet_NaN()) == 0, "NaN height colour is not deterministic");
  bool threw = false;
  try { okvis::LidarHeightColour invalid(2, 2); }
  catch (const std::invalid_argument&) { threw = true; }
  check(threw, "Invalid height bounds were accepted");

  okvis::LidarPointCloudMap::ArchiveOptions options;
  options.directory = directory.string();
  options.maxResidentSubmaps = 1;
  options.chunkPointLimit = 8;
  okvis::LidarPointCloudMap map(.1, options);
  map.enableGlobalPreview(.5, 16);
  const auto identity = Eigen::Isometry3d::Identity();
  okvis::LidarPointCloudMap::Points input;
  for (size_t i = 0; i < 200; ++i) input.emplace_back(.03 + .25*i, .03, -1.97 + .02*(i%100));
  map.addPoints(1, identity, input);
  map.addPoints(2, identity, input);
  auto corrected = identity;
  corrected.translation().z() = 1;
  map.updatePose(1, corrected, true, 1);
  map.updatePose(2, corrected, true, 1);
  map.flush();
  const auto snapshot = map.snapshot();
  for (const auto& block : snapshot)
    check(block.id == 1 || block.id == 2, "Archive snapshot lost stable keyframe IDs");
  const auto file = (directory / "rgb.pcd").string();
  check(okvis::LidarPointCloudMap::saveColoredPcd(snapshot, .1, file,
      [&](const Eigen::Vector3d& point) { return colour(point.z()); }, 4096),
      "Bounded coloured external PCD export failed");
  std::ifstream stream(file, std::ios::binary);
  std::string line;
  size_t count = 0;
  bool rgb = false, types = false;
  while (std::getline(stream, line)) {
    rgb |= line == "FIELDS x y z rgb";
    types |= line == "TYPE F F F F";
    if (line.rfind("POINTS ", 0) == 0) count = std::stoull(line.substr(7));
    if (line == "DATA binary") break;
  }
  check(rgb && types && count == input.size(), "Coloured PCD layout or global deduplication is incorrect");
  for (size_t i = 0; i < count; ++i) {
    float xyz[3];
    uint32_t packed;
    stream.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
    stream.read(reinterpret_cast<char*>(&packed), sizeof(packed));
    check(bool(stream), "Truncated XYZRGB binary record");
    check(std::abs(xyz[0] - input[i].x()) < 1e-5
        && std::abs(xyz[2] - input[i].z() - 1.0) < 1e-5,
        "Coloured PCD did not apply corrected anchor pose");
    check(packed == colour(input[i].z() + 1.0), "PCD colour is not evaluated in corrected world coordinates");
  }
  check(stream.peek() == std::char_traits<char>::eof(), "Unexpected PCD padding or extra points");
  check(okvis::LidarPointCloudMap::assemble(map.visualizationSnapshot(),
      map.visualizationVoxelSize(), 16).size() <= 16, "Preview cap did not survive global pose corrections");
  // The new sidecar's display walks the COMPLETE archive with a fixed stride;
  // a stride must continue across chunk/submap boundaries rather than restart.
  const auto full = okvis::LidarPointCloudMap::assembleUnfiltered(snapshot);
  for (size_t step : {size_t(1), size_t(3), size_t(13)}) {
    const size_t expected = (full.size() + step - 1) / step;
    check(okvis::LidarPointCloudMap::sampledPointCount(snapshot, step) == expected,
        "Fixed-step point count changed across archive chunks");
    size_t visited = 0;
    okvis::LidarPointCloudMap::forEachSampledPoint(snapshot, step, [&](const Eigen::Vector3d& point) {
      check((point - full.at(visited*step)).norm() < 1e-10,
          "Archive sampling restarted at a chunk boundary or lost corrected pose");
      ++visited;
    });
    check(visited == expected, "Fixed-step archive display lost selected points");
  }
  {
    size_t visited = 0;
    okvis::LidarPointCloudMap::forEachSampledPoint(snapshot, 13, [&](const Eigen::Vector3d& point) {
      check((point - full.at(2 + visited*13)).norm() < 1e-10,
          "Incremental block sampling lost its global phase");
      ++visited;
    }, 2);
    check(visited == okvis::LidarPointCloudMap::sampledPointCount(snapshot, 13, 2),
        "Initial-skip point count differs from traversal");
  }
  auto dense = std::make_shared<okvis::LidarPointCloudMap::Points>();
  constexpr size_t denseCount = 200003;
  dense->reserve(denseCount);
  for (size_t i = 0; i < denseCount; ++i) dense->emplace_back(.25*i, .03, .04);
  const okvis::LidarPointCloudMap::Snapshot large{{corrected, dense, {}}};
  size_t visited = 0;
  okvis::LidarPointCloudMap::forEachSampledPoint(large, 1, [&](const Eigen::Vector3d& point) {
    check((point - corrected*dense->at(visited)).norm() < 1e-10, "Dense display changed world coordinates");
    ++visited;
  });
  check(visited == denseCount && okvis::LidarPointCloudMap::sampledPointCount(large, 1) == denseCount,
      "Fixed-step display retained the old 200000-point cap");
  threw = false;
  try { okvis::LidarPointCloudMap::sampledPointCount(large, 0); }
  catch (const std::invalid_argument&) { threw = true; }
  check(threw, "Zero display sampling step was not rejected");
  // An export callback failure must preserve the previous completed file.
  const auto before = fs::file_size(file);
  threw = false;
  try {
    okvis::LidarPointCloudMap::saveColoredPcd(snapshot, .1, file,
        [](const Eigen::Vector3d&) -> uint32_t { throw std::runtime_error("test colour failure"); }, 4096);
  } catch (const std::runtime_error&) { threw = true; }
  check(threw && fs::file_size(file) == before, "Failed colour export replaced the previous PCD");
}
} // namespace

int main() {
  try { run(); std::cout << "Coloured PCD, correction, deduplication and uncapped fixed-step display passed\n"; }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
