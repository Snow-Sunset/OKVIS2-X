// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/PlaneMap.hpp>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <fcntl.h>
#include <unistd.h>

namespace okvis {

PlaneMap::PlaneMap(const Config& config) : config_(config) {
  if (!(config_.voxelSize > 0.0) || !std::isfinite(config_.voxelSize)
      || !(config_.maxPlaneVariance > 0.0) || !std::isfinite(config_.maxPlaneVariance)
      || !(config_.maxEigenvalueRatio > 0.0 && config_.maxEigenvalueRatio < 1.0)
      || !(config_.maxMatchDistance > 0.0) || !std::isfinite(config_.maxMatchDistance)
      || !(config_.noiseFloor > 0.0) || !std::isfinite(config_.noiseFloor)
      || !(config_.sigmaGate > 0.0) || !std::isfinite(config_.sigmaGate)
      || config_.minPoints < 3 || config_.maxPoints < config_.minPoints) {
    throw std::invalid_argument("Invalid point-to-plane map configuration");
  }
}

size_t PlaneMap::Hash::operator()(const Key& key) const {
  size_t seed = 0;
  for (const auto value : key) {
    seed ^= std::hash<int64_t>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }
  return seed;
}

PlaneMap::Key PlaneMap::key(const Eigen::Vector3d& point) const {
  return {{int64_t(std::floor(point.x() / config_.voxelSize)),
           int64_t(std::floor(point.y() / config_.voxelSize)),
           int64_t(std::floor(point.z() / config_.voxelSize))}};
}

void PlaneMap::addPoints(const Points& points) {
  if (finalized_) throw std::logic_error("Cannot update a completed plane map");
  std::unordered_set<Key, Hash> changed;
  bool capacityPruned = false;
  for (const auto& point : points) {
    if (!point.allFinite()) continue;
    const auto scaled = (point / config_.voxelSize).array().floor();
    if ((scaled <= double(std::numeric_limits<int64_t>::min()) + 1024.0).any()
        || (scaled >= double(std::numeric_limits<int64_t>::max()) - 1024.0).any()) continue;
    if (windowHalfExtent_ > 0.0
        && (point - windowCenter_).cwiseAbs().maxCoeff() > windowHalfExtent_) continue;
    const auto index = key(point);
    if (config_.maxCells && cells_.size() >= config_.maxCells && !cells_.count(index)) {
      // A deferred window prune must not let stale trailing cells starve newly
      // visible cells at the leading edge. Scan at most once per input batch.
      if (!capacityPruned && windowHalfExtent_ > 0.0) {
        pruneWindow(windowCenter_, windowHalfExtent_);
        lastWindowPruneCenter_ = windowCenter_;
        lastWindowPruneHalfExtent_ = windowHalfExtent_;
        windowPruneInitialized_ = true;
        capacityPruned = true;
      }
      if (cells_.size() >= config_.maxCells) continue;
    }
    auto& cell = cells_[index];
    if (cell.frozen) continue;
    bounds_.extend(point);
    ++cell.seenPoints;
    if (cell.points.size() < config_.maxPoints) {
      const size_t oldCapacity = cell.points.capacity();
      cell.points.push_back(point);
      sampleCapacityBytes_ += (cell.points.capacity() - oldCapacity) * sizeof(Eigen::Vector3d);
      changed.insert(index);
    } else {
      // Deterministic reservoir sampling: do not freeze a cell on its first
      // scan lines, which can leave a planar surface looking like a line.
      uint64_t sample = cell.seenPoints + 0x9e3779b97f4a7c15ULL;
      sample = (sample ^ (sample >> 30)) * 0xbf58476d1ce4e5b9ULL;
      sample = (sample ^ (sample >> 27)) * 0x94d049bb133111ebULL;
      sample ^= sample >> 31;
      const uint64_t slot = sample % cell.seenPoints;
      if (slot < config_.maxPoints) {
        cell.points[slot] = point;
        changed.insert(index);
      }
    }
  }
  for (const auto& index : changed) {
    auto& cell = cells_.at(index);
    fit(cell);
    if (config_.freezeStableCells && cell.plane.valid && cell.points.size() >= config_.maxPoints) {
      sampleCapacityBytes_ -= cell.points.capacity() * sizeof(Eigen::Vector3d);
      Points().swap(cell.points);
      cell.frozen = true;
    }
  }
}

size_t PlaneMap::residentBytes() const {
  return sizeof(*this) + cells_.bucket_count() * sizeof(void*) + sampleCapacityBytes_
      + cells_.size() * (sizeof(std::pair<const Key, Cell>) + 3 * sizeof(void*));
}

void PlaneMap::setWindow(const Eigen::Vector3d& center, double halfExtent) {
  if (finalized_ || !center.allFinite() || !std::isfinite(halfExtent) || halfExtent <= 0.0)
    throw std::invalid_argument("Invalid active plane window");
  // Keep filtering centred on the latest pose, but prune the hash only after
  // meaningful accumulated motion. Scanning every active cell for every LiDAR
  // packet makes local-window maintenance scale with map size rather than
  // vehicle motion.
  windowCenter_ = center;
  windowHalfExtent_ = halfExtent;
  const double pruneDistance = std::max(config_.voxelSize, 0.1 * halfExtent);
  const bool extentChanged = !windowPruneInitialized_
      || std::abs(halfExtent - lastWindowPruneHalfExtent_) > 1e-12;
  const bool movedEnough = !windowPruneInitialized_
      || (center - lastWindowPruneCenter_).cwiseAbs().maxCoeff() >= pruneDistance;
  if (!extentChanged && !movedEnough) return;
  lastWindowPruneCenter_ = center;
  lastWindowPruneHalfExtent_ = halfExtent;
  windowPruneInitialized_ = true;
  pruneWindow(center, halfExtent);
}

bool PlaneMap::pruneWindow(const Eigen::Vector3d& center, double halfExtent) {
  bool removed = false;
  for (auto it = cells_.begin(); it != cells_.end();) {
    const Eigen::Vector3d cellCenter = (Eigen::Vector3d(double(it->first[0]), double(it->first[1]),
        double(it->first[2])) + Eigen::Vector3d::Constant(0.5)) * config_.voxelSize;
    if ((cellCenter - center).cwiseAbs().maxCoeff() > halfExtent + config_.voxelSize) {
      sampleCapacityBytes_ -= it->second.points.capacity() * sizeof(Eigen::Vector3d);
      it = cells_.erase(it);
      removed = true;
    }
    else ++it;
  }
  if (removed) recomputeBounds();
  return removed;
}

void PlaneMap::recomputeBounds() {
  bounds_.setEmpty();
  // Conservative cell support remains available after stable samples are freed.
  for (const auto& entry : cells_) {
    const Eigen::Vector3d minimum(double(entry.first[0]) * config_.voxelSize,
        double(entry.first[1]) * config_.voxelSize, double(entry.first[2]) * config_.voxelSize);
    bounds_.extend(minimum);
    bounds_.extend(minimum + Eigen::Vector3d::Constant(config_.voxelSize));
  }
}

void PlaneMap::finalize() {
  for (auto it = cells_.begin(); it != cells_.end();) {
    if (!it->second.plane.valid) it = cells_.erase(it);
    else { Points().swap(it->second.points); it->second.frozen = true; ++it; }
  }
  sampleCapacityBytes_ = 0;
  finalized_ = true;
  recomputeBounds();
}

void PlaneMap::save(const std::string& filename) const {
  if (!finalized_) throw std::logic_error("Archive only completed plane maps");
  const auto temp = filename + ".tmp";
  std::ofstream out(temp, std::ios::binary | std::ios::trunc);
  out.exceptions(std::ios::failbit | std::ios::badbit);
  const uint64_t magic = 0x31504d5356494b4fULL, count = cells_.size();
  out.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
  out.write(reinterpret_cast<const char*>(&count), sizeof(count));
  out.write(reinterpret_cast<const char*>(bounds_.min().data()), 3 * sizeof(double));
  out.write(reinterpret_cast<const char*>(bounds_.max().data()), 3 * sizeof(double));
  for (const auto& entry : cells_) {
    out.write(reinterpret_cast<const char*>(entry.first.data()), 3 * sizeof(int64_t));
    const auto& p = entry.second.plane;
    out.write(reinterpret_cast<const char*>(p.normal.data()), 3 * sizeof(double));
    out.write(reinterpret_cast<const char*>(p.center.data()), 3 * sizeof(double));
    out.write(reinterpret_cast<const char*>(&p.variance), sizeof(double));
    out.write(reinterpret_cast<const char*>(&p.radius), sizeof(double));
  }
  out.close();
  const int descriptor = ::open(temp.c_str(), O_RDONLY);
  if (descriptor < 0) throw std::runtime_error("Cannot sync plane archive: " + temp);
  const int synced = ::fsync(descriptor);
  ::close(descriptor);
  if (synced != 0) throw std::runtime_error("Cannot sync plane archive: " + temp);
  std::filesystem::rename(temp, filename);
  const auto parent = std::filesystem::path(filename).parent_path();
  const int directory = ::open((parent.empty() ? std::filesystem::path(".") : parent).c_str(), O_RDONLY | O_DIRECTORY);
  if (directory < 0) throw std::runtime_error("Cannot sync plane archive directory");
  const int directorySynced = ::fsync(directory);
  ::close(directory);
  if (directorySynced != 0) throw std::runtime_error("Cannot sync plane archive directory");
}

std::shared_ptr<PlaneMap> PlaneMap::load(const std::string& filename, const Config& config,
                                      size_t maxBytes) {
  std::ifstream in(filename, std::ios::binary);
  in.exceptions(std::ios::failbit | std::ios::badbit);
  uint64_t magic = 0, count = 0;
  in.read(reinterpret_cast<char*>(&magic), sizeof(magic));
  in.read(reinterpret_cast<char*>(&count), sizeof(count));
  if (magic != 0x31504d5356494b4fULL || count > maxBytes / (sizeof(Cell) + sizeof(Key) + 40))
    throw std::runtime_error("Invalid or over-budget plane archive: " + filename);
  auto result = std::make_shared<PlaneMap>(config);
  Eigen::Vector3d minimum, maximum;
  in.read(reinterpret_cast<char*>(minimum.data()), 3 * sizeof(double));
  in.read(reinterpret_cast<char*>(maximum.data()), 3 * sizeof(double));
  result->bounds_ = Eigen::AlignedBox3d(minimum, maximum);
  if (count && (!minimum.allFinite() || !maximum.allFinite() || (minimum.array() > maximum.array()).any()))
    throw std::runtime_error("Corrupt plane bounds: " + filename);
  for (uint64_t i = 0; i < count; ++i) {
    Key index;
    Cell cell;
    auto& p = cell.plane;
    in.read(reinterpret_cast<char*>(index.data()), 3 * sizeof(int64_t));
    in.read(reinterpret_cast<char*>(p.normal.data()), 3 * sizeof(double));
    in.read(reinterpret_cast<char*>(p.center.data()), 3 * sizeof(double));
    in.read(reinterpret_cast<char*>(&p.variance), sizeof(double));
    in.read(reinterpret_cast<char*>(&p.radius), sizeof(double));
    if (!p.normal.allFinite() || !p.center.allFinite() || std::abs(p.normal.norm() - 1.0) > 1e-6
        || !std::isfinite(p.variance) || p.variance < 0.0 || !std::isfinite(p.radius) || p.radius < 0.0)
      throw std::runtime_error("Corrupt plane archive: " + filename);
    p.valid = true;
    cell.frozen = true;
    result->cells_.emplace(index, std::move(cell));
  }
  result->finalized_ = true;
  return result;
}

void PlaneMap::fit(Cell& cell) {
  cell.plane.valid = false;
  if (cell.points.size() < config_.minPoints) return;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (const auto& point : cell.points) center += point;
  center /= double(cell.points.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto& point : cell.points) {
    const Eigen::Vector3d delta = point - center;
    covariance.noalias() += delta * delta.transpose();
  }
  covariance /= double(cell.points.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) return;
  const auto eigenvalues = solver.eigenvalues().cwiseMax(0.0).eval();
  // Reject lines and ill-conditioned normals as well as non-planar cells.
  if (eigenvalues[1] < 1e-6 || eigenvalues[0] > config_.maxPlaneVariance
      || eigenvalues[0] > config_.maxEigenvalueRatio * eigenvalues[1]) return;
  cell.plane.center = center;
  cell.plane.normal = solver.eigenvectors().col(0);
  cell.plane.variance = eigenvalues[0];
  cell.plane.radius = 3.0 * std::sqrt(eigenvalues[2]);
  cell.plane.valid = true;
}

bool PlaneMap::match(const Eigen::Vector3d& point, double sensorSigma,
                     Plane& plane, double& residualSigma) const {
  if (!point.allFinite() || !std::isfinite(sensorSigma) || sensorSigma < 0.0) return false;
  const auto scaled = (point / config_.voxelSize).array().floor();
  if ((scaled <= double(std::numeric_limits<int64_t>::min()) + 1024.0).any()
      || (scaled >= double(std::numeric_limits<int64_t>::max()) - 1024.0).any()) return false;
  const auto index = key(point);
  double bestScore = std::numeric_limits<double>::infinity();
  bool matched = false;
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = -1; z <= 1; ++z) {
        const Key candidate{{index[0] + x, index[1] + y, index[2] + z}};
        const auto it = cells_.find(candidate);
        if (it == cells_.end() || !it->second.plane.valid) continue;
        const auto& p = it->second.plane;
        const Eigen::Vector3d delta = point - p.center;
        const double distance = std::abs(p.normal.dot(delta));
        const double tangentDistance = std::sqrt(std::max(0.0, delta.squaredNorm() - distance * distance));
        if (distance > config_.maxMatchDistance || tangentDistance > p.radius) continue;
        const double variance = sensorSigma * sensorSigma + p.variance
                                + config_.noiseFloor * config_.noiseFloor;
        const double score = distance * distance / variance;
        if (score > config_.sigmaGate * config_.sigmaGate) continue;
        // Gaussian likelihood, including the variance normalization term.
        const double likelihoodScore = score + std::log(variance);
        if (likelihoodScore < bestScore) {
          bestScore = likelihoodScore;
          plane = p;
          residualSigma = std::sqrt(variance);
          matched = true;
        }
      }
    }
  }
  return matched;
}

size_t PlaneMap::numPlanes() const {
  size_t count = 0;
  for (const auto& cell : cells_) count += cell.second.plane.valid;
  return count;
}

} // namespace okvis
