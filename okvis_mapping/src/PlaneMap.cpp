// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/PlaneMap.hpp>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

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
  std::unordered_set<Key, Hash> changed;
  for (const auto& point : points) {
    if (!point.allFinite()) continue;
    bounds_.extend(point);
    const auto index = key(point);
    auto& cell = cells_[index];
    ++cell.seenPoints;
    if (cell.points.size() < config_.maxPoints) {
      cell.points.push_back(point);
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
  for (const auto& index : changed) fit(cells_.at(index));
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
