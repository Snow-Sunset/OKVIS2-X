// SPDX-License-Identifier: BSD-3-Clause
#ifndef OKVIS_PLANE_MAP_HPP
#define OKVIS_PLANE_MAP_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <array>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

namespace okvis {

/// Local planes expressed in the submap anchor frame. Only the mapping thread
/// writes the active map; completed maps are published as shared_ptr<const>.
class PlaneMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct Config {
    double voxelSize = 1.0;              ///< metres
    double maxPlaneVariance = 0.01;      ///< smallest PCA eigenvalue, m^2
    double maxEigenvalueRatio = 0.1;
    double maxMatchDistance = 0.3;       ///< metres
    double noiseFloor = 0.03;            ///< standard deviation, metres
    double sigmaGate = 3.0;
    size_t minPoints = 6;
    size_t maxPoints = 100;
  };
  struct Plane {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double variance = 0.0; ///< local surface scatter, not parameter covariance
    double radius = 0.0;
    bool valid = false;
  };
  using Points = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

  explicit PlaneMap(const Config& config);
  void addPoints(const Points& points);
  bool match(const Eigen::Vector3d& point, double sensorSigma,
             Plane& plane, double& residualSigma) const;
  size_t numPlanes() const;
  const Eigen::AlignedBox3d& bounds() const { return bounds_; }

 private:
  using Key = std::array<int64_t, 3>;
  struct Hash {
    size_t operator()(const Key& key) const;
  };
  struct Cell {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Points points;
    Plane plane;
    uint64_t seenPoints = 0;
  };
  Key key(const Eigen::Vector3d& point) const;
  void fit(Cell& cell);
  Config config_;
  Eigen::AlignedBox3d bounds_;
  std::unordered_map<Key, Cell, Hash> cells_;
};

using PlaneMapPtr = std::shared_ptr<const PlaneMap>;

} // namespace okvis
#endif
