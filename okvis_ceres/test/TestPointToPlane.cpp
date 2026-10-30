// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/PlaneMap.hpp>
#include <okvis/ceres/PointToPlaneError.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <ceres/ceres.h>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
using Pose = std::array<double, 7>;
Pose pose(const Eigen::Vector3d& t, const Eigen::Quaterniond& q) {
  return {{t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()}};
}
void testMap() {
  okvis::PlaneMap::Config config;
  okvis::PlaneMap map(config);
  okvis::PlaneMap::Plane plane;
  double sigma;
  check(!map.match({0.3, 0.3, 0.22}, 0.01, plane, sigma), "Empty map matched");
  okvis::PlaneMap::Points points;
  for (int x = 0; x < 4; ++x) {
    for (int y = 0; y < 4; ++y) points.emplace_back(0.1 + x * 0.2, 0.1 + y * 0.2, 0.2);
  }
  map.addPoints(points);
  check(map.numPlanes() == 1, "Planar cell not fitted");
  check(map.match({0.3, 0.3, 0.22}, 0.01, plane, sigma), "Plane association failed");
  check(std::abs(std::abs(plane.normal.z()) - 1.0) < 1e-10, "Wrong plane normal");
  check(std::abs(sigma - std::sqrt(0.001)) < 1e-10, "Wrong whitening variance");
  check(map.match({-0.01, 0.3, 0.22}, 0.01, plane, sigma), "Neighbour/negative voxel lookup failed");
  check(!map.match({0.3, 0.3, 0.5}, 0.01, plane, sigma), "Distance gate failed");
  check(!map.match({0.3, 0.3, 0.22}, -0.01, plane, sigma), "Invalid noise accepted");
  okvis::PlaneMap line(config);
  points.clear();
  for (int x = 0; x < 12; ++x) points.emplace_back(0.1 + x * 0.05, 0.3, 0.2);
  line.addPoints(points);
  check(line.numPlanes() == 0, "Line accepted as plane");
  // A full cell must recover from an initially collinear scan.
  config.maxPoints = 12;
  okvis::PlaneMap recovery(config);
  recovery.addPoints(points);
  points.clear();
  for (int repeat = 0; repeat < 20; ++repeat) {
    for (int x = 0; x < 4; ++x) {
      for (int y = 0; y < 4; ++y) points.emplace_back(0.1 + x * 0.2, 0.1 + y * 0.2, 0.2);
    }
  }
  recovery.addPoints(points);
  check(recovery.numPlanes() == 1, "Full cell did not recover from line samples");
  config.voxelSize = 0.0;
  bool threw = false;
  try { okvis::PlaneMap invalid(config); } catch (const std::invalid_argument&) { threw = true; }
  check(threw, "Invalid map configuration accepted");
}

void testFactor() {
  Pose a = pose({0.2, -0.4, 0.3}, Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d(1, 2, 3).normalized())));
  Pose b = pose({-0.3, 0.2, 0.6}, Eigen::Quaterniond(Eigen::AngleAxisd(-0.2, Eigen::Vector3d(2, -1, 1).normalized())));
  const Eigen::Vector3d point(0.8, -0.7, 1.2), center(0.1, 0.2, 0.3), normal = Eigen::Vector3d(1, 2, -1).normalized();
  okvis::ceres::PointToPlaneError factor(point, center, normal, 0.07);
  const double* params[] = {a.data(), b.data()};
  Eigen::Matrix<double, 1, 7, Eigen::RowMajor> ambient[2];
  Eigen::Matrix<double, 1, 6, Eigen::RowMajor> minimal[2];
  double* j[] = {ambient[0].data(), ambient[1].data()};
  double* jm[] = {minimal[0].data(), minimal[1].data()};
  double residual, residualOnly;
  check(factor.EvaluateWithMinimalJacobians(params, &residual, j, jm), "Evaluate failed");
  factor.Evaluate(params, &residualOnly, nullptr);
  check(std::abs(residual - residualOnly) < 1e-12 && std::abs(residualOnly) > 0.01,
        "Residual-only evaluation differs");
  const okvis::kinematics::Transformation ta(Eigen::Vector3d(a[0], a[1], a[2]), Eigen::Quaterniond(a[6], a[3], a[4], a[5]));
  const okvis::kinematics::Transformation tb(Eigen::Vector3d(b[0], b[1], b[2]), Eigen::Quaterniond(b[6], b[3], b[4], b[5]));
  const auto tab = ta.inverse() * tb;
  check(std::abs(residual - normal.dot(tab.C() * point + tab.r() - center) / 0.07) < 1e-12,
        "Wrong geometric residual");
  for (int block = 0; block < 2; ++block) {
    Eigen::Matrix<double, 7, 6, Eigen::RowMajor> plus;
    okvis::ceres::PoseManifold::plusJacobian(params[block], plus.data());
    check((ambient[block] * plus - minimal[block]).norm() < 1e-10, "Ambient lift differs");
    for (int col = 0; col < 6; ++col) {
      Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
      delta[col] = 1e-6;
      Pose positive, negative;
      okvis::ceres::PoseManifold::plus(params[block], delta.data(), positive.data());
      delta[col] = -1e-6;
      okvis::ceres::PoseManifold::plus(params[block], delta.data(), negative.data());
      const double* perturbed[] = {a.data(), b.data()};
      double rp, rn;
      perturbed[block] = positive.data(); factor.Evaluate(perturbed, &rp, nullptr);
      perturbed[block] = negative.data(); factor.Evaluate(perturbed, &rn, nullptr);
      check(std::abs((rp - rn) / 2e-6 - minimal[block][col]) < 1e-7, "Finite difference Jacobian failed");
    }
  }
  // The minimal-only API must work without ambient Jacobian buffers.
  factor.EvaluateWithMinimalJacobians(params, &residualOnly, nullptr, jm);
  const okvis::kinematics::Transformation worldChange({1.0, 2.0, -0.5},
      Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY())));
  const auto newA = worldChange * ta, newB = worldChange * tb;
  Pose ap = pose(newA.r(), newA.q()), bp = pose(newB.r(), newB.q());
  const double* shifted[] = {ap.data(), bp.data()};
  double changed;
  factor.Evaluate(shifted, &changed, nullptr);
  check(std::abs(changed - residual) < 1e-10, "Global frame invariance failed");
}

void testOptimization() {
  Pose a{{0, 0, 0, 0, 0, 0, 1}};
  Pose b = pose({0.1, -0.08, 0.05}, Eigen::Quaterniond(Eigen::AngleAxisd(0.08, Eigen::Vector3d(1, 2, 3).normalized())));
  ::ceres::Problem problem;
  problem.AddParameterBlock(a.data(), 7, new okvis::ceres::PoseManifold);
  problem.AddParameterBlock(b.data(), 7, new okvis::ceres::PoseManifold);
  problem.SetParameterBlockConstant(a.data());
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::Vector3d normal = Eigen::Vector3d::Unit(axis);
    for (int u = -1; u <= 1; ++u) {
      for (int v = -1; v <= 1; ++v) {
        Eigen::Vector3d point = normal;
        point[(axis + 1) % 3] = u * 0.5;
        point[(axis + 2) % 3] = v * 0.5;
        problem.AddResidualBlock(new okvis::ceres::PointToPlaneError(point, point, normal, 0.03),
                                 nullptr, a.data(), b.data());
      }
    }
  }
  ::ceres::Solver::Options options;
  options.linear_solver_type = ::ceres::DENSE_QR;
  options.max_num_iterations = 20;
  ::ceres::Solver::Summary summary;
  ::ceres::Solve(options, &problem, &summary);
  check(summary.IsSolutionUsable() && summary.final_cost < 1e-12, "Point-to-plane optimization failed");
  check(Eigen::Map<Eigen::Vector3d>(b.data()).norm() < 1e-7, "Translation did not converge");
  check(Eigen::Map<Eigen::Vector3d>(b.data() + 3).norm() < 1e-7, "Rotation did not converge");
}
} // namespace

int main() {
  try {
    testMap(); testFactor(); testOptimization();
    std::cout << "Plane map, residual, Jacobians and Ceres optimization passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
