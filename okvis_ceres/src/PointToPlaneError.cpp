// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/ceres/PointToPlaneError.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/kinematics/operators.hpp>
#include <cmath>
#include <stdexcept>

namespace okvis {
namespace ceres {

PointToPlaneError::PointToPlaneError(const Eigen::Vector3d& pointB,
    const Eigen::Vector3d& centerA, const Eigen::Vector3d& normalA, double sigma)
    : pointB_(pointB), centerA_(centerA), normalA_(normalA), sqrtInformation_(1.0 / sigma) {
  if (!pointB.allFinite() || !centerA.allFinite() || !normalA.allFinite()
      || normalA.norm() < 1e-12 || !std::isfinite(sigma) || sigma <= 0.0) {
    throw std::invalid_argument("Invalid point-to-plane measurement");
  }
  normalA_.normalize();
}

bool PointToPlaneError::Evaluate(double const* const* parameters,
    double* residuals, double** jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

bool PointToPlaneError::EvaluateWithMinimalJacobians(double const* const* parameters,
    double* residuals, double** jacobians, double** jacobiansMinimal) const {
  const Eigen::Map<const Eigen::Vector3d> tA(parameters[0]);
  const Eigen::Map<const Eigen::Vector3d> tB(parameters[1]);
  const Eigen::Quaterniond qA(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
  const Eigen::Quaterniond qB(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
  const Eigen::Matrix3d RaT = qA.toRotationMatrix().transpose();
  const Eigen::Vector3d rotatedB = qB.toRotationMatrix() * pointB_;
  const Eigen::Vector3d delta = rotatedB + tB - tA;
  residuals[0] = sqrtInformation_ * normalA_.dot(RaT * delta - centerA_);

  const Eigen::RowVector3d deDp = sqrtInformation_ * normalA_.transpose();
  for (int block = 0; block < 2; ++block) {
    if (!(jacobians && jacobians[block]) && !(jacobiansMinimal && jacobiansMinimal[block])) continue;
    Eigen::Matrix<double, 3, 6> dp;
    if (block == 0) {
      dp.leftCols<3>() = -RaT;
      dp.rightCols<3>() = RaT * kinematics::crossMx(delta);
    } else {
      dp.leftCols<3>() = RaT;
      dp.rightCols<3>() = -RaT * kinematics::crossMx(rotatedB);
    }
    const Eigen::Matrix<double, 1, 6> minimal = deDp * dp;
    if (jacobians && jacobians[block]) {
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> lift;
      PoseManifold::minusJacobian(parameters[block], lift.data());
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> mapped(jacobians[block]);
      mapped = minimal * lift;
    }
    if (jacobiansMinimal && jacobiansMinimal[block]) {
      Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> mapped(jacobiansMinimal[block]);
      mapped = minimal;
    }
  }
  return true;
}

} // namespace ceres
} // namespace okvis
