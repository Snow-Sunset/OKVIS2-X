// SPDX-License-Identifier: BSD-3-Clause
#ifndef OKVIS_CERES_POINT_TO_PLANE_ERROR_HPP
#define OKVIS_CERES_POINT_TO_PLANE_ERROR_HPP

#include <ceres/sized_cost_function.h>
#include <okvis/ceres/ErrorInterface.hpp>
#include <Eigen/Core>

namespace okvis {
namespace ceres {

/// A fixed point/plane correspondence between two submap anchor poses.
/// Plane parameters and whitening stay fixed for the lifetime of this factor.
class PointToPlaneError : public ::ceres::SizedCostFunction<1, 7, 7>,
                          public ErrorInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PointToPlaneError(const Eigen::Vector3d& pointB,
                   const Eigen::Vector3d& centerA,
                   const Eigen::Vector3d& normalA, double sigma);
  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override;
  bool EvaluateWithMinimalJacobians(double const* const* parameters,
      double* residuals, double** jacobians, double** jacobiansMinimal) const override;
  int residualDim() const override { return 1; }
  int parameterBlocks() const override { return 2; }
  int parameterBlockDim(int id) const override { return parameter_block_sizes().at(id); }
  std::string typeInfo() const override { return "PointToPlaneError"; }

 private:
  Eigen::Vector3d pointB_, centerA_, normalA_;
  double sqrtInformation_;
};

} // namespace ceres
} // namespace okvis
#endif
