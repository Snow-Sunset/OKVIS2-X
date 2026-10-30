/**
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense 
 * Depth or LiDAR, and GNSS
 *
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich 
 * and ETH Zurich
 *
 * SPDX-License-Identifier: BSD-3-Clause, see LICENESE file for details
 */

#ifndef INCLUDE_OKVIS_LIDARMOTIONUNDISTORTION_HPP
#define INCLUDE_OKVIS_LIDARMOTIONUNDISTORTION_HPP

#include <okvis/PlaneMap.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/ViInterface.hpp>
#include <okvis/mapTypedefs.hpp>
#include <se/map/map.hpp>
#include <okvis/VoxelGridFilter.hpp>

namespace okvis{
    class LidarMotionUndistortion{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        LidarMotionUndistortion() = delete;

        /// \brief Default move Constructor.
        LidarMotionUndistortion(LidarMotionUndistortion && ) = default;

        /// \brief Default copy Constructor.
        LidarMotionUndistortion(const LidarMotionUndistortion & ) = default;

        /// \brief Constructor
        LidarMotionUndistortion(const State& initialState, const kinematics::Transformation& T_WS_live,
                                const kinematics::Transformation& T_SL,
                                const LidarMeasurementDeque& lidarMeasurementDeque,
                                const ImuMeasurementDeque& imuMeasurementDeque);

        /// \brief Function doing motion undistortion based on latest state and imu measurements
        bool deskew();

        /// \brief Function doing voxel hash grid based downsampling
        bool downsample(size_t num_output_points, double voxel_grid_resolution=0.1);

        /// \brief Spatially sample candidates without imposing a random point limit.
        /// Invalid resolutions throw; nonfinite or unindexable points are removed.
        bool voxelDownsample(double voxel_grid_resolution=0.1);

        /// \brief Return reference to downsampled, deskewed point cloud
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& deskewedDownsampledPointCloud(){
          return lidarScanPostProcessed_;
        }

        /// \brief Check which points are actually observed
        /// @param map
        /// @param T_WM
        size_t filterPlanes(PlaneMapPtr map, const kinematics::Transformation& T_WM,
                      double sensorSigma);

  size_t filterObserved(const SupereightMapType* map, const okvis::kinematics::Transformation& T_WM);

        void randomDownsample(size_t num_output_points);

    private:

        State initialState_; /// < T_WS0, sb_0, biases_0, state propagated from
        kinematics::Transformation T_WS_live_; ///< Pose of the frame propagated to
        kinematics::Transformation T_SW_live_; ///< Inverse
        kinematics::Transformation T_SL_; ///< Extrinsics IMU (S) <-> Lidar (L)
        LidarMeasurementDeque lidarScan_;
        Point3Cloud<float> lidarScanPostProcessed_;
        ImuMeasurementDeque imuMeasurementDeque_;

    };
}

#endif //INCLUDE_OKVIS_LIDARMOTIONUNDISTORTION_HPP
