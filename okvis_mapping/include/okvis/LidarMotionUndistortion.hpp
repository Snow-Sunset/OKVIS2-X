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
#include <memory>

namespace okvis{
    class LidarMotionUndistortion{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        LidarMotionUndistortion() = delete;

        /// \brief Default move Constructor.
        LidarMotionUndistortion(LidarMotionUndistortion && ) = default;

        /// \brief Default copy Constructor.
        LidarMotionUndistortion(const LidarMotionUndistortion & ) = default;

        /// \brief Borrow input measurements; lvalues must outlive this object.
        LidarMotionUndistortion(const State& initialState, const kinematics::Transformation& T_WS_live,
                                const kinematics::Transformation& T_SL,
                                const LidarMeasurementDeque& lidarMeasurementDeque,
                                const ImuMeasurementDeque& imuMeasurementDeque);

        /// \brief Own temporary inputs while continuing to borrow any lvalue input.
        LidarMotionUndistortion(const State& initialState, const kinematics::Transformation& T_WS_live,
                                const kinematics::Transformation& T_SL,
                                LidarMeasurementDeque&& lidarMeasurementDeque,
                                const ImuMeasurementDeque& imuMeasurementDeque);
        LidarMotionUndistortion(const State& initialState, const kinematics::Transformation& T_WS_live,
                                const kinematics::Transformation& T_SL,
                                const LidarMeasurementDeque& lidarMeasurementDeque,
                                ImuMeasurementDeque&& imuMeasurementDeque);
        LidarMotionUndistortion(const State& initialState, const kinematics::Transformation& T_WS_live,
                                const kinematics::Transformation& T_SL,
                                LidarMeasurementDeque&& lidarMeasurementDeque,
                                ImuMeasurementDeque&& imuMeasurementDeque);

        /// \brief Motion undistortion based on the latest state and IMU measurements.
        /// Clears prior output; unsupported propagation returns false with empty output.
        bool deskew();

        /// \brief Function doing voxel hash grid based downsampling
        bool downsample(size_t num_output_points, double voxel_grid_resolution=0.1);

        /// \brief Spatially sample candidates without imposing a random point limit.
        /// Invalid resolutions throw; nonfinite or unindexable points are removed.
        bool voxelDownsample(double voxel_grid_resolution=0.1);

        /// \brief Limit spatially sampled plane candidates using deterministic voxel priorities.
        /// Call after voxelDownsample(). Zero keeps all candidates. Increasing the
        /// budget on the same original cloud retains the previously selected set.
        /// This does not query the map or change occupancy-path sampling.
        void selectPlaneCandidates(size_t maxCandidates, double voxelResolution=0.1);

        /// \brief Return reference to downsampled, deskewed point cloud
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& deskewedDownsampledPointCloud(){
          return lidarScanPostProcessed_;
        }

        /// \brief Check which points are actually observed
        /// @param map
        /// @param T_WM
        size_t filterPlanes(PlaneMapPtr map, const kinematics::Transformation& T_WM,
                      double sensorSigma);

        struct PlaneFilterStatistics {
          size_t initial = 0, maximum = 0, queried = 0;
          bool expanded = false;
        };
        /// Query a deterministic spatial subset, expanding only on weak overlap.
        /// Each candidate is queried at most once. Output preserves source order.
        /// Call after voxelDownsample(); zero maximum means unlimited.
        size_t filterPlaneCandidates(PlaneMapPtr map, const kinematics::Transformation& T_WM,
            double sensorSigma, size_t initialCandidates, size_t maxCandidates,
            size_t minMatches, double voxelResolution,
            PlaneFilterStatistics* statistics = nullptr);

  size_t filterObserved(const SupereightMapType* map, const okvis::kinematics::Transformation& T_WM);

        void randomDownsample(size_t num_output_points);

    private:

        State initialState_; /// < T_WS0, sb_0, biases_0, state propagated from
        kinematics::Transformation T_WS_live_; ///< Pose of the frame propagated to
        kinematics::Transformation T_SW_live_; ///< Inverse
        kinematics::Transformation T_SL_; ///< Extrinsics IMU (S) <-> Lidar (L)
        std::shared_ptr<const LidarMeasurementDeque> ownedLidarScan_;
        std::shared_ptr<const ImuMeasurementDeque> ownedImuMeasurements_;
        const LidarMeasurementDeque* lidarScan_;
        Point3Cloud<float> lidarScanPostProcessed_;
        const ImuMeasurementDeque* imuMeasurementDeque_;

    };
}

#endif //INCLUDE_OKVIS_LIDARMOTIONUNDISTORTION_HPP
