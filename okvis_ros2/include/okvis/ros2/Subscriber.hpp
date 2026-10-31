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

/**
 * @file Subscriber.hpp
 * @brief Header file for the Subscriber class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_SUBSCRIBER_HPP_
#define INCLUDE_OKVIS_SUBSCRIBER_HPP_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

#include <boost/shared_ptr.hpp>
#include <chrono>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>) // requires GCC >= 5
  #include <cv_bridge/cv_bridge.hpp>
#else
  #include <cv_bridge/cv_bridge.h> // ros2 changed to .hpp some point...
#endif
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <okvis/Time.hpp>
#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/ThreadedSlam.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/ViParametersReader.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/ros2/Publisher.hpp>
#include <okvis/SubmappingInterface.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * @brief This class handles all the buffering of incoming data.
 */
class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// @brief Destructor (trivial).
  ~Subscriber();

  /**
   * @brief Constructor. This will either subscribe to the relevant ROS topics or
   *        start up the sensor and register the callbacks directly there.
   * @param node The ROS node.
   * @param viInterfacePtr Pointer to the ViInterface.
   * @param publisher  Pointer to publisher
   * @param se_interface Pointer to the supereight interface
   * @param parameters Parameters for the ViInterface.
   * @param isDepthCamera Flag to indicate if depth camera is used
   * @param isLiDAR Flag to indicate if LiDAR is used
   */
  Subscriber(std::shared_ptr<rclcpp::Node> node, okvis::ViInterface* viInterfacePtr,
             okvis::Publisher* publisher,
             const okvis::ViParameters& parameters,
             okvis::SubmappingInterface* se_interface = nullptr,
             bool isDepthCamera = false, bool isLiDAR = false);

  /// @brief Set the node handle. This sets up the callbacks. This is called in the constructor.
  void setNodeHandle(std::shared_ptr<rclcpp::Node> node, bool isDepthCamera = false, bool isLiDAR = false);
    
  /// @brief Stop subscriptions and join the LiDAR worker.
  /// The owning ROS executor must be stopped first so no image/IMU callback is
  /// still using the interfaces while this object is being torn down.
  void shutdown();

 protected:
  /// @name ROS callbacks
  /// @{

  /// @brief The image callback.
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                     unsigned int cameraIndex, bool isColour = false);

  /// @brief The IMU callback.
  void imuCallback(const sensor_msgs::msg::Imu& msg);

  /// @brief The depth image callback
  /// @param msg the depth image ROS message
  /// @param cameraIndex the index of the depth cmera, which will make it associated to one of the stereo cameras
  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg, unsigned int cameraIndex);

  /// @brief The lidar sensor callback
  /// @param msg the lidar sensor ROS message
  void lidarCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  /// @brief Decode and submit queued LiDAR scans outside the ROS executor.
  void lidarWorkerLoop();

  /// @brief function that performs the synchronization of the different ir and depth images for the slam system
  void synchronizeData();

  /// @}
  /// @name Node and subscriber related
  /// @{
  std::shared_ptr<rclcpp::Node> node_; ///< The node handle.
  std::shared_ptr<image_transport::ImageTransport> imgTransport_; ///< The image transport.
  std::vector<image_transport::Subscriber> imageSubscribers_; ///< The image message subscriber.
  std::vector<image_transport::Subscriber> depthImageSubscribers_; ///< The depth image message subscriber
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;  ///< The IMU message subscriber.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLiDAR_;  ///< The LiDAR message subscriber.
  std::mutex time_mutex_; ///< Lock when accessing time

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gtPoses_;
  rclcpp::Subscription<geometry_msgs::msg::Transform>::SharedPtr subGoal_;
  rclcpp::Subscription<geometry_msgs::msg::Transform>::SharedPtr subStartPlannerPose_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subExploration_;
  uint16_t im_counter_ = 0;

  /// @}
  
  okvis::ViInterface* viInterface_ = nullptr;   ///< The VioInterface. (E.g. ThreadedSlam).
  okvis::SubmappingInterface* seInterface_ = nullptr; ///< The interface with SuperEight2
  okvis::Publisher* publisher_ = nullptr;  ///< Publisher for IMU propagation.
  okvis::ViParameters parameters_;  ///< The parameters and settings.
  
  std::vector<std::map<uint64_t, cv::Mat>> imagesReceived_; ///< Images obtained&buffered (to sync).
  std::vector<std::map<uint64_t, cv::Mat>> depthImagesReceived_; ///> The depth images obtained and buffered (to sync)
  bool syncDepthImages_ = false;
  std::string lidarPointTimeReference_ = "scan_start";
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> lidarScanQueue_;
  std::mutex lidarScanMutex_;
  std::condition_variable lidarScanCv_;
  std::thread lidarWorker_;
  size_t lidarWorkerQueueDepth_ = 3;
  bool lidarWorkerStop_ = false; ///< Guarded by lidarScanMutex_.
  std::atomic_bool shutdownRequested_{false};
  std::atomic<uint64_t> lidarScansReceived_{0};
  std::atomic<uint64_t> lidarScansProcessed_{0};
  std::atomic<uint64_t> lidarScansDropped_{0};
  std::atomic<uint64_t> lidarPointsReceived_{0};
  std::atomic<uint64_t> lidarInvalidPoints_{0};
  std::atomic<uint64_t> lidarRegressingPoints_{0};
  std::atomic<uint64_t> lidarEstimatorRejectedBatches_{0};
  std::atomic<uint64_t> lidarSubmappingRejectedBatches_{0};
  double lidarCallbackMaxMs_ = 0.0;
  std::chrono::steady_clock::time_point lidarLastReport_ = std::chrono::steady_clock::now();
};
}

#endif /* INCLUDE_OKVIS_SUBSCRIBER_HPP_ */
