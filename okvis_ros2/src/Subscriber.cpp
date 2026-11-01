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
 * @file Subscriber.cpp
 * @brief Source file for the Subscriber class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <algorithm>
#include <iterator>
#include <stdexcept>

#include <glog/logging.h>
#include <okvis/ros2/Subscriber.hpp>
#include <okvis/ros2/PointCloudUtilities.hpp>

#ifdef OKVIS_USE_NN
#include <okvis/Processor.hpp>
#endif

#define OKVIS_THRESHOLD_SYNC 0.01 ///< Sync threshold in seconds.

/// \brief okvis Main namespace of this package.
namespace okvis {

Subscriber::~Subscriber()
{
  shutdown();
  if (imgTransport_ != nullptr)
    imgTransport_.reset();
}

Subscriber::Subscriber(std::shared_ptr<rclcpp::Node> node, 
                       okvis::ViInterface* viInterfacePtr,
                       okvis::Publisher* publisher, 
                       const okvis::ViParameters& parameters,
                       okvis::SubmappingInterface* seInterface,
                       bool isDepthCamera, bool isLiDAR)
{
  viInterface_ = viInterfacePtr;
  seInterface_ = seInterface;
  publisher_ = publisher;
  parameters_ = parameters;
  setNodeHandle(node, isDepthCamera, isLiDAR);
}

void Subscriber::setNodeHandle(std::shared_ptr<rclcpp::Node> node,
                               bool isDepthCamera, bool isLiDAR)
{
  node_ = node;

  imageSubscribers_.resize(parameters_.nCameraSystem.numCameras());
  depthImageSubscribers_.resize(parameters_.nCameraSystem.numCameras());
  imagesReceived_.resize(parameters_.nCameraSystem.numCameras());
  depthImagesReceived_.resize(parameters_.nCameraSystem.numCameras());

  // set up image reception
  if (imgTransport_ != nullptr)
    imgTransport_.reset();
  imgTransport_ = std::make_shared<image_transport::ImageTransport>(node_);

  // set up callbacks
  for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
    imageSubscribers_[i] = imgTransport_->subscribe(
        "/okvis/cam" + std::to_string(i) +"/image_raw",
        30 * parameters_.nCameraSystem.numCameras(),
        std::bind(&Subscriber::imageCallback, this, std::placeholders::_1, i,
          parameters_.nCameraSystem.cameraType(i).isColour));
  }

  subImu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      "/okvis/imu0", 1000, 
      std::bind(&Subscriber::imuCallback, this, std::placeholders::_1));

  if(isDepthCamera){
    syncDepthImages_ = true;

    // set up callbacks
    for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
      depthImageSubscribers_[i] = imgTransport_->subscribe(
        "/okvis/depth" + std::to_string(i) + "/image_raw",
        30 * parameters_.nCameraSystem.numCameras(),
        std::bind(&Subscriber::depthCallback, this, std::placeholders::_1, i));
    }
  }

  if(isLiDAR){
    if (!node_->has_parameter("lidar_ouster_time_reference"))
      node_->declare_parameter<std::string>("lidar_ouster_time_reference", "scan_start");
    lidarPointTimeReference_ = node_->get_parameter("lidar_ouster_time_reference").as_string();
    if (lidarPointTimeReference_ != "scan_start" && lidarPointTimeReference_ != "scan_end"
        && lidarPointTimeReference_ != "header")
      throw std::invalid_argument("lidar_ouster_time_reference must be scan_start, scan_end or header");
    auto lidarQos = rclcpp::SensorDataQoS();
    lidarQos.keep_last(1);
    subLiDAR_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/okvis/lidar", lidarQos,
      std::bind(&Subscriber::lidarCallback, this, std::placeholders::_1));
    lidarWorker_ = std::thread(&Subscriber::lidarWorkerLoop, this);
    LOG(INFO) << "LiDAR asynchronous batch input: DDS depth 1, one latest waiting scan; "
              << "active scan is retained; Ouster UINT32 t in ns, header reference "
              << lidarPointTimeReference_;
  }
}

void Subscriber::shutdown() {
  if (shutdownRequested_.exchange(true)) return;

  // stop callbacks
  for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
    imageSubscribers_[i].shutdown();
    depthImageSubscribers_[i].shutdown();
  }
  subImu_.reset();
  subLiDAR_.reset();

  {
    std::lock_guard<std::mutex> lock(lidarScanMutex_);
    lidarWorkerStop_ = true;
  }
  lidarScanCv_.notify_all();
  if (lidarWorker_.joinable()) lidarWorker_.join();
}

void Subscriber::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                               unsigned int cameraIndex, bool isColour)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  cv::Mat raw;
  try
  {
    if(!isColour){
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } else {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    raw = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat filtered;
  filtered = raw.clone();

  // adapt timestamp
  okvis::Time t(msg->header.stamp.sec, msg->header.stamp.nanosec);

  // insert
  std::lock_guard<std::mutex> lock(time_mutex_);
  imagesReceived_.at(cameraIndex)[t.toNSec()] = filtered;
  
  // try sync
  synchronizeData();
}

void Subscriber::imuCallback(const sensor_msgs::msg::Imu& msg)
{
  // construct measurement
  okvis::Time timestamp(msg.header.stamp.sec, msg.header.stamp.nanosec);
  Eigen::Vector3d acc(msg.linear_acceleration.x, msg.linear_acceleration.y,
                      msg.linear_acceleration.z);
  Eigen::Vector3d gyr(msg.angular_velocity.x, msg.angular_velocity.y,
                      msg.angular_velocity.z);                    
  
  // forward to estimator
  viInterface_->addImuMeasurement(timestamp, acc, gyr);
  
   // also forward for realtime prediction
  if(seInterface_) {
    seInterface_->realtimePredict(timestamp, acc, gyr);
  }
  else if(publisher_) {
    publisher_->realtimePredictAndPublish(timestamp, acc, gyr);
  }
}

void Subscriber::depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg, 
                               unsigned int cameraIndex){
  // TODO now we work only with one camera of depth images,
  // as it is what the submapping interface accepts
  cv_bridge::CvImageConstPtr cv_ptr;
  cv::Mat raw;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    raw = cv_ptr->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  okvis::Time t(msg->header.stamp.sec, msg->header.stamp.nanosec);

  if(!viInterface_->addDepthMeasurement(t, raw)){
    //LOG(WARNING) << "Dropped last depth image frame for okvis interface";
  }

  // try sync
  std::lock_guard<std::mutex> lock(time_mutex_);
  depthImagesReceived_.at(cameraIndex)[t.toNSec()] = raw;
  //add here the depth images to the receiver
  synchronizeData();
}

void Subscriber::lidarCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
  const auto begin = std::chrono::steady_clock::now();
  lidarScansReceived_.fetch_add(1, std::memory_order_relaxed);
  lidarPointsReceived_.fetch_add(uint64_t(msg->width) * msg->height,
                                std::memory_order_relaxed);

  size_t pending = 0;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr replaced;
  {
    std::lock_guard<std::mutex> lock(lidarScanMutex_);
    if (lidarWorkerStop_) return;
    if (pendingLidarScan_) {
      replaced = std::move(pendingLidarScan_);
      lidarScansDropped_.fetch_add(1, std::memory_order_relaxed);
    }
    pendingLidarScan_ = std::move(msg);
    pending = 1;
  }
  lidarScanCv_.notify_one();
  // Release a potentially large old PointCloud2 outside the worker mutex.
  replaced.reset();

  const auto now = std::chrono::steady_clock::now();
  lidarCallbackMaxMs_ = std::max(lidarCallbackMaxMs_,
      std::chrono::duration<double, std::milli>(now - begin).count());
  if (now - lidarLastReport_ >= std::chrono::seconds(1)) {
    LOG(INFO) << "LiDAR callback totals: received "
              << lidarScansReceived_.load(std::memory_order_relaxed)
              << ", processed " << lidarScansProcessed_.load(std::memory_order_relaxed)
              << ", dropped scans " << lidarScansDropped_.load(std::memory_order_relaxed)
              << ", pending " << pending << "; callback max "
              << lidarCallbackMaxMs_ << " ms in reporting interval";
    lidarCallbackMaxMs_ = 0.0;
    lidarLastReport_ = now;
  }
}

void Subscriber::lidarWorkerLoop() {
  LidarMeasurementBatch lidarBatch;
  std::optional<Time> lastLidarPointTime;
  auto lastReport = std::chrono::steady_clock::now();
  double decodeMaxMs = 0.0;
  double pureDecodeMaxMs = 0.0;
  double estimatorSubmitMaxMs = 0.0;
  double mappingSubmitMaxMs = 0.0;
  double latestScanSpanMs = 0.0;

  while (true) {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
    {
      std::unique_lock<std::mutex> lock(lidarScanMutex_);
      lidarScanCv_.wait(lock, [&] { return lidarWorkerStop_ || bool(pendingLidarScan_); });
      if (!pendingLidarScan_) {
        if (lidarWorkerStop_) break;
        continue;
      }
      msg = std::move(pendingLidarScan_);
    }

    try {
      const auto begin = std::chrono::steady_clock::now();
      auto reference = pointcloud_ros::OusterTimeReference::ScanStart;
      if (lidarPointTimeReference_ == "scan_end") reference = pointcloud_ros::OusterTimeReference::ScanEnd;
      if (lidarPointTimeReference_ == "header") reference = pointcloud_ros::OusterTimeReference::Header;
      const auto decodeBegin = std::chrono::steady_clock::now();
      const auto invalidPoints = pointcloud_ros::lidar2batch(*msg, lidarBatch, reference);
      pureDecodeMaxMs = std::max(pureDecodeMaxMs,
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - decodeBegin).count());
      lidarInvalidPoints_.fetch_add(invalidPoints, std::memory_order_relaxed);
      // Report the decoded scan before trimming overlapping input, so an unusual
      // point-time span remains visible rather than being hidden by the watermark.
      latestScanSpanMs = lidarBatch.empty() ? 0.0
          : (lidarBatch.back().timeStamp - lidarBatch.front().timeStamp).toSec() * 1000.0;

      // Both consumers require a monotonic FIFO. Account for overlapping/replayed
      // timestamps explicitly instead of hiding an older point behind a newer one.
      if (lastLidarPointTime) {
        const auto first = std::lower_bound(lidarBatch.begin(), lidarBatch.end(), *lastLidarPointTime,
            [](const LidarMeasurement& measurement, const Time& time) {
              return measurement.timeStamp < time;
            });
        const auto count = size_t(std::distance(lidarBatch.begin(), first));
        lidarRegressingPoints_.fetch_add(count, std::memory_order_relaxed);
        if (count) {
          LOG_EVERY_N(WARNING, 40) << "Discarded " << count
                                 << " LiDAR points with timestamps preceding the previous scan tail";
          lidarBatch.erase(lidarBatch.begin(), first);
        }
      }
      if (!lidarBatch.empty()) {
        const Time scanTail = lidarBatch.back().timeStamp;
        // A decoded scan is never retried. Advance the input watermark before
        // submitting so an exception in one consumer cannot make the other
        // consumer receive an overlapping prefix again on the next scan.
        lastLidarPointTime = scanTail;

        // Deliver live-factor input before mapping filtering or optional spool I/O.
        // Each consumer still receives the same monotonic scan independently.
        const auto estimatorBegin = std::chrono::steady_clock::now();
        try {
          if (!viInterface_->addLidarMeasurements(lidarBatch))
            lidarEstimatorRejectedBatches_.fetch_add(1, std::memory_order_relaxed);
        } catch (const std::exception& error) {
          lidarEstimatorRejectedBatches_.fetch_add(1, std::memory_order_relaxed);
          LOG(ERROR) << "Estimator rejected LiDAR scan with exception: " << error.what();
        } catch (...) {
          lidarEstimatorRejectedBatches_.fetch_add(1, std::memory_order_relaxed);
          LOG(ERROR) << "Estimator rejected LiDAR scan with unknown exception";
        }
        estimatorSubmitMaxMs = std::max(estimatorSubmitMaxMs,
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - estimatorBegin).count());
        if (seInterface_) {
          const auto mappingBegin = std::chrono::steady_clock::now();
          try {
            if (!seInterface_->addLidarMeasurements(lidarBatch))
              lidarSubmappingRejectedBatches_.fetch_add(1, std::memory_order_relaxed);
          } catch (const std::exception& error) {
            lidarSubmappingRejectedBatches_.fetch_add(1, std::memory_order_relaxed);
            LOG(ERROR) << "Submapping rejected LiDAR scan with exception: " << error.what();
          } catch (...) {
            lidarSubmappingRejectedBatches_.fetch_add(1, std::memory_order_relaxed);
            LOG(ERROR) << "Submapping rejected LiDAR scan with unknown exception";
          }
          mappingSubmitMaxMs = std::max(mappingSubmitMaxMs,
              std::chrono::duration<double, std::milli>(
                  std::chrono::steady_clock::now() - mappingBegin).count());
        }
      }
      lidarScansProcessed_.fetch_add(1, std::memory_order_relaxed);

      const auto now = std::chrono::steady_clock::now();
      decodeMaxMs = std::max(decodeMaxMs,
          std::chrono::duration<double, std::milli>(now - begin).count());
      if (now - lastReport >= std::chrono::seconds(1)) {
        size_t pending = 0;
        {
          std::lock_guard<std::mutex> lock(lidarScanMutex_);
          pending = pendingLidarScan_ ? 1 : 0;
        }
        LOG(INFO) << "LiDAR worker totals: scans "
                  << lidarScansProcessed_.load(std::memory_order_relaxed)
                  << ", points " << lidarPointsReceived_.load(std::memory_order_relaxed)
                  << ", invalid " << lidarInvalidPoints_.load(std::memory_order_relaxed)
                  << ", time-regressing " << lidarRegressingPoints_.load(std::memory_order_relaxed)
                  << ", estimator rejected batches "
                  << lidarEstimatorRejectedBatches_.load(std::memory_order_relaxed)
                  << ", submapping rejected batches "
                  << lidarSubmappingRejectedBatches_.load(std::memory_order_relaxed)
                  << ", pending " << pending << "; decode/submit max "
                  << decodeMaxMs << " ms, decode only max " << pureDecodeMaxMs
                  << " ms, estimator submit max " << estimatorSubmitMaxMs
                  << " ms, mapping submit max " << mappingSubmitMaxMs
                  << " ms in reporting interval; latest scan span " << latestScanSpanMs
                  << " ms, Ouster header reference " << lidarPointTimeReference_;
        decodeMaxMs = 0.0;
        pureDecodeMaxMs = 0.0;
        estimatorSubmitMaxMs = 0.0;
        mappingSubmitMaxMs = 0.0;
        lastReport = now;
      }
    } catch (const std::exception& error) {
      LOG(ERROR) << "Failed to process LiDAR scan: " << error.what();
    } catch (...) {
      LOG(ERROR) << "Failed to process LiDAR scan with unknown exception";
    }
  }
}

void Subscriber::synchronizeData() {
  std::set<uint64_t> allTimes;
  const int numCameras = imagesReceived_.size();
  for(int i=0; i < numCameras; ++i) {
    if(!parameters_.nCameraSystem.cameraType(i).isUsed) continue; // Only consider SLAM cameras here
    for(const auto & entry : imagesReceived_.at(i)) {
      allTimes.insert(entry.first);
    }
  } //We are synchronizing the depth cameras against the ir cameras

  for(const auto & time : allTimes) {
    // note: ordered old to new
    std::vector<uint64_t> syncedTimes(numCameras, 0);
    uint64_t depthSyncedTime; //Assume that there is only one depth image (currently supported by the submapping interface)
    std::map<uint64_t, cv::Mat> images;
    std::map<uint64_t, cv::Mat> depthImages;

    std::map<uint64_t, std::pair<okvis::Time, cv::Mat>> timestampedImages;
    std::map<uint64_t, std::pair<okvis::Time, cv::Mat>> timestampedDepthImages;

    okvis::Time tcheck;
    tcheck.fromNSec(time);
    bool synced = true;
    for(int i=0; i < numCameras; ++i) {
      bool syncedi = false;
      for(const auto & entry : imagesReceived_.at(i)) {
        okvis::Time ti;
        ti.fromNSec(entry.first);
        bool slam_use = !parameters_.nCameraSystem.cameraType(i).isUsed;
        if(fabs((tcheck-ti).toSec()) < OKVIS_THRESHOLD_SYNC && (tcheck >= ti || slam_use)) {
          syncedTimes.at(i) = entry.first;
          images[i] = imagesReceived_.at(i).at(entry.first);
          timestampedImages[i] = std::make_pair(ti, imagesReceived_.at(i).at(entry.first));
          syncedi = true;
          break;
        } 
      }

      if(!syncedi) {
        synced = false;
        break;
      }
    }

    bool syncedDepth = true;
    if(syncDepthImages_){
      syncedDepth = false;
      for(int i = 0; i < numCameras; ++i) {
        for(const auto & entry : depthImagesReceived_.at(i)){
          okvis::Time tdepth;
          tdepth.fromNSec(entry.first);
          //There is a higher unsynchronization between depth images and ir images from the realsense
          if(std::abs((tcheck - tdepth).toSec()) < OKVIS_THRESHOLD_SYNC && tcheck>=tdepth) {
            depthSyncedTime = entry.first;
            depthImages[i] = depthImagesReceived_.at(i).at(entry.first);
            timestampedDepthImages[i] = std::make_pair(tdepth, depthImagesReceived_.at(i).at(entry.first));
            syncedDepth = true;
            break;
          }
        }

        if(syncedDepth){
          //For now we assume there is only one depth image per synchronization process, needs discussion
          break;
        }
      }
    }

    if(synced && syncedDepth) {
      bool isProcessor = false;
      #ifdef OKVIS_USE_NN
      okvis::Processor* casted_processor = dynamic_cast<okvis::Processor*>(viInterface_);
      if (casted_processor) {
        isProcessor = true;
        if(!casted_processor->addImages(timestampedImages, timestampedDepthImages)) {
          LOG(WARNING) << "Frame not added to Processor at t="<< timestampedImages.at(0).first;
        }
      }
      else if(!viInterface_->addImages(tcheck, images, depthImages)) {
        LOG(WARNING) << "Frame not added at t="<< tcheck;
      }
      #else
      if(!viInterface_->addImages(tcheck, images, depthImages)) {
        LOG(WARNING) << "Frame not added at t="<< tcheck;
      }
      #endif
      if(!isProcessor && syncDepthImages_) {
        //If we have depth images, then we send the data to SI from here, if not the Processor handles it. As for LiDAR, we need to decide
        //how to continue with it. Do we want colour fusion at all?
        std::map<size_t, std::vector<okvis::CameraMeasurement>> cameraMeasurements;
        
        for(const auto& it : timestampedImages) {
          okvis::CameraMeasurement cameraMeasure;
          cameraMeasure.timeStamp = tcheck;
          cameraMeasure.measurement.image = it.second.second.clone();
          cameraMeasure.sensorId = it.first;
          cameraMeasurements[it.first] = {cameraMeasure};
        }

        for(const auto& it : timestampedDepthImages) {
          if(cameraMeasurements.find(it.first) != cameraMeasurements.end()) {
            //Check if they are time synchronized or not
            bool imageAdded = false;
            for(auto& image : cameraMeasurements.at(it.first)) {
              if(image.timeStamp == it.second.first){
                image.measurement.depthImage = it.second.second.clone();
                imageAdded = true;
              }
            }

            if(!imageAdded) {
              okvis::CameraMeasurement cameraMeasure;
              cameraMeasure.timeStamp = it.second.first;
              cameraMeasure.measurement.depthImage = it.second.second.clone();
              cameraMeasure.sensorId = it.first;
              cameraMeasurements[it.first] = {cameraMeasure};
            }
          } else {
            okvis::CameraMeasurement cameraMeasure;
            cameraMeasure.timeStamp = it.second.first;
            cameraMeasure.measurement.depthImage = it.second.second.clone();
            cameraMeasure.sensorId = it.first;
            cameraMeasurements[it.first] = {cameraMeasure};
          }
        }
        
        if(!seInterface_->addDepthMeasurement(cameraMeasurements)){
          LOG(WARNING) << "Frame not added to SI at t="<< cameraMeasurements.at(0)[0].timeStamp;
        }
      }

      // remove all the older stuff from buffer
      for(int i=0; i < numCameras; ++i) {
        auto end = imagesReceived_.at(i).find(syncedTimes.at(i));
        if(end!=imagesReceived_.at(i).end()) {
          ++end;
        }
        imagesReceived_.at(i).erase(imagesReceived_.at(i).begin(), end);
      }

      if(syncDepthImages_) {
        for(int i=0; i < numCameras; ++i) {
          auto end = std::find_if(depthImagesReceived_.at(i).begin(), depthImagesReceived_.at(i).end(),
                                 [depthSyncedTime](const auto& x) { return x.first > depthSyncedTime; });
          depthImagesReceived_.at(i).erase(depthImagesReceived_.at(i).begin(), end);
        }
      }
    }
  }
}

} // namespace okvis
