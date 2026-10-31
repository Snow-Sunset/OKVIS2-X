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
 * @file ThreadedSlam.cpp
 * @brief Source file for the ThreadedSlam3 class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <chrono>
#include <map>

#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <glog/logging.h>

#include <okvis/ThreadedSlam.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/ceres/ImuError.hpp>
#include <okvis/LidarMotionUndistortion.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

/// \brief okvis Main namespace of this package.
namespace okvis
{

static const int cameraInputQueueSize = 2;

// overlap of imu data before and after two consecutive frames [seconds]:
static const double imuTemporalOverlap = 0.02;

// Constructor.
ThreadedSlam::ThreadedSlam(ViParameters &parameters, std::string dBowDir, se::SubMapConfig subMapConfig) :
  visualizer_(parameters),
  hasStarted_(false),
  frontend_(parameters.nCameraSystem.numCameras(), dBowDir),
  parameters_(parameters),
  submapConfig_(subMapConfig),
  useAlignmentFactors_(submapConfig_.useMap2LiveFactors)
{
  setBlocking(false);
  init();
  
  ///// HACK: multi-session and multi-agent //////
  //frontend_.loadComponent(
  //  "/Users/leuteneg/Documents/datasets/euroc/V2_03_difficult/mav0/okvis2-slam-final_map.g2o",
  //  parameters_.imu, parameters_.nCameraSystem);
  ////////////////////////////////////////////////
}

// Initialises settings and calls startThreads().
void ThreadedSlam::init()
{
  assert(parameters_.nCameraSystem.numCameras() > 0);
  const size_t numCameras = parameters_.nCameraSystem.numCameras();
  shutdown_ = false;

  // setup frontend
  frontend_.setBriskDetectionOctaves(size_t(parameters_.frontend.octaves));
  frontend_.setBriskDetectionThreshold(parameters_.frontend.detection_threshold);
  frontend_.setBriskDetectionAbsoluteThreshold(parameters_.frontend.absolute_threshold);
  frontend_.setBriskMatchingThreshold(parameters_.frontend.matching_threshold);
  frontend_.setBriskDetectionMaximumKeypoints(size_t(parameters_.frontend.max_num_keypoints));
  frontend_.setKeyframeInsertionOverlapThreshold(float(parameters_.frontend.keyframe_overlap));

  if (parameters_.estimator.memory_bounded) {
    estimator_.configureInMemoryHistory();
    LOG(INFO) << "SLAM history: RAM only; cold images are released, loop-closure features "
                 "and graph observations remain in memory; disk archive disabled.";
  }

  // setup estimator
  estimator_.addImu(parameters_.imu);
  for (size_t im = 0; im < numCameras; ++im) {
    // parameters_.camera_extrinsics is never set (default 0's)...
    // do they ever change?
    estimator_.addCamera(parameters_.camera);
  }
  if(parameters_.gps){
    estimator_.addGps(*parameters_.gps);
  }
  estimator_.setDetectorUniformityRadius(parameters_.frontend.detection_threshold);

  // time limit if requested
  if(parameters_.estimator.enforce_realtime) {
    estimator_.setOptimisationTimeLimit(
          parameters_.estimator.realtime_time_limit,
          parameters_.estimator.realtime_min_iterations);
  }

  // Set calibration parameters for live-to-map factor
  // TODO: generalize to n > 1 mapping camera
  if (parameters_.lidar) submapConfig_.validateLidarMapping();
  if(parameters_.output.enable_submapping) {
    if(parameters_.lidar){
      T_SD_ = parameters_.lidar.value().T_SL;
    }
    else{
      std::shared_ptr<const cameras::CameraBase> camera;
      kinematics::Transformation T_SD;
      bool mapping_camera_found = false;
      for(size_t i = 0; i < parameters_.nCameraSystem.numCameras(); i++){
        if(parameters_.nCameraSystem.cameraType(i).isUsedMapping) {
          mapping_camera_found = true;
          if (!parameters_.nCameraSystem.cameraType(i).depthType.needRectify) {
            camera = parameters_.nCameraSystem.cameraGeometry(i);
            T_SD = *parameters_.nCameraSystem.T_SC(i);
          } else {
            camera = parameters_.nCameraSystem.rectifyCameraGeometry(i);
            T_SD = *parameters_.nCameraSystem.rectifyT_SC(i);
            T_rect_ = parameters_.nCameraSystem.T_SC(i)->inverse() * T_SD;
          }
          break;
        }
      }
      if(!mapping_camera_found){
        LOG(WARNING) << "Neither LiDAR nor camera for mapping specified! Running in VI mode without depth." \
        "If you want to run without depth specify enabling_submapping: false in the okvis_config. If you want to use" \
        "the LiDAR, please specify a lidar section in the okvis_config. If you wanted to use depth from cameras " \
        ", please specify mapping: true for the camera from which depth will be integrated.";
        useAlignmentFactors_ = false;
      } else {
        se::PinholeCamera::Config depthCameraConfig;
        Eigen::VectorXd intrinsics;
        camera->getIntrinsics(intrinsics);
        depthCameraConfig.fx = intrinsics(0);
        depthCameraConfig.fy = intrinsics(1);
        depthCameraConfig.cx = intrinsics(2);
        depthCameraConfig.cy = intrinsics(3);
        depthCameraConfig.T_BS = T_SD.T().cast<float>();
        depthCameraConfig.width = camera->imageWidth();
        depthCameraConfig.height = camera->imageHeight();
        depthCameraConfig.near_plane = submapConfig_.near_plane;
        depthCameraConfig.far_plane = submapConfig_.far_plane;
        depthCamera_.reset(new se::PinholeCamera(depthCameraConfig, submapConfig_.depthImageResDownsampling));
        T_SD_ = T_SD;
      }
    }
  } else if(useAlignmentFactors_){
    useAlignmentFactors_ = false;
    LOG(WARNING) << "Alignment factors have been set in the se2_config but the enable_submapping in the okvis_config is set to false." \
    " Due to this inconsistency you will be running VI mode no submap constraints.";
  }

  if(useAlignmentFactors_ && parameters_.output.enable_submapping){
    LOG(INFO) << "[Info SLAM] SLAM will be using Submap Alignment Constraints.";
  }

  startThreads();
}

// Start all threads.
void ThreadedSlam::startThreads()
{

  // visualisation
  if(parameters_.output.display_matches) {
    visualisationThread_ = std::thread(&ThreadedSlam::visualisationLoop, this);
  }

  //setMainThreadPriority(SCHED_RR, 99);

  // publishing
  publishingThread_ = std::thread(&ThreadedSlam::publishingLoop, this);

}

// Destructor. This calls Shutdown() for all threadsafe queues and joins all threads.
ThreadedSlam::~ThreadedSlam()
{

  // shutdown and join threads
  stopThreading();
}

// Add a new image.
bool ThreadedSlam::addImages(const okvis::Time & stamp,
                             const std::map<size_t, cv::Mat> & images,
                             const std::map<size_t, cv::Mat> & depthImages)
{
  // remove image delay:
  // timestamp_camera_correct = timestamp_camera - image_delay
  const Time stampCorrected = stamp - Duration(parameters_.camera.image_delay);

  // assemble frame
  const size_t numCameras = parameters_.nCameraSystem.numCameras();
  std::vector<okvis::CameraMeasurement> frames(numCameras);
  frames.at(0).timeStamp = stampCorrected; // slight hack -- always have the timestamp here.

  bool useFrame = false;
  for(const auto & image : images) {
    if(parameters_.nCameraSystem.isCameraConfigured(image.first) && 
       parameters_.nCameraSystem.cameraType(image.first).isUsed) {
      if(image.second.channels()==1) {
        frames.at(image.first).measurement.image = image.second;
      } else {
        cv::cvtColor(image.second, frames.at(image.first).measurement.image, cv::COLOR_BGR2GRAY);
      }
      frames.at(image.first).timeStamp = stampCorrected;
      frames.at(image.first).sensorId = image.first;
      frames.at(image.first).measurement.deliversKeypoints = false;
      useFrame = true;
    }
  }

  for(const auto & depthImage : depthImages) {
    if(parameters_.nCameraSystem.cameraType(depthImage.first).isUsed) {
      frames.at(depthImage.first).measurement.depthImage = depthImage.second;
      frames.at(depthImage.first).timeStamp = stampCorrected;
      frames.at(depthImage.first).sensorId = depthImage.first;
      frames.at(depthImage.first).measurement.deliversKeypoints = false;
    }
  }

  if(!useFrame) {
    return true; // the frame is not used, so quitely ignore.
  }

  if (blocking_)
  {
    return cameraMeasurementsReceived_.PushBlockingIfFull(frames,1);
  }
  else
  {
    if(cameraMeasurementsReceived_.PushNonBlockingDroppingIfFull(frames, cameraInputQueueSize)) {
      DLOG(WARNING) << "frame drop ";
      return false;
    }
    return true;
  }
}

// Add an IMU measurement.
bool ThreadedSlam::addImuMeasurement(const okvis::Time& stamp,
                                     const Eigen::Vector3d& alpha,
                                     const Eigen::Vector3d& omega)
{
  static bool warnOnce = true;
  if (!parameters_.imu.use) {
    if (warnOnce) {
      LOG(WARNING) << "imu measurement added, but IMU disabled";
      warnOnce = false;
    }
    return false;
  }
  okvis::ImuMeasurement imu_measurement;
  imu_measurement.measurement.accelerometers.x() = parameters_.imu.s_a.x() * alpha.x();
  imu_measurement.measurement.accelerometers.y() = parameters_.imu.s_a.y() * alpha.y();
  imu_measurement.measurement.accelerometers.z() = parameters_.imu.s_a.z() * alpha.z();
  imu_measurement.measurement.gyroscopes = omega;
  imu_measurement.timeStamp = stamp;

  const int imuQueueSize = 5000;

  if (blocking_)
  {
    return imuMeasurementsReceived_.PushBlockingIfFull(imu_measurement, size_t(imuQueueSize));
  }
  else
  {
    if(imuMeasurementsReceived_.PushNonBlockingDroppingIfFull(
         imu_measurement, size_t(imuQueueSize))) {
      LOG(WARNING) << "imu measurement drop ";
      return false;
    }
    return true;
  }

}

// Add a LiDAR measurement.
bool ThreadedSlam::addLidarMeasurement(const okvis::Time &stamp,
                                       const Eigen::Vector3d &rayMeasurement)
{
  if(!useAlignmentFactors_){
    return false;
  }

  okvis::LidarMeasurement lidarMeasurement;
  lidarMeasurement.measurement.rayMeasurement = rayMeasurement;
  lidarMeasurement.measurement.intensity = 0;
  lidarMeasurement.timeStamp = stamp;

  const int lidarQueueSize = 500000;
  sensorMeasurementDownsamplingCounter_ ++;
  bool drop = false;
  if(sensorMeasurementDownsamplingCounter_ % submapConfig_.sensorMeasurementDownsampling == 0){
    if (blocking_)
    {
      return lidarMeasurementsReceived_.PushBlockingIfFull(lidarMeasurement, size_t(lidarQueueSize));
    }
    else
    {
      drop = lidarMeasurementsReceived_.PushNonBlockingDroppingIfFull(
              lidarMeasurement, size_t(lidarQueueSize));
      if(drop)
       DLOG(WARNING) << "lidar measurement drop ";
    }
    return drop;
  }
  return true;
}

bool ThreadedSlam::addDepthMeasurement(const okvis::Time &stamp,
                                       const cv::Mat &depthImage,
                                       const std::optional<cv::Mat> &sigmaImage) {
  CameraMeasurement depthMeasurement;
  
  // This assumes the image_delay is also applied to the depth camera.
  // That is valid for most of RGB-D cameras and network depths.
  // But, caution is needed if a depth camera has a different timestamp delay than gray (or RGB) camera.
  depthMeasurement.timeStamp = stamp - Duration(parameters_.camera.image_delay);
  depthMeasurement.measurement.depthImage = submapConfig_.depthScalingFactor*depthImage;
  if (sigmaImage) {
    depthMeasurement.measurement.sigmaImage = sigmaImage.value();
  }

  // We want to buffer 4 depth images to give OKVIS to have time to compute the states
  const size_t depthQueueSize = 4;
  sensorMeasurementDownsamplingCounter_ ++;
  bool drop = false;
  if(sensorMeasurementDownsamplingCounter_ % submapConfig_.sensorMeasurementDownsampling == 0){
    sensorMeasurementDownsamplingCounter_ = 0;
    if (blocking_) {
      return depthMeasurementsReceived_.PushBlockingIfFull(depthMeasurement, depthQueueSize);
    }
    else {
      drop = depthMeasurementsReceived_.PushNonBlockingDroppingIfFull(depthMeasurement, depthQueueSize);
      if (drop) 
        LOG(WARNING) << "Depth measurement drop ";
    }
    return drop;
  }
  return true;
}

// Add a GPS measurement.
bool ThreadedSlam::addGpsMeasurement(const okvis::Time& stamp,
                                     const Eigen::Vector3d& pos,
                                     const Eigen::Vector3d& err)
{
  okvis::GpsMeasurement  gps_measurement;
  okvis::GpsSensorReadings gpsReading(pos, err(0), err(1), err(2));
  gps_measurement.timeStamp = stamp;
  gps_measurement.measurement = gpsReading;
  const int gpsQueueSize = 5000;

  if (blocking_)
  {
    return gpsMeasurementsReceived_.PushBlockingIfFull(gps_measurement, size_t(gpsQueueSize));
  }
  else
  {
    if(gpsMeasurementsReceived_.PushNonBlockingDroppingIfFull(gps_measurement, size_t(gpsQueueSize))) {
      LOG(WARNING) << "gps measurement drop ";
      return false;
    }

    return true;
  }

}

// Add a GPS measurement (geodetic input).
bool ThreadedSlam::addGeodeticGpsMeasurement(const okvis::Time& stamp,
                                             double lat, double lon, double height,
                                             double hAcc, double vAcc)
{

  okvis::GpsMeasurement  gps_measurement;
  okvis::GpsSensorReadings gpsReading(lat, lon, height, hAcc, vAcc);
  gps_measurement.timeStamp = stamp;
  gps_measurement.measurement = gpsReading;
  const int gpsQueueSize = 5000;

  if (blocking_)
  {
    return gpsMeasurementsReceived_.PushBlockingIfFull(gps_measurement, size_t(gpsQueueSize));
  }
  else
  {
    if(gpsMeasurementsReceived_.PushNonBlockingDroppingIfFull(gps_measurement, size_t(gpsQueueSize))) {
      LOG(WARNING) << "gps measurement drop ";
      return false;
    }

    return true;
  }

}

// Add Submap alignment constraints to estimator
bool ThreadedSlam::addSubmapAlignmentConstraints(const SupereightMapType* submap_A_ptr,
                                                 const SupereightMapType* submap_B_ptr,
                                                 const uint64_t& frame_A_id, const uint64_t& frame_B_id,
                                                 std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& pointCloud,
                                                 std::vector<float> sensorError,
                                                 PlaneMapPtr planeMapA, PlaneMapPtr planeMapB) {

  const AlignmentTerm term(submap_A_ptr, submap_B_ptr, frame_A_id, frame_B_id,
                           pointCloud, sensorError, planeMapA, planeMapB);
  if (parameters_.estimator.memory_bounded) {
    if (shutdown_) return false;
    if (!submapAlignmentFactorsReceived_.PushNonBlockingIfNotFull(term, 4)) {
      if (submapAlignmentFactorsReceived_.shutdown_) return false;
      LOG(ERROR) << "Submap alignment queue exceeded 4 batches. Stop processing and slow dataset replay; "
                    "historical alignment factors have not been discarded.";
      throw std::runtime_error("Bounded submap alignment queue overflow");
    }
  } else {
    submapAlignmentFactorsReceived_.PushNonBlocking(term);
  }
  return true;
}

// Set the blocking variable that indicates whether the addMeasurement() functions
// should return immediately (blocking=false), or only when the processing is complete.
void ThreadedSlam::setBlocking(bool blocking)
{
  blocking_ = blocking;
  // disable time limit for optimization
  if(blocking_)
  {
    /// \todo Lock estimator
    //estimator_.setOptimizationTimeLimit(-1.0,parameters_.optimization.max_iterations);
  }
}

bool ThreadedSlam::getNextFrame(MultiFramePtr &multiFrame) {
  std::vector<okvis::CameraMeasurement> frames;
  if(!cameraMeasurementsReceived_.getCopyOfFront(&frames)) {
    return false;
  }
  const size_t numCameras = frames.size();
  multiFrame.reset(new okvis::MultiFrame(parameters_.nCameraSystem, frames.at(0).timeStamp, 0));
  bool success = false;
  for(size_t im = 0; im < numCameras; ++im)
  {
    cv::Mat filtered = frames.at(im).measurement.image;
    if(!filtered.empty()) {
      success = true; // require at least one image, depth-only is not considered a success.
    }
    multiFrame->setImage(im, filtered);
    multiFrame->setDepthImage(im, frames.at(im).measurement.depthImage);
  }
  return success;
}

bool ThreadedSlam::processPendingSubmapAlignments() {
  // The application processing thread owns estimator mutations. A mapping
  // callback only enqueues work; it never waits on or modifies the estimator.
  if (submapAlignmentFactorsReceived_.Empty()) return false;
  if (optimisationThread_.joinable()) optimisationThread_.join();
  // Map to Map Alignment Factors are added here...
  AlignmentTerm alignmentTerm;
  bool processed = false;
  while (submapAlignmentFactorsReceived_.PopNonBlocking(&alignmentTerm)) {
    processed = true;
    hasAlignmentUpdates_ = true;

    // Set reference submap for live constraints
    // Prioritize the older submap
    if(parameters_.lidar) {
      // add submap alignment constraints between map keyframes A and B
      if (alignmentTerm.submap_A_ptr || alignmentTerm.planeMapA) {
        estimator_.addSubmapAlignmentConstraints(alignmentTerm.submap_A_ptr, alignmentTerm.frame_A_id,
                                                 alignmentTerm.frame_B_id, alignmentTerm.pointCloud_B,
                                                 alignmentTerm.sensorError, true, "Tukey", alignmentTerm.planeMapA,
                                                 submapConfig_.usePointToPlaneFactors);
      }

      previousSubmap_ = alignmentTerm.submap_B_ptr;
      previousPlaneMap_ = alignmentTerm.planeMapB;
      previousSubmapId_ = alignmentTerm.frame_B_id;
    }
    else {
      // add submap alignment constraints between map keyframes A and B
      if (alignmentTerm.submap_A_ptr) {
        estimator_.addSubmapAlignmentConstraints(alignmentTerm.submap_A_ptr, alignmentTerm.frame_A_id,
                                                alignmentTerm.frame_B_id, alignmentTerm.pointCloud_B,
                                                alignmentTerm.sensorError, false, "Tukey");
        // saveAlignedPoints(alignmentTerm.frame_B_id, alignmentTerm.pointCloud_B,
        //                 "map2map_"+std::to_string(alignmentTerm.frame_A_id));
      }
      if (alignmentTerm.frame_A_id != 0) {
        previousSubmap_ = alignmentTerm.submap_A_ptr;
        previousPlaneMap_ = alignmentTerm.planeMapA;
        previousSubmapId_ = alignmentTerm.frame_A_id;
      }
      else {
        previousSubmap_ = alignmentTerm.submap_B_ptr;
        previousPlaneMap_ = alignmentTerm.planeMapB;
        previousSubmapId_ = alignmentTerm.frame_B_id;
      }
    }
    LOG(INFO) << "[ThreadedSlam] Registered new reference submap with ID " << previousSubmapId_;
  }

  return processed;
}

void ThreadedSlam::flushPendingUpdates() {
  if (optimisationThread_.joinable()) optimisationThread_.join();
  while (publicationPending_.load() != 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  processPendingSubmapAlignments();
}

bool ThreadedSlam::processFrame() {
  // Process completed submaps even while cameras are idle or have stopped.
  processPendingSubmapAlignments();
  MultiFramePtr multiFrame;
  ImuMeasurement imuMeasurement;
  LidarMeasurement lidarMeasurement;
  GpsMeasurement gpsMeasurement;
  CameraMeasurement depthMeasurement;
  const size_t numCameras = parameters_.nCameraSystem.numCameras();

  kinematics::Transformation T_WS;
  SpeedAndBias speedAndBias;

  bool ranDetection = false;
  if(firstFrame_) {
    // in the very beginning, we need to wait for the IMU
    if(parameters_.imu.use && !imuMeasurementsReceived_.getCopyOfFront(&imuMeasurement)) {
      return false;
    }

    // now get the frames (synchronised timestamp)
    if(!getNextFrame(multiFrame)) {
      return false;
    }

    if(parameters_.imu.use) {
      if(multiFrame->timestamp()-Duration(imuTemporalOverlap) <= imuMeasurement.timeStamp) {
        // that's bad, we have frames without IMU measurements. Discard too old frames
        LOG(WARNING) << "startup: dropping frame because IMU measurements are newer, t="
                     << imuMeasurement.timeStamp;
        std::vector<okvis::CameraMeasurement> frames;
        cameraMeasurementsReceived_.PopBlocking(&frames);
        return false;
      }

      // also make sure we have enough IMU measuerements
      if(!imuMeasurementsReceived_.getCopyOfBack(&imuMeasurement)) {
        return false;
      }
      if(imuMeasurement.timeStamp < multiFrame->timestamp() + Duration(imuTemporalOverlap)) {
        return false; // wait for more IMU measurements
      }

      // now get all relevant IMU measurements we have received thus far
      do {
        if(imuMeasurementsReceived_.PopNonBlocking(&imuMeasurement))
        {
          imuMeasurementDeque_.push_back(imuMeasurement);
        } else {
          return false;
        }
      } while(imuMeasurementDeque_.back().timeStamp
              < multiFrame->timestamp() + Duration(imuTemporalOverlap));
    }

    // Drop LiDAR MEasurements before first frame ToDo: is this necessary?
    while(!lidarMeasurementsReceived_.Empty()
      && lidarMeasurementsReceived_.queue_.front().timeStamp < multiFrame->timestamp()){
      lidarMeasurementsReceived_.PopBlocking(&lidarMeasurement);
    }

    // Drop GPS Measurements that are older than the first frame (= first state)
    while(!gpsMeasurementsReceived_.Empty() && gpsMeasurementsReceived_.queue_.front().timeStamp < multiFrame ->timestamp()){
        gpsMeasurementsReceived_.PopBlocking(&gpsMeasurement);
    } // nothing else to do here for GPS


    // Drop the initial depth measurements. Otherwise, the queue would be full.
    // This leads to blocking the whole thread. 
    while(!depthMeasurementsReceived_.Empty()
      && depthMeasurementsReceived_.queue_.front().timeStamp < multiFrame->timestamp()){
      depthMeasurementsReceived_.PopBlocking(&depthMeasurement);
    }

    firstFrame_ = false;
  } else {
    // wait for next frame
    if(!getNextFrame(multiFrame)) {
      if(optimisationThread_.joinable()) {
        // in the very beginning, we can't join because it was not started
        optimisationThread_.join();
      }

      return false;
    }
    // now get all relevant IMU measurements we have received thus far
    if(parameters_.imu.use) {
      while(!shutdown_ && imuMeasurementDeque_.back().timeStamp <
            multiFrame->timestamp() + Duration(imuTemporalOverlap))
      {
        if(imuMeasurementsReceived_.PopNonBlocking(&imuMeasurement))
        {
          imuMeasurementDeque_.push_back(imuMeasurement);
        } else {
          return false;
        }
      }
    }
  }
  if (!lastOptimisedState_.id.isInitialised()) {
    // initial state
    if (parameters_.imu.use) {
      bool success = ceres::ImuError::initPose(imuMeasurementDeque_, T_WS);
      OKVIS_ASSERT_TRUE_DBG(Exception,
                            success,
                            "pose could not be initialized from imu measurements.")
      (void) (success); // avoid warning on unused variable
    } else {
      // otherwise we assume the camera is vertical & upright
      kinematics::Transformation T_WC;
      T_WC.set(T_WC.r(), T_WC.q() * Eigen::Quaterniond(-sqrt(2), sqrt(2), 0, 0));
      T_WS = T_WC * parameters_.nCameraSystem.T_SC(0)->inverse();
    }

    // detection -- needed to check if we can start up
    TimerSwitchable detectTimer("1 DetectAndDescribe");
    if(parameters_.frontend.parallelise_detection) {
      std::vector<std::shared_ptr<std::thread>> detectionThreads;
      for(size_t im = 1; im < numCameras; ++im) {
        kinematics::Transformation T_WC = T_WS * (*parameters_.nCameraSystem.T_SC(im));
        if(!multiFrame->image(im).empty()) {
          detectionThreads.emplace_back(
              new std::thread(&Frontend::detectAndDescribe, &frontend_,
                              im,  multiFrame, T_WC, nullptr));
        }
      }
      kinematics::Transformation T_WC0 = T_WS * (*parameters_.nCameraSystem.T_SC(0));
      if(!multiFrame->image(0).empty()) {
        frontend_.detectAndDescribe(0, multiFrame, T_WC0, nullptr);
      }
      for(size_t i = 0; i < detectionThreads.size(); ++i) {
        detectionThreads[i]->join();
      }
    } else {
      for(size_t im = 0; im < numCameras; ++im) {
        kinematics::Transformation T_WC = T_WS * (*parameters_.nCameraSystem.T_SC(im));
        if(!multiFrame->image(im).empty()) {
          frontend_.detectAndDescribe(im,  multiFrame, T_WC, nullptr);
        }
      }
    }
    if(!frontend_.isInitialized() && multiFrame->numKeypoints() < 15) {
      LOG(WARNING) << "Not enough keypoints (" << multiFrame->numKeypoints()
                   << ") -- cannot initialise yet.";
      std::vector<okvis::CameraMeasurement> frames;
      cameraMeasurementsReceived_.PopBlocking(&frames);
      return false;
    }
    ranDetection = true;
    detectTimer.stop();

  } else {
    // propagate to have a sensible pose estimate (as needed for detection)
    if (parameters_.imu.use) {
      // NOTE: we are not allowed to access the Estimator object here, as optimisation might run.
      T_WS = lastOptimisedState_.T_WS;
      kinematics::Transformation T_WS_prev = T_WS;
      speedAndBias.head<3>() = lastOptimisedState_.v_W;
      speedAndBias.segment<3>(3) = lastOptimisedState_.b_g;
      speedAndBias.tail<3>() = lastOptimisedState_.b_a;
      ceres::ImuError::propagation(imuMeasurementDeque_,
                                           parameters_.imu,
                                           T_WS,
                                           speedAndBias,
                                           lastOptimisedState_.timestamp,
                                           multiFrame->timestamp());
    } else {
      T_WS = lastOptimisedState_.T_WS;
      if (preLastOptimisedState_.id.isInitialised()) {
        const double r = (multiFrame->timestamp() - lastOptimisedState_.timestamp).toSec()
        / (lastOptimisedState_.timestamp - preLastOptimisedState_.timestamp).toSec();
        kinematics::Transformation T_WS_m1 = preLastOptimisedState_.T_WS;
        Eigen::Vector3d dr = r * (T_WS.r() - T_WS_m1.r());
        Eigen::AngleAxisd daa(T_WS.q() * T_WS_m1.q().inverse());
        daa.angle() *= r;
        T_WS.set(T_WS.r() + dr, T_WS.q() * Eigen::Quaterniond(daa));
      }
    }
    // now also get all relevant GPS measurements received thus far
    while(!shutdown_ && !gpsMeasurementsReceived_.Empty() && gpsMeasurementsReceived_.queue_.front().timeStamp < multiFrame->timestamp())
    {
        if(gpsMeasurementsReceived_.PopBlocking(&gpsMeasurement))
        {
          gpsMeasurementDeque_.push_back(gpsMeasurement);
        }
    }

    // now also get all relevant LiDAR measurements received thus far
    while(!shutdown_ && !lidarMeasurementsReceived_.Empty() && lidarMeasurementsReceived_.queue_.front().timeStamp < multiFrame->timestamp())
    {
      if(lidarMeasurementsReceived_.PopBlocking(&lidarMeasurement))
      {
        lidarMeasurementDeque_.push_back(lidarMeasurement);
      }
    }

    // Get depth measurements: in stereo network, depth and multiFrame are perfectly synced.
    while(!shutdown_ && !depthMeasurementsReceived_.Empty() && depthMeasurementsReceived_.queue_.front().timeStamp <= multiFrame->timestamp())
    {
      if (depthMeasurementsReceived_.PopBlocking(&depthMeasurement))
      {
        depthMeasurementDeque_.push_back(depthMeasurement);
      }
    }
  }
  // success, frame processed. Need to remove from the queue, as getNextFrame only obtains a copy.
  std::vector<okvis::CameraMeasurement> frames;
  cameraMeasurementsReceived_.PopBlocking(&frames);

  // detection
  if(!ranDetection) {
    TimerSwitchable detectTimer("1 DetectAndDescribe");
    if(parameters_.frontend.parallelise_detection) {
      std::vector<std::shared_ptr<std::thread>> detectionThreads;
      for(size_t im = 1; im < numCameras; ++im) {
        kinematics::Transformation T_WC = T_WS * (*parameters_.nCameraSystem.T_SC(im));
        if(!multiFrame->image(im).empty()) {
          detectionThreads.emplace_back(
              new std::thread(&Frontend::detectAndDescribe, &frontend_,
                              im,  multiFrame, T_WC, nullptr));
        }
      }
      kinematics::Transformation T_WC0 = T_WS * (*parameters_.nCameraSystem.T_SC(0));
      if(!multiFrame->image(0).empty()) {
        frontend_.detectAndDescribe(0, multiFrame, T_WC0, nullptr);
      }
      for(size_t i = 0; i < detectionThreads.size(); ++i) {
        detectionThreads[i]->join();
      }
    } else {
      for(size_t im = 0; im < numCameras; ++im) {
        kinematics::Transformation T_WC = T_WS * (*parameters_.nCameraSystem.T_SC(im));
        if(!multiFrame->image(im).empty()) {
          frontend_.detectAndDescribe(im,  multiFrame, T_WC, nullptr);
        }
      }
    }
    if(!frontend_.isInitialized() && multiFrame->numKeypoints() < 15) {
      LOG(WARNING) << "Not enough keypoints (" << multiFrame->numKeypoints()
                   << ") -- cannot initialise yet.";
      return true;
    }
    detectTimer.stop();
  }

  // IMPORTANT: the matcher needs the optimiser to be finished:
  if(optimisationThread_.joinable()) {
    // in the very beginning, we can't join because it was not started
    optimisationThread_.join();
  }

  // now store last optimised state for later use
  if (estimator_.numFrames() > 0) {
    const StateId currentId = estimator_.currentStateId();
    lastOptimisedState_.T_WS = estimator_.pose(currentId);
    SpeedAndBias speedAndBias = estimator_.speedAndBias(currentId);
    lastOptimisedState_.v_W = speedAndBias.head<3>();
    lastOptimisedState_.b_g = speedAndBias.segment<3>(3);
    lastOptimisedState_.b_a = speedAndBias.tail<3>();
    lastOptimisedState_.id = currentId;
    lastOptimisedState_.timestamp = estimator_.timestamp(currentId);
    if (estimator_.numFrames() > 1) {
      const StateId previousId(currentId.value() - 1);
      preLastOptimisedState_.T_WS = estimator_.pose(previousId);
      preLastOptimisedState_.id = previousId;
      preLastOptimisedState_.timestamp = estimator_.timestamp(previousId);
    }
  }

  // break here since all local threads joined
  if(shutdown_) {
    return false;
  }

  // remove imuMeasurements from deque
  if (estimator_.numFrames() > 0) {
    if(parameters_.imu.use) {
      while (!shutdown_
             && (imuMeasurementDeque_.front().timeStamp
                 < lastOptimisedState_.timestamp - Duration(imuTemporalOverlap))) {
        auto imuMeasurement = imuMeasurementDeque_.front();
        imuMeasurementDeque_.pop_front();
        if (imuMeasurementDeque_.empty()
            || (imuMeasurementDeque_.front().timeStamp
                > lastOptimisedState_.timestamp - Duration(imuTemporalOverlap))) {
          imuMeasurementDeque_.push_front(imuMeasurement); // re-add
          break; // finished popping
        }
      }
    }
  }

  // start the matching
  Time matchingStart = Time::now();
  TimerSwitchable matchTimer("2 Match");
  bool asKeyframe = false;

  if (!estimator_.addStates(multiFrame, imuMeasurementDeque_, asKeyframe)) {
    LOG(ERROR)<< "Failed to add state! will drop multiframe.";
    matchTimer.stop();
    return true;
  }
  else{

    // Also add Lidar Measurements as Live Factors; doing it here before the dataAssociationAndInitialization
    // ensures that we can add the factors before the first optimization

    if(useAlignmentFactors_)
    {
      TimerSwitchable tProcessLiveDepthLidar("8 Processing data for map-to-frame factors");

      // Motion Compensation of LiDAR Point Cloud
      kinematics::Transformation T_WS_live = estimator_.pose(StateId(multiFrame->id()));
      if(parameters_.lidar) {
        if (submapConfig_.usePointToPlaneFactors ? bool(previousPlaneMap_) : previousSubmap_ != nullptr) {

          LidarMotionUndistortion motionUndistortion(lastOptimisedState_, T_WS_live, T_SD_,
                                                    lidarMeasurementDeque_, imuMeasurementDeque_);

          TimerSwitchable tLiveUndistortion("8.1 Live undistortion");
          motionUndistortion.deskew();
          tLiveUndistortion.stop();          

          TimerSwitchable tFilterObserved("8.2 Filter observed points");
          size_t observed_points = 0;
          if (submapConfig_.usePointToPlaneFactors) {
            // Bound association work spatially before the 27-cell plane search;
            // retain all voxel candidates until validity has been checked.
            motionUndistortion.voxelDownsample(submapConfig_.voxelGridResolution);
            observed_points = motionUndistortion.filterPlanes(previousPlaneMap_,
                estimator_.pose(StateId(previousSubmapId_)), submapConfig_.sensorError);
          } else {
            observed_points = motionUndistortion.filterObserved(previousSubmap_, estimator_.pose(StateId(previousSubmapId_)));
          }
          // For point-to-plane, count valid spatial candidates before the final
          // factor limit, rather than raw duplicates or the truncated factor set.
          if(observed_points < submapConfig_.numSubmapFactors){
              noOverlapCounter_++;
          }
          else{
              noOverlapCounter_ = 0;
          }
          tFilterObserved.stop();

          TimerSwitchable tDownsampling("8.3 Downsampling");
          motionUndistortion.downsample(submapConfig_.numSubmapFactors, submapConfig_.voxelGridResolution);
          tDownsampling.stop();
          
          if(alignmentPublishCallback_){
            alignmentPublishCallback_(multiFrame->timestamp(), T_WS_live, motionUndistortion.deskewedDownsampledPointCloud(), false);
          } 
          // Also add LiDAR Factors (live factors only)
          std::vector<float> sensorErrors(motionUndistortion.deskewedDownsampledPointCloud().size(), submapConfig_.sensorError);
          estimator_.addSubmapAlignmentConstraints(
                  previousSubmap_, previousSubmapId_, multiFrame->id(),
                  motionUndistortion.deskewedDownsampledPointCloud(), sensorErrors, true, "Tukey",
                  previousPlaneMap_, submapConfig_.usePointToPlaneFactors);
        }
      }
      else {
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> livePoints;
        std::vector<float> liveSigmas;
        computeLiveDepthMeasurements(T_WS_live, multiFrame->timestamp(), livePoints, liveSigmas);

        if (livePoints.size() > 0.05*submapConfig_.numSubmapFactors) {
          if(alignmentPublishCallback_){
            alignmentPublishCallback_(multiFrame->timestamp(), T_WS_live, livePoints, false);
          }

          estimator_.addSubmapAlignmentConstraints(previousSubmap_, previousSubmapId_, multiFrame->id(),
                                                    livePoints, liveSigmas, false, "Tukey");
          float sigmaMean = 0;
          for (size_t ii = 0; ii < liveSigmas.size(); ii++) {
            sigmaMean += liveSigmas[ii];
          }
          DLOG(INFO) << "Map-to-frame factor added "<< previousSubmapId_ << "-" 
            << multiFrame->id() << " with " << livePoints.size()
            << " with uncertainty " << sigmaMean / liveSigmas.size() << std::endl;
          // saveAlignedPoints(multiFrame->id(), livePoints,
          //                   "frame2map_"+std::to_string(previousSubmapId_));
        }  
      }
      tProcessLiveDepthLidar.stop();
    }
  }
  // Check if we need new kf due to lidar overlap
  bool kfPrior = false;
  if(submapConfig_.useMap2LiveFactors && parameters_.lidar){
    kfPrior = needsNewLidarKeyframe();
    if(kfPrior) {
      lidarKeyframes_.insert(StateId(multiFrame->id()));
    }
  }

  // call the matcher
  if(!frontend_.dataAssociationAndInitialization(
        estimator_, parameters_, multiFrame, kfPrior, &asKeyframe) && !frontend_.isInitialized()) {
    LOG(WARNING) << "Not enough matches, cannot initialise";
    frontend_.clear();
    estimator_.clear();
    return false;
  }
  estimator_.setKeyframe(StateId(multiFrame->id()), asKeyframe);
  matchTimer.stop();

  // Add GPS Measurements
  estimator_.addGpsMeasurementsOnAllGraphs(gpsMeasurementDeque_, imuMeasurementDeque_);

  // remove gpsMeasurements from deque
  while(!shutdown_ && !gpsMeasurementDeque_.empty() && gpsMeasurementDeque_.front().timeStamp < multiFrame->timestamp() )
  {
    gpsMeasurementDeque_.pop_front();
  }

  // remove lidarMeasurements from deque
  while(!shutdown_ && !lidarMeasurementDeque_.empty() && lidarMeasurementDeque_.front().timeStamp < multiFrame->timestamp() )
  {
    lidarMeasurementDeque_.pop_front();
  }

  // remove depthMeasurements from deque
  while(!shutdown_ && !depthMeasurementDeque_.empty() && depthMeasurementDeque_.front().timeStamp <= multiFrame->timestamp() ) {
    depthMeasurementDeque_.pop_front();
  }

  processPendingSubmapAlignments();

  // start optimise, publish&visualise, marginalise:
  Eigen::Vector3d gyr(0,0,0);
  auto riter = imuMeasurementDeque_.rbegin();
  while(riter!=imuMeasurementDeque_.rend() && riter->timeStamp > multiFrame->timestamp()) {
    gyr = riter->measurement.gyroscopes;
    ++riter;
  }
  Time now = Time::now();
  double dt = parameters_.estimator.realtime_time_limit-(now-matchingStart).toSec();
  if(dt < 0.0) {
    dt = 0.01;
  }
  if(parameters_.estimator.enforce_realtime) {
    estimator_.setOptimisationTimeLimit(dt,
          parameters_.estimator.realtime_min_iterations);
  }
  // The publication/mapping pipeline may still need this frame interval for
  // asynchronous sensor propagation. It is carried only with the in-flight
  // update and released once all consumers finish; no per-frame history is kept.
  ImuMeasurementDeque frameImuMeasurements = imuMeasurementDeque_;
  optimisationThread_ = std::thread(&ThreadedSlam::optimisePublishMarginalise,
                                    this, multiFrame, gyr, std::move(frameImuMeasurements));

  // kick off posegraph optimisation, if needed, too...
  if(estimator_.needsFullGraphOptimisation()) {
    if(fullGraphOptimisationThread_.joinable()) {
      fullGraphOptimisationThread_.join();
    }
    //std::cout << "launching full graph optimisation" << std::endl;
    // hack: call full graph optimisation
    fullGraphOptimisationThread_ = std::thread(
          &ViSlamBackend::optimiseFullGraph, &estimator_,
          parameters_.estimator.full_graph_iterations,
          std::ref(posegraphOptimisationSummary_),
          parameters_.estimator.full_graph_num_threads, false);
  }

  return true;
}

void ThreadedSlam::optimisePublishMarginalise(MultiFramePtr multiFrame,
                                              const Eigen::Vector3d& gyroReading,
                                              ImuMeasurementDeque frameImuMeasurements) {
  kinematics::Transformation T_WS;
  SpeedAndBias speedAndBiases;
  const size_t numCameras = parameters_.nCameraSystem.numCameras();

  // optimise (if initialised)
  TimerSwitchable optimiseTimer("3 Optimise");
  std::vector<StateId> updatedStatesRealtime;
  estimator_.optimiseRealtimeGraph(
      parameters_.estimator.realtime_max_iterations, updatedStatesRealtime,
      parameters_.estimator.realtime_num_threads,
      false, false, frontend_.isInitialized());
  optimiseTimer.stop();

  // import pose graph optimisation
  std::vector<StateId> updatedStatesSync;
  if(estimator_.isLoopClosureAvailable()) {
    OKVIS_ASSERT_TRUE(Exception,
                      !estimator_.isLoopClosing(),
                      "loop closure available, but still loop closing -- bug")
    TimerSwitchable synchronisationTimer("5 Import full optimisation");
    estimator_.synchroniseRealtimeAndFullGraph(updatedStatesSync);
    synchronisationTimer.stop();
  }

  // prepare for publishing
  TimerSwitchable publishTimer("4 Prepare publishing");
  T_WS = estimator_.pose(StateId(multiFrame->id()));
  speedAndBiases = estimator_.speedAndBias(StateId(multiFrame->id()));
  StateId id(multiFrame->id());
  State state;
  state.id = id;
  state.T_WS = T_WS;
  state.v_W = speedAndBiases.head<3>();
  state.b_g = speedAndBiases.segment<3>(3);
  state.b_a = speedAndBiases.tail<3>();
  state.omega_S = gyroReading - speedAndBiases.segment<3>(3);
  state.timestamp = multiFrame->timestamp();
  state.previousImuMeasurements = frameImuMeasurements;
  state.isKeyframe = estimator_.isKeyframe(id);
  state.isOnlineExtrinsics = parameters_.camera.online_calibration.do_extrinsics;
  if (state.isOnlineExtrinsics && depthCamera_) {
    for (size_t camIndex = 0; camIndex < numCameras; ++camIndex) {
      if (parameters_.nCameraSystem.cameraType(camIndex).isUsedMapping) {
        if (parameters_.nCameraSystem.cameraType(camIndex).depthType.needRectify) {
          state.extrinsics[camIndex] = estimator_.extrinsics(id, camIndex) * T_rect_;
        }
        else {
          state.extrinsics[camIndex] = estimator_.extrinsics(id, camIndex);
        }
        T_SD_ = state.extrinsics[camIndex]; // update T_SD (D: corresponding camera)
      }
    }
  }

  AlignedMap<uint64_t, kinematics::Transformation> T_AiW;
  if (estimator_.T_AiS_.count(id)) {
    AlignedMap<uint64_t, kinematics::Transformation> T_AiS = estimator_.T_AiS_.at(id);
    for (const auto &T_AS : T_AiS) {
      T_AiW[T_AS.first] = T_AS.second * T_WS.inverse();
    }
  }
  state.T_AiW = T_AiW;
  estimator_.getObservedIds(state.id, state.covisibleFrameIds);

  TrackingState trackingState;
  if(lidarKeyframes_.count(id) == 0){
    trackingState.isLidarKeyframe = false;
  }
  else{
    trackingState.isLidarKeyframe = true;
  }
  trackingState.id = id;
  trackingState.isKeyframe = estimator_.isKeyframe(id);
  trackingState.recognisedPlace = estimator_.closedLoop(id);
  const double trackingQuality = estimator_.trackingQuality(id);
  if(trackingQuality < 0.01) {
    trackingState.trackingQuality = TrackingQuality::Lost;
  } else if (trackingQuality < 0.3){
    trackingState.trackingQuality = TrackingQuality::Marginal;
  } else {
    trackingState.trackingQuality = TrackingQuality::Good;
  }
  trackingState.currentKeyframeId = estimator_.mostOverlappedStateId(id, false);

  // re-propagate
  hasStarted_.store(true);

  // now publish
  if(optimisedGraphCallback_) {
    // current state & tracking info via State and Tracking State.
    // the graph:
    std::vector<StateId> updatedStateIds;
    PublicationData publicationData;
    publicationData.state = state;
    publicationData.trackingState = trackingState;
    publicationData.updatedStates.reset(new AlignedMap<StateId, State>());
    if(updatedStatesSync.size()>0) {
      updatedStateIds = updatedStatesSync;
    } else {
      updatedStateIds = updatedStatesRealtime;
    }
    for(const auto & id : updatedStateIds) {
      kinematics::Transformation T_WS = estimator_.pose(id);
      SpeedAndBias speedAndBias = estimator_.speedAndBias(id);
      Time timestamp = estimator_.timestamp(id);
      const bool isKeyframe = estimator_.isKeyframe(id);
      const bool isCurrentState = id == state.id;
      ImuMeasurementDeque imuMeasurements = isCurrentState ? frameImuMeasurements : ImuMeasurementDeque{};
      const Eigen::Vector3d omega_S = isCurrentState ? state.omega_S : Eigen::Vector3d::Zero();
      std::set<StateId> observedIds;
      estimator_.getObservedIds(id, observedIds);
      AlignedVector<Eigen::Vector3d> gpsPoints;
      estimator_.gpsMeasurements(id, gpsPoints);
      (*publicationData.updatedStates)[id] = State{T_WS, speedAndBias.head<3>(),
                                    speedAndBias.segment<3>(3), speedAndBias.tail<3>(),
                                    omega_S, timestamp, id, imuMeasurements, isKeyframe,
                                    AlignedMap<uint64_t, kinematics::Transformation>(),
                                    observedIds, true,
                                    false, AlignedMap<uint64_t, kinematics::Transformation>(),
                                    estimator_.T_GW(), gpsPoints};
    }
    for (const auto &id : affectedStates_) {
      kinematics::Transformation T_WS = estimator_.pose(id);
      SpeedAndBias speedAndBias = estimator_.speedAndBias(id);
      Time timestamp = estimator_.timestamp(id);
      const bool isCurrentState = id == state.id;
      ImuMeasurementDeque imuMeasurements = isCurrentState ? frameImuMeasurements : ImuMeasurementDeque{};
      const Eigen::Vector3d omega_S = isCurrentState ? state.omega_S : Eigen::Vector3d::Zero();
      std::set<StateId> observedIds;
      estimator_.getObservedIds(id, observedIds);
      AlignedVector<Eigen::Vector3d> gpsPoints;
      estimator_.gpsMeasurements(id, gpsPoints);
      (*publicationData.updatedStates)[id]
        = State{T_WS,
                speedAndBias.head<3>(),
                speedAndBias.segment<3>(3),
                speedAndBias.tail<3>(),
                omega_S,
                timestamp,
                id,
                imuMeasurements,
                estimator_.isKeyframe(id),
                AlignedMap<uint64_t, kinematics::Transformation>(),
                observedIds, false,
                false, AlignedMap<uint64_t, kinematics::Transformation>(),
                estimator_.T_GW(), gpsPoints};
    }
    affectedStates_.clear();

    // landmarks:
    publicationData.landmarksPublish.reset(new MapPointVector());
    MapPoints landmarks;
    estimator_.getLandmarks(landmarks);
    publicationData.landmarksPublish->reserve(landmarks.size());
    for(const auto & lm : landmarks) {
      auto latestObservedFrameId = *lm.second.observations.rbegin();
      publicationData.landmarksPublish->push_back(
            MapPoint(lm.first.value(), lm.second.point, lm.second.quality,
                     latestObservedFrameId.getFrameId()));
    }

    publicationPending_.fetch_add(1);
    if (blocking_ || parameters_.estimator.memory_bounded) {
      if (!publicationQueue_.PushBlockingIfFull(publicationData, blocking_ ? 1 : 3))
        publicationPending_.fetch_sub(1);
    } else {
      const bool overrun = publicationQueue_.PushNonBlockingDroppingIfFull(publicationData, 3);
      if (overrun) {
        publicationPending_.fetch_sub(1); // account for the replaced queued update.
        LOG(ERROR) << "publication (full update) overrun: dropping";
      }
    }

    // if(realtimePropagation_) {
    //   // pass on into IMU processing loop for realtime propagation later.
    //   // queue size 1 because we can afford to lose them; re-prop. just needs newest stuff.
    //   const bool overrun2 = lastOptimisedQueue_.PushNonBlockingDroppingIfFull(publicationData,3);
    //   if(overrun2) {
    //     LOG(WARNING) << "publication (full update) overrun 2: dropping";
    //   }
    // }

    // Update Realtime Trajectory object
    std::set<StateId> affectedStateIds;
    trajectory_.update(publicationData.trackingState, publicationData.updatedStates, affectedStateIds);
  }
  publishTimer.stop();

  // visualise
  TimerSwitchable visualisationTimer("6 Visualising");
  if(parameters_.output.display_overhead) {
    TimerSwitchable visualisation1Timer("6.1 Visualising overhead");
    // draw debug overhead image
    cv::Mat image(580, 580, CV_8UC3);
    image.setTo(cv::Scalar(10, 10, 10));
    estimator_.drawOverheadImage(image);
    overheadImages_.PushNonBlockingDroppingIfFull(image,1);
    visualisation1Timer.stop();
  }
  if(parameters_.output.display_matches) {
    TimerSwitchable visualisation2Timer("6.2 Prepare visualising matches");
    // draw matches
    // fill in information that requires access to estimator.
    ViVisualizer::VisualizationData::Ptr visualizationDataPtr(
          new ViVisualizer::VisualizationData());
    visualizationDataPtr->observations.resize(multiFrame->numKeypoints());
    okvis::MapPoint2 landmark;
    okvis::ObservationVector::iterator it = visualizationDataPtr
        ->observations.begin();
    for (size_t camIndex = 0; camIndex < numCameras; ++camIndex) {
      visualizationDataPtr->T_SCi.push_back(
        estimator_.extrinsics(estimator_.currentStateId(), camIndex));
      for (size_t k = 0; k < multiFrame->numKeypoints(camIndex); ++k) {
        OKVIS_ASSERT_TRUE_DBG(Exception,it != visualizationDataPtr->observations.end(),
                              "Observation-vector not big enough")
        it->keypointIdx = k;
        multiFrame->getKeypoint(camIndex, k, it->keypointMeasurement);
        multiFrame->getKeypointSize(camIndex, k, it->keypointSize);
        it->cameraIdx = camIndex;
        it->frameId = multiFrame->id();
        it->landmarkId = multiFrame->landmarkId(camIndex, k);
        if (estimator_.isLandmarkAdded(LandmarkId(it->landmarkId)) &&
            estimator_.isObserved(KeypointIdentifier(it->frameId, camIndex, k))) {
          estimator_.getLandmark(LandmarkId(it->landmarkId), landmark);
          it->landmark_W = landmark.point;
          it->classification = landmark.classification;
          if (estimator_.isLandmarkInitialised(LandmarkId(it->landmarkId)))
            it->isInitialized = true;
          else
            it->isInitialized = false;
        } else {
          it->landmark_W = Eigen::Vector4d(0, 0, 0, 0);
          // set to infinity to tell visualizer that landmark is not added...
        }
        ++it;
      }
    }
    visualizationDataPtr->T_WS = estimator_.pose(estimator_.currentStateId());
    visualizationDataPtr->currentFrames = multiFrame;
    visualizationDataPtr->isKeyframe = trackingState.isKeyframe;
    visualizationDataPtr->recognisedPlace = trackingState.recognisedPlace;
    if(trackingState.trackingQuality == TrackingQuality::Lost) {
      visualizationDataPtr->trackingQuality =
          ViVisualizer::VisualizationData::TrackingQuality::Lost;
    } else if(trackingState.trackingQuality == TrackingQuality::Marginal) {
      visualizationDataPtr->trackingQuality =
          ViVisualizer::VisualizationData::TrackingQuality::Marginal;
    }
    visualisation2Timer.stop();
    visualisationData_.PushNonBlockingDroppingIfFull(visualizationDataPtr,1);
  }
  visualisationTimer.stop();

  // apply marginalisation strategy
  bool expand = true;
  TimerSwitchable marginaliseTimer("7 Marginalise");
  estimator_.applyStrategy(
        size_t(parameters_.estimator.num_keyframes),
        size_t(parameters_.estimator.num_loop_closure_frames),
    size_t(parameters_.estimator.num_imu_frames), affectedStates_, expand);
  marginaliseTimer.stop();
}

bool ThreadedSlam::needsNewLidarKeyframe()
{
  if (submapConfig_.usePointToPlaneFactors ? bool(previousPlaneMap_) : previousSubmap_ != nullptr) {
    if(noOverlapCounter_ > 1){
      DLOG(INFO) << "Manually triggering new KF for lidar tracking!!! For lastOptimisedState.id = " << lastOptimisedState_.id.value();
      noOverlapCounter_ = 0;
      return true;
    }
  }
  return false;
}


// Loop to process visualisations.
void ThreadedSlam::visualisationLoop()
{
  while (!shutdown_) {
    ViVisualizer::VisualizationData::Ptr visualisationData;
    if(visualisationData_.PopBlocking(&visualisationData)) {
      std::vector<cv::Mat> outImages(parameters_.nCameraSystem.numCameras());
      for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
        if(parameters_.nCameraSystem.cameraType(i).isUsed
           && !visualisationData->currentFrames->image(i).empty()) {
          outImages[i] = visualizer_.drawMatches(visualisationData, i);
        }
      }
      visualisationImages_.PushNonBlockingDroppingIfFull(outImages,1);
    } else {
      return;
    }
  }
}

// Loop to process publishing.
void ThreadedSlam::publishingLoop()
{
  while (!shutdown_) {
    PublicationData publicationData;
    if(publicationQueue_.PopBlocking(&publicationData)) {
      if(optimisedGraphCallback_) {
        optimisedGraphCallback_(publicationData.state, publicationData.trackingState,
                                publicationData.updatedStates, publicationData.landmarksPublish);
      }
      publicationPending_.fetch_sub(1);
    } else {
      return;
    }
  }
}

// trigger display (needed because OSX won't allow threaded display)
void ThreadedSlam::display(std::map<std::string, cv::Mat> &images)
{

  std::vector<cv::Mat> outImages(parameters_.nCameraSystem.numCameras());
  if(visualisationImages_.PopNonBlocking(&outImages)) {
    // draw
    for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); i++) {
      std::string name;
      if(parameters_.nCameraSystem.cameraType(i).isColour) {
        name = "rgb"+std::to_string(i);
      } else {
        name = "cam"+std::to_string(i);
      }
      if(!outImages[i].empty()) {
        images[name] = outImages[i];
      }
    }
  }

  // top view
  cv::Mat topDebugImg;
  if(overheadImages_.PopNonBlocking(&topDebugImg)) {
    if(!topDebugImg.empty()) {
      images["Top Debug View"] = topDebugImg;
    }
  }
}

void ThreadedSlam::stopThreading() {
  if (shutdown_) {
    return;
  }

  // process all received
  while (processFrame()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  flushPendingUpdates();

  // stop CNN stuff
  frontend_.endCnnThreads();

  if(optimisationThread_.joinable()) {
    optimisationThread_.join(); // this should not be necessary after having called processFrame.
  }

  // shutdown queues
  imuMeasurementsReceived_.Shutdown();
  cameraMeasurementsReceived_.Shutdown();
  gpsMeasurementsReceived_.Shutdown();
  lidarMeasurementsReceived_.Shutdown();
  depthMeasurementsReceived_.Shutdown();
  submapAlignmentFactorsReceived_.Shutdown();
  visualisationImages_.Shutdown();
  visualisationData_.Shutdown();
  publicationQueue_.Shutdown();
  lastOptimisedQueue_.Shutdown();

  // Force background optimisation to finish, including final mapping factors.
  if(fullGraphOptimisationThread_.joinable() || hasAlignmentUpdates_) {
    LOG(INFO) << "wait for background optimisation to finish...";
    if (fullGraphOptimisationThread_.joinable()) fullGraphOptimisationThread_.join();
    std::vector<StateId> precedingFinalSync;
    if (hasAlignmentUpdates_ && estimator_.numFrames() > 0) {
      if (estimator_.isLoopClosureAvailable())
        estimator_.synchroniseRealtimeAndFullGraph(precedingFinalSync);
      LOG(INFO) << "Optimising final sparse graph with remaining submap constraints";
      estimator_.optimiseFullGraph(parameters_.estimator.full_graph_iterations,
          posegraphOptimisationSummary_, parameters_.estimator.full_graph_num_threads, false);
      hasAlignmentUpdates_ = false;
    }
    // import pose graph optimisation
    if (estimator_.isLoopClosureAvailable()) {
      OKVIS_ASSERT_TRUE(Exception,
                        !estimator_.isLoopClosing(),
                        "loop closure available, but still loop closing -- bug")
      LOG(INFO) << "import final background optimisation and publish...";
      TimerSwitchable synchronisationTimer("5 Import full optimisation");
      std::vector<StateId> updatedStates;
      estimator_.synchroniseRealtimeAndFullGraph(updatedStates);
      updatedStates.insert(updatedStates.end(), precedingFinalSync.begin(), precedingFinalSync.end());
      std::sort(updatedStates.begin(), updatedStates.end());
      updatedStates.erase(std::unique(updatedStates.begin(), updatedStates.end()), updatedStates.end());
      synchronisationTimer.stop();

      // prepare publishing
      StateId currentId = estimator_.currentStateId();
      kinematics::Transformation T_WS = estimator_.pose(currentId);
      SpeedAndBias speedAndBiases = estimator_.speedAndBias(currentId);
      State state;
      state.id = currentId;
      state.T_WS = T_WS;
      state.v_W = speedAndBiases.head<3>();
      state.b_g = speedAndBiases.segment<3>(3);
      state.b_a = speedAndBiases.tail<3>();
      state.omega_S.setZero(); // FIXME: use actual value
      state.timestamp = estimator_.timestamp(currentId);
      state.isKeyframe = estimator_.isKeyframe(currentId);
      TrackingState trackingState;
      trackingState.id = currentId;
      trackingState.isKeyframe = estimator_.isKeyframe(currentId);
      trackingState.recognisedPlace = estimator_.closedLoop(currentId);
      const double trackingQuality = estimator_.trackingQuality(currentId);
      if (trackingQuality < 0.01) {
        trackingState.trackingQuality = TrackingQuality::Lost;
      } else if (trackingQuality < 0.3) {
        trackingState.trackingQuality = TrackingQuality::Marginal;
      } else {
        trackingState.trackingQuality = TrackingQuality::Good;
      }
      trackingState.currentKeyframeId = estimator_.mostOverlappedStateId(currentId, false);
      hasStarted_.store(true);

      // now publish
      if (optimisedGraphCallback_) {
        // current state & tracking info via State and Tracking State.
        PublicationData publicationData;
        publicationData.state = state;
        publicationData.trackingState = trackingState;
        publicationData.updatedStates.reset(new AlignedMap<StateId, State>());
        for (const auto &id : updatedStates) {
          kinematics::Transformation T_WS = estimator_.pose(id);
          SpeedAndBias speedAndBias = estimator_.speedAndBias(id);
          Time timestamp = estimator_.timestamp(id);
          ImuMeasurementDeque imuMeasurements;
          const Eigen::Vector3d omega_S = Eigen::Vector3d::Zero();
          std::set<StateId> observedIds;
          estimator_.getObservedIds(id, observedIds);
          (*publicationData.updatedStates)[id] =
            State{T_WS, speedAndBias.head<3>(), speedAndBias.segment<3>(3), speedAndBias.tail<3>(),
                  omega_S, timestamp, id, imuMeasurements, estimator_.isKeyframe(id),
                  AlignedMap<uint64_t, kinematics::Transformation>(), observedIds, true,
                  false, AlignedMap<uint64_t, kinematics::Transformation>(),
                  kinematics::Transformation::Identity(), AlignedVector<Eigen::Vector3d>()};
          affectedStates_.erase(id); // do not separately publish as affected state...
        }
        for (const auto &id : affectedStates_) {
          kinematics::Transformation T_WS = estimator_.pose(id);
          SpeedAndBias speedAndBias = estimator_.speedAndBias(id);
          Time timestamp = estimator_.timestamp(id);
          ImuMeasurementDeque imuMeasurements;
          const Eigen::Vector3d omega_S = Eigen::Vector3d::Zero();
          std::set<StateId> observedIds;
          estimator_.getObservedIds(id, observedIds);
          (*publicationData.updatedStates)[id]
            = State{T_WS,
                    speedAndBias.head<3>(),
                    speedAndBias.segment<3>(3),
                    speedAndBias.tail<3>(),
                    omega_S,
                    timestamp,
                    id,
                    imuMeasurements,
                    estimator_.isKeyframe(id),
                    AlignedMap<uint64_t, kinematics::Transformation>(),
                    observedIds, false,
                    false, AlignedMap<uint64_t, kinematics::Transformation>(),
                    kinematics::Transformation::Identity(), AlignedVector<Eigen::Vector3d>()};
        }
        affectedStates_.clear();

        // landmarks:
        publicationData.landmarksPublish.reset(new MapPointVector());
        MapPoints landmarks;
        estimator_.getLandmarks(landmarks);
        publicationData.landmarksPublish->reserve(landmarks.size());
        for (const auto &lm : landmarks) {
          publicationData.landmarksPublish->push_back(
            MapPoint(lm.first.value(), lm.second.point, lm.second.quality));
        }

        // and publish
        optimisedGraphCallback_(publicationData.state,
                                publicationData.trackingState,
                                publicationData.updatedStates,
                                publicationData.landmarksPublish);
      }
    }
  }

  // end remaining threads
  shutdown_ = true;
  if(visualisationThread_.joinable()) {
    visualisationThread_.join();
  }
  if(publishingThread_.joinable()) {
    publishingThread_.join();
  }
}

void ThreadedSlam::writeFinalTrajectoryCsv()
{
  // thread safety -- join running stuff
  stopThreading();

  // trajectory writing
  if(!finalTrajectoryCsvFileName_.empty()) {
    estimator_.writeFinalCsvTrajectory(finalTrajectoryCsvFileName_, rpg_);
  }
}

void ThreadedSlam::writeGlobalTrajectoryCsv(const std::string& csvFileName)
{

  estimator_.writeGlobalCsvTrajectory(csvFileName);
}

void ThreadedSlam::doFinalBa()
{
  processPendingSubmapAlignments();

  // thread safety -- join running stuff
  stopThreading();

  if (parameters_.estimator.memory_bounded) {
    LOG(INFO) << "Skipping final full-history bundle adjustment in bounded-memory mode; "
                 "the corrected sparse graph and full trajectory remain available.";
    return;
  }

  // now call it
  const int numThreads = parameters_.estimator.realtime_num_threads
      +parameters_.estimator.full_graph_num_threads;
  const bool do_extrinsics_final_ba =
      parameters_.camera.online_calibration.do_extrinsics_final_ba;
  const double sigma_r = do_extrinsics_final_ba ?
                         parameters_.camera.online_calibration.sigma_r_final_ba :
                         0.0;
  const double sigma_alpha = do_extrinsics_final_ba ?
                             parameters_.camera.online_calibration.sigma_alpha_final_ba :
                             0.0;
  std::set<StateId> updatedStatesBa;
  estimator_.doFinalBa(100, posegraphOptimisationSummary_, updatedStatesBa,
      sigma_r, sigma_alpha, numThreads, true);

  // prepare publishing
  StateId currentId = estimator_.currentStateId();
  kinematics::Transformation T_WS = estimator_.pose(currentId);
  SpeedAndBias speedAndBiases = estimator_.speedAndBias(currentId);
  State state;
  state.id = currentId;
  state.T_WS = T_WS;
  state.v_W = speedAndBiases.head<3>();
  state.b_g = speedAndBiases.segment<3>(3);
  state.b_a = speedAndBiases.tail<3>();
  state.omega_S.setZero(); // FIXME: use actual value
  state.timestamp = estimator_.timestamp(currentId);
  state.isKeyframe = estimator_.isKeyframe(currentId);
  TrackingState trackingState;
  trackingState.id = currentId;
  trackingState.isKeyframe = estimator_.isKeyframe(currentId);
  trackingState.recognisedPlace = estimator_.closedLoop(currentId);
  const double trackingQuality = estimator_.trackingQuality(currentId);
  if(trackingQuality < 0.01) {
    trackingState.trackingQuality = TrackingQuality::Lost;
  } else if (trackingQuality < 0.3){
    trackingState.trackingQuality = TrackingQuality::Marginal;
  } else {
    trackingState.trackingQuality = TrackingQuality::Good;
  }
  trackingState.currentKeyframeId = estimator_.mostOverlappedStateId(currentId, false);
  hasStarted_.store(true);

  // now publish
  if(optimisedGraphCallback_) {
    // current state & tracking info via State and Tracking State.
    std::vector<StateId> updatedStateIds;
    PublicationData publicationData;
    publicationData.state = state;
    publicationData.trackingState = trackingState;
    publicationData.updatedStates.reset(new AlignedMap<StateId, State>());
    for(const auto & id : updatedStatesBa) {
      kinematics::Transformation T_WS = estimator_.pose(id);
      SpeedAndBias speedAndBias = estimator_.speedAndBias(id);
      Time timestamp = estimator_.timestamp(id);
      ImuMeasurementDeque imuMeasurements;
      const Eigen::Vector3d omega_S = Eigen::Vector3d::Zero();
      std::set<StateId> observedIds;
      estimator_.getObservedIds(id, observedIds);
      (*publicationData.updatedStates)[id] =
          State{T_WS, speedAndBias.head<3>(),speedAndBias.segment<3>(3),
          speedAndBias.tail<3>(),omega_S, timestamp, id, imuMeasurements,
          estimator_.isKeyframe(id),
          AlignedMap<uint64_t, kinematics::Transformation>(), observedIds, true,
          false, AlignedMap<uint64_t, kinematics::Transformation>(),
          kinematics::Transformation::Identity(), AlignedVector<Eigen::Vector3d>()};
    }
    for (const auto &id : affectedStates_) {
      kinematics::Transformation T_WS = estimator_.pose(id);
      SpeedAndBias speedAndBias = estimator_.speedAndBias(id);
      Time timestamp = estimator_.timestamp(id);
      ImuMeasurementDeque imuMeasurements;
      const Eigen::Vector3d omega_S = Eigen::Vector3d::Zero();
      std::set<StateId> observedIds;
      estimator_.getObservedIds(id, observedIds);
      (*publicationData.updatedStates)[id]
        = State{T_WS,
                speedAndBias.head<3>(),
                speedAndBias.segment<3>(3),
                speedAndBias.tail<3>(),
                omega_S,
                timestamp,
                id,
                imuMeasurements,
                estimator_.isKeyframe(id),
                AlignedMap<uint64_t, kinematics::Transformation>(),
                observedIds, false,
                false, AlignedMap<uint64_t, kinematics::Transformation>(),
                kinematics::Transformation::Identity(), AlignedVector<Eigen::Vector3d>()};
    }
    affectedStates_.clear();

    // landmarks:
    publicationData.landmarksPublish.reset(new MapPointVector());
    MapPoints landmarks;
    estimator_.getLandmarks(landmarks);
    publicationData.landmarksPublish->reserve(landmarks.size());
    for(const auto & lm : landmarks) {
      publicationData.landmarksPublish->push_back(
            MapPoint(lm.first.value(), lm.second.point, lm.second.quality));
    }

    // and publish
    optimisedGraphCallback_(publicationData.state, publicationData.trackingState,
                            publicationData.updatedStates, publicationData.landmarksPublish);
  }
}

bool ThreadedSlam::saveMap() {
  // thread safety -- join running stuff
  stopThreading();

  // now call it
  if(!finalTrajectoryCsvFileName_.empty()) {
    return estimator_.saveMap(mapCsvFileName_);
  }
  return false;
}

void ThreadedSlam::dumpGpsResiduals(const std::string &gpsResCsvFileName)
{
  // thread safety -- join running stuff
  stopThreading();

  // call residual writer on graph
  estimator_.dumpGpsResiduals(gpsResCsvFileName);


}

void ThreadedSlam::computeLiveDepthMeasurements(
  const kinematics::Transformation& T_WS_live,
  const okvis::Time& curTime,
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& livePoints,
  std::vector<float>& liveSigmas) {

  livePoints.clear();
  liveSigmas.clear();

  if (previousSubmap_ && !depthMeasurementDeque_.empty()) {
    // {M}: previous submap pose
    // {D}: depth camera pose in multiFrame
    const kinematics::Transformation T_WD = T_WS_live * T_SD_;
    const kinematics::Transformation T_WM = estimator_.pose(StateId(previousSubmapId_));
    const kinematics::Transformation T_MD = T_WM.inverse() * T_WD;

    for (size_t i = 0; i < depthMeasurementDeque_.size(); i ++) {
      CameraMeasurement liveDepth = depthMeasurementDeque_[i];

      // Precompute point in depth camera frame
      cv::Mat lowDepth, lowSigma;
      cv::resize(liveDepth.measurement.depthImage, lowDepth, cv::Size(),
        1.0 / submapConfig_.depthImageResDownsampling, 
        1.0 / submapConfig_.depthImageResDownsampling, cv::INTER_NEAREST);

      if (submapConfig_.useUncertainty && !liveDepth.measurement.sigmaImage.empty()) {
        cv::resize(liveDepth.measurement.sigmaImage, lowSigma, cv::Size(), 
          1.0 / submapConfig_.depthImageResDownsampling,
          1.0 / submapConfig_.depthImageResDownsampling, cv::INTER_NEAREST);
      }

      int depthWidth = lowDepth.cols;
      int depthHeight = lowDepth.rows;

      // T_delta = T_{D}{D_meas} where the latter is depth frame when measurement actually happens.
      kinematics::Transformation T_delta(Eigen::Matrix4d::Identity());
      if (depthMeasurementDeque_[i].timeStamp != curTime) {
        // Pose interpolation 
        okvis::State stateMeas;
        if (!trajectory_.getState(depthMeasurementDeque_[i].timeStamp, stateMeas)) {
          okvis::State latestTrajState;
          const std::set<StateId> allIds = trajectory_.stateIds();
          if (!allIds.empty()) {
            StateId latestTrajId = *allIds.rbegin(); // get the most recently added id
            trajectory_.getState(latestTrajId, latestTrajState);
            double deltaTime = (curTime-latestTrajState.timestamp).toSec();
            double deltaTime0 = (depthMeasurementDeque_[i].timeStamp-latestTrajState.timestamp).toSec();
            float interpWeight = deltaTime0/deltaTime;
            if (interpWeight > 0 && interpWeight < 1) {
              Eigen::Vector3d interpPos = (1-interpWeight)*latestTrajState.T_WS.r() + interpWeight*T_WS_live.r();
              Eigen::Quaterniond interpQuat = latestTrajState.T_WS.q().slerp(interpWeight, T_WS_live.q());
              okvis::kinematics::Transformation T_interp(interpPos, interpQuat);
              stateMeas.T_WS = T_interp;
            }
          }
        }
        T_delta = T_WD.inverse() * (stateMeas.T_WS * T_SD_);
      }

      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> observedPoints;
      std::vector<float> observedSigmas;

      // Uniformly sample depths and sigmas
      for (int x = 0; x < depthWidth; x+=10) {
        for (int y = 0; y < depthHeight; y+=10) {
          Eigen::Vector2f imagePoint_xy(static_cast<float>(x), static_cast<float>(y));
          Eigen::Vector3f cameraPoint;
          depthCamera_->model.backProject(imagePoint_xy, &cameraPoint);

          float pointZ = lowDepth.at<float>(y,x);
          float sigmaZ = submapConfig_.sensorError;
          if (submapConfig_.useUncertainty && !lowSigma.empty()) {
            sigmaZ = lowSigma.at<float>(y,x);
          }

          if (pointZ < submapConfig_.near_plane || pointZ > submapConfig_.far_plane || sigmaZ > 3.0) continue;
          
          cameraPoint = cameraPoint * pointZ;

          Eigen::Vector3f p_M = (T_MD*T_delta).T3x4().cast<float>() * cameraPoint.homogeneous();
          std::optional<float> occ = okvis::interpFieldMeanOccup<se::Safe::On>(*previousSubmap_, p_M);
          std::optional<Eigen::Vector3f> gradf = okvis::gradFieldMeanOccup<se::Safe::On>(*previousSubmap_, p_M);
          const float occ_margin = 2.0f;
          if (occ && gradf) {
            if (occ.value() > -occ_margin && occ.value() < occ_margin && gradf.value().norm() >= 1e-03) {
              Eigen::Vector3f p_S = (T_SD_*T_delta).T3x4().cast<float>() * cameraPoint.homogeneous();
              observedPoints.push_back(p_S);
              observedSigmas.push_back(sigmaZ);
            }
          } 
        }
      }
      // Downample points based on uncertainty
      if (observedPoints.size() > submapConfig_.numSubmapFactors) {
        okvis::downsamplePointsUncertainty(observedPoints, observedSigmas, 
                                    livePoints, liveSigmas,
                                    submapConfig_.numSubmapFactors);
      } 
      else {
        livePoints = observedPoints;
        liveSigmas = observedSigmas;
      }     
    }
  }
}

void ThreadedSlam::saveAlignedPoints(size_t id,
                                     std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points,
                                     std::string type) {
  kinematics::Transformation T_WS = estimator_.pose(StateId(id));
  size_t Npoints = points.size();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = Npoints;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.resize (cloud.width * cloud.height);
  size_t tmpCnt = 0;
  for (auto& point: cloud) {
    Eigen::Vector3d point_S = points[tmpCnt].cast<double>();
    Eigen::Vector3d point_W = (T_WS.T() * point_S.homogeneous()).head<3>();
    point.x = point_W(0);
    point.y = point_W(1);
    point.z = point_W(2);
    tmpCnt ++;
  }
  
  std::string mesh_numbering;
  if (id < 10) {
    mesh_numbering = "00000" + std::to_string(id);
  }
  else if (id < 100) {
    mesh_numbering = "0000" + std::to_string(id);
  }
  else if (id < 1000) {
    mesh_numbering = "000" + std::to_string(id);
  }
  else if (id < 10000) {
    mesh_numbering = "00" + std::to_string(id);
  }
  else if (id < 100000) {
    mesh_numbering = "0" + std::to_string(id);
  }
  else if (id < 1000000) {
    mesh_numbering = std::to_string(id);
  }
  
  std::string saveName = finalTrajectoryCsvFileName_.substr(0, finalTrajectoryCsvFileName_.find("okvis2"))
    + type + '_' + mesh_numbering + ".ply";
  pcl::io::savePLYFileASCII (saveName, cloud);
}

void ThreadedSlam::writeDebugStatisticsCsv(const std::string& csvFilePrefix)
{

  estimator_.writeLidarDebugStatisticsCsv(csvFilePrefix);
 
}

}  // namespace okvis
