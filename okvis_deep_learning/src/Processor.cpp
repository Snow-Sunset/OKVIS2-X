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

#include <chrono>
#include <thread>
#include <okvis/Processor.hpp>
#include <okvis/DepthFusionProcessor.hpp>

namespace okvis {

Processor::Processor(okvis::ViParameters& parameters, 
                    okvis::DeepLearningProcessor* dlProcessor,
                    std::string dBowDir,
                    const okvis::SupereightMapType::Config &mapConfig,
                    const SupereightMapType::DataType::Config &dataConfig,
                    const se::SubMapConfig &submapConfig)
: 
  deepLearningProcessor_(dlProcessor),
  slam_(parameters, dBowDir, submapConfig),
  se_interface_(mapConfig, dataConfig, submapConfig, parameters) {

  // Connect the SLAM optimized graph callback to supereight.
  slam_.setOptimisedGraphCallback(std::bind(
    &Processor::internalOptimizedGraphCallback,
    this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4
  ));

  if (dynamic_cast<DepthFusionProcessor*>(deepLearningProcessor_)) {
    deepLearningProcessor_->setLiveDepthImageCallback([&] (const okvis::Time &stamp,
      const cv::Mat &depthImage,
      const std::optional<cv::Mat> &sigmaImage) {
      bool estimatorAdd = slam_.addDepthMeasurement(stamp, depthImage, sigmaImage);
      return estimatorAdd;
    });
  
    deepLearningProcessor_->setImageCallback([&] (std::map<size_t, std::vector<okvis::CameraMeasurement>>& frames){
      bool mapAdd = se_interface_.addDepthMeasurement(frames);
      return mapAdd;
    });
  }
  else {
    deepLearningProcessor_->setImageCallback([&] (std::map<size_t, std::vector<okvis::CameraMeasurement>>& frames){
      bool estimatorAdd = false;
      for(const auto& cam_idx_frames : frames) {
        for(const auto& cam_measurement : cam_idx_frames.second) {
          if(!cam_measurement.measurement.depthImage.empty() && !cam_measurement.measurement.sigmaImage.empty()) {
            estimatorAdd = slam_.addDepthMeasurement(cam_measurement.timeStamp, cam_measurement.measurement.depthImage);
          } else if (!cam_measurement.measurement.depthImage.empty()) {
            estimatorAdd = slam_.addDepthMeasurement(cam_measurement.timeStamp, cam_measurement.measurement.depthImage, cam_measurement.measurement.sigmaImage);
          }
        }
      }
      bool mapAdd = se_interface_.addDepthMeasurement(frames);
      return (estimatorAdd && mapAdd);
    });
  }
  // Connect the submap alignment callback to supereight.
  se_interface_.setAlignCallback(std::bind(&okvis::ThreadedSlam::addSubmapAlignmentConstraints, &slam_,
                                             std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                                             std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
                                             std::placeholders::_7, std::placeholders::_8));

  // Launch supereight.
  se_interface_.start();
}

void Processor::display(std::map<std::string, cv::Mat> & images) {
  slam_.display(images);
  if(images.empty()){
    return;
  }
  deepLearningProcessor_->display(images);
}

void Processor::collectInfo() {
  se_interface_.collectInfo();
}

void Processor::finish() {
  while (slam_.processFrame()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  slam_.flushPendingUpdates();
  while (!se_interface_.finishedIntegrating()) {
    slam_.processPendingSubmapAlignments();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  slam_.processPendingSubmapAlignments();
  se_interface_.setFinished();
  slam_.stopThreading();
}

void Processor::setBlocking(bool blocking) {
  slam_.setBlocking(blocking);
  deepLearningProcessor_->setBlocking(blocking);
  se_interface_.setBlocking(blocking);
}

void Processor::setT_BS(const okvis::kinematics::Transformation& T_BS) {
  se_interface_.setT_BS(T_BS);
}

bool Processor::processFrame() {
  return slam_.processFrame();
}

void Processor::setSubmapCallback(const okvis::submapCallback &callback) {
  se_interface_.setSubmapCallback(callback);
}

void Processor::setFieldSliceCallback(const okvis::fieldCallback &callback) {
  se_interface_.setFieldSliceCallback(callback);
}

void Processor::setAlignmentPublishCallback(const okvis::alignmentPublishCallback &callback) {
  slam_.setAlignmentPublishCallback(callback);
  se_interface_.setAlignmentPublishCallback(callback);
}

void Processor::setOptimizedGraphCallback(const okvis::ViInterface::OptimisedGraphCallback &callback) {
  optimizedGraphCallback_ = callback;
}

void Processor::internalOptimizedGraphCallback(
  const okvis::State &state, 
  const okvis::TrackingState &trackingState,
  std::shared_ptr<const okvis::AlignedMap<okvis::StateId, okvis::State>> alignedMapPtr,
  std::shared_ptr<const okvis::MapPointVector> mapPointVectorPtr) {

  // Update supereight state.
  se_interface_.stateUpdateCallback(state, trackingState, alignedMapPtr);
  if (dynamic_cast<DepthFusionProcessor*>(deepLearningProcessor_)) {
    deepLearningProcessor_->stateUpdateCallback(state, trackingState, alignedMapPtr, mapPointVectorPtr);
  }

  // If defined, call external optimized graph callback.
  if(optimizedGraphCallback_) {
    optimizedGraphCallback_(state, trackingState, alignedMapPtr, mapPointVectorPtr);
  }
}

bool Processor::addImages(const std::map<size_t, std::pair<okvis::Time, cv::Mat>> & images,
                         const std::map<size_t, std::pair<okvis::Time, cv::Mat>> & depthImages) {
  okvis::Time stamp;
  std::map<size_t, cv::Mat> irImages;
  std::map<size_t, cv::Mat> depthImage;
  stamp = images.begin()->second.first;

  for(const auto& irImage : images) {
    irImages[irImage.first] = irImage.second.second;
  }

  for(const auto& depth : depthImages){
    depthImage[depth.first] = depth.second.second;
  }
  bool slamSuccess = slam_.addImages(stamp, irImages, depthImage);
  if(!deepLearningProcessor_->addImages(images, depthImages)) {
    DLOG(INFO) << "Frame not added to deep learning processor at t=" << stamp;
  }
  return slamSuccess;
}

bool Processor::addImuMeasurement(const okvis::Time &stamp, const Eigen::Vector3d &alpha, const Eigen::Vector3d &omega) {
  return slam_.addImuMeasurement(stamp, alpha, omega);
}

bool Processor::addLidarMeasurement(const okvis::Time & stamp,
                                    const Eigen::Vector3d & rayMeasurement) {
  return slam_.addLidarMeasurement(stamp, rayMeasurement);
}

bool Processor::addLidarMeasurements(const LidarMeasurementBatch& measurements) {
  return slam_.addLidarMeasurements(measurements);
}

bool Processor::addDepthMeasurement(const okvis::Time &stamp,
                                    const cv::Mat &depthImage,
                                    const std::optional<cv::Mat> &sigmaImage) {
  return slam_.addDepthMeasurement(stamp, depthImage);}

}
