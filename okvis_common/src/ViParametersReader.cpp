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
 * @file ViParametersReader.cpp
 * @brief Source file for the VioParametersReader class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <algorithm>

#include <glog/logging.h>

#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>
#include <okvis/cameras/EucmCamera.hpp>

#include <opencv2/core/core.hpp>

#include <okvis/ViParametersReader.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

// The default constructor.
ViParametersReader::ViParametersReader()
    : readConfigFile_(false) {
}

// The constructor. This calls readConfigFile().
ViParametersReader::ViParametersReader(const std::string& filename) {
  // reads
  readConfigFile(filename);
}

// Read and parse a config file.
void ViParametersReader::readConfigFile(const std::string& filename) {

  // reads
  
  cv::FileStorage file(filename, cv::FileStorage::READ);

  OKVIS_ASSERT_TRUE(Exception, file.isOpened(),
                    "Could not open config file: " << filename)
  LOG(INFO) << "Opened configuration file: " << filename;

  // camera calibration
  std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> calibrations;
  if(!getCameraCalibration(calibrations, file)) {
    LOG(FATAL) << "Did not find any calibration!";
  }

  // Count the number of cameras for mapping and rectification to decide
  // whether stereo rectification for the stereo depth network is actually needed.
  size_t numRectifiedCamera = 0;
  for (const auto& calibration : calibrations) {
    if (calibration.cameraType.depthType.needRectify) {
      numRectifiedCamera ++;
    }
  }

  // Assign parameters for stereo rectification.
  double fov_scale;
  if (file["camera_parameters"]["fov_scale"].isReal()) {
    parseEntry(file["camera_parameters"], "fov_scale", fov_scale);
  }
  else {
    if (numRectifiedCamera != 0) {
      LOG(FATAL) << "Please set fov_scale for the stereo rectification.\n"
        << "If you don't want to run the stereo depth network, "
        << "please set mapping_rectification: false in okvis2.yaml.";
    }
  }

  std::vector<size_t> sidx;
  if (file["camera_parameters"]["deep_stereo_indices"].isSeq()) {
    cv::FileNode T_idx = file["camera_parameters"]["deep_stereo_indices"];
    for(auto iter = T_idx.begin(); iter != T_idx.end(); ++iter) {
      sidx.push_back(int(*iter));
    }
  }
  else {
    if (numRectifiedCamera != 0) {
      LOG(FATAL) << "Please set deep_stereo_indices for the stereo depth network.\n"
        << "If you don't want to run the stereo depth network, "
        << "please set mapping_rectification: false in okvis2.yaml.";
    }
  }
  if (sidx.size() == 2) {
    computeRectifyMap(calibrations[sidx[0]], calibrations[sidx[1]], sidx, fov_scale);
  } else {
    LOG(WARNING) << "No stereo rectification";
  }
  
  size_t camIdx = 0;
  for (size_t i = 0; i < calibrations.size(); ++i) {

    std::shared_ptr<const kinematics::Transformation> T_SC_okvis_ptr(
          new kinematics::Transformation(calibrations[i].T_SC.r(),
                                                calibrations[i].T_SC.q().normalized()));

    if(strcmp(calibrations[i].cameraModel.c_str(), "eucm") == 0){
      std::shared_ptr<okvis::cameras::EucmCamera> cam;
      cam.reset(new okvis::cameras::EucmCamera(calibrations[i].imageDimension[0],
                                               calibrations[i].imageDimension[1],
                                               calibrations[i].focalLength[0],
                                               calibrations[i].focalLength[1],
                                               calibrations[i].principalPoint[0],
                                               calibrations[i].principalPoint[1],
                                               calibrations[i].eucmParameters[0],
                                               calibrations[i].eucmParameters[1]));
      cam->initialiseCameraAwarenessMaps();
      viParameters_.nCameraSystem.addCamera(
              T_SC_okvis_ptr,
              std::static_pointer_cast<const okvis::cameras::CameraBase>(cam),
              okvis::cameras::NCameraSystem::NoDistortion, true,
              calibrations[i].cameraType);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "EUCM camera " << camIdx
                << " with T_SC=\n" << s.str();
    }
    else if (strcmp(calibrations[i].distortionType.c_str(), "equidistant") == 0) {
      std::shared_ptr<okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>> cam;
      cam.reset(new okvis::cameras::PinholeCamera<
                  okvis::cameras::EquidistantDistortion>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  cameras::EquidistantDistortion(
                    calibrations[i].distortionCoefficients[0],
                    calibrations[i].distortionCoefficients[1],
                    calibrations[i].distortionCoefficients[2],
                    calibrations[i].distortionCoefficients[3])/*, id ?*/));
      cam->initialiseUndistortMaps(); // set up undistorters
      cam->initialiseCameraAwarenessMaps();
      viParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          std::static_pointer_cast<const cameras::CameraBase>(cam),
          cameras::NCameraSystem::Equidistant, true,
          calibrations[i].cameraType);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Equidistant pinhole camera " << camIdx
                << " with T_SC=\n" << s.str();
    } else if (strcmp(calibrations[i].distortionType.c_str(), "radialtangential") == 0
               || strcmp(calibrations[i].distortionType.c_str(), "plumb_bob") == 0) {
      std::shared_ptr<cameras::PinholeCamera<cameras::RadialTangentialDistortion>> cam;
      cam.reset(new cameras::PinholeCamera<
                  cameras::RadialTangentialDistortion>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  cameras::RadialTangentialDistortion(
                    calibrations[i].distortionCoefficients[0],
                    calibrations[i].distortionCoefficients[1],
                    calibrations[i].distortionCoefficients[2],
                    calibrations[i].distortionCoefficients[3])/*, id ?*/));
      cam->initialiseUndistortMaps(); // set up undistorters
      cam->initialiseCameraAwarenessMaps();
      viParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          std::static_pointer_cast<const cameras::CameraBase>(cam),
          cameras::NCameraSystem::RadialTangential, true,
          calibrations[i].cameraType);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Radial tangential pinhole camera " << camIdx
                << " with T_SC=\n" << s.str();
    } else if (strcmp(calibrations[i].distortionType.c_str(), "radialtangential8") == 0
               || strcmp(calibrations[i].distortionType.c_str(), "plumb_bob8") == 0) {
      std::shared_ptr<cameras::PinholeCamera<cameras::RadialTangentialDistortion8>> cam;
      cam.reset(new cameras::PinholeCamera<cameras::RadialTangentialDistortion8>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  cameras::RadialTangentialDistortion8(
                    calibrations[i].distortionCoefficients[0],
                    calibrations[i].distortionCoefficients[1],
                    calibrations[i].distortionCoefficients[2],
                    calibrations[i].distortionCoefficients[3],
                    calibrations[i].distortionCoefficients[4],
                    calibrations[i].distortionCoefficients[5],
                    calibrations[i].distortionCoefficients[6],
                    calibrations[i].distortionCoefficients[7])/*, id ?*/));
      cam->initialiseUndistortMaps(); // set up undistorters
      cam->initialiseCameraAwarenessMaps();
      viParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          std::static_pointer_cast<const cameras::CameraBase>(cam),
          cameras::NCameraSystem::RadialTangential8, true,
          calibrations[i].cameraType);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Radial tangential 8 pinhole camera " << camIdx
                << " with T_SC=\n" << s.str();
    } else {
      LOG(ERROR) << "unrecognized distortion type " << calibrations[i].distortionType;
    }
    ++camIdx;
  }

  if(file["lidar"].isMap()){
    viParameters_.lidar = okvis::LidarParameters(); 
    if(!getLiDARCalibration(file["lidar"], *viParameters_.lidar)){
      LOG(ERROR) << "Could not parse the LiDAR config file";
    } else {
      std::stringstream s;
      s << (*viParameters_.lidar).T_SL.T();
      LOG(INFO) << "Parsed LiDAR with the following characteristics: \n" 
                << "elevation_resolution_angle: " << std::to_string((*viParameters_.lidar).elevation_resolution_angle) << " \n"
                << "azimuth_resolution_angle: " << std::to_string((*viParameters_.lidar).azimuth_resolution_angle) << "\n"
                << "T_SL:\n " << s.str();
    }
  } else {
    LOG(INFO) << "No LiDAR declared";
  }

  //camera parameters.
  parseEntry(file["camera_parameters"], "timestamp_tolerance",
             viParameters_.camera.timestamp_tolerance);
  cv::FileNode T = file["camera_parameters"]["sync_cameras"];
  OKVIS_ASSERT_TRUE(
    Exception, T.isSeq(),
    "missing real array parameter " << "camera_parameters" << ": " << "sync_cameras")
  for(auto iter = T.begin(); iter != T.end(); ++iter) {
    viParameters_.camera.sync_cameras.insert(int(*iter));
  }
  parseEntry(file["camera_parameters"], "image_delay",
             viParameters_.camera.image_delay);
  parseEntry(file["camera_parameters"]["online_calibration"], "do_extrinsics",
             viParameters_.camera.online_calibration.do_extrinsics);
  parseEntry(file["camera_parameters"]["online_calibration"], "do_extrinsics_final_ba",
             viParameters_.camera.online_calibration.do_extrinsics_final_ba);
  parseEntry(file["camera_parameters"]["online_calibration"], "sigma_r",
             viParameters_.camera.online_calibration.sigma_r);
  parseEntry(file["camera_parameters"]["online_calibration"], "sigma_alpha",
             viParameters_.camera.online_calibration.sigma_alpha);
  parseEntry(file["camera_parameters"]["online_calibration"], "sigma_r_final_ba",
             viParameters_.camera.online_calibration.sigma_r_final_ba);
  parseEntry(file["camera_parameters"]["online_calibration"], "sigma_alpha_final_ba",
             viParameters_.camera.online_calibration.sigma_alpha_final_ba);
  viParameters_.camera.stereo_indices = sidx;

  //IMU parameters.
  parseEntry(file["imu_parameters"], "use",
             viParameters_.imu.use);
  Eigen::Matrix4d T_BS;
  parseEntry(file["imu_parameters"], "T_BS", T_BS);
  viParameters_.imu.T_BS = kinematics::Transformation(T_BS);
  parseEntry(file["imu_parameters"], "a_max",
             viParameters_.imu.a_max);
  parseEntry(file["imu_parameters"], "g_max",
             viParameters_.imu.g_max);
  parseEntry(file["imu_parameters"], "sigma_g_c",
             viParameters_.imu.sigma_g_c);
  parseEntry(file["imu_parameters"], "sigma_bg",
             viParameters_.imu.sigma_bg);
  parseEntry(file["imu_parameters"], "sigma_a_c",
             viParameters_.imu.sigma_a_c);
  parseEntry(file["imu_parameters"], "sigma_ba",
             viParameters_.imu.sigma_ba);
  parseEntry(file["imu_parameters"], "sigma_gw_c",
             viParameters_.imu.sigma_gw_c);
  parseEntry(file["imu_parameters"], "sigma_aw_c",
             viParameters_.imu.sigma_aw_c);
  parseEntry(file["imu_parameters"], "a0",
             viParameters_.imu.a0);
  parseEntry(file["imu_parameters"], "g0",
             viParameters_.imu.g0);
  parseEntry(file["imu_parameters"], "g",
             viParameters_.imu.g);
  parseEntry(file["imu_parameters"], "s_a",
             viParameters_.imu.s_a);

  // Parameters for detection etc.
  parseEntry(file["frontend_parameters"], "detection_threshold",
             viParameters_.frontend.detection_threshold);
  parseEntry(file["frontend_parameters"], "absolute_threshold",
             viParameters_.frontend.absolute_threshold);
  parseEntry(file["frontend_parameters"], "matching_threshold",
             viParameters_.frontend.matching_threshold);
  parseEntry(file["frontend_parameters"], "octaves",
             viParameters_.frontend.octaves);
  parseEntry(file["frontend_parameters"], "max_num_keypoints",
             viParameters_.frontend.max_num_keypoints);
  parseEntry(file["frontend_parameters"], "keyframe_overlap",
             viParameters_.frontend.keyframe_overlap);
  parseEntry(file["frontend_parameters"], "use_cnn",
             viParameters_.frontend.use_cnn);
  parseEntry(file["frontend_parameters"], "parallelise_detection",
             viParameters_.frontend.parallelise_detection);
  parseEntry(file["frontend_parameters"], "num_matching_threads",
             viParameters_.frontend.num_matching_threads);

  // Parameters regarding the estimator.
  const auto estimatorNode = file["estimator_parameters"];
  if (!estimatorNode["memory_bounded"].empty())
    parseEntry(estimatorNode, "memory_bounded", viParameters_.estimator.memory_bounded);
  parseEntry(file["estimator_parameters"], "num_keyframes",
             viParameters_.estimator.num_keyframes);
  parseEntry(file["estimator_parameters"], "num_loop_closure_frames",
             viParameters_.estimator.num_loop_closure_frames);
  parseEntry(file["estimator_parameters"], "num_imu_frames",
             viParameters_.estimator.num_imu_frames);
  parseEntry(file["estimator_parameters"], "do_loop_closures",
             viParameters_.estimator.do_loop_closures);
  parseEntry(file["estimator_parameters"], "do_final_ba",
             viParameters_.estimator.do_final_ba);
  parseEntry(file["estimator_parameters"], "enforce_realtime",
             viParameters_.estimator.enforce_realtime);
  parseEntry(file["estimator_parameters"], "realtime_min_iterations",
             viParameters_.estimator.realtime_min_iterations);
  parseEntry(file["estimator_parameters"], "realtime_max_iterations",
             viParameters_.estimator.realtime_max_iterations);
  parseEntry(file["estimator_parameters"], "realtime_time_limit",
             viParameters_.estimator.realtime_time_limit);
  parseEntry(file["estimator_parameters"], "realtime_num_threads",
             viParameters_.estimator.realtime_num_threads);
  parseEntry(file["estimator_parameters"], "full_graph_iterations",
             viParameters_.estimator.full_graph_iterations);
  parseEntry(file["estimator_parameters"], "full_graph_num_threads",
             viParameters_.estimator.full_graph_num_threads);
  parseEntry(file["estimator_parameters"], "p_dbow",
             viParameters_.estimator.p_dbow);
  parseEntry(file["estimator_parameters"], "drift_percentage_heuristic",
             viParameters_.estimator.drift_percentage_heuristic);

  // Some options for how and what to output.
  parseEntry(file["output_parameters"], "display_topview",
             viParameters_.output.display_topview);
  parseEntry(file["output_parameters"], "display_matches",
             viParameters_.output.display_matches);
  parseEntry(file["output_parameters"], "display_overhead",
             viParameters_.output.display_overhead);
  parseEntry(file["output_parameters"], "enable_submapping",
             viParameters_.output.enable_submapping);

  // GPS Parameters
  if(file["gps_parameters"].isMap()){
    viParameters_.gps = okvis::GpsParameters(); 
    if(!getGpsCalibration(file["gps_parameters"], *viParameters_.gps)){
      LOG(ERROR) << "Could not parse the GPS config";
    } else {
      LOG(INFO) << "Parsed GPS with the following characteristics: \n" 
                << "\tdata_type: " << (*viParameters_.gps).type << " \n"
                << "\tr_SA: " << (*viParameters_.gps).r_SA.transpose() << " \n"
                << "\tyaw_error_threshold: " << std::to_string((*viParameters_.gps).yawErrorThreshold) << " \n"
                << "\trobust_gps_init: " << std::boolalpha << (*viParameters_.gps).robustGpsInit;
    }
  } else {
    LOG(INFO) << "No GPS declared";
  }

  // done!
  readConfigFile_ = true;
}

void ViParametersReader::parseEntry(const cv::FileNode& file, std::string name, int& readValue) {
  OKVIS_ASSERT_TRUE(Exception, file[name].isInt(),
                    "missing integer parameter " << file.name() << ": " << name)
  file[name] >> readValue;
}

void ViParametersReader::parseEntry(const cv::FileNode &file,
                                    std::string name, double& readValue) {
  OKVIS_ASSERT_TRUE(Exception, file[name].isReal(),
                    "missing real parameter " << file.name() << ": " << name)
  file[name] >> readValue;
}

// Parses booleans from a cv::FileNode. OpenCV sadly has no implementation like this.
void ViParametersReader::parseEntry(const cv::FileNode& file, std::string name, bool& readValue) {
  OKVIS_ASSERT_TRUE(Exception, file[name].isInt() || file[name].isString(),
                    "missing boolean parameter " << file.name() << ": " << name)
  if (file[name].isInt()) {
    readValue = int(file[name]) != 0;
    return;
  }
  if (file[name].isString()) {
    std::string str = std::string(file[name]);
    // cut out first word. str currently contains everything including comments
    str = str.substr(0,str.find(" "));
    // transform it to all lowercase
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    /* from yaml.org/type/bool.html:
     * Booleans are formatted as English words
     * (“true”/“false”, “yes”/“no” or “on”/“off”)
     * for readability and may be abbreviated as
     * a single character “y”/“n” or “Y”/“N”. */
    if (str.compare("false")  == 0
        || str.compare("no")  == 0
        || str.compare("n")   == 0
        || str.compare("off") == 0) {
      readValue = false;
      return;
    }
    if (str.compare("true")   == 0
        || str.compare("yes") == 0
        || str.compare("y")   == 0
        || str.compare("on")  == 0) {
      readValue = true;
      return;
    }
    OKVIS_THROW(Exception, "Boolean with uninterpretable value " << str)
  }
  return;
}

void ViParametersReader::parseEntry(const cv::FileNode &file, std::string name,
                                    Eigen::Matrix4d& readValue) {
  cv::FileNode T = file[name];
  OKVIS_ASSERT_TRUE(Exception, T.isSeq(),
                    "missing real array parameter " << file.name() << ": " << name)
  readValue << T[0], T[1], T[2], T[3],
               T[4], T[5], T[6], T[7],
               T[8], T[9], T[10], T[11],
               T[12], T[13], T[14], T[15];
}

void ViParametersReader::parseEntry(const cv::FileNode& file, std::string name,
                                    Eigen::Vector3d& readValue) {
  cv::FileNode T = file[name];
  OKVIS_ASSERT_TRUE(Exception, T.isSeq(),
                    "missing real array parameter " << file.name() << ": " << name)
  readValue << T[0], T[1], T[2];
}

void ViParametersReader::parseEntry(const cv::FileNode& file, std::string name,
                                    std::string& readValue) {
  cv::FileNode T = file[name];
  OKVIS_ASSERT_TRUE(Exception, T.isString(),
                    "missing string parameter " << file.name() << ": " << name)
  readValue = std::string(T);
}

bool ViParametersReader::getCameraCalibration(
    std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations,
    cv::FileStorage& configurationFile) {

  bool success = getCalibrationViaConfig(calibrations, configurationFile["cameras"]);
  return success;
}

// Get the camera calibration via the configuration file.
bool ViParametersReader::getCalibrationViaConfig(
    std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations,
    cv::FileNode cameraNode) const {

  calibrations.clear();
  bool gotCalibration = false;
  // first check if calibration is available in config file
  if (cameraNode.isSeq()
     && cameraNode.size() > 0) {
    size_t camIdx = 0;
    for (cv::FileNodeIterator it = cameraNode.begin();
        it != cameraNode.end(); ++it) {
      if ((*it).isMap()
          && (*it)["T_SC"].isSeq()
          && (*it)["image_dimension"].isSeq()
          && (*it)["image_dimension"].size() == 2
          && (*it)["distortion_coefficients"].isSeq()
          && (*it)["distortion_coefficients"].size() >= 4
          && (*it)["distortion_type"].isString()
          && (*it)["focal_length"].isSeq()
          && (*it)["focal_length"].size() == 2
          && (*it)["principal_point"].isSeq()
          && (*it)["principal_point"].size() == 2) {
        LOG(INFO) << "Found calibration in configuration file for camera " << camIdx;
        gotCalibration = true;
      } else {
        LOG(WARNING) << "Found incomplete calibration in configuration file for camera " << camIdx
                     << ". Will not use the calibration from the configuration file.";
        return false;
      }

      ++camIdx;
    }
  }
  else
    LOG(INFO) << "Did not find a calibration in the configuration file.";

  if (gotCalibration) {
    for (cv::FileNodeIterator it = cameraNode.begin();
        it != cameraNode.end(); ++it) {

      CameraCalibration calib;

      if((std::string)((*it)["cam_model"]) == "pinhole"){
        cv::FileNode T_SC_node = (*it)["T_SC"];
        cv::FileNode imageDimensionNode = (*it)["image_dimension"];
        cv::FileNode distortionCoefficientNode = (*it)["distortion_coefficients"];
        cv::FileNode focalLengthNode = (*it)["focal_length"];
        cv::FileNode principalPointNode = (*it)["principal_point"];

        // extrinsics
        Eigen::Matrix4d T_SC;
        T_SC << T_SC_node[0], T_SC_node[1], T_SC_node[2], T_SC_node[3],
                T_SC_node[4], T_SC_node[5], T_SC_node[6], T_SC_node[7],
                T_SC_node[8], T_SC_node[9], T_SC_node[10], T_SC_node[11],
                T_SC_node[12], T_SC_node[13], T_SC_node[14], T_SC_node[15];
        calib.T_SC = kinematics::Transformation(T_SC);

        calib.imageDimension << imageDimensionNode[0], imageDimensionNode[1];
        calib.distortionCoefficients.resize(int(distortionCoefficientNode.size()));
        for(int i=0; i<int(distortionCoefficientNode.size()); ++i) {
          calib.distortionCoefficients[i] = distortionCoefficientNode[i];
        }
        calib.focalLength << focalLengthNode[0], focalLengthNode[1];
        calib.principalPoint << principalPointNode[0], principalPointNode[1];
        calib.distortionType = std::string((*it)["distortion_type"]);
        calib.cameraModel = "pinhole";
      } else if ( (std::string)((*it)["cam_model"]) == "eucm"){
        cv::FileNode T_SC_node = (*it)["T_SC"];
        cv::FileNode imageDimensionNode = (*it)["image_dimension"];
        cv::FileNode focalLengthNode = (*it)["focal_length"];
        cv::FileNode principalPointNode = (*it)["principal_point"];
        cv::FileNode eucmParamNode= (*it)["eucm_parameters"];

        // extrinsics
        Eigen::Matrix4d T_SC;
        T_SC << T_SC_node[0], T_SC_node[1], T_SC_node[2], T_SC_node[3],
                T_SC_node[4], T_SC_node[5], T_SC_node[6], T_SC_node[7],
                T_SC_node[8], T_SC_node[9], T_SC_node[10], T_SC_node[11],
                T_SC_node[12], T_SC_node[13], T_SC_node[14], T_SC_node[15];
        calib.T_SC = okvis::kinematics::Transformation(T_SC);

        calib.imageDimension << imageDimensionNode[0], imageDimensionNode[1];
        calib.focalLength << focalLengthNode[0], focalLengthNode[1];
        calib.principalPoint << principalPointNode[0], principalPointNode[1];
        calib.eucmParameters << eucmParamNode[0], eucmParamNode[1];
        calib.distortionType = "None";
        calib.cameraModel = "eucm";
      }

      // parse additional stuff
      if((*it)["camera_type"].isString()){
        std::string camera_type = std::string((*it)["camera_type"]);
        if(camera_type.compare(0,4,"gray")==0) {
          calib.cameraType.isColour = false;
        } else {
          calib.cameraType.isColour = true;
        }
        if(camera_type.size()>=6){
          if(camera_type.compare(camera_type.size()-6,6,"+depth")==0) {
            calib.cameraType.depthType.isDepthCamera = true;
          } else {
            calib.cameraType.depthType.isDepthCamera = false;
          }
        }
      }
      if((*it)["slam_use"].isString()){
        std::string slam_use = std::string((*it)["slam_use"]);
        if(slam_use.compare(0,5,"okvis")==0) {
          calib.cameraType.isUsed = true;
        } else {
          calib.cameraType.isUsed = false;
        }
        if(slam_use.size()>=6){
          if(slam_use.compare(slam_use.size()-6,6,"-depth")==0) {
            calib.cameraType.depthType.createDepth = true;
          } else {
            calib.cameraType.depthType.createDepth = false;
          }
        }
        if(slam_use.size()>=8){
          if(slam_use.compare(slam_use.size()-8,8,"-virtual")==0) {
            calib.cameraType.depthType.createVirtual = true;
          } else {
            calib.cameraType.depthType.createVirtual = false;
          }
        }
      }
      if((*it)["sigma_pixels"].isReal()){
        calib.cameraType.depthType.sigmaPixels = ((*it)["sigma_pixels"]);
      }
      if((*it)["sigma_depth"].isReal()){
        calib.cameraType.depthType.sigmaPixels = ((*it)["sigma_depth"]);
      }

      if((*it)["mapping"].isString() && (std::string)((*it)["mapping"]) == "true"){
        calib.cameraType.isUsedMapping = true;
      }

      if((*it)["mapping_rectification"].isString() && (std::string)((*it)["mapping_rectification"]) == "true"){
        calib.cameraType.depthType.needRectify = true;
      }

      calibrations.push_back(calib);
    }
  }
  return gotCalibration;
}

bool ViParametersReader::getLiDARCalibration(const cv::FileNode& calibrationNode, okvis::LidarParameters& lidarParameters){
  
  if(!calibrationNode["T_SL"].isSeq() || !calibrationNode["elevation_resolution_angle"].isReal() || !calibrationNode["azimuth_resolution_angle"].isReal()){
    return false;
  }

  cv::FileNode T_SL_node = calibrationNode["T_SL"];
  Eigen::Matrix4d T_SL;
  T_SL << T_SL_node[0], T_SL_node[1], T_SL_node[2], T_SL_node[3],
          T_SL_node[4], T_SL_node[5], T_SL_node[6], T_SL_node[7],
          T_SL_node[8], T_SL_node[9], T_SL_node[10], T_SL_node[11],
          T_SL_node[12], T_SL_node[13], T_SL_node[14], T_SL_node[15];
  lidarParameters.T_SL = okvis::kinematics::Transformation(T_SL);
  lidarParameters.elevation_resolution_angle = calibrationNode["elevation_resolution_angle"];
  lidarParameters.azimuth_resolution_angle = calibrationNode["azimuth_resolution_angle"];

  return true;
}

bool ViParametersReader::getGpsCalibration(const cv::FileNode& calibrationNode, okvis::GpsParameters& gpsParameters){

  parseEntry(calibrationNode, "data_type",
             gpsParameters.type);
  parseEntry(calibrationNode, "r_SA",
             gpsParameters.r_SA);
  parseEntry(calibrationNode, "yaw_error_threshold",
             gpsParameters.yawErrorThreshold);
  parseEntry(calibrationNode, "robust_gps_init",
             gpsParameters.robustGpsInit);

  return true;
}

bool ViParametersReader::computeRectifyMap(CameraCalibration& leftCalibration,
                                           CameraCalibration& rightCalibration,
                                           const std::vector<size_t>& stereo_indices,
                                           const double& fov_scale) {
  // Precompute rectification map of stereo network for mapping
  if (leftCalibration.cameraType.depthType.needRectify && rightCalibration.cameraType.depthType.needRectify &&
      leftCalibration.cameraModel == "pinhole" && rightCalibration.cameraModel == "pinhole") {

    LOG(INFO) << "Found a stereo camera for rectification";

    if (leftCalibration.distortionType != "radialtangential" &&
        leftCalibration.distortionType != "equidistant") {
      LOG(ERROR) << "Radial-tangential or equidistant distortion models are supported in stereo rectification.";
      return false;
    }

    cv::Size rawSize = cv::Size(leftCalibration.imageDimension[0],
      leftCalibration.imageDimension[1]); // [width, height]
    int rectWidth = 512;
    const int rectHeight = 384;
    double aspectRatio = static_cast<double>(rawSize.width)/static_cast<double>(rawSize.height);
    // The native resolution of the depth networks (stereo & MVS) is 512x384. 
    // Deviate from this native resolution only for wide images.
    if (aspectRatio > 1.6) {
      rectWidth = static_cast<int>(aspectRatio*rectHeight);
      if (rectWidth % 8 != 0) {
        // Make sure that the image width divisible by 8.
        rectWidth += 8 - (rectWidth % 8);
      }
    }
    cv::Size rectSize(rectWidth, rectHeight);
    LOG(INFO) << "Original resolution: " << rawSize.width << "x" << rawSize.height
              <<",  Rectification resolution: " << rectSize.width << "x" << rectSize.height;
    double array_dcl[4] = {leftCalibration.distortionCoefficients[0],
                            leftCalibration.distortionCoefficients[1],
                            leftCalibration.distortionCoefficients[2],
                            leftCalibration.distortionCoefficients[3]};
    double array_dcr[4] = {rightCalibration.distortionCoefficients[0],
                            rightCalibration.distortionCoefficients[1],
                            rightCalibration.distortionCoefficients[2],
                            rightCalibration.distortionCoefficients[3]};

    double array_Kl[9] = {leftCalibration.focalLength[0], 0.0, leftCalibration.principalPoint[0],
                          0.0, leftCalibration.focalLength[1], leftCalibration.principalPoint[1],
                          0.0, 0.0, 1.0};
    double array_Kr[9] = {rightCalibration.focalLength[0], 0.0, rightCalibration.principalPoint[0],
                          0.0, rightCalibration.focalLength[1], rightCalibration.principalPoint[1],
                          0.0, 0.0, 1.0};

    Eigen::Matrix4d T_rl = rightCalibration.T_SC.T().inverse() * leftCalibration.T_SC.T();
    double array_Rrl[9] = {T_rl(0,0), T_rl(0,1), T_rl(0,2), 
                            T_rl(1,0), T_rl(1,1), T_rl(1,2),
                            T_rl(2,0), T_rl(2,1), T_rl(2,2)};
    double array_trl[3] = {T_rl(0,3), T_rl(1,3), T_rl(2,3)};

    cv::Mat Kl = cv::Mat(3, 3, CV_64F, array_Kl);
    cv::Mat Kr = cv::Mat(3, 3, CV_64F, array_Kr);
    cv::Mat dcl = cv::Mat(4, 1, CV_64F, array_dcl);
    cv::Mat dcr = cv::Mat(4, 1, CV_64F, array_dcr);
    cv::Mat R = cv::Mat(3, 3, CV_64F, array_Rrl);
    cv::Mat T = cv::Mat(3, 1, CV_64F, array_trl);

    // P1 is the rectified camera parameters, R1 is R_{rect}{unrect}
    cv::Mat R1, R2, P1, P2, Q;
    cv::Mat rectMapLeft1, rectMapLeft2, rectMapRight1, rectMapRight2;
    if (leftCalibration.distortionType == "radialtangential") {
      // alpha = 0 for only valid pixels after undistortion
      cv::stereoRectify(Kl, dcl, Kr, dcr, rawSize, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 
                        0, rectSize);
      cv::initUndistortRectifyMap(Kl, dcl, R1, P1, rectSize, CV_16SC2, rectMapLeft1, rectMapLeft2);
      cv::initUndistortRectifyMap(Kr, dcr, R2, P2, rectSize, CV_16SC2, rectMapRight1, rectMapRight2);
    }
    else if (leftCalibration.distortionType == "equidistant") {
      // balance = 0.0 and fov_scale from config for only valid pixels after undistortion
      cv::fisheye::stereoRectify(Kl, dcl, Kr, dcr, rawSize, R, T, R1, R2, P1, P2, Q,
                                  cv::CALIB_ZERO_DISPARITY, rectSize, 0.0, fov_scale);
      cv::fisheye::initUndistortRectifyMap(Kl, dcl, R1, P1, rectSize, CV_16SC2, rectMapLeft1, rectMapLeft2);
      cv::fisheye::initUndistortRectifyMap(Kr, dcr, R2, P2, rectSize, CV_16SC2, rectMapRight1, rectMapRight2);
    }

    Eigen::Matrix4d T1, T2;
    T1 << R1.at<double>(0,0), R1.at<double>(0,1), R1.at<double>(0,2), 0.0,
          R1.at<double>(1,0), R1.at<double>(1,1), R1.at<double>(1,2), 0.0,
          R1.at<double>(2,0), R1.at<double>(2,1), R1.at<double>(2,2), 0.0,
                        0.0,                 0.0,                0.0, 1.0;
    T2 << R2.at<double>(0,0), R2.at<double>(0,1), R2.at<double>(0,2), 0.0,
          R2.at<double>(1,0), R2.at<double>(1,1), R2.at<double>(1,2), 0.0,
          R2.at<double>(2,0), R2.at<double>(2,1), R2.at<double>(2,2), 0.0,
                        0.0,                 0.0,                0.0, 1.0;
    kinematics::Transformation T1_rect_unrect(T1);
    kinematics::Transformation T2_rect_unrect(T2);

    // Rectified left camera
    kinematics::Transformation T_SCl = leftCalibration.T_SC * T1_rect_unrect.inverse();
    std::shared_ptr<const kinematics::Transformation> T_SCl_okvis_ptr(
      new kinematics::Transformation(T_SCl.r(), T_SCl.q().normalized()));
    std::shared_ptr<cameras::PinholeCamera<cameras::NoDistortion>> rectify_caml;
    rectify_caml.reset(new cameras::PinholeCamera<cameras::NoDistortion>(
                        rectSize.width, rectSize.height,
                        P1.at<double>(0,0), P1.at<double>(1,1),
                        P1.at<double>(0,2), P1.at<double>(1,2),
                        cameras::NoDistortion()));
    rectify_caml->setRectifyMap(rectMapLeft1, rectMapLeft2);
    viParameters_.nCameraSystem.addRectifyCamera(stereo_indices[0], T_SCl_okvis_ptr, rectify_caml);

    // Rectified right camera
    kinematics::Transformation T_SCr = rightCalibration.T_SC * T2_rect_unrect.inverse();
        std::shared_ptr<const kinematics::Transformation> T_SCr_okvis_ptr(
      new kinematics::Transformation(T_SCr.r(), T_SCr.q().normalized()));
    std::shared_ptr<cameras::PinholeCamera<cameras::NoDistortion>> rectify_camr;
    rectify_camr.reset(new cameras::PinholeCamera<cameras::NoDistortion>(
                        rectSize.width, rectSize.height,
                        P2.at<double>(0,0), P2.at<double>(1,1),
                        P2.at<double>(0,2), P2.at<double>(1,2),
                        cameras::NoDistortion()));
    rectify_camr->setRectifyMap(rectMapRight1, rectMapRight2);
    viParameters_.nCameraSystem.addRectifyCamera(stereo_indices[1], T_SCr_okvis_ptr, rectify_camr);
    return true;
  } else {
    return false;
  }
}

}  // namespace okvis
