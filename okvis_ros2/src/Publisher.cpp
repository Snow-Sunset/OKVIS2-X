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
 * @file Publisher.cpp
 * @brief Source file for the Publisher class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <glog/logging.h>
#include <okvis/ros2/Publisher.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include <okvis/FrameTypedefs.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

// Default constructor.
Publisher::Publisher(
  std::shared_ptr<rclcpp::Node> node,
  std::shared_ptr<ThreadedPublisher> threadedOdometryPublisher,
  std::shared_ptr<ThreadedPublisher> threadedImagePublisher,
  std::shared_ptr<ThreadedPublisher> threadedPublisher)
    : threadedOdometryPublisher_(threadedOdometryPublisher),
      threadedImagePublisher_(threadedImagePublisher),
      threadedPublisher_(threadedPublisher),
      trajectoryOutput_(false),
      trajectoryLocked_(false)
{
  setupNode(node);
  text_embedding_vector_ = Descriptor(okvis::chair_embedding.data());
}

void Publisher::setupNode(std::shared_ptr<rclcpp::Node> node)
{
  // set up node
  node_ = node;
  const auto pathLimit = node_->declare_parameter<int64_t>("path_max_points", 10000);
  const auto landmarkLimit = node_->declare_parameter<int64_t>("landmark_max_points", 100000);
  if (pathLimit < 0 || landmarkLimit < 0)
    throw std::invalid_argument("Path and landmark display point limits must be nonnegative");
  pathMaxPoints_ = static_cast<size_t>(pathLimit);
  landmarkMaxPoints_ = static_cast<size_t>(landmarkLimit);

  // set up publishers
  slice_pub_ =              threadedPublisher_->registerPublisher<visualization_msgs::msg::Marker>("se_map_slice");
  pubObometry_ =            threadedOdometryPublisher_->registerPublisher<nav_msgs::msg::Odometry>("okvis_odometry");
  pubPath_ =                threadedPublisher_->registerPublisher<nav_msgs::msg::Path>("okvis_path");
  pubFrontendOdometry_ =    threadedOdometryPublisher_->registerPublisher<nav_msgs::msg::Odometry>("okvis_frontend_odometry");
  pubFrontendPath_ =        threadedPublisher_->registerPublisher<nav_msgs::msg::Path>("okvis_frontend_path");
  pubTransform_ =           threadedPublisher_->registerPublisher<geometry_msgs::msg::TransformStamped>("okvis_transform");
  pubMesh_ =                threadedPublisher_->registerPublisher<visualization_msgs::msg::Marker>("okvis_mesh");
  pubSubmapMesh_ =          threadedPublisher_->registerPublisher<visualization_msgs::msg::MarkerArray>("okvis_submap_mesh");
  pubPointsMatched_ =       threadedPublisher_->registerPublisher<sensor_msgs::msg::PointCloud2>("okvis_points_matched");
  pubPointsAlignment_ =     threadedPublisher_->registerPublisher<sensor_msgs::msg::PointCloud2>("okvis_points_alignment");
  pubLidarMap_ = threadedPublisher_->registerPublisher<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>(
      "okvis_lidar_map", nullptr, rclcpp::QoS(1).transient_local().reliable());
      
  // get the mesh, if there is one
  // where to get the mesh from
  std::string mesh_file;
  bool loaded_mesh;
  node_->declare_parameter("mesh_file", "");
  loaded_mesh = node_->get_parameter("mesh_file", mesh_file);
  meshMsg_ = std::make_shared<visualization_msgs::msg::Marker>();
  if (loaded_mesh) {
    meshMsg_->mesh_resource = mesh_file;

    // fill orientation
    meshMsg_->pose.orientation.x = 0;
    meshMsg_->pose.orientation.y = 0;
    meshMsg_->pose.orientation.z = 0;
    meshMsg_->pose.orientation.w = 1;

    // fill position
    meshMsg_->pose.position.x = 0;
    meshMsg_->pose.position.y = 0;
    meshMsg_->pose.position.z = 0;

    // scale -- needed
    meshMsg_->scale.x = 1.0;
    meshMsg_->scale.y = 1.0;
    meshMsg_->scale.z = 1.0;

    meshMsg_->action = visualization_msgs::msg::Marker::ADD;
    meshMsg_->color.a = 1.0; // Don't forget to set the alpha!
    meshMsg_->color.r = 1.0;
    meshMsg_->color.g = 1.0;
    meshMsg_->color.b = 1.0;

    // embedded material / colour
    //meshMsg_->mesh_use_embedded_materials = true;
  } else {
    LOG(INFO) << "no mesh found for visualisation, set ros param mesh_file, if desired";
    meshMsg_->mesh_resource = "";
  }
}

Publisher::~Publisher()
{
}

void Publisher::setBodyTransform(const okvis::kinematics::Transformation& T_BS) {
  T_BS_ = T_BS;
  T_SB_ = T_BS_.inverse();
}

void Publisher::setCsvFile(const std::string & filename, bool rpg)
{
  trajectoryOutput_.setCsvFile(filename, rpg);
}

bool Publisher::realtimePredictAndPublish(const okvis::Time& stamp,
                         const Eigen::Vector3d& alpha,
                         const Eigen::Vector3d& omega) {

  // store in any case
  imuMeasurements_.push_back(ImuMeasurement(stamp, ImuSensorReadings(omega,alpha)));

  // add to Trajectory if possible
  bool success = false;
  State state;
  if(!trajectoryLocked_) {
    trajectoryLocked_ = true;
    for(auto & imuMeasurement : imuMeasurements_) {
      State propagatedState;
      if(trajectory_.addImuMeasurement(imuMeasurement.timeStamp,
                                       imuMeasurement.measurement.accelerometers,
                                       imuMeasurement.measurement.gyroscopes,
                                       propagatedState)) {
        state = propagatedState;
        success = true;
      }
    }
    imuMeasurements_.clear();
    trajectoryLocked_ = false;
    if(!success) {
      return false;
    }
  } else {
    return false;
  }

  // only publish according to rate
  if((state.timestamp - lastTime_).toSec()
      < (1.0/double(odometryPublishingRate_))) {
    return false;
  }
  lastTime_ = state.timestamp;

  rclcpp::Time t(state.timestamp.sec, state.timestamp.nsec); // Header timestamp.
  const okvis::kinematics::Transformation T_WS = state.T_WS;
  const kinematics::Transformation T_SB = T_BS_.inverse();
  const okvis::kinematics::Transformation T_WB = T_WS * T_SB;

  // Odometry
  auto odometryMsg = std::make_shared<nav_msgs::msg::Odometry>();  // Odometry message.
  odometryMsg->header.frame_id = "world";
  odometryMsg->child_frame_id = "body";
  odometryMsg->header.stamp = t;

  // fill orientation
  const Eigen::Quaterniond q = T_WB.q();
  odometryMsg->pose.pose.orientation.x = q.x();
  odometryMsg->pose.pose.orientation.y = q.y();
  odometryMsg->pose.pose.orientation.z = q.z();
  odometryMsg->pose.pose.orientation.w = q.w();

  // fill position
  const Eigen::Vector3d r = T_WB.r();
  odometryMsg->pose.pose.position.x = r[0];
  odometryMsg->pose.pose.position.y = r[1];
  odometryMsg->pose.pose.position.z = r[2];

  // note: velocity and angular velocity needs to be expressed in child frame, i.e. "body"
  // see http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html

  // fill velocity
  const kinematics::Transformation T_BW = T_WB.inverse();
  const Eigen::Vector3d v = T_BW.C() * state.v_W + T_BS_.C() * T_SB.r().cross(state.omega_S);
  // ...of body orig. represented in body
  odometryMsg->twist.twist.linear.x = v[0];
  odometryMsg->twist.twist.linear.y = v[1];
  odometryMsg->twist.twist.linear.z = v[2];

  // fill angular velocity
  const Eigen::Matrix3d C_BS = T_BS_.C();
  const Eigen::Vector3d omega_B = C_BS * state.omega_S; // of body represented in body
  odometryMsg->twist.twist.angular.x = omega_B[0];
  odometryMsg->twist.twist.angular.y = omega_B[1];
  odometryMsg->twist.twist.angular.z = omega_B[2];

  // publish odometry
  pubObometry_.publish(odometryMsg);
  return true;
}

void Publisher::publishFrontendState(const State& state, const TrackingState& trackingState,
                                     uint64_t feedbackVersion) {
  if (trackingState.trackingQuality == TrackingQuality::Lost
      || !state.T_WS.T().allFinite() || !state.v_W.allFinite()
      || !state.omega_S.allFinite()) return;

  std::lock_guard<std::mutex> lock(frontendPublicationMutex_);
  if (hasFrontendState_ && (state.timestamp <= lastFrontendTime_
      || feedbackVersion < lastFrontendFeedbackVersion_)) return;

  const kinematics::Transformation T_SB = T_BS_.inverse();
  const kinematics::Transformation T_WB = state.T_WS * T_SB;
  const Eigen::Vector3d velocity_B = T_WB.C().transpose() * state.v_W
      + T_BS_.C() * state.omega_S.cross(T_SB.r());
  const Eigen::Vector3d omega_B = T_BS_.C() * state.omega_S;
  const auto timestamp = rclcpp::Time(state.timestamp.sec, state.timestamp.nsec);

  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->header.frame_id = "world";
  odometry->header.stamp = timestamp;
  odometry->child_frame_id = "body";
  auto& pose = odometry->pose.pose;
  pose.position.x = T_WB.r().x();
  pose.position.y = T_WB.r().y();
  pose.position.z = T_WB.r().z();
  pose.orientation.x = T_WB.q().x();
  pose.orientation.y = T_WB.q().y();
  pose.orientation.z = T_WB.q().z();
  pose.orientation.w = T_WB.q().w();
  // nav_msgs/Odometry twist is expressed at the child origin in the child frame.
  odometry->twist.twist.linear.x = velocity_B.x();
  odometry->twist.twist.linear.y = velocity_B.y();
  odometry->twist.twist.linear.z = velocity_B.z();
  odometry->twist.twist.angular.x = omega_B.x();
  odometry->twist.twist.angular.y = omega_B.y();
  odometry->twist.twist.angular.z = omega_B.z();

  geometry_msgs::msg::PoseStamped pathPose;
  pathPose.header = odometry->header;
  pathPose.pose = pose;
  frontendPathPoses_.push_back(std::move(pathPose));
  if (pathMaxPoints_)
    while (frontendPathPoses_.size() > pathMaxPoints_) frontendPathPoses_.pop_front();
  auto path = std::make_shared<nav_msgs::msg::Path>();
  path->header = odometry->header;
  path->poses.assign(frontendPathPoses_.begin(), frontendPathPoses_.end());

  lastFrontendTime_ = state.timestamp;
  lastFrontendFeedbackVersion_ = feedbackVersion;
  hasFrontendState_ = true;
  pubFrontendOdometry_.publish(odometry);
  pubFrontendPath_.publish(path);
  VLOG(2) << "Frontend pose t=" << state.timestamp
          << " feedback_version=" << feedbackVersion;
}

void Publisher::publishEstimatorUpdate(
  const State& state, const TrackingState & trackingState,
  std::shared_ptr<const AlignedMap<StateId, State>> updatedStates,
  std::shared_ptr<const MapPointVector> landmarks) {

  // forward to existing writer
  trajectoryOutput_.processState(state, trackingState, updatedStates, landmarks);

  // pose
  rclcpp::Time t(state.timestamp.sec, state.timestamp.nsec); // Header timestamp.
  const okvis::kinematics::Transformation T_WS = state.T_WS;
  const kinematics::Transformation T_SB = T_BS_.inverse();
  const okvis::kinematics::Transformation T_WB = T_WS * T_SB;

  // publish pose:
  auto poseMsg = std::make_shared<geometry_msgs::msg::TransformStamped>(); // Pose message.
  poseMsg->child_frame_id = "body";
  poseMsg->header.frame_id = "world";
  poseMsg->header.stamp = t;
  //if ((node_->now() - t).seconds() > 10.0)
  //  poseMsg->header.stamp = node_->now(); // hack for dataset processing

  // fill orientation
  Eigen::Quaterniond q = T_WB.q();
  poseMsg->transform.rotation.x = q.x();
  poseMsg->transform.rotation.y = q.y();
  poseMsg->transform.rotation.z = q.z();
  poseMsg->transform.rotation.w = q.w();

  // fill position
  Eigen::Vector3d r = T_WB.r();
  poseMsg->transform.translation.x = r[0];
  poseMsg->transform.translation.y = r[1];
  poseMsg->transform.translation.z = r[2];

  // publish current pose
  pubTransform_.publish(poseMsg);

  // also do the mesh
  meshMsg_->header.frame_id = "world";
  meshMsg_->header.stamp = t;
  //if ((node_->now() - t).seconds() > 10.0)
  //  meshMsg_->header.stamp = node_->now(); // hack for dataset processing
  meshMsg_->type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  
  meshMsg_->pose.position.x = T_WS.r().x();
  meshMsg_->pose.position.y = T_WS.r().y();
  meshMsg_->pose.position.z = T_WS.r().z();

  meshMsg_->pose.orientation.x = T_WS.q().x();
  meshMsg_->pose.orientation.y = T_WS.q().y();
  meshMsg_->pose.orientation.z = T_WS.q().z();
  meshMsg_->pose.orientation.w = T_WS.q().w();

  // publish mesh
  if(meshPublishingEnabled_ && !meshMsg_->mesh_resource.empty())
    pubMesh_.publish(meshMsg_);  //publish stamped mesh

  // check if empty
  if(updatedStates->empty()) {
    return; // otherwise it will override landmarks and path (plus create unnecessary traffic)
  }

  // now handle all the (key)frames & path:
  for(const auto & updatedState : *updatedStates) {
    rclcpp::Time updatedState_t(state.timestamp.sec, state.timestamp.nsec); // Header timestamp.
    const okvis::kinematics::Transformation T_WS = updatedState.second.T_WS;

    auto updatedStatePoseMsg = std::make_shared<geometry_msgs::msg::TransformStamped>(); // Pose message.
    const okvis::kinematics::Transformation T_WB = T_WS * T_SB;
    updatedStatePoseMsg->child_frame_id = "body_"+std::to_string(updatedState.second.id.value());
    updatedStatePoseMsg->header.frame_id = "world";
    updatedStatePoseMsg->header.stamp = updatedState_t;
    updatedStatePoseMsg->transform.rotation.x = T_WB.q().x();
    updatedStatePoseMsg->transform.rotation.y = T_WB.q().y();
    updatedStatePoseMsg->transform.rotation.z = T_WB.q().z();
    updatedStatePoseMsg->transform.rotation.w = T_WB.q().w();
    updatedStatePoseMsg->transform.translation.x = T_WB.r()[0];
    updatedStatePoseMsg->transform.translation.y = T_WB.r()[1];
    updatedStatePoseMsg->transform.translation.z = T_WB.r()[2];

    // publish
    pubTransform_.publish(updatedStatePoseMsg);
  }

  // update Trajectory object
  std::set<okvis::StateId> affectedStateIds;
  while(trajectoryLocked_);
  trajectoryLocked_ = true;
  trajectory_.update(trackingState, updatedStates, affectedStateIds);
  trajectoryLocked_ = false;

  // Bound display message memory independently of the complete trajectory.
  auto path = std::make_shared<nav_msgs::msg::Path>();
  path->header.stamp = t;
  path->header.frame_id = "world";
  const auto& stateIds = trajectory_.stateIds();
  const size_t pathLimit = pathMaxPoints_ ? std::min(pathMaxPoints_, stateIds.size()) : stateIds.size();
  path->poses.reserve(pathLimit);
  size_t pathIndex = 0;
  size_t nextPathIndex = pathLimit == 1 ? stateIds.size() - 1 : 0;
  for(const auto& stateId : stateIds) {
    if (pathIndex++ != nextPathIndex) continue;
    // Include both endpoints when at least two poses fit in the display budget.
    if (pathLimit > 1)
      nextPathIndex = (path->poses.size() + 1) * (stateIds.size() - 1) / (pathLimit - 1);
    else
      nextPathIndex = stateIds.size();
    okvis::State state;
    if(!trajectory_.getState(stateId, state)) {
      continue;
    }

    const okvis::kinematics::Transformation T_WB_p = state.T_WS * T_SB;
    const Eigen::Vector3d& r = T_WB_p.r();
    const Eigen::Quaterniond& q = T_WB_p.q();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.header.stamp = rclcpp::Time(state.timestamp.sec, state.timestamp.nsec);
    pose.pose.position.x = r[0];
    pose.pose.position.y = r[1];
    pose.pose.position.z = r[2];
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    path->poses.push_back(pose);
  }
  pubPath_.publish(path);

  // finally the landmarks
  pcl::PointCloud<pcl::PointXYZRGB> pointsMatched; // Point cloud for matched points.
  const size_t landmarkLimit = landmarkMaxPoints_ ? std::min(landmarkMaxPoints_, landmarks->size()) : landmarks->size();
  pointsMatched.reserve(landmarkLimit);
  const size_t landmarkStride = landmarkLimit ? (landmarks->size() + landmarkLimit - 1) / landmarkLimit : 1;
  size_t landmarkIndex = 0;

  // transform points into custom world frame:
  /// \todo properly from ros params -- also landmark thresholds below
  for (const auto & lm : *landmarks) {
    if (landmarkIndex++ % landmarkStride != 0) continue;
    // check infinity
    if (fabs((double) (lm.point[3])) < 1.0e-8)
      continue;

    // check quality
    if (lm.quality < 0.01)
      continue;

    pointsMatched.push_back(pcl::PointXYZRGB());
    const Eigen::Vector4d point = lm.point;
    pointsMatched.back().x = point[0] / point[3];
    pointsMatched.back().y = point[1] / point[3];
    pointsMatched.back().z = point[2] / point[3];
    pointsMatched.back().g = 255 * (std::min(0.1f, (float)lm.quality) / 0.1f);
  }
  pointsMatched.header.frame_id = "world";
  auto pointsMatchedMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(pointsMatched, *pointsMatchedMsg);
  pointsMatchedMsg->header.frame_id = "world";

#if PCL_VERSION >= PCL_VERSION_CALC(1,7,0)
  std_msgs::msg::Header header;
  header.stamp = t;
  pointsMatched.header.stamp = pcl_conversions::toPCL(header).stamp;
#else
  pointsMatched.header.stamp=_t;
#endif

  // and now publish them:
  pubPointsMatched_.publish(pointsMatchedMsg);

}

void Publisher::setupImageTopics(const okvis::cameras::NCameraSystem & nCameraSystem) {
  pubImages_.clear();
  bool rgb_found = false;
  for(size_t i=0; i< nCameraSystem.numCameras(); ++i) {
    std::string name;
    if(nCameraSystem.cameraType(i).isColour) {
      name = "rgb"+std::to_string(i);
      rgb_found = true;
    } else {
      name = "cam"+std::to_string(i);
    }
    pubImages_[name] = threadedImagePublisher_->registerPublisher<sensor_msgs::msg::Image>(name+"_matches");
  }
  std::string name = "Top Debug View";
  pubImages_[name] = threadedImagePublisher_->registerPublisher<sensor_msgs::msg::Image>("top_debug_view");
  skip_uncolored_ = rgb_found;
#ifdef OKVIS_COLIDMAP
  name = "language_rgb";
  pubImages_[name] = threadedImagePublisher_->registerPublisher<sensor_msgs::msg::Image>("language_rgb");
  name = "sam_masks";
  pubImages_[name] = threadedImagePublisher_->registerPublisher<sensor_msgs::msg::Image>("sam_masks");  
#endif
}

void Publisher::setMeshesPath(std::string meshesDir){
  meshesDir_ = meshesDir;
}

bool Publisher::publishImages(const std::map<std::string, cv::Mat>& images) const {
  for(const auto & image : images) {
    sensor_msgs::msg::Image::SharedPtr msg
      = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image.second).toImageMsg();
    const auto & pubIter = pubImages_.find(image.first);
    if(pubIter == pubImages_.end()) {
      continue;
    }
    pubIter->second.publish(msg);
  }
  return true;
}

void Publisher::setupNetworkTopics(const std::string & topicName) {
  // TODO: properly set topic names
  std::string name = topicName + "Depth";
  pubImages_[name] = threadedImagePublisher_->registerPublisher<sensor_msgs::msg::Image>("network_depth");

  name = topicName + "Sigma";
  pubImages_[name] = threadedImagePublisher_->registerPublisher<sensor_msgs::msg::Image>("network_sigma");
}

void Publisher::publishSubmapsAsCallback(std::unordered_map<uint64_t, okvis::kinematics::Transformation, std::hash<uint64_t>, std::equal_to<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, okvis::kinematics::Transformation>>> submapPoseLookup,
                                         std::unordered_map<uint64_t, std::shared_ptr<okvis::SupereightMapType>> submapLookup,
                                         std::shared_ptr<okvis::ObjectMap> objectMap) 
{
  if (!meshPublishingEnabled_) return;
  constexpr size_t n = okvis::SupereightMapType::SurfaceMesh::value_type::num_vertexes;

  std_msgs::msg::Header header;
  header.stamp = node_->now();
  header.frame_id = "world";

  // iterate over updated submaps and add them to our attribute holding the different submaps
  for (auto& it: submapLookup) {
    visualization_msgs::msg::Marker meshMarker;
    meshMarker.header = header;
    meshMarker.pose.orientation.w = 1;
    meshMarker.ns = "submaps"; 
    meshMarker.id = it.first;
    meshMarker.scale.x = 1;
    meshMarker.scale.y = 1;
    meshMarker.scale.z = 1;
    meshMarker.color.r = 0.8; 
    meshMarker.color.g = 0.8;
    meshMarker.color.b = 0.8;
    meshMarker.color.a = 1;
    meshMarker.lifetime = rclcpp::Duration(std::chrono::seconds(0)); // 0 for infinity
    meshMarker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    meshMarker.action = visualization_msgs::msg::Marker::ADD;

    bool containsObjects = objectMap->containsMap(it.first);

    if(submapSurfaceMesh_.find(it.first) == submapSurfaceMesh_.end()){
      submapSurfaceMesh_[it.first] = se::algorithms::marching_cube(it.second->getOctree());
      if(containsObjects) {
        submapObjects_[it.first] = & objectMap->submapObjects(it.first);
      }
    }
    auto & mesh = submapSurfaceMesh_[it.first];

    meshMarker.points.clear();
    meshMarker.points.resize(n * mesh.size());

    Eigen::Matrix4f T_OW = submapPoseLookup[it.first].T().cast<float>();
    Eigen::Matrix4f T_WM_scale = it.second->getTWM().matrix();
    T_WM_scale.topLeftCorner<3, 3>() *= it.second->getRes();
    const Eigen::Matrix4f T_OM = T_OW * T_WM_scale;

    submapPoses_[it.first] = T_OM;
    size_t valMeshCnt = 0;

    #ifdef OKVIS_COLIDMAP
    if(skip_uncolored_){
      meshMarker.colors.clear();
      meshMarker.colors.resize(n * mesh.size());
    }
    if( publishMode_ == "Colors" ) {
      for (size_t i = 0; i < mesh.size(); i++) {
        const Eigen::Vector3f v0_W = (T_OM * (mesh[i].vertexes[0].homogeneous())).template head<3>();
        const Eigen::Vector3f v1_W = (T_OM * (mesh[i].vertexes[1].homogeneous())).template head<3>();
        const Eigen::Vector3f v2_W = (T_OM * (mesh[i].vertexes[2].homogeneous())).template head<3>();
        const float triangle_max_z_W = Eigen::Vector3f(v0_W.z(), v1_W.z(), v2_W.z()).maxCoeff();
        if (triangle_max_z_W > mesh_cutoff_z_) {
          continue;
        }

        meshMarker.points.resize(n * mesh.size());
        tf::pointEigenToMsg(v0_W.template cast<double>(), meshMarker.points[n * valMeshCnt + 0]);
        tf::pointEigenToMsg(v1_W.template cast<double>(), meshMarker.points[n * valMeshCnt + 1]);
        tf::pointEigenToMsg(v2_W.template cast<double>(), meshMarker.points[n * valMeshCnt + 2]);
        if constexpr(okvis::SupereightMapType::SurfaceMesh::value_type::col_ == se::Colour::On) { // Check for valid colours
          if(!skip_uncolored_){
            goto skip_color;
          }
          if(!mesh[i].colour.vertexes) {
            continue;
          }
          for(size_t v = 0; v < n; v++) {
            auto& msg_colour = meshMarker.colors[n * valMeshCnt + v];
            const auto& mesh_colour = mesh[i].colour.vertexes.value()[v];
            msg_colour.r = float(mesh_colour.r) / UINT8_MAX;
            msg_colour.g = float(mesh_colour.g) / UINT8_MAX;
            msg_colour.b = float(mesh_colour.b) / UINT8_MAX;
            msg_colour.a = 1.0;
          }
          skip_color:;
        }
        valMeshCnt ++;
      }
      if(skip_uncolored_) {
        meshMarker.points.resize(n * valMeshCnt);
        meshMarker.colors.resize(n * valMeshCnt);
      }
      submapMeshLookup_rgb_[it.first] = meshMarker;
    }
    else if ( containsObjects && publishMode_ == "Activations" ) {
      Descriptor query(text_embedding_vector_.data());
      auto& submab_objects = objectMap->submapObjects(it.first);
      // Eigen::Vector3f light_dir_in_K = T_OW.topLeftCorner<3,3>().transpose() * Eigen::Vector3f(0., -1., -1.); 
      colour_mesh_by_match(mesh, submab_objects, query, tinycolormap::ColormapType::Jet, 0.1, 0.5);

      for (size_t i = 0; i < mesh.size(); i++) {
        const Eigen::Vector3f v0_W = (T_OM * (mesh[i].vertexes[0].homogeneous())).template head<3>();
        const Eigen::Vector3f v1_W = (T_OM * (mesh[i].vertexes[1].homogeneous())).template head<3>();
        const Eigen::Vector3f v2_W = (T_OM * (mesh[i].vertexes[2].homogeneous())).template head<3>();
        const float triangle_max_z_W = Eigen::Vector3f(v0_W.z(), v1_W.z(), v2_W.z()).maxCoeff();
        if (triangle_max_z_W > mesh_cutoff_z_) {
          continue;
        }
        for (size_t j = 0; j < n; j++) {

          if(!(mesh[i].id.id == se::g_not_mapped || mesh[i].id.id == se::g_no_id)){
            meshMarker.points.emplace_back();
            const Eigen::Vector3f vertex_W = (T_OM * mesh[i].vertexes[j].homogeneous()).template head<3>();
            meshMarker.points[valMeshCnt].x = vertex_W.x();
            meshMarker.points[valMeshCnt].y = vertex_W.y();
            meshMarker.points[valMeshCnt].z = vertex_W.z();
            std_msgs::msg::ColorRGBA face_color;
            face_color.a = 1;
            face_color.r = float(mesh[i].colour.face.value().r) / UINT8_MAX;
            face_color.g = float(mesh[i].colour.face.value().g) / UINT8_MAX;
            face_color.b = float(mesh[i].colour.face.value().b) / UINT8_MAX;
            meshMarker.colors[valMeshCnt] = face_color;
            valMeshCnt ++;
          }
        }
      }
      meshMarker.points.resize(valMeshCnt);
      meshMarker.colors.resize(valMeshCnt);
      submapMeshLookup_embedding_[it.first] = meshMarker;
    }
    #else
    for (size_t i = 0; i < mesh.size(); i++) {
      const Eigen::Vector3f v0_W = (T_OM * (mesh[i].vertexes[0].homogeneous())).template head<3>();
      const Eigen::Vector3f v1_W = (T_OM * (mesh[i].vertexes[1].homogeneous())).template head<3>();
      const Eigen::Vector3f v2_W = (T_OM * (mesh[i].vertexes[2].homogeneous())).template head<3>();
      const float triangle_max_z_W = Eigen::Vector3f(v0_W.z(), v1_W.z(), v2_W.z()).maxCoeff();
      if (triangle_max_z_W > mesh_cutoff_z_) {
        continue;
      }
  
      tf::pointEigenToMsg(v0_W.template cast<double>(), meshMarker.points[n * valMeshCnt + 0]);
      tf::pointEigenToMsg(v1_W.template cast<double>(), meshMarker.points[n * valMeshCnt + 1]);
      tf::pointEigenToMsg(v2_W.template cast<double>(), meshMarker.points[n * valMeshCnt + 2]);
      valMeshCnt ++;
    }
    meshMarker.points.resize(n * valMeshCnt);
    submapMeshLookup_rgb_[it.first] = meshMarker;
    #endif
  }

  auto markerarraymsg_ = std::make_shared<visualization_msgs::msg::MarkerArray>();
  markerarraymsg_->markers.clear();
  //If too many submaps are to be updated at once (> 1GB) ROS considers it has gone out of sync and does not update.
  // To prevent this, we generate buffers of submaps and send them in smaller packets to bypass this issue
  int submap_publisher_buffer = 0;

  if(publishMode_ == "Colors"){
    for(auto it: submapMeshLookup_rgb_){
      it.second.header = header;
      markerarraymsg_->markers.push_back(it.second);
      submap_publisher_buffer++;

      if(submap_publisher_buffer == 30){
        submap_publisher_buffer = 0;
        pubSubmapMesh_.publish(markerarraymsg_);
        markerarraymsg_->markers.clear();
      }
    }
  }
  else if(publishMode_ == "Activations"){
    for(auto it: submapMeshLookup_embedding_){
      it.second.header = header;
      markerarraymsg_->markers.push_back(it.second);
      submap_publisher_buffer++;

      if(submap_publisher_buffer == 30){
        submap_publisher_buffer = 0;
        pubSubmapMesh_.publish(markerarraymsg_);
        markerarraymsg_->markers.clear();
      }
    }
  }
  
  if(markerarraymsg_->markers.size() > 0) {
      pubSubmapMesh_.publish(markerarraymsg_);
  }
  
  return;
}

void Publisher::publishFieldSliceAsCallback(
    const State& latest_state,
    const AlignedUnorderedMap<uint64_t, se::Submap<okvis::SupereightMapType>>& seSubmapLookup)
{
  // Get the latest submap.
  const auto it = std::max_element(seSubmapLookup.begin(),
                                   seSubmapLookup.end(),
                                   [](const auto& lhs, const auto& rhs) -> bool {
                                     return lhs.first < rhs.first;
                                   });
  if (it == seSubmapLookup.end()) {
    return;
  }
  const kinematics::Transformation T_WK(seSubmapLookup.at(it->first).T_WK.matrix().cast<double>());
  const kinematics::Transformation T_KW = T_WK.inverse();
  const Eigen::Matrix4f T_WK_e = T_WK.T().cast<float>();
  auto msg = std::make_shared<visualization_msgs::msg::Marker>();
  msg->header.stamp.sec = latest_state.timestamp.sec;
  msg->header.stamp.nanosec = latest_state.timestamp.nsec;
  msg->header.frame_id = "world";
  msg->ns = "field_slice";
  msg->id = 0;
  msg->type = visualization_msgs::msg::Marker::TRIANGLE_LIST; //CUBE_LIST
  msg->action = visualization_msgs::msg::Marker::ADD;
  msg->pose.orientation.w = 1;
  msg->scale.x = 1;
  msg->scale.y = 1;
  msg->scale.z = 1;
  const Eigen::Vector3d t_WB = (latest_state.T_WS * T_SB_).r();
  // The slice frame Sl is axis-aligned with the world frame W and has the same
  // origin {O} as the body frame B.
  const kinematics::Transformation T_WSl(t_WB, Eigen::Quaterniond(1, 0, 0, 0));
  Eigen::Matrix4f T_KO = (T_KW * T_WSl).T().cast<float>();
  se::QuadMesh<se::Colour::On, se::Id::Off> quad_mesh;
  quad_mesh.reserve(se::math::sq(it->second.map->getOctree().getSize()));
  const Eigen::Array3f aabb_min = it->second.map->aabb().min();
  const Eigen::Array3f aabb_max = it->second.map->aabb().max();
  const Eigen::Vector3f dim = it->second.map->getDim();
  float res = it->second.map->getRes();
  for (float y = -dim.y(); y < dim.y(); y += res) {
      for (float x = -dim.x(); x < dim.x(); x += res) {
          const Eigen::Vector3f point_O(x, y, 0);
          const Eigen::Vector3f point_W = (T_KO * point_O.homogeneous()).head<3>();
          if ((point_W.array() < aabb_min).any() || (point_W.array() > aabb_max).any()) {
              // Skip points outside the map AABB for speed.
              continue;
          }
          Eigen::Vector3i voxel_coord;
          it->second.map->pointToVoxel<se::Safe::Off>(point_W, voxel_coord);

          // Check occupancy to skip faces in unknown space
          se::field_t occ = se::get_field(se::visitor::getData(it->second.map->getOctree(), voxel_coord));
          se::RGB slice_face_color{0, 0, 0};
          if (occ > 0) {
            slice_face_color.r = 255;
          } else if (occ < -15.0f) {
            slice_face_color.g = 255;
          } else {
            continue;
          }

          quad_mesh.emplace_back();
          quad_mesh.back().colour.face = slice_face_color;
          quad_mesh.back().vertexes[0] = point_O;
          quad_mesh.back().vertexes[1] = point_O + Eigen::Vector3f(res, 0, 0);
          quad_mesh.back().vertexes[2] = point_O + Eigen::Vector3f(res, res, 0);
          quad_mesh.back().vertexes[3] = point_O + Eigen::Vector3f(0, res, 0);
          for (auto& v : quad_mesh.back().vertexes) {
              v = (T_KO * v.homogeneous()).template head<3>();
          }
      }
  }

  // Convert Mesh to triangle mesh
  auto mesh = se::quad_to_triangle_mesh(quad_mesh);
  constexpr int num_face_vertexes = 3;
  msg->points.reserve(num_face_vertexes * mesh.size());
  msg->colors.reserve(num_face_vertexes * mesh.size());
  for (const auto& face : mesh) {
    std_msgs::msg::ColorRGBA face_color;
    face_color.a = 1.0f;
    face_color.r = face.colour.face.value().r / 255.0f;
    face_color.g = face.colour.face.value().g / 255.0f;
    face_color.b = face.colour.face.value().b / 255.0f;
    for (size_t j = 0; j < num_face_vertexes; j++) {
      msg->points.emplace_back();
      const Eigen::Vector3f vertex_W = (T_WK_e * face.vertexes[j].homogeneous()).template head<3>();
      msg->points.back().x = vertex_W.x();
      msg->points.back().y = vertex_W.y();
      msg->points.back().z = vertex_W.z();
      msg->colors.push_back(face_color);
    }
  }
  slice_pub_.publish(msg);
}

void Publisher::publishLidarPointCloudMap(const Time& timestamp, const LidarPointCloudMap::Points& points) {
  // Write packed XYZ directly into the ROS payload. A full global map must not
  // require an additional PCL point array or per-point alignment padding.
  static_assert(sizeof(float) == 4, "PointCloud2 FLOAT32 requires 4-byte floats");
  constexpr uint32_t pointStep = 3 * sizeof(float);
  // An unorganised cloud has height 1, so this also bounds width and prevents
  // the uint32 row_step multiplication from overflowing.
  if (points.size() > std::numeric_limits<uint32_t>::max() / pointStep)
    throw std::length_error("LiDAR PointCloud2 row exceeds the ROS message size limit");
  auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
  if (points.size() > message->data.max_size() / pointStep)
    throw std::length_error("LiDAR PointCloud2 payload exceeds vector capacity");
  message->header.frame_id = "world";
  message->header.stamp = rclcpp::Time(timestamp.sec, timestamp.nsec);
  message->height = 1;
  message->width = static_cast<uint32_t>(points.size());
  message->point_step = pointStep;
  message->row_step = message->width * pointStep;
  const uint16_t byteOrder = 1;
  message->is_bigendian = *reinterpret_cast<const uint8_t*>(&byteOrder) == 0;
  message->is_dense = true;
  message->fields.resize(3);
  const char* names[] = {"x", "y", "z"};
  for (size_t axis = 0; axis < 3; ++axis) {
    auto& field = message->fields[axis];
    field.name = names[axis];
    field.offset = static_cast<uint32_t>(axis * sizeof(float));
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
  }
  message->data.resize(points.size() * pointStep);
  size_t offset = 0;
  for (const auto& point : points) {
    const float xyz[] = {float(point.x()), float(point.y()), float(point.z())};
    std::memcpy(message->data.data() + offset, xyz, pointStep);
    message->is_dense = message->is_dense && std::isfinite(xyz[0]) &&
        std::isfinite(xyz[1]) && std::isfinite(xyz[2]);
    offset += pointStep;
  }
  pubLidarMap_.publish(message);
}

void Publisher::publishAlignmentPointsAsCallback(const okvis::Time& timestamp, const okvis::kinematics::Transformation& T_WS,
                                                 const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& alignPointCloud,
                                                 bool isMapFrame){

  // finally the landmarks
  pcl::PointCloud<pcl::PointXYZRGB> alignPoints; // Point cloud for matched points.
  alignPoints.reserve(alignPointCloud.size());

  rclcpp::Time t(timestamp.sec, timestamp.nsec); // Header timestamp.

  // transform points into custom world frame:
  /// \todo properly from ros params -- also landmark thresholds below
  for (const auto & pt : alignPointCloud) {

    alignPoints.push_back(pcl::PointXYZRGB());
    const Eigen::Vector3f point = T_WS.T3x4().cast<float>() * pt.homogeneous();
    alignPoints.back().x = point[0];
    alignPoints.back().y = point[1];
    alignPoints.back().z = point[2];
    if(isMapFrame){
      alignPoints.back().r = 255.0f;
      alignPoints.back().g = 140.0f;
      alignPoints.back().b = 0.0f;
    }
    else{
      alignPoints.back().r = 255.0f;
      alignPoints.back().g = 215.0f;
      alignPoints.back().b = 0.0f;
    }
  }
  alignPoints.header.frame_id = "world";

#if PCL_VERSION >= PCL_VERSION_CALC(1,7,0)
  std_msgs::msg::Header header;
  header.stamp = t;
  alignPoints.header.stamp = pcl_conversions::toPCL(header).stamp;
#else
  alignPoints.header.stamp=_t;
#endif

  // and now publish them:
  auto alignPointsMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(alignPoints, *alignPointsMsg);
  pubPointsAlignment_.publish(alignPointsMsg);
}

void Publisher::publishRealTimePropagation(const okvis::Time& time, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, 
                                           const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity)
{
  rclcpp::Time t(time.sec, time.nsec);
  
  // Odometry
  auto odometryMsg = std::make_shared<nav_msgs::msg::Odometry>();  // Odometry message.
  odometryMsg->header.frame_id = "world";
  odometryMsg->child_frame_id = "body";
  odometryMsg->header.stamp = t;

  // fill orientation
  odometryMsg->pose.pose.orientation.x = orientation.x();
  odometryMsg->pose.pose.orientation.y = orientation.y();
  odometryMsg->pose.pose.orientation.z = orientation.z();
  odometryMsg->pose.pose.orientation.w = orientation.w();

  // fill position
  odometryMsg->pose.pose.position.x = position[0];
  odometryMsg->pose.pose.position.y = position[1];
  odometryMsg->pose.pose.position.z = position[2];

  // note: velocity and angular velocity needs to be expressed in child frame, i.e. "body"
  // see http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html

  // fill velocity
  // ...of body orig. represented in body
  odometryMsg->twist.twist.linear.x = linear_velocity[0];
  odometryMsg->twist.twist.linear.y = linear_velocity[1];
  odometryMsg->twist.twist.linear.z = linear_velocity[2];

  // fill angular velocity
  odometryMsg->twist.twist.angular.x = angular_velocity[0];
  odometryMsg->twist.twist.angular.y = angular_velocity[1];
  odometryMsg->twist.twist.angular.z = angular_velocity[2];

  // publish odometry
  pubObometry_.publish(odometryMsg);
  lastOdom_ = *odometryMsg;
}

void Publisher::republishMeshes()
{
  if(submapSurfaceMesh_.size() == 0){
    LOG(ERROR) << "No meshes present for re-meshing";
    return;
  }
  else{
    LOG(INFO) << "Re-Meshing " << submapSurfaceMesh_.size() << " meshes for text query";
  }

  constexpr size_t n = SupereightMapType::SurfaceMesh::value_type::num_vertexes;

  std_msgs::msg::Header header;
  header.stamp = node_->now();
  header.frame_id = "world";

  // Note: ToDo: Text_embedding_vector_ is set before
  for(auto cached_mesh : submapSurfaceMesh_){
    visualization_msgs::msg::Marker meshMarker;
    meshMarker.header = header;
    meshMarker.pose.orientation.w = 1;
    meshMarker.ns = "submaps"; 
    meshMarker.id = cached_mesh.first;
    meshMarker.scale.x = 1;
    meshMarker.scale.y = 1;
    meshMarker.scale.z = 1;
    meshMarker.color.r = 0.8;
    meshMarker.color.g = 0.8;
    meshMarker.color.b = 0.8;
    meshMarker.color.a = 1;
    meshMarker.lifetime = rclcpp::Duration(std::chrono::seconds(0)); // 0 for infinity
    meshMarker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    meshMarker.action = visualization_msgs::msg::Marker::ADD;

    auto & mesh = submapSurfaceMesh_[cached_mesh.first];

    meshMarker.points.clear();
    meshMarker.points.resize(n * mesh.size());
    size_t valMeshCnt = 0;

    #ifdef OKVIS_COLIDMAP
    meshMarker.colors.clear();
    meshMarker.colors.resize(n * mesh.size());
    if(publishMode_ == "Colors")
    {
      for (size_t i = 0; i < mesh.size(); i++) {
        const Eigen::Vector3f v0_W = (submapPoses_[cached_mesh.first] * (mesh[i].vertexes[0].homogeneous())).template head<3>();
        const Eigen::Vector3f v1_W = (submapPoses_[cached_mesh.first] * (mesh[i].vertexes[1].homogeneous())).template head<3>();
        const Eigen::Vector3f v2_W = (submapPoses_[cached_mesh.first] * (mesh[i].vertexes[2].homogeneous())).template head<3>();
        const float triangle_max_z_W = Eigen::Vector3f(v0_W.z(), v1_W.z(), v2_W.z()).maxCoeff();
        if (triangle_max_z_W > mesh_cutoff_z_) {
          continue;
        }

        tf::pointEigenToMsg(v0_W.template cast<double>(), meshMarker.points[n * valMeshCnt + 0]);
        tf::pointEigenToMsg(v1_W.template cast<double>(), meshMarker.points[n * valMeshCnt + 1]);
        tf::pointEigenToMsg(v2_W.template cast<double>(), meshMarker.points[n * valMeshCnt + 2]);
      
        if constexpr(okvis::SupereightMapType::SurfaceMesh::value_type::col_ == se::Colour::On) {
          // Check for valid colours
          if(!mesh[i].colour.vertexes) {
            continue;
          }
          for(size_t v = 0; v < n; v++) {
            auto& msg_colour = meshMarker.colors[n * valMeshCnt + v];
            const auto& mesh_colour = mesh[i].colour.vertexes.value()[v];
            msg_colour.r = float(mesh_colour.r) / UINT8_MAX;
            msg_colour.g = float(mesh_colour.g) / UINT8_MAX;
            msg_colour.b = float(mesh_colour.b) / UINT8_MAX;
            msg_colour.a = 1.0;
          }
        }
        valMeshCnt ++;
      }
      meshMarker.points.resize(n * valMeshCnt);
      meshMarker.colors.resize(n * valMeshCnt);
      submapMeshLookup_rgb_[cached_mesh.first] = meshMarker;
    }
    else if(publishMode_ == "Activations")
    {
      Descriptor query(text_embedding_vector_.data());
      auto& submab_objects = submapObjects_[cached_mesh.first];
      // Eigen::Vector3f light_dir_in_K = T_OW.topLeftCorner<3,3>().transpose() * Eigen::Vector3f(0., -1., -1.); 
      colour_mesh_by_match(mesh, *submab_objects, query, tinycolormap::ColormapType::Jet, 0.1, 0.5);

      for (size_t i = 0; i < mesh.size(); i++) {
        const Eigen::Vector3f v0_W = (submapPoses_[cached_mesh.first] * (mesh[i].vertexes[0].homogeneous())).template head<3>();
        const Eigen::Vector3f v1_W = (submapPoses_[cached_mesh.first] * (mesh[i].vertexes[1].homogeneous())).template head<3>();
        const Eigen::Vector3f v2_W = (submapPoses_[cached_mesh.first] * (mesh[i].vertexes[2].homogeneous())).template head<3>();
        const float triangle_max_z_W = Eigen::Vector3f(v0_W.z(), v1_W.z(), v2_W.z()).maxCoeff();
        if (triangle_max_z_W > mesh_cutoff_z_) {
          continue;
        }
        for (size_t j = 0; j < n; j++) {
          if(!(mesh[i].id.id == se::g_not_mapped || mesh[i].id.id == se::g_no_id)){
            meshMarker.points.emplace_back();
            const Eigen::Vector3f vertex_W = (submapPoses_[cached_mesh.first] * mesh[i].vertexes[j].homogeneous()).template head<3>();
            meshMarker.points[valMeshCnt].x = vertex_W.x();
            meshMarker.points[valMeshCnt].y = vertex_W.y();
            meshMarker.points[valMeshCnt].z = vertex_W.z();
            std_msgs::msg::ColorRGBA face_color;
            face_color.a = 1;
            face_color.r = float(mesh[i].colour.face.value().r) / UINT8_MAX;
            face_color.g = float(mesh[i].colour.face.value().g) / UINT8_MAX;
            face_color.b = float(mesh[i].colour.face.value().b) / UINT8_MAX;
            meshMarker.colors[valMeshCnt] = face_color;
            valMeshCnt ++;
          }
        }
      }
      meshMarker.points.resize(valMeshCnt);
      meshMarker.colors.resize(valMeshCnt);
      submapMeshLookup_embedding_[cached_mesh.first] = meshMarker;
    }
    #else
    for (size_t i = 0; i < mesh.size(); i++) {
      const Eigen::Vector3f v0_W = (submapPoses_[cached_mesh.first] * (mesh[i].vertexes[0].homogeneous())).template head<3>();
      const Eigen::Vector3f v1_W = (submapPoses_[cached_mesh.first] * (mesh[i].vertexes[1].homogeneous())).template head<3>();
      const Eigen::Vector3f v2_W = (submapPoses_[cached_mesh.first] * (mesh[i].vertexes[2].homogeneous())).template head<3>();
      const float triangle_max_z_W = Eigen::Vector3f(v0_W.z(), v1_W.z(), v2_W.z()).maxCoeff();
      if (triangle_max_z_W > mesh_cutoff_z_) {
        continue;
      }

      tf::pointEigenToMsg(v0_W.template cast<double>(), meshMarker.points[n * valMeshCnt + 0]);
      tf::pointEigenToMsg(v1_W.template cast<double>(), meshMarker.points[n * valMeshCnt + 1]);
      tf::pointEigenToMsg(v2_W.template cast<double>(), meshMarker.points[n * valMeshCnt + 2]);
      valMeshCnt ++;
    }
    meshMarker.points.resize(n * valMeshCnt);
    submapMeshLookup_rgb_[cached_mesh.first] = meshMarker;
    #endif
  }

  auto markerarraymsg_ = std::make_shared<visualization_msgs::msg::MarkerArray>();
  markerarraymsg_->markers.clear();
  //If too many submaps are to be updated at once (> 1GB) ROS considers it has gone out of sync and does not update.
  // To prevent this, we generate buffers of submaps and send them in smaller packets to bypass this issue
  int submap_publisher_buffer = 0;
  if(publishMode_ == "Colors"){
    for(auto it: submapMeshLookup_rgb_){
      it.second.header = header;
      markerarraymsg_->markers.push_back(it.second);
      submap_publisher_buffer++;

      if(submap_publisher_buffer == 30){
        submap_publisher_buffer = 0;
        pubSubmapMesh_.publish(markerarraymsg_);
        markerarraymsg_->markers.clear();
      }
    }
  }
  else if(publishMode_ == "Activations"){
    for(auto it: submapMeshLookup_embedding_){
      it.second.header = header;
      markerarraymsg_->markers.push_back(it.second);
      submap_publisher_buffer++;

      if(submap_publisher_buffer == 30){
        submap_publisher_buffer = 0;
        pubSubmapMesh_.publish(markerarraymsg_);
        markerarraymsg_->markers.clear();
      }
    }
  }
  
  if(markerarraymsg_->markers.size() > 0) {
      pubSubmapMesh_.publish(markerarraymsg_);
  }

  return;
}

void Publisher::setMeshPublishingMode(const std::string &publishMode) {
  if(! (publishMode == "Colors" || publishMode == "Activations")){
    LOG(ERROR) << "Unknown MeshPublishing Mode given: " << publishMode << " - Setting to Colors.";
    return;
  }
  LOG(INFO) << "Set MeshPublishing Mode to " << publishMode;
  publishMode_ = publishMode;
  return;
}

}  // namespace okvis
