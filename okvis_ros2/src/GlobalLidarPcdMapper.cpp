// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/ros2/GlobalLidarPcdMapper.hpp>
#include <okvis/ros2/PointCloudUtilities.hpp>
#include <okvis/ros2/GlobalLidarScanTiming.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <algorithm>
#include <array>
#include <cstring>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <vector>
#ifdef __linux__
#include <sys/resource.h>
#endif

namespace okvis {
namespace {
constexpr size_t kPoseBatch = 4096;
constexpr size_t kUpdateLimit = 32;
constexpr size_t kPendingScanLimit = 8;
constexpr size_t kPendingLivePoseLimit = 64;

Eigen::Isometry3d isometry(const kinematics::Transformation& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.matrix() = pose.T();
  return result;
}
} // namespace

GlobalLidarPcdMapper::GlobalLidarPcdMapper(const std::shared_ptr<rclcpp::Node>& node,
    const ViParameters& parameters, const std::string& outputDirectory)
    : node_(node) {
  // Declare on this node so launch/CLI can enable the feature without changing
  // any existing SLAM configuration or input processing.
  const auto parameter = [&](const std::string& suffix, const auto& fallback) {
    using T = std::decay_t<decltype(fallback)>;
    const std::string name = "global_lidar_pcd." + suffix;
    if (!node_->has_parameter(name)) node_->declare_parameter<T>(name, fallback);
    return node_->get_parameter(name).get_value<T>();
  };
  try {
    enabled_ = parameter("enabled", false);
    if (!enabled_) return;
    if (!parameters.lidar) throw std::invalid_argument("Global LiDAR PCD requires lidar calibration");
    T_SL_ = parameters.lidar->T_SL;
    voxelSize_ = parameter("voxel_size", .1);
    publishInterval_ = parameter("publish_interval", .5);
    keyframeMatchTolerance_ = parameter("keyframe_match_tolerance", .05);
    inputReliability_ = parameter("input_reliability", std::string("auto"));
    if (inputReliability_ != "auto" && inputReliability_ != "reliable"
        && inputReliability_ != "best_effort")
      throw std::invalid_argument("global_lidar_pcd.input_reliability must be auto, reliable or best_effort");
    minimumRange_ = parameter("minimum_range", .5);
    maximumRange_ = parameter("maximum_range", 150.0);
    workerCpuBudget_ = parameter("worker_cpu_budget", .35);
    if (!std::isfinite(workerCpuBudget_) || workerCpuBudget_ <= 0 || workerCpuBudget_ > 1)
      throw std::invalid_argument("global_lidar_pcd.worker_cpu_budget must be in (0, 1]");
    colour_ = LidarHeightColour(parameter("height_min", -2.0), parameter("height_max", 5.0));
    saveOnShutdown_ = parameter("save_on_shutdown", true);
    filename_ = parameter("output_file", std::string());
    archiveDirectory_ = parameter("archive_directory",
        (std::filesystem::path(outputDirectory) / "global_lidar_pcd").string());
    const auto count = [&](const std::string& name, int64_t fallback, int64_t maximum) {
      const int64_t value = parameter(name, fallback);
      if (value <= 0 || value > maximum)
        throw std::invalid_argument("global_lidar_pcd." + name + " is outside its safe range");
      return static_cast<size_t>(value);
    };
    const std::string samplingName = "global_lidar_pcd.preview_sampling_step";
    if (!node_->has_parameter(samplingName)) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.description = "Display every Nth retained map point; 1 shows all. No display point cap.";
      rcl_interfaces::msg::IntegerRange range;
      range.from_value = 1;
      range.to_value = std::numeric_limits<int64_t>::max();
      range.step = 1;
      descriptor.integer_range.push_back(range);
      node_->declare_parameter<int64_t>(samplingName, 1, descriptor);
    }
    previewSamplingStep_ = count("preview_sampling_step", 1, std::numeric_limits<int64_t>::max());
    maxScanPoints_ = count("max_scan_points", 262144, 2000000);
    maxScanBytes_ = count("max_scan_bytes", 16 * 1024 * 1024, 256 * 1024 * 1024);
    maxPendingScanBytes_ = count("max_pending_scan_bytes", 32 * 1024 * 1024, 256 * 1024 * 1024);
    maxUpdateBytes_ = count("max_pending_update_bytes", 16 * 1024 * 1024, 256 * 1024 * 1024);
    if (!std::isfinite(voxelSize_) || voxelSize_ <= 0
        || !std::isfinite(publishInterval_) || publishInterval_ < .1
        || !std::isfinite(keyframeMatchTolerance_) || keyframeMatchTolerance_ < 0
        || keyframeMatchTolerance_ > 1.0
        || !std::isfinite(minimumRange_) || !std::isfinite(maximumRange_)
        || minimumRange_ < 0 || maximumRange_ <= minimumRange_
        || maximumRange_ > 100000 || archiveDirectory_.empty())
      throw std::invalid_argument("Invalid global_lidar_pcd resolution, period, range or archive directory");
    const std::string referenceName = "lidar_ouster_time_reference";
    if (!node_->has_parameter(referenceName)) node_->declare_parameter(referenceName, "scan_start");
    timeReference_ = node_->get_parameter(referenceName).as_string();
    if (timeReference_ != "scan_start" && timeReference_ != "scan_end" && timeReference_ != "header")
      throw std::invalid_argument("Invalid lidar_ouster_time_reference for global PCD");

    inputGroup_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::ExecutorOptions inputOptions;
    inputOptions.context = node_->get_node_base_interface()->get_context();
    inputExecutor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(inputOptions);
    inputExecutor_->add_callback_group(inputGroup_, node_->get_node_base_interface());
    rclcpp::PublisherOptions fullOptions;
    fullOptions.callback_group = inputGroup_;
    fullOptions.event_callbacks.matched_callback = [this](const auto& event) {
      if (event.total_count_change > 0) fullMapReplay_.fetch_add(1);
    };
    // Reliable supports both Reliable and Best Effort RViz readers. Volatile,
    // depth one still prevents a queue of historical global-map samples.
    publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "global_lidar_map", rclcpp::QoS(1).reliable().durability_volatile(), fullOptions);
    incrementalDisplay_ = std::make_unique<GlobalLidarMapDisplay>(node_, colour_, voxelSize_, inputGroup_);
    if (parameter("publish_lidar_tf", true)) {
      liveTransforms_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
      liveWorker_ = std::thread(&GlobalLidarPcdMapper::publishLiveTransforms, this);
    }
    refreshInputSubscription();
    inputWorker_ = std::thread(&GlobalLidarPcdMapper::receiveScans, this);
    previewWorker_ = std::thread(&GlobalLidarPcdMapper::publishLoop, this);
    worker_ = std::thread(&GlobalLidarPcdMapper::run, this);
  } catch (const std::exception& error) {
    enabled_ = false;
    failed_ = true;
    stopInputSubscription();
    {
      std::lock_guard<std::mutex> lock(previewMutex_);
      previewStopping_ = true;
    }
    previewCondition_.notify_one();
    if (previewWorker_.joinable()) previewWorker_.join();
    {
      std::lock_guard<std::mutex> lock(liveMutex_);
      liveStopping_ = true;
    }
    liveCondition_.notify_one();
    if (liveWorker_.joinable()) liveWorker_.join();
    RCLCPP_ERROR(node_->get_logger(), "Global LiDAR PCD disabled: %s", error.what());
  }
}

GlobalLidarPcdMapper::~GlobalLidarPcdMapper() { shutdown(); }

void GlobalLidarPcdMapper::refreshInputSubscription() {
  // Only this sidecar changes its reader. The estimator's input, threads and
  // keyframe policy are untouched. Use the remapped name for graph discovery.
  std::lock_guard<std::mutex> lock(subscriptionMutex_);
  if (inputStopped_) return;
  const auto topic = node_->get_node_topics_interface()->resolve_topic_name("/okvis/lidar");
  bool reliable = inputReliability_ == "reliable";
  if (inputReliability_ == "auto") {
    const auto publishers = node_->get_publishers_info_by_topic(topic);
    reliable = !publishers.empty();
    for (const auto& publisher : publishers)
      if (publisher.qos_profile().get_rmw_qos_profile().reliability
          != RMW_QOS_POLICY_RELIABILITY_RELIABLE) reliable = false;
  }
  if (subscription_ && reliable == inputReliable_) return;
  auto qos = rclcpp::QoS(1).durability_volatile();
  if (reliable) qos.reliable(); else qos.best_effort();
  // Construct before replacing so discovery/creation failures leave the old
  // reader alive. A possible handover duplicate is rejected by scan timing/KF
  // selection, never integrated as an additional scan for the same keyframe.
  rclcpp::SubscriptionOptions options;
  options.callback_group = inputGroup_;
  auto next = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/okvis/lidar", qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr scan) { enqueueScan(std::move(scan)); },
      options);
  subscription_ = std::move(next);
  inputReliable_ = reliable;
  RCLCPP_INFO(node_->get_logger(), "Global PCD input: %s; %s, depth 1 (mode %s)",
      topic.c_str(), reliable ? "Reliable" : "Best Effort", inputReliability_.c_str());
}

void GlobalLidarPcdMapper::stopInputSubscription() {
  std::lock_guard<std::mutex> shutdownLock(inputShutdownMutex_);
  {
    std::lock_guard<std::mutex> lock(subscriptionMutex_);
    inputStopped_ = true;
    inputStopping_ = true;
    subscription_.reset();
  }
  if (inputExecutor_) inputExecutor_->cancel();
  if (inputWorker_.joinable()) inputWorker_.join();
}

void GlobalLidarPcdMapper::receiveScans() try {
#ifdef __linux__
  (void)::setpriority(PRIO_PROCESS, 0, 5);
#endif
  // spin_once plus the stop flag also handles shutdown racing thread startup:
  // cancel() issued before spin() would otherwise be forgotten by spin().
  while (!inputStopping_ && rclcpp::ok(node_->get_node_base_interface()->get_context())) {
    inputExecutor_->spin_once(std::chrono::milliseconds(20));
  }
} catch (const std::exception& error) {
  inputFailed_ = true;
  RCLCPP_ERROR(node_->get_logger(), "Global PCD input executor stopped: %s; SLAM input is independent", error.what());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++generation_;
  }
  condition_.notify_one();
} catch (...) {
  inputFailed_ = true;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++generation_;
  }
  condition_.notify_one();
  RCLCPP_ERROR(node_->get_logger(), "Global PCD input executor failed; SLAM input is independent");
}

void GlobalLidarPcdMapper::enqueueScan(sensor_msgs::msg::PointCloud2::ConstSharedPtr scan) noexcept try {
  if (!scan || failed_) return;
  receivedScans_.fetch_add(1, std::memory_order_relaxed);
  if (scan->data.size() > std::min(maxScanBytes_, maxPendingScanBytes_)
      || uint64_t(scan->width) * scan->height > maxScanPoints_) {
    droppedScans_.fetch_add(1);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Global PCD scan exceeds configured byte/point cap; existing SLAM input is unaffected");
    return;
  }
  if (liveTransforms_ && !scan->header.frame_id.empty()) {
    {
      std::lock_guard<std::mutex> lock(liveMutex_);
      lidarFrameId_ = scan->header.frame_id;
    }
    liveCondition_.notify_one();
  }
  // Releasing megabyte payloads can enter the allocator. Never do that under
  // the same short mutex used by the estimator's state handoff.
  std::array<sensor_msgs::msg::PointCloud2::ConstSharedPtr, kPendingScanLimit> discarded;
  size_t discardCount = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) return;
    // Preserve a short FIFO while delayed optimisation/IMU catches up. A
    // single latest slot used to overwrite the scan needed by a new keyframe.
    while (!pendingScans_.empty() && (pendingScans_.size() >= kPendingScanLimit
        || scan->data.size() > maxPendingScanBytes_ - pendingScanBytes_)) {
      pendingScanBytes_ -= pendingScans_.front()->data.size();
      discarded[discardCount++] = std::move(pendingScans_.front());
      pendingScans_.pop_front();
      droppedScans_.fetch_add(1);
    }
    const size_t bytes = scan->data.size();
    pendingScans_.push_back(std::move(scan));
    pendingScanBytes_ += bytes;
    ++generation_;
  }
  condition_.notify_one();
} catch (...) {
  droppedScans_.fetch_add(1);
  RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Global PCD scan buffering failed; existing SLAM input is unaffected");
}

void GlobalLidarPcdMapper::stateUpdateCallback(
    const TrackingState& tracking, std::shared_ptr<const States> states) noexcept try {
  if (!enabled_ || failed_ || !states || states->empty()) return;
  // Forward the small fallback pose immediately. It must not wait behind
  // queued archive writes/deskew when an entry point has no frontend callback.
  const auto current = states->find(tracking.id);
  if (current != states->end()) frontendStateCallback(current->second, tracking);
  // A shared immutable update avoids copying poses/IMU on the backend thread.
  // Bound both the queue length and an estimate including dynamic IMU storage.
  size_t bytes = 0;
  for (const auto& entry : *states) {
    const auto& state = entry.second;
    const size_t dynamic = state.previousImuMeasurements.size() * sizeof(ImuMeasurement)
        + state.covisibleFrameIds.size() * (sizeof(StateId) + 4 * sizeof(void*))
        + state.extrinsics.size() * (sizeof(kinematics::Transformation) + 5 * sizeof(void*))
        + state.T_AiW.size() * (sizeof(kinematics::Transformation) + 5 * sizeof(void*))
        + state.gpsPoints.capacity() * sizeof(Eigen::Vector3d);
    if (bytes > maxUpdateBytes_ || dynamic > maxUpdateBytes_
        || sizeof(State) + 4 * sizeof(void*) > maxUpdateBytes_ - std::min(bytes, maxUpdateBytes_)
        || dynamic > maxUpdateBytes_ - bytes - sizeof(State) - 4 * sizeof(void*)) {
      bytes = maxUpdateBytes_ + 1;
      break;
    }
    bytes += sizeof(State) + 4 * sizeof(void*) + dynamic;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || updateOverflow_) return;
    if (updates_.size() >= kUpdateLimit || bytes > maxUpdateBytes_ - updateBytes_) {
      updateOverflow_ = true;
      rejectedUpdates_.fetch_add(1);
    } else {
      updates_.push_back({tracking, std::move(states), bytes});
      updateBytes_ += bytes;
    }
    ++generation_;
  }
  condition_.notify_one();
} catch (...) {
  // A sidecar allocation/queue failure must not escape into the estimator's
  // optimisation/publication thread. The worker reports and preserves a partial map.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    updateOverflow_ = true;
    rejectedUpdates_.fetch_add(1);
    ++generation_;
  }
  condition_.notify_one();
}

void GlobalLidarPcdMapper::shutdown() {
  std::lock_guard<std::mutex> shutdownLock(shutdownMutex_);
  stopInputSubscription();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    ++generation_;
  }
  condition_.notify_one();
  if (worker_.joinable()) worker_.join();
  {
    std::lock_guard<std::mutex> lock(previewMutex_);
    previewStopping_ = true;
  }
  previewCondition_.notify_one();
  if (previewWorker_.joinable()) previewWorker_.join();
  {
    std::lock_guard<std::mutex> lock(liveMutex_);
    liveStopping_ = true;
  }
  liveCondition_.notify_one();
  if (liveWorker_.joinable()) liveWorker_.join();
}

void GlobalLidarPcdMapper::frontendStateCallback(const State& state,
    const TrackingState& tracking) noexcept try {
  if (!enabled_ || !liveTransforms_ || tracking.trackingQuality == TrackingQuality::Lost
      || !state.T_WS.T().allFinite()) return;
  {
    std::lock_guard<std::mutex> lock(liveMutex_);
    if (liveStopping_ || (latestLivePoseTime_ && state.timestamp <= *latestLivePoseTime_)) return;
    pendingLivePoses_.push_back(LivePose{state.timestamp, state.T_WS});
    while (pendingLivePoses_.size() > kPendingLivePoseLimit) pendingLivePoses_.pop_front();
    latestLivePoseTime_ = state.timestamp;
  }
  liveCondition_.notify_one();
} catch (...) {
  // Raw-scan visualisation must never throw into the tracking thread.
}

void GlobalLidarPcdMapper::applyUpdate(const Update& update) {
  if (update.states->find(update.tracking.id) == update.states->end())
    throw std::runtime_error("Global PCD update does not contain its current state");
  std::set<StateId> affected;
  trajectory_.update(update.tracking, update.states, affected);
  const auto currentTime = update.states->at(update.tracking.id).timestamp.toNSec();
  if (!latestStateTimeNs_ || currentTime > *latestStateTimeNs_) latestStateTimeNs_ = currentTime;
  ++sequence_;
  for (const auto& id : affected) {
    State state{};
    if (!trajectory_.getState(id, state)) continue;
    if (state.isKeyframe) keyframesByTime_[state.timestamp.toNSec()] = id;
    if (map_->hasSubmap(id.value())) map_->updatePose(id.value(), isometry(state.T_WS), true, sequence_);
  }
}

bool GlobalLidarPcdMapper::prepareScan(bool draining) {
  if (!activeRawScan_) return true;
  auto reference = pointcloud_ros::OusterTimeReference::ScanStart;
  if (timeReference_ == "scan_end") reference = pointcloud_ros::OusterTimeReference::ScanEnd;
  if (timeReference_ == "header") reference = pointcloud_ros::OusterTimeReference::Header;
  try {
    if (!activeRawBounds_)
      activeRawBounds_ = detail::globalLidarScanTimeBounds(*activeRawScan_, reference);
    if (!activeRawBounds_ || (lastConsumedTime_
        && activeRawBounds_->second <= lastConsumedTime_->toNSec())) {
      invalidPoints_ += uint64_t(activeRawScan_->width) * activeRawScan_->height;
    } else {
      // Wait for the same state watermark as exact keyframe matching, but
      // retain the raw message instead of allocating/sorting 65k measurements.
      const uint64_t first = lastConsumedTime_
          ? std::max(activeRawBounds_->first, lastConsumedTime_->toNSec() + 1)
          : activeRawBounds_->first;
      const uint64_t last = activeRawBounds_->second;
      const uint64_t tolerance = static_cast<uint64_t>(keyframeMatchTolerance_ * 1e9);
      const uint64_t lower = first > tolerance ? first - tolerance : 0;
      const uint64_t upper = last > std::numeric_limits<uint64_t>::max() - tolerance
          ? std::numeric_limits<uint64_t>::max() : last + tolerance;
      if (!draining && (!latestStateTimeNs_ || *latestStateTimeNs_ < upper)) return false;
      bool candidate = false, alreadyMerged = false;
      for (auto it = keyframesByTime_.lower_bound(lower); it != keyframesByTime_.end()
          && it->first <= upper; ++it) {
        if (!selectedKeyframes_.count(it->second)) { candidate = true; break; }
        alreadyMerged = true;
      }
      if (candidate) {
        ++decodedScans_;
        invalidPoints_ += pointcloud_ros::lidar2batch(*activeRawScan_, activeScan_, reference);
        if (lastConsumedTime_) {
          const auto begin = std::upper_bound(activeScan_.begin(), activeScan_.end(),
              *lastConsumedTime_, [](const Time& time, const LidarMeasurement& point) {
                return time < point.timeStamp;
              });
          invalidPoints_ += static_cast<size_t>(begin - activeScan_.begin());
          activeScan_.erase(activeScan_.begin(), begin);
        }
      } else if (alreadyMerged) {
        ++skippedDuplicateKeyframeScans_;
      } else {
        ++skippedNonKeyframeScans_;
      }
    }
  } catch (const std::exception& error) {
    activeScan_.clear();
    droppedScans_.fetch_add(1);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Global PCD rejected malformed scan: %s", error.what());
  }
  activeRawScan_.reset();
  activeRawBounds_.reset();
  activeOffset_ = 0;
  activeKeyframe_.reset();
  return true;
}

bool GlobalLidarPcdMapper::integrateScan(bool draining) {
  if (activeScan_.empty()) return true;
  Time earliest;
  if (!trajectory_.earliestPoseTimestamp(earliest)
      || activeScan_.back().timeStamp < earliest) {
    if (!draining && !latestStateTimeNs_) return false;
    invalidPoints_ += activeScan_.size() - activeOffset_;
    activeScan_.clear();
    activeKeyframe_.reset();
    return true;
  }
  if (!activeKeyframe_) {
    // Find an unmerged keyframe in/near this scan's time interval. Matching
    // the midpoint against ALL frames wrongly vetoed scans whose midpoint
    // was a normal frame even though the scan covered an actual keyframe.
    const uint64_t first = activeScan_.front().timeStamp.toNSec();
    const uint64_t last = activeScan_.back().timeStamp.toNSec();
    const uint64_t middle = first + (last - first) / 2;
    const uint64_t tolerance = static_cast<uint64_t>(keyframeMatchTolerance_ * 1e9);
    const uint64_t lower = first > tolerance ? first - tolerance : 0;
    const uint64_t upper = last > std::numeric_limits<uint64_t>::max() - tolerance
        ? std::numeric_limits<uint64_t>::max() : last + tolerance;
    // State callbacks may lag the raw scan. Wait until the complete matching
    // window is known rather than rejecting it before its keyframe arrives.
    if (!draining && (!latestStateTimeNs_ || *latestStateTimeNs_ < upper)) return false;
    auto frame = keyframesByTime_.end();
    uint64_t distance = std::numeric_limits<uint64_t>::max();
    bool alreadyMerged = false;
    for (auto it = keyframesByTime_.lower_bound(lower); it != keyframesByTime_.end()
        && it->first <= upper; ++it) {
      if (selectedKeyframes_.count(it->second)) { alreadyMerged = true; continue; }
      const uint64_t delta = it->first >= middle ? it->first - middle : middle - it->first;
      if (delta < distance) { frame = it; distance = delta; }
    }
    if (frame == keyframesByTime_.end()) {
      if (alreadyMerged) ++skippedDuplicateKeyframeScans_;
      else ++skippedNonKeyframeScans_;
      activeScan_.clear();
      return true;
    }
    activeKeyframe_ = frame->second;
  }
  while (activeOffset_ < activeScan_.size()) {
    workBudget_->checkpoint(draining);
    const auto timestamp = activeScan_[activeOffset_].timeStamp;
    if (timestamp < earliest) {
      const auto begin = activeScan_.begin() + activeOffset_;
      const auto supported = std::lower_bound(begin, activeScan_.end(), earliest,
          [](const LidarMeasurement& point, const Time& time) { return point.timeStamp < time; });
      const size_t expired = static_cast<size_t>(supported - begin);
      activeOffset_ += expired;
      invalidPoints_ += expired;
      continue;
    }
    std::vector<Time> times;
    std::vector<size_t> runEnds;
    const size_t end = std::min(activeScan_.size(), activeOffset_ + kPoseBatch);
    times.reserve(end - activeOffset_);
    runEnds.reserve(end - activeOffset_);
    for (size_t i = activeOffset_; i < end; ++i) {
      const auto& time = activeScan_[i].timeStamp;
      if (times.empty() || time != times.back()) {
        times.push_back(time);
        runEnds.push_back(i + 1);
      } else {
        runEnds.back() = i + 1;
      }
    }
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> poses;
    const auto lookup = trajectory_.getStatePosePrefix(times, poses);
    if (!poses.empty()) {
      State state{};
      if (!trajectory_.getState(*activeKeyframe_, state))
        throw std::runtime_error("Global PCD lost its keyframe anchor");
      const auto T_WK = isometry(state.T_WS);
      const auto T_KW = T_WK.inverse();
      LidarPointCloudMap::Points local;
      local.reserve(runEnds[poses.size() - 1] - activeOffset_);
      size_t first = activeOffset_;
      const double minimumSquared = minimumRange_ * minimumRange_;
      const double maximumSquared = maximumRange_ * maximumRange_;
      for (size_t i = 0; i < poses.size(); ++i) {
        // Organised LiDAR rings share a firing time. Query one pose and
        // compose one complete LiDAR->keyframe transform for that time,
        // rather than copying a 4x4 pose and composing transforms per ray.
        const Eigen::Matrix4d T_KL = T_KW.matrix() * poses[i] * T_SL_.T();
        for (size_t p = first; p < runEnds[i]; ++p) {
          const auto& ray = activeScan_[p].measurement.rayMeasurement;
          const double squared = ray.squaredNorm();
          if (!ray.allFinite() || squared < minimumSquared || squared > maximumSquared) {
            ++invalidPoints_;
            continue;
          }
          const Eigen::Vector3d point = T_KL.topLeftCorner<3, 3>() * ray + T_KL.topRightCorner<3, 1>();
          if (!point.allFinite()) { ++invalidPoints_; continue; }
          local.push_back(point);
        }
        first = runEnds[i];
      }
      if (!local.empty()) {
        map_->addPoints(activeKeyframe_->value(), T_WK, local);
        selectedKeyframes_.insert(*activeKeyframe_);
      }
      activeOffset_ = runEnds[poses.size() - 1];
      lastConsumedTime_ = times[poses.size() - 1];
      latestTimestamp_ = *lastConsumedTime_;
    }
    if (lookup.status == Trajectory::PoseLookupStatus::Complete) continue;
    if (lookup.status == Trajectory::PoseLookupStatus::ExpiredImu
        || lookup.status == Trajectory::PoseLookupStatus::Invalid) {
      // Lookup counts now refer to UNIQUE times. Convert back to point
      // offsets so every ring survives, including a run spanning batches.
      const size_t skipRuns = std::min(runEnds.size() - poses.size(),
                                      std::max(size_t(1), lookup.expiredCount));
      const size_t skip = runEnds[poses.size() + skipRuns - 1] - activeOffset_;
      activeOffset_ += skip;
      invalidPoints_ += skip;
      continue;
    }
    if (!draining) return false;
    invalidPoints_ += activeScan_.size() - activeOffset_;
    RCLCPP_WARN(node_->get_logger(), "Global PCD omitted %zu trailing points without optimised pose/IMU coverage",
        activeScan_.size() - activeOffset_);
    activeOffset_ = activeScan_.size();
  }
  activeScan_.clear();
  activeKeyframe_.reset();
  activeOffset_ = 0;
  ++integratedScans_;
  if (integratedScans_ % 100 == 0) {
    RCLCPP_INFO(node_->get_logger(), "Global PCD: %lu keyframe scans, %lu replaced/rejected scans, %lu invalid/uncovered points; resident cloud estimate %.1f MiB, archived points %lu",
        static_cast<unsigned long>(integratedScans_), static_cast<unsigned long>(droppedScans_.load()),
        static_cast<unsigned long>(invalidPoints_), double(map_->residentBytes()) / (1024.0*1024.0),
        static_cast<unsigned long>(map_->archivedPointCount()));
  }
  return true;
}

void GlobalLidarPcdMapper::queuePreview(bool force) noexcept try {
  const bool subscribed = publisher_->get_subscription_count() > 0
      || publisher_->get_intra_process_subscription_count() > 0
      || incrementalDisplay_->subscribed();
  if (!subscribed && !force) return;
  const auto revision = map_->revision();
  if (!force && lastQueuedRevision_ && *lastQueuedRevision_ == revision) return;
  // Copying the growing anchor/chunk index and freezing active voxels is not
  // free. Coalesce at the source, rather than building snapshots which the
  // display thread's latest-only slot would immediately discard.
  const auto now = std::chrono::steady_clock::now();
  if (!force && now < nextPreview_) return;
  Preview preview{map_->snapshot(), latestTimestamp_, revision};
  {
    std::lock_guard<std::mutex> lock(previewMutex_);
    pendingPreview_ = std::move(preview); // At most one waiting snapshot.
  }
  lastQueuedRevision_ = revision;
  nextPreview_ = now + std::chrono::milliseconds(100);
  previewCondition_.notify_one();
} catch (const std::exception& error) {
  RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Global PCD snapshot: %s; map ingestion/export continues", error.what());
} catch (...) {
  RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Global PCD snapshot failed; map ingestion/export continues");
}

void GlobalLidarPcdMapper::publishLiveTransforms() try {
  std::optional<LivePose> lastPose;
  std::string lastFrame;
  while (true) {
    std::deque<LivePose> poses;
    std::string frame;
    bool stopping;
    {
      std::unique_lock<std::mutex> lock(liveMutex_);
      liveCondition_.wait(lock, [&] {
        return liveStopping_ || (!lidarFrameId_.empty() && lidarFrameId_ != "world"
            && (!pendingLivePoses_.empty() || (lastPose && lidarFrameId_ != lastFrame)));
      });
      stopping = liveStopping_;
      frame = lidarFrameId_;
      if (!frame.empty()) poses.swap(pendingLivePoses_);
    }
    if (!frame.empty() && frame != "world") {
      if (poses.empty() && lastPose && frame != lastFrame) poses.push_back(*lastPose);
      std::vector<geometry_msgs::msg::TransformStamped> transforms;
      transforms.reserve(poses.size());
      for (const auto& live : poses) {
        // Preserve every queued sensor timestamp. RViz needs poses on both
        // sides of a raw scan's header stamp; a latest-only TF slot loses
        // that history when several frontend frames arrive in one burst.
        const auto pose = live.pose * T_SL_;
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "world";
        transform.child_frame_id = frame;
        transform.header.stamp = rclcpp::Time(live.timestamp.sec, live.timestamp.nsec);
        transform.transform.translation.x = pose.r().x();
        transform.transform.translation.y = pose.r().y();
        transform.transform.translation.z = pose.r().z();
        transform.transform.rotation.x = pose.q().x();
        transform.transform.rotation.y = pose.q().y();
        transform.transform.rotation.z = pose.q().z();
        transform.transform.rotation.w = pose.q().w();
        transforms.push_back(std::move(transform));
      }
      if (!transforms.empty()) {
        try {
          liveTransforms_->sendTransform(transforms);
        } catch (const std::exception& error) {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
              "Live LiDAR TF: %s; global map ingestion/export continues", error.what());
        }
        lastPose = poses.back();
        lastFrame = frame;
      }
    }
    if (stopping) break;
  }
} catch (const std::exception& error) {
  RCLCPP_ERROR(node_->get_logger(), "Live LiDAR TF stopped: %s; global map continues", error.what());
} catch (...) {
  RCLCPP_ERROR(node_->get_logger(), "Live LiDAR TF stopped; global map continues");
}

void GlobalLidarPcdMapper::publishLoop() try {
#ifdef __linux__
  (void)::setpriority(PRIO_PROCESS, 0, 5);
#endif
  detail::GlobalLidarWorkBudget workBudget(workerCpuBudget_);
  std::optional<Preview> latest;
  uint64_t builtRevision = 0;
  uint64_t publishedReplay = 0;
  size_t builtStep = 0;
  bool havePublished = false;
  auto lastPublished = std::chrono::steady_clock::now();
  auto lastIncremental = lastPublished - std::chrono::milliseconds(100);
  // Keep ONE packed payload for late readers. Unchanged maps are sent only
  // on a reader match, avoiding periodic serialization of millions of points.
  auto message = std::make_unique<sensor_msgs::msg::PointCloud2>();
  message->header.frame_id = "world";
  message->height = 1;
  message->point_step = 16;
  const uint16_t endian = 1;
  message->is_bigendian = *reinterpret_cast<const uint8_t*>(&endian) == 0;
  message->is_dense = true;
  message->fields.resize(4);
  const char* names[] = {"x", "y", "z", "rgb"};
  for (size_t i = 0; i < 4; ++i) {
    message->fields[i].name = names[i];
    message->fields[i].offset = static_cast<uint32_t>(4 * i);
    message->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    message->fields[i].count = 1;
  }
  while (true) {
    bool stopping;
    {
      std::unique_lock<std::mutex> lock(previewMutex_);
      previewCondition_.wait_for(lock, std::chrono::milliseconds(50),
          [&] { return previewStopping_ || pendingPreview_; });
      if (pendingPreview_) {
        latest = std::move(pendingPreview_);
        pendingPreview_.reset();
      }
      stopping = previewStopping_;
    }
    workBudget.checkpoint(stopping);
    try {
      const int64_t step = node_->get_parameter("global_lidar_pcd.preview_sampling_step").as_int();
      if (step <= 0) throw std::invalid_argument("LiDAR display sampling step must be positive");
      const auto incrementalNow = std::chrono::steady_clock::now();
      // Map snapshots may arrive faster than RViz can consume replay batches.
      if (latest && (stopping || incrementalNow - lastIncremental >= std::chrono::milliseconds(100))) {
        incrementalDisplay_->publish(latest->snapshot, static_cast<size_t>(step), latest->revision,
            [&] { workBudget.checkpoint(stopping); });
        lastIncremental = incrementalNow;
      }
      const double interval = node_->get_parameter("global_lidar_pcd.publish_interval").as_double();
      if (!std::isfinite(interval) || interval < .1)
        throw std::invalid_argument("global_lidar_pcd.publish_interval must be at least 0.1 s");
      const auto now = std::chrono::steady_clock::now();
      const bool subscribed = publisher_->get_subscription_count() > 0
          || publisher_->get_intra_process_subscription_count() > 0;
      if (!subscribed && !message->data.empty()) {
        std::vector<uint8_t>().swap(message->data);
        builtStep = 0;
        havePublished = false;
      }
      const uint64_t replay = fullMapReplay_.load();
      const bool changed = latest && (builtStep != static_cast<size_t>(step) || builtRevision != latest->revision);
      if (latest && subscribed && (!havePublished || changed || publishedReplay != replay)
          && (stopping || !havePublished
              || std::chrono::duration<double>(now - lastPublished).count() >= interval)) {
        if (builtStep != static_cast<size_t>(step) || builtRevision != latest->revision) {
          const uint64_t count = LidarPointCloudMap::sampledPointCount(latest->snapshot, step);
          if (count > std::numeric_limits<uint32_t>::max() / message->point_step)
            throw std::length_error("Global map exceeds PointCloud2 row size; increase preview_sampling_step");
          message->width = static_cast<uint32_t>(count);
          message->row_step = message->width * message->point_step;
          message->header.stamp = rclcpp::Time(latest->timestamp.sec, latest->timestamp.nsec);
          // Release the old payload BEFORE a larger allocation; do not hold
          // two full world clouds during vector growth or density changes.
          if (message->row_step > message->data.capacity()
              || message->row_step < message->data.capacity() / 2)
            std::vector<uint8_t>().swap(message->data);
          message->data.resize(message->row_step);
          size_t offset = 0;
          size_t visited = 0;
          LidarPointCloudMap::forEachSampledPoint(latest->snapshot, step, [&](const Eigen::Vector3d& point) {
            if (++visited % kPoseBatch == 0) workBudget.checkpoint(stopping);
            const float xyz[] = {float(point.x()), float(point.y()), float(point.z())};
            std::memcpy(message->data.data() + offset, xyz, sizeof(xyz));
            const uint32_t rgb = colour_(point.z());
            std::memcpy(message->data.data() + offset + sizeof(xyz), &rgb, sizeof(rgb));
            offset += message->point_step;
          });
          builtRevision = latest->revision;
          builtStep = static_cast<size_t>(step);
        }
        publisher_->publish(*message);
        publishedReplay = replay;
        havePublished = true;
        lastPublished = std::chrono::steady_clock::now();
      }
    } catch (const std::exception& error) {
      // Display failures/backpressure must never stop map ingestion or export.
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "Global PCD display: %s; map ingestion/export continues", error.what());
    }
    if (stopping) break;
  }
} catch (const std::exception& error) {
  RCLCPP_ERROR(node_->get_logger(), "Global PCD display stopped: %s; map ingestion/export continues", error.what());
} catch (...) {
  RCLCPP_ERROR(node_->get_logger(), "Global PCD display stopped with unknown exception; map ingestion/export continues");
}

void GlobalLidarPcdMapper::save() {
  if (!map_) return;
  map_->flush();
  if (!saveOnShutdown_) return;
  const std::string output = failed_ ? filename_ + ".partial.pcd" : filename_;
  if (!std::filesystem::path(output).parent_path().empty())
    std::filesystem::create_directories(std::filesystem::path(output).parent_path());
  const bool saved = LidarPointCloudMap::saveColoredPcd(map_->snapshot(), voxelSize_, output,
      [this](const Eigen::Vector3d& point) { return colour_(point.z()); });
  if (saved) RCLCPP_INFO(node_->get_logger(), "Global coloured LiDAR PCD saved: %s", output.c_str());
  else RCLCPP_WARN(node_->get_logger(), "Global coloured LiDAR PCD is empty; no file written");
}

void GlobalLidarPcdMapper::run() {
#ifdef __linux__
  // Lower only this worker's scheduling priority on Linux; SLAM keeps its policy.
  (void)::setpriority(PRIO_PROCESS, 0, 5);
#endif
  try {
    LidarPointCloudMap::ArchiveOptions archive;
    archive.directory = archiveDirectory_;
    archive.maxResidentSubmaps = 1;
    archive.chunkPointLimit = 65536;
    map_ = std::make_unique<LidarPointCloudMap>(voxelSize_, archive);
    workBudget_ = std::make_unique<detail::GlobalLidarWorkBudget>(workerCpuBudget_);
    if (filename_.empty()) filename_ = map_->archiveDirectory() + "/global_lidar_map_rgb.pcd";
    RCLCPP_INFO(node_->get_logger(), "Global PCD worker: %s; one scan per SLAM keyframe; RViz every %zu-th point, no display point cap, every %.2f s; soft CPU budget %.0f%% of one core per map/display worker; archive: %s",
        filename_.c_str(), previewSamplingStep_, publishInterval_, 100 * workerCpuBudget_, map_->archiveDirectory().c_str());
    auto nextInputCheck = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    auto nextCoverageReport = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (true) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= nextInputCheck) {
        nextInputCheck = now + std::chrono::seconds(1);
        try { refreshInputSubscription(); }
        catch (const std::exception& error) {
          RCLCPP_WARN(node_->get_logger(), "Global PCD input QoS discovery: %s; keeping current reader", error.what());
        }
      }
      std::deque<Update> updates;
      bool stopping, overflow;
      uint64_t observed;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        updates.swap(updates_);
        // In-flight update storage counts towards the queue budget until applied.
        stopping = stopping_;
        overflow = updateOverflow_;
        observed = generation_;
        if (!activeRawScan_ && activeScan_.empty() && !pendingScans_.empty()) {
          activeRawScan_ = std::move(pendingScans_.front());
          pendingScans_.pop_front();
          pendingScanBytes_ -= activeRawScan_->data.size();
        }
      }
      while (!updates.empty()) {
        workBudget_->checkpoint(stopping);
        auto update = std::move(updates.front());
        updates.pop_front();
        applyUpdate(update);
        update.states.reset();
        std::lock_guard<std::mutex> lock(mutex_);
        updateBytes_ -= update.bytes;
      }
      if (overflow) throw std::runtime_error("Global PCD pose update backlog exceeded its bound; stopping sidecar to preserve pose consistency");
      if (inputFailed_) throw std::runtime_error("Global PCD input executor failed; stopping sidecar");
      workBudget_->checkpoint(stopping);
      const bool hadScan = bool(activeRawScan_) || !activeScan_.empty();
      const bool prepared = prepareScan(stopping);
      const bool integrated = integrateScan(stopping);
      if (hadScan && prepared && integrated) completedScans_.fetch_add(1, std::memory_order_relaxed);
      queuePreview();
      if (now >= nextCoverageReport) {
        nextCoverageReport = now + std::chrono::seconds(5);
        RCLCPP_INFO(node_->get_logger(),
            "Global PCD coverage: %lu received scans; %zu known SLAM keyframes, %zu with map points; %lu non-keyframe scans skipped; %lu replaced/rejected after receive; %lu fully decoded scans. DDS losses before receive are not counted",
            static_cast<unsigned long>(receivedScans_.load(std::memory_order_relaxed)),
            keyframesByTime_.size(), selectedKeyframes_.size(),
            static_cast<unsigned long>(skippedNonKeyframeScans_),
            static_cast<unsigned long>(droppedScans_.load()), static_cast<unsigned long>(decodedScans_.load()));
      }
      std::unique_lock<std::mutex> lock(mutex_);
      if (stopping_ && updates_.empty() && pendingScans_.empty() && !activeRawScan_ && activeScan_.empty()) break;
      if (!activeRawScan_ && activeScan_.empty() && !pendingScans_.empty()) continue;
      condition_.wait_for(lock, std::chrono::milliseconds(50),
          [&] { return generation_ != observed; });
    }
    queuePreview(true);
    save();
  } catch (const std::exception& error) {
    failed_ = true;
    RCLCPP_ERROR(node_->get_logger(), "Global PCD worker stopped: %s; existing SLAM continues", error.what());
    try { save(); }
    catch (const std::exception& failure) {
      RCLCPP_ERROR(node_->get_logger(), "Global PCD recovery export failed: %s; retained archive: %s",
          failure.what(), map_ ? map_->archiveDirectory().c_str() : archiveDirectory_.c_str());
    }
  } catch (...) {
    failed_ = true;
    RCLCPP_ERROR(node_->get_logger(), "Global PCD worker stopped with unknown exception; existing SLAM continues");
  }
  std::array<std::optional<Update>, kUpdateLimit> discardedUpdates;
  std::array<sensor_msgs::msg::PointCloud2::ConstSharedPtr, kPendingScanLimit> discardedScans;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t index = 0;
    while (!updates_.empty()) {
      discardedUpdates[index++].emplace(std::move(updates_.front()));
      updates_.pop_front();
    }
    updateBytes_ = 0;
    index = 0;
    while (!pendingScans_.empty()) {
      discardedScans[index++] = std::move(pendingScans_.front());
      pendingScans_.pop_front();
    }
    pendingScanBytes_ = 0;
  }
  activeRawScan_.reset();
  activeRawBounds_.reset();
  // On a sidecar failure also release its DDS reader: merely ignoring callbacks
  // would keep deserializing large scans after mapping had already stopped.
  stopInputSubscription();
  RCLCPP_INFO(node_->get_logger(), "Global PCD complete: %lu keyframe scans, %lu non-keyframe scans skipped, %lu duplicate-keyframe scans skipped, %lu replaced/rejected scans, %lu invalid/uncovered points, %lu rejected pose updates",
      static_cast<unsigned long>(integratedScans_), static_cast<unsigned long>(skippedNonKeyframeScans_),
      static_cast<unsigned long>(skippedDuplicateKeyframeScans_), static_cast<unsigned long>(droppedScans_.load()),
      static_cast<unsigned long>(invalidPoints_), static_cast<unsigned long>(rejectedUpdates_.load()));
  RCLCPP_INFO(node_->get_logger(), "Global PCD final coverage: %lu received scans; %zu known SLAM keyframes, %zu with map points; DDS losses before receive are not counted",
      static_cast<unsigned long>(receivedScans_.load(std::memory_order_relaxed)),
      keyframesByTime_.size(), selectedKeyframes_.size());
}

} // namespace okvis
