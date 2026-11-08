// SPDX-License-Identifier: BSD-3-Clause
#ifndef OKVIS_ROS2_GLOBAL_LIDAR_PCD_MAPPER_HPP
#define OKVIS_ROS2_GLOBAL_LIDAR_PCD_MAPPER_HPP

#include <okvis/LidarMapColour.hpp>
#include <okvis/LidarPointCloudMap.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/ViInterface.hpp>
#include <okvis/ros2/GlobalLidarMapDisplay.hpp>
#include <okvis/ros2/GlobalLidarWorkBudget.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <set>
#include <thread>

namespace okvis {

/// Optional read-only sidecar. ROS callbacks enqueue immutable input only; one
/// mapping worker owns decoding, trajectory, deskew, archive and PCD export.
/// A private executor receives scans outside the SLAM node's callback group.
/// A separate display worker receives immutable snapshots through a latest-only
/// slot, so full-map assembly and DDS/RViz cannot stall keyframe ingestion.
/// A third, lightweight worker publishes raw-scan TF independently of map I/O.
/// Each SLAM keyframe selects at most one temporally adjacent scan. Ordinary
/// frames cannot veto that selection. No calls into Estimator/SubmappingInterface.
/// Display density is controlled
/// only by a fixed sampling step; the global display has no point-count cap.
class GlobalLidarPcdMapper {
 public:
  using States = AlignedMap<StateId, State>;
  GlobalLidarPcdMapper(const std::shared_ptr<rclcpp::Node>& node,
                      const ViParameters& parameters, const std::string& outputDirectory);
  ~GlobalLidarPcdMapper();
  GlobalLidarPcdMapper(const GlobalLidarPcdMapper&) = delete;
  GlobalLidarPcdMapper& operator=(const GlobalLidarPcdMapper&) = delete;

  /// Call alongside the existing optimised graph callback. Never does map I/O.
  void stateUpdateCallback(const TrackingState& tracking, std::shared_ptr<const States> states) noexcept;
  /// Queue a small frontend pose only; a separate live thread publishes TF.
  void frontendStateCallback(const State& state, const TrackingState& tracking) noexcept;
  /// Stop sensor input/estimator callbacks first. Idempotent; drains accepted
  /// inputs and exports on the worker, joining before ROS is shut down.
  void shutdown();
  bool enabled() const { return enabled_; }
  bool failed() const { return failed_.load(); }
  struct Statistics { uint64_t receivedScans, decodedScans, completedScans; };
  /// Independent counters for diagnostics; not a consistent estimator snapshot.
  Statistics statistics() const noexcept {
    return {receivedScans_.load(std::memory_order_relaxed), decodedScans_.load(std::memory_order_relaxed),
            completedScans_.load(std::memory_order_relaxed)};
  }

 private:
  struct Update { TrackingState tracking; std::shared_ptr<const States> states; size_t bytes; };
  struct Preview { LidarPointCloudMap::Snapshot snapshot; Time timestamp; uint64_t revision; };
  struct LivePose { Time timestamp; kinematics::Transformation pose; };
  void enqueueScan(sensor_msgs::msg::PointCloud2::ConstSharedPtr scan) noexcept;
  void refreshInputSubscription();
  void stopInputSubscription();
  void receiveScans();
  void run();
  void applyUpdate(const Update& update);
  bool prepareScan(bool draining);
  bool integrateScan(bool draining);
  void queuePreview(bool force = false) noexcept;
  void publishLoop();
  void publishLiveTransforms();
  void save();

  std::shared_ptr<rclcpp::Node> node_;
  kinematics::Transformation T_SL_;
  bool enabled_ = false, saveOnShutdown_ = true;
  std::string filename_, archiveDirectory_, timeReference_;
  std::string inputReliability_ = "auto";
  double voxelSize_ = .1, publishInterval_ = .5, keyframeMatchTolerance_ = .05;
  double minimumRange_ = .5, maximumRange_ = 150.0;
  double workerCpuBudget_ = .35;
  LidarHeightColour colour_{-2.0, 5.0};
  size_t previewSamplingStep_ = 1, maxScanPoints_ = 262144;
  size_t maxScanBytes_ = 16 * 1024 * 1024, maxUpdateBytes_ = 16 * 1024 * 1024;
  size_t maxPendingScanBytes_ = 32 * 1024 * 1024, pendingScanBytes_ = 0;
  size_t updateBytes_ = 0;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  // Explicitly excluded from the node's SLAM executor/default callback group.
  rclcpp::CallbackGroup::SharedPtr inputGroup_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> inputExecutor_;
  std::thread inputWorker_;
  std::atomic_bool inputStopping_{false};
  std::mutex subscriptionMutex_, inputShutdownMutex_;
  bool inputStopped_ = false, inputReliable_ = false;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::unique_ptr<GlobalLidarMapDisplay> incrementalDisplay_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> liveTransforms_;
  std::mutex mutex_, shutdownMutex_;
  std::condition_variable condition_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pendingScans_;
  std::deque<Update> updates_;
  bool stopping_ = false, updateOverflow_ = false;
  uint64_t generation_ = 0;
  std::thread worker_;
  std::mutex previewMutex_;
  std::condition_variable previewCondition_;
  std::optional<Preview> pendingPreview_;
  bool previewStopping_ = false;
  std::thread previewWorker_;
  // Raw-scan TF must never wait for archive reads, map uploads or DDS map
  // backpressure. Keep a short ordered pose history for scan-time interpolation
  // in RViz instead of replacing every pending pose with only the latest one.
  std::mutex liveMutex_;
  std::condition_variable liveCondition_;
  std::deque<LivePose> pendingLivePoses_;
  std::optional<Time> latestLivePoseTime_;
  std::string lidarFrameId_;
  bool liveStopping_ = false;
  std::thread liveWorker_;
  std::atomic_bool failed_{false};
  std::atomic_bool inputFailed_{false};
  std::atomic<uint64_t> receivedScans_{0}, droppedScans_{0}, rejectedUpdates_{0};
  std::atomic<uint64_t> fullMapReplay_{1};
  std::atomic<uint64_t> decodedScans_{0}, completedScans_{0};

  // Worker-owned state. Only small pose/chunk metadata grows with trajectory;
  // dense history lives on disk. A sampled world cloud is streamed directly
  // into one ROS payload; display memory grows with the selected point count.
  Trajectory trajectory_;
  std::unique_ptr<LidarPointCloudMap> map_;
  std::unique_ptr<detail::GlobalLidarWorkBudget> workBudget_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr activeRawScan_;
  std::optional<std::pair<uint64_t, uint64_t>> activeRawBounds_;
  LidarMeasurementBatch activeScan_;
  size_t activeOffset_ = 0;
  std::map<uint64_t, StateId> keyframesByTime_;
  std::optional<uint64_t> latestStateTimeNs_;
  std::set<StateId> selectedKeyframes_;
  std::optional<StateId> activeKeyframe_;
  uint64_t sequence_ = 0, integratedScans_ = 0, invalidPoints_ = 0, rangeFilteredPoints_ = 0;
  uint64_t skippedNonKeyframeScans_ = 0, skippedDuplicateKeyframeScans_ = 0;
  std::optional<Time> lastConsumedTime_;
  Time latestTimestamp_;
  std::optional<uint64_t> lastQueuedRevision_;
  std::chrono::steady_clock::time_point nextPreview_{};
};

} // namespace okvis
#endif
