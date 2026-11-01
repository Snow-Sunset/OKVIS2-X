// SPDX-License-Identifier: BSD-3-Clause
#ifndef OKVIS_ROS2_GLOBAL_LIDAR_MAP_DISPLAY_HPP
#define OKVIS_ROS2_GLOBAL_LIDAR_MAP_DISPLAY_HPP

#include <okvis/LidarMapColour.hpp>
#include <okvis/LidarPointCloudMap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <array>
#include <atomic>
#include <map>
#include <functional>
#include <optional>

namespace okvis {

/// Incremental RViz view. Retained archive blocks have stable marker IDs;
/// keyframe corrections update small TF anchors instead of uploading all points.
/// publish() is owned exclusively by the mapper's display thread. No estimator
/// access and no retained dense point cloud or unbounded DDS history.
class GlobalLidarMapDisplay {
 public:
  struct Result { size_t blocks = 0, points = 0, poses = 0, indexed = 0; };
  GlobalLidarMapDisplay(const std::shared_ptr<rclcpp::Node>& node,
      const LidarHeightColour& colour, double colourResolution,
      const rclcpp::CallbackGroup::SharedPtr& callbackGroup = nullptr);
  bool subscribed() const;
  Result publish(const LidarPointCloudMap::Snapshot& snapshot, size_t samplingStep,
      std::optional<uint64_t> revision = std::nullopt, const std::function<void()>& checkpoint = {});

 private:
  using Key = std::pair<uint64_t, size_t>;
  struct Sent {
    std::weak_ptr<const LidarPointCloudMap::Points> points;
    std::weak_ptr<const LidarPointCloudMap::ArchiveChunk> archive;
    std::array<double, 4> height{};
    double radius = 0;
    size_t step = 0, skip = 0;
    uint64_t replay = 0;
    uint64_t version = 0;
  };
  struct Candidate { size_t index, skip; int priority; };
  std::string frame(uint64_t id) const;
  LidarHeightColour colour_;
  double colourResolution_;
  std::string prefix_;
  std::atomic<uint64_t> replay_{1};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  tf2_ros::StaticTransformBroadcaster transforms_;
  std::map<Key, Sent> sent_;
  std::map<uint64_t, std::array<double, 16>> poses_;
  // Reuse the plan across replay batches and idle ticks. A snapshot revision,
  // density change or new reader invalidates the index, never a 10 Hz timer.
  std::optional<uint64_t> indexedRevision_;
  size_t indexedStep_ = 0, nextCandidate_ = 0;
  uint64_t indexedReplay_ = 0;
  std::vector<Candidate> candidates_;
  bool initialised_ = false;
  uint64_t version_ = 0;
};
} // namespace okvis
#endif
