// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/ros2/GlobalLidarMapDisplay.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unistd.h>

namespace okvis {
namespace {
uint64_t pointCount(const LidarPointCloudMap::SubmapSnapshot& block) {
  return block.archive ? block.archive->pointCount : (block.points ? block.points->size() : 0);
}
std::array<double, 4> heightRow(const Eigen::Isometry3d& pose) {
  return {pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3)};
}
} // namespace

GlobalLidarMapDisplay::GlobalLidarMapDisplay(const std::shared_ptr<rclcpp::Node>& node,
    const LidarHeightColour& colour, double colourResolution,
    const rclcpp::CallbackGroup::SharedPtr& callbackGroup)
    : colour_(colour), colourResolution_(colourResolution),
      prefix_("okvis_lidar_" + std::to_string(::getpid()) + "_"
          + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())),
      transforms_(node, rclcpp::QoS(1).reliable().transient_local()) {
  if (!std::isfinite(colourResolution_) || colourResolution_ <= 0)
    throw std::invalid_argument("Incremental LiDAR colour resolution must be positive");
  rclcpp::PublisherOptions options;
  options.callback_group = callbackGroup;
  options.event_callbacks.matched_callback = [this](const auto& event) {
    // A quick disconnect/reconnect may have zero net current_count_change.
    // Cumulative new matches must still trigger a complete history replay.
    if (event.total_count_change > 0) replay_.fetch_add(1);
  };
  publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "global_lidar_map_markers", rclcpp::QoS(4).reliable().durability_volatile(), options);
}

bool GlobalLidarMapDisplay::subscribed() const {
  return publisher_->get_subscription_count() > 0
      || publisher_->get_intra_process_subscription_count() > 0;
}

std::string GlobalLidarMapDisplay::frame(uint64_t id) const {
  return prefix_ + "/kf_" + std::to_string(id);
}

GlobalLidarMapDisplay::Result GlobalLidarMapDisplay::publish(
    const LidarPointCloudMap::Snapshot& snapshot, size_t samplingStep, std::optional<uint64_t> revision,
    const std::function<void()>& checkpoint) {
  if (!samplingStep) throw std::invalid_argument("Incremental LiDAR sampling step must be positive");
  Result result;
  if (!subscribed()) return result;
  const uint64_t replay = replay_.load();
  if (!revision || indexedRevision_ != revision || indexedStep_ != samplingStep || indexedReplay_ != replay) {
    // No history traversal/sort on static-map ticks or subsequent replay batches.
    // Rebuild conservatively when no revision is supplied (e.g. protocol tests).
    indexedRevision_.reset();
    std::vector<geometry_msgs::msg::TransformStamped> poses;
    for (const auto& block : snapshot) {
      auto previous = poses_.find(block.id);
      std::array<double, 16> matrix;
      std::copy_n(block.pose.matrix().data(), 16, matrix.data());
      if (previous != poses_.end() && previous->second == matrix) continue;
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id = "world";
      transform.child_frame_id = frame(block.id);
      const Eigen::Quaterniond q(block.pose.linear());
      transform.transform.translation.x = block.pose.translation().x();
      transform.transform.translation.y = block.pose.translation().y();
      transform.transform.translation.z = block.pose.translation().z();
      transform.transform.rotation.x = q.x(); transform.transform.rotation.y = q.y();
      transform.transform.rotation.z = q.z(); transform.transform.rotation.w = q.w();
      poses.push_back(std::move(transform));
      poses_[block.id] = matrix;
    }
    if (!poses.empty()) transforms_.sendTransform(poses);
    result.poses = poses.size();

    candidates_.clear();
    nextCandidate_ = 0;
    uint64_t offset = 0;
    for (size_t i = 0; i < snapshot.size(); ++i) {
      const auto& block = snapshot[i];
      const size_t skip = (samplingStep - offset % samplingStep) % samplingStep;
      if (pointCount(block) > std::numeric_limits<uint64_t>::max() - offset)
        throw std::overflow_error("Incremental LiDAR point count overflow");
      offset += pointCount(block);
      auto previous = sent_.find({block.id, block.chunkIndex});
      if (previous == sent_.end()) { candidates_.push_back({i, skip, 0}); continue; }
      const auto& sent = previous->second;
      const bool geometryChanged = sent.points.lock() != block.points || sent.archive.lock() != block.archive
          || sent.step != samplingStep || sent.skip != skip;
      const auto height = heightRow(block.pose);
      double angle = 0;
      for (size_t a = 0; a < 3; ++a) angle += std::pow(height[a] - sent.height[a], 2);
      const bool colourChanged = std::sqrt(angle)*sent.radius + std::abs(height[3] - sent.height[3])
          >= colourResolution_;
      if (geometryChanged || colourChanged) candidates_.push_back({i, skip, 1});
      else if (sent.replay != replay) candidates_.push_back({i, skip, 2});
    }
    // New live keyframes precede historical replay/recolouring. Two archive
    // blocks per batch bounds work, not total visible points; no block is capped.
    std::sort(candidates_.begin(), candidates_.end(), [](const Candidate& a, const Candidate& b) {
      return a.priority != b.priority ? a.priority < b.priority : a.index > b.index;
    });
    indexedRevision_ = revision;
    indexedStep_ = samplingStep;
    indexedReplay_ = replay;
    result.indexed = snapshot.size();
  }
  visualization_msgs::msg::MarkerArray message;
  if (!initialised_) {
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "world";
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    message.markers.push_back(std::move(clear));
  }
  std::vector<std::pair<Key, Sent>> committed;
  const size_t work = std::min(size_t(2), candidates_.size() - nextCandidate_);
  for (size_t c = 0; c < work; ++c) {
    const auto candidate = candidates_[nextCandidate_ + c];
    const auto& block = snapshot[candidate.index];
    if (block.chunkIndex > size_t(std::numeric_limits<int32_t>::max()))
      throw std::length_error("Incremental LiDAR chunk ID exceeds Marker format");
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame(block.id);
    marker.ns = "global_lidar_" + std::to_string(block.id);
    marker.id = static_cast<int32_t>(block.chunkIndex);
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.frame_locked = true;
    marker.pose.orientation.w = 1;
    marker.scale.x = marker.scale.y = .03;
    marker.color.a = 1;
    // Content identity lets a slow RViz skip already applied blocks during
    // recovery replay, without hashing or retaining historical Marker arrays.
    const auto previous = sent_.find({block.id, block.chunkIndex});
    const uint64_t version = candidate.priority == 2 && previous != sent_.end()
        ? previous->second.version : ++version_;
    marker.text = std::to_string(version);
    auto local = block;
    local.pose = Eigen::Isometry3d::Identity();
    const LidarPointCloudMap::Snapshot one{local};
    const auto count = LidarPointCloudMap::sampledPointCount(one, samplingStep, candidate.skip);
    marker.points.reserve(count); marker.colors.reserve(count);
    const auto height = heightRow(block.pose);
    double radiusSquared = 0;
    size_t visited = 0;
    LidarPointCloudMap::forEachSampledPoint(one, samplingStep, [&](const Eigen::Vector3d& point) {
      if (checkpoint && ++visited % 4096 == 0) checkpoint();
      geometry_msgs::msg::Point xyz;
      xyz.x = point.x(); xyz.y = point.y(); xyz.z = point.z();
      marker.points.push_back(xyz);
      radiusSquared = std::max(radiusSquared, point.squaredNorm());
      const uint32_t rgb = colour_(height[0]*point.x() + height[1]*point.y()
          + height[2]*point.z() + height[3]);
      std_msgs::msg::ColorRGBA colour;
      colour.r = float((rgb >> 16) & 255) / 255;
      colour.g = float((rgb >> 8) & 255) / 255;
      colour.b = float(rgb & 255) / 255;
      colour.a = 1;
      marker.colors.push_back(colour);
    }, candidate.skip);
    if (!count) marker.action = visualization_msgs::msg::Marker::DELETE;
    result.points += count; ++result.blocks;
    message.markers.push_back(std::move(marker));
    committed.push_back({{block.id, block.chunkIndex},
        {block.points, block.archive, height, std::sqrt(radiusSquared), samplingStep, candidate.skip, replay, version}});
  }
  if (!message.markers.empty()) {
    publisher_->publish(message);
    for (auto& entry : committed) sent_[entry.first] = std::move(entry.second);
    nextCandidate_ += work;
    initialised_ = true;
  }
  return result;
}
} // namespace okvis
