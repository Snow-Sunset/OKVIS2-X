// SPDX-License-Identifier: BSD-3-Clause
#ifndef OKVIS_ROS2_GLOBAL_LIDAR_MAP_RVIZ_INBOX_HPP
#define OKVIS_ROS2_GLOBAL_LIDAR_MAP_RVIZ_INBOX_HPP

#include <visualization_msgs/msg/marker_array.hpp>
#include <algorithm>
#include <deque>
#include <map>
#include <mutex>
#include <optional>

namespace okvis {
namespace detail {

/// Transport/render-thread boundary. Bounds outstanding payloads, not the
/// persistent map. Contains no Ogre or estimator state and is independently
/// testable without a GPU. Callbacks capture shared ownership of this inbox.
class GlobalLidarMapRvizInbox {
 public:
  using Array = visualization_msgs::msg::MarkerArray;
  using Marker = visualization_msgs::msg::Marker;
  using Key = std::pair<std::string, int32_t>;
  using Version = std::pair<std::string, std::string>; // session frame, content revision
  struct Pending { Array::ConstSharedPtr message; size_t index; };
  static constexpr size_t capacity = 8;

  static Key key(const Marker& marker) { return {marker.ns, marker.id}; }
  static Version version(const Marker& marker) { return {marker.header.frame_id, marker.text}; }

  void accept(Array::ConstSharedPtr message) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < message->markers.size(); ++i) {
      const auto& marker = message->markers[i];
      if (marker.action == Marker::DELETEALL) {
        pending_.clear(); applied_.clear();
        pending_.push_back({message, i});
        continue;
      }
      if (marker.action != Marker::ADD && marker.action != Marker::DELETE) continue;
      const auto old = applied_.find(key(marker));
      // Empty text is accepted for manual debugging but not deduplicated.
      if (!marker.text.empty() && old != applied_.end() && version(marker) == old->second) continue;
      auto queued = std::find_if(pending_.begin(), pending_.end(), [&](const Pending& item) {
        const auto& previous = item.message->markers[item.index];
        return previous.action != Marker::DELETEALL && key(previous) == key(marker);
      });
      if (queued != pending_.end()) { *queued = {message, i}; continue; }
      // Overload requests replay. Applied versions skip completed blocks on
      // subsequent replay rounds, allowing even a slow viewer to catch up.
      if (pending_.size() >= capacity) { replay_ = true; continue; }
      pending_.push_back({message, i});
    }
  }
  std::optional<Pending> take() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_.empty()) return {};
    Pending next = std::move(pending_.front()); pending_.pop_front();
    return next;
  }
  void acknowledge(const Marker& marker) {
    std::lock_guard<std::mutex> lock(mutex_);
    applied_[key(marker)] = version(marker);
  }
  void cleared() {
    std::lock_guard<std::mutex> lock(mutex_);
    applied_.clear();
  }
  void requestReplay() {
    std::lock_guard<std::mutex> lock(mutex_);
    replay_ = true;
  }
  bool replayWhenDrained() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!replay_ || !pending_.empty()) return false;
    replay_ = false;
    return true;
  }
  size_t pendingCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_.size();
  }

 private:
  mutable std::mutex mutex_;
  std::deque<Pending> pending_;
  std::map<Key, Version> applied_;
  bool replay_ = false;
};
} // namespace detail
} // namespace okvis
#endif
