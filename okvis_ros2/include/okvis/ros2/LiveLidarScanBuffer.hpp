// SPDX-License-Identifier: BSD-3-Clause
#ifndef OKVIS_ROS2_LIVE_LIDAR_SCAN_BUFFER_HPP
#define OKVIS_ROS2_LIVE_LIDAR_SCAN_BUFFER_HPP

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

namespace okvis {
namespace detail {

/// A bounded short history is essential: replacing the latest scan alone
/// starves display when every scan arrives just BEFORE its corresponding TF.
/// Contains no estimator, Ogre, global-map state or retained displayed cloud.
class LiveLidarScanBuffer {
 public:
  using Cloud = sensor_msgs::msg::PointCloud2;
  using Clock = std::chrono::steady_clock;
  struct Entry { uint64_t sequence; Cloud::ConstSharedPtr cloud; };
  struct View {
    std::vector<Entry> pending;
    uint64_t received = 0, rejected = 0;
    Clock::time_point lastReceived{};

    template<class Ready> const Entry* newestReady(Ready ready) const {
      for (auto it = pending.rbegin(); it != pending.rend(); ++it)
        if (ready(*it->cloud)) return &*it;
      return nullptr;
    }
  };
  static constexpr size_t capacity = 4;
  static constexpr size_t maxBytes = 16*1024*1024;
  static constexpr size_t maxPoints = 262144;

  void accept(Cloud::ConstSharedPtr cloud) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++received_;
    if (!cloud || cloud->header.frame_id.empty() || cloud->data.size() > maxBytes
        || uint64_t(cloud->width)*cloud->height > maxPoints) {
      ++rejected_; return;
    }
    pending_.push_back({received_, std::move(cloud)});
    while (pending_.size() > capacity) pending_.pop_front();
    lastReceived_ = Clock::now();
  }
  View view() const {
    std::lock_guard<std::mutex> lock(mutex_);
    View result;
    result.pending.assign(pending_.begin(), pending_.end());
    result.received = received_;
    result.rejected = rejected_;
    result.lastReceived = lastReceived_;
    return result;
  }
  void consumeThrough(uint64_t sequence) {
    std::lock_guard<std::mutex> lock(mutex_);
    while (!pending_.empty() && pending_.front().sequence <= sequence) pending_.pop_front();
  }

  /// Decode finite XYZ only, retaining original scan coordinates and point
  /// count. Respect row padding, FLOAT32/FLOAT64 fields and message endianness.
  template<class Visit> static size_t forEachPoint(const Cloud& cloud, Visit visit) {
    using Field = sensor_msgs::msg::PointField;
    if (cloud.data.size() > maxBytes || uint64_t(cloud.width)*cloud.height > maxPoints
        || cloud.point_step == 0 || uint64_t(cloud.width)*cloud.point_step > cloud.row_step
        || uint64_t(cloud.height)*cloud.row_step > cloud.data.size())
      throw std::invalid_argument("Invalid/oversized PointCloud2 dimensions");
    std::array<const Field*, 3> fields{};
    const char* names[] = {"x", "y", "z"};
    for (size_t axis = 0; axis < fields.size(); ++axis) {
      for (const auto& field : cloud.fields) if (field.name == names[axis]) { fields[axis] = &field; break; }
      const auto* field = fields[axis];
      if (!field || field->count != 1
          || (field->datatype != Field::FLOAT32 && field->datatype != Field::FLOAT64)
          || uint64_t(field->offset) + (field->datatype == Field::FLOAT32 ? 4 : 8) > cloud.point_step)
        throw std::invalid_argument("PointCloud2 requires valid FLOAT32/FLOAT64 XYZ fields");
    }
    const uint16_t endian = 1;
    const bool swap = cloud.is_bigendian != (*reinterpret_cast<const uint8_t*>(&endian) == 0);
    size_t count = 0;
    for (size_t row = 0; row < cloud.height; ++row) for (size_t col = 0; col < cloud.width; ++col) {
      const auto* data = cloud.data.data() + row*cloud.row_step + col*cloud.point_step;
      std::array<float, 3> xyz;
      bool finite = true;
      for (size_t axis = 0; axis < fields.size(); ++axis) {
        const auto* field = fields[axis];
        const auto* value = data + field->offset;
        const double coordinate = field->datatype == Field::FLOAT32
            ? scalar<float>(value, swap) : scalar<double>(value, swap);
        if (!std::isfinite(coordinate) || std::abs(coordinate) > std::numeric_limits<float>::max()) {
          finite = false; break;
        }
        xyz[axis] = static_cast<float>(coordinate);
      }
      if (!finite) continue;
      visit(xyz); ++count;
    }
    return count;
  }

 private:
  template<class T> static T scalar(const uint8_t* value, bool swap) {
    std::array<uint8_t, sizeof(T)> bytes;
    std::copy_n(value, bytes.size(), bytes.begin());
    if (swap) std::reverse(bytes.begin(), bytes.end());
    T result;
    std::memcpy(&result, bytes.data(), sizeof(result));
    return result;
  }
  mutable std::mutex mutex_;
  std::deque<Entry> pending_;
  uint64_t received_ = 0, rejected_ = 0;
  Clock::time_point lastReceived_{};
};
} // namespace detail
} // namespace okvis
#endif
