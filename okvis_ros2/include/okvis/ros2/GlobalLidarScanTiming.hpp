// SPDX-License-Identifier: BSD-3-Clause
#ifndef OKVIS_ROS2_GLOBAL_LIDAR_SCAN_TIMING_HPP
#define OKVIS_ROS2_GLOBAL_LIDAR_SCAN_TIMING_HPP

#include <okvis/ros2/PointCloudUtilities.hpp>
#include <optional>
#include <utility>

namespace okvis {
namespace detail {

/// Scan interval without allocating measurements, sorting, or constructing
/// Eigen point vectors. Used ONLY by the PCD sidecar's keyframe prefilter.
/// Validate finite XYZ so a NaN point with an extreme timestamp cannot stall
/// the state watermark. Exact selection still precedes point integration.
inline std::optional<std::pair<uint64_t, uint64_t>> globalLidarScanTimeBounds(
    const sensor_msgs::msg::PointCloud2& msg, pointcloud_ros::OusterTimeReference reference) {
  using Field = sensor_msgs::msg::PointField;
  using namespace pointcloud_ros::detail;
  if (!msg.width || !msg.height) return std::nullopt;
  if (!msg.point_step || uint64_t(msg.width) * msg.point_step > msg.row_step
      || uint64_t(msg.height) * msg.row_step > msg.data.size()
      || msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000u)
    throw std::invalid_argument("Invalid global PCD scan layout or timestamp");
  const Field* xyz[] = {field(msg, "x"), field(msg, "y"), field(msg, "z")};
  for (const auto* f : xyz) {
    if (!f || (f->datatype != Field::FLOAT32 && f->datatype != Field::FLOAT64))
      throw std::invalid_argument("Global PCD XYZ fields must be float32 or float64");
    requireField(msg, f, f->datatype, f->datatype == Field::FLOAT32 ? 4 : 8);
  }
  const auto* absolute = field(msg, "timestamp");
  const auto* high = field(msg, "stamp_high");
  const auto* low = field(msg, "stamp_low");
  const auto* offset = field(msg, "t");
  const bool blk = !absolute && high && low;
  const bool ouster = !absolute && !blk && offset && field(msg, "ring");
  if (absolute) requireField(msg, absolute, Field::FLOAT64, 8);
  if (blk) { requireField(msg, high, Field::UINT32, 4); requireField(msg, low, Field::UINT32, 4); }
  if (ouster) requireField(msg, offset, Field::UINT32, 4);
  const uint64_t header = uint64_t(msg.header.stamp.sec) * 1000000000ull + msg.header.stamp.nanosec;
  if (!absolute && !blk && (!ouster || reference == pointcloud_ros::OusterTimeReference::Header))
    return std::make_pair(header, header);
  const uint16_t probe = 1;
  const bool swap = msg.is_bigendian != (*reinterpret_cast<const uint8_t*>(&probe) == 0);
  const uint64_t maximum = uint64_t(std::numeric_limits<uint32_t>::max()) * 1000000000ull + 999999999ull;
  uint64_t first = std::numeric_limits<uint64_t>::max(), last = 0;
  uint64_t maximumOffset = 0;
  for (size_t row = 0; row < msg.height; ++row) for (size_t col = 0; col < msg.width; ++col) {
    const auto* point = msg.data.data() + size_t(row) * msg.row_step + col * msg.point_step;
    uint64_t ns;
    if (absolute) {
      const double seconds = scalar<double>(point + absolute->offset, swap);
      if (!std::isfinite(seconds) || seconds < 0
          || seconds >= double(std::numeric_limits<uint32_t>::max()) + 1.0) continue;
      Time time;
      time.fromSec(seconds);
      ns = time.toNSec();
    } else if (blk) {
      ns = (uint64_t(scalar<uint32_t>(point + high->offset, swap)) << 32)
          | scalar<uint32_t>(point + low->offset, swap);
    } else {
      // Keep offsets until both extrema are known: ScanEnd uses the maximum
      // of ALL offsets, exactly like lidar2batch(), including invalid XYZ.
      ns = scalar<uint32_t>(point + offset->offset, swap);
      maximumOffset = std::max(maximumOffset, ns);
    }
    if (ns > maximum) continue;
    bool finite = true;
    for (const auto* f : xyz) {
      if (f->datatype == Field::FLOAT32) finite = finite && std::isfinite(scalar<float>(point + f->offset, swap));
      else finite = finite && std::isfinite(scalar<double>(point + f->offset, swap));
    }
    if (!finite) continue;
    first = std::min(first, ns); last = std::max(last, ns);
  }
  if (first == std::numeric_limits<uint64_t>::max()) return std::nullopt;
  if (ouster) {
    if (reference == pointcloud_ros::OusterTimeReference::ScanEnd) {
      if (maximumOffset > header) throw std::invalid_argument("Ouster scan end precedes its duration");
      first += header - maximumOffset;
      last += header - maximumOffset;
    } else {
      first += header; last += header;
    }
    if (first > maximum) return std::nullopt;
    last = std::min(last, maximum);
  }
  return std::make_pair(first, last);
}

} // namespace detail
} // namespace okvis
#endif
