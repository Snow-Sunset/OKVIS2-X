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
 * @file PointCloudUtilities.hpp
 * @brief Some functions to work with sensor_msgs::msg::PointCloud2 messages.
 * @author Simon Boche
 */

#ifndef INCLUDE_OKVIS_PCL_UTILITIES_HPP_
#define INCLUDE_OKVIS_PCL_UTILITIES_HPP_

#include <string>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <okvis/Time.hpp>
#include <okvis/Measurements.hpp>

namespace okvis {
namespace pointcloud_ros{

/**
 * @ brief check if a certain data field exists in the message
 */
inline bool has_field(const sensor_msgs::msg::PointCloud2& pointcloud, const std::string & field_name)
{
  for (const auto & field : pointcloud.fields) {
    if (field.name == field_name) {
      return true;
    }
  }
  return false;
}

enum class OusterTimeReference { ScanStart, ScanEnd, Header };

namespace detail {
inline const sensor_msgs::msg::PointField* field(
    const sensor_msgs::msg::PointCloud2& msg, const char* name) {
  for (const auto& f : msg.fields) if (f.name == name) return &f;
  return nullptr;
}

// PointCloud2 records need not be aligned (the VBR point_step is 29 bytes).
template<class T>
T scalar(const uint8_t* bytes, bool swapEndian) {
  T value;
  std::memcpy(&value, bytes, sizeof(T));
  if (swapEndian) {
    auto* begin = reinterpret_cast<uint8_t*>(&value);
    std::reverse(begin, begin + sizeof(T));
  }
  return value;
}

inline void requireField(const sensor_msgs::msg::PointCloud2& msg,
    const sensor_msgs::msg::PointField* f, uint8_t type, size_t bytes) {
  if (!f || f->datatype != type || f->count != 1
      || size_t(f->offset) > msg.point_step
      || bytes > size_t(msg.point_step) - f->offset)
    throw std::invalid_argument("Missing or invalid LiDAR PointCloud2 field");
}
} // namespace detail

/// Decode directly into one contiguous transport batch, retaining point times.
/// Native Ouster `t` is a UINT32 nanosecond offset from the scan start. The
/// reference is explicit for converted bags whose header denotes the scan end.
inline size_t lidar2batch(const sensor_msgs::msg::PointCloud2& msg,
    LidarMeasurementBatch& output,
    OusterTimeReference ousterReference = OusterTimeReference::ScanStart) {
  using Field = sensor_msgs::msg::PointField;
  output.clear();
  if (!msg.width || !msg.height) return 0;
  if (!msg.point_step || uint64_t(msg.width) * msg.point_step > msg.row_step
      || uint64_t(msg.height) * msg.row_step > msg.data.size()
      || msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000u)
    throw std::invalid_argument("Invalid LiDAR PointCloud2 layout or timestamp");
  const auto* x = detail::field(msg, "x");
  const auto* y = detail::field(msg, "y");
  const auto* z = detail::field(msg, "z");
  for (const auto* f : {x, y, z}) {
    if (!f || (f->datatype != Field::FLOAT32 && f->datatype != Field::FLOAT64))
      throw std::invalid_argument("LiDAR XYZ fields must be float32 or float64");
    detail::requireField(msg, f, f->datatype, f->datatype == Field::FLOAT32 ? 4 : 8);
  }
  const auto* absolute = detail::field(msg, "timestamp");
  const auto* high = detail::field(msg, "stamp_high");
  const auto* low = detail::field(msg, "stamp_low");
  const auto* offset = detail::field(msg, "t");
  const bool hesai = absolute != nullptr;
  const bool blk = !hesai && high && low;
  const bool ouster = !hesai && !blk && offset && detail::field(msg, "ring");
  if (hesai) detail::requireField(msg, absolute, Field::FLOAT64, 8);
  if (blk) {
    detail::requireField(msg, high, Field::UINT32, 4);
    detail::requireField(msg, low, Field::UINT32, 4);
  }
  if (ouster) detail::requireField(msg, offset, Field::UINT32, 4);
  const uint16_t endianProbe = 1;
  const bool hostBigEndian = *reinterpret_cast<const uint8_t*>(&endianProbe) == 0;
  const bool swapEndian = msg.is_bigendian != hostBigEndian;
  uint64_t headerNs = uint64_t(msg.header.stamp.sec) * 1000000000ull + msg.header.stamp.nanosec;
  if (ouster && ousterReference == OusterTimeReference::ScanEnd) {
    uint32_t maxOffset = 0;
    for (size_t row = 0; row < msg.height; ++row) {
      for (size_t col = 0; col < msg.width; ++col) {
        const auto* p = msg.data.data() + row * msg.row_step + col * msg.point_step;
        maxOffset = std::max(maxOffset, detail::scalar<uint32_t>(p + offset->offset, swapEndian));
      }
    }
    if (maxOffset > headerNs) throw std::invalid_argument("Ouster scan end precedes its duration");
    headerNs -= maxOffset;
  }
  const uint64_t maxTimeNs = uint64_t(std::numeric_limits<uint32_t>::max())
      * 1000000000ull + 999999999ull;
  size_t invalid = 0;
  output.reserve(size_t(msg.width) * msg.height);
  const auto append = [&](size_t row, size_t col) {
    const auto* p = msg.data.data() + row * msg.row_step + col * msg.point_step;
    Eigen::Vector3d ray;
    const Field* fields[] = {x, y, z};
    for (size_t axis = 0; axis < 3; ++axis) {
      const auto* f = fields[axis];
      ray[axis] = f->datatype == Field::FLOAT32
          ? double(detail::scalar<float>(p + f->offset, swapEndian))
          : detail::scalar<double>(p + f->offset, swapEndian);
    }
    if (!ray.allFinite()) { ++invalid; return; }
    LidarMeasurement measurement;
    measurement.measurement.rayMeasurement = ray;
    measurement.measurement.intensity = 0;
    if (hesai) {
      const double seconds = detail::scalar<double>(p + absolute->offset, swapEndian);
      if (!std::isfinite(seconds) || seconds < 0.0
          || seconds >= double(std::numeric_limits<uint32_t>::max()) + 1.0) {
        ++invalid; return;
      }
      measurement.timeStamp.fromSec(seconds);
      // Time::fromSec rounds nanoseconds but does not carry a rounded 1e9
      // into seconds. Normalize before comparisons or ordering this batch.
      const uint64_t ns = measurement.timeStamp.toNSec();
      if (ns > maxTimeNs) { ++invalid; return; }
      measurement.timeStamp.fromNSec(ns);
    } else {
      uint64_t ns = headerNs;
      if (blk) ns = (uint64_t(detail::scalar<uint32_t>(p + high->offset, swapEndian)) << 32)
          | detail::scalar<uint32_t>(p + low->offset, swapEndian);
      else if (ouster && ousterReference != OusterTimeReference::Header)
        ns += detail::scalar<uint32_t>(p + offset->offset, swapEndian);
      if (ns > maxTimeNs) { ++invalid; return; }
      measurement.timeStamp.fromNSec(ns);
    }
    output.push_back(measurement);
  };
  if (ouster && msg.height > 1) {
    // Organized Ouster clouds are stored ring-major; column-major decoding
    // groups equal timestamps and usually avoids an O(N log N) time sort.
    for (size_t col = 0; col < msg.width; ++col)
      for (size_t row = 0; row < msg.height; ++row) append(row, col);
  } else {
    for (size_t row = 0; row < msg.height; ++row)
      for (size_t col = 0; col < msg.width; ++col) append(row, col);
  }
  const auto byTime = [](const LidarMeasurement& a, const LidarMeasurement& b) {
    return a.timeStamp < b.timeStamp;
  };
  if (!std::is_sorted(output.begin(), output.end(), byTime))
    std::stable_sort(output.begin(), output.end(), byTime);
  return invalid;
}

/**
 * @ brief Get point-wise timestamps and measurements for BLK2FLY type PointCloud msgs
 */
inline void blk_lidar2points(const sensor_msgs::msg::PointCloud2& msg,
                             std::vector<okvis::Time, Eigen::aligned_allocator<okvis::Time>>& timestamps,
                             std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& rays) 
{

  timestamps.reserve(msg.width * msg.height);
  rays.reserve(msg.width * msg.height);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x"), iter_y(msg, "y"), iter_z(msg,"z");
  sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_stamp_high(msg, "stamp_high"), iter_stamp_low(msg, "stamp_low");
  size_t counter=0;
  while (iter_x != iter_x.end()){
    rays.push_back(Eigen::Vector3d(*iter_x, *iter_y, *iter_z));
    okvis::Time ts;
    ts.fromNSec((static_cast<uint64_t>(*iter_stamp_high) << 32) | *iter_stamp_low);
    timestamps.push_back(ts);
    ++iter_x;
    ++iter_y;
    ++iter_z;
    if(iter_x != iter_x.end()){
      ++iter_stamp_low;
      ++iter_stamp_high;
    }
    counter++;
  }
}

/**
 * @ brief Get point-wise timestamps and measurements for Hesai type PointCloud msgs (Hilti22 SLAM Challenge)
 */
inline void hesai_lidar2points(const sensor_msgs::msg::PointCloud2& msg,
                               std::vector<okvis::Time, Eigen::aligned_allocator<okvis::Time>>& timestamps,
                               std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& rays)
{
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x"), iter_y(msg, "y"), iter_z(msg,"z");
  sensor_msgs::PointCloud2ConstIterator<double> iter_stamp(msg, "timestamp");

  while (iter_x != iter_x.end()){
    rays.push_back(Eigen::Vector3d(*iter_x, *iter_y, *iter_z));
    okvis::Time ts;
    ts.fromSec(*iter_stamp);
    timestamps.push_back(ts);
    ++iter_x;
    ++iter_y;
    ++iter_z;
    if(iter_x != iter_x.end()){
      ++iter_stamp;
    }
  }
}

} // namespace pointcloud_ros
} // namespace okvis

#endif // INCLUDE_OKVIS_PCL_UTILITIES_HPP_
