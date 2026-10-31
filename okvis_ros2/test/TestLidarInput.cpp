// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/ros2/PointCloudUtilities.hpp>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

template<class T>
void write(sensor_msgs::msg::PointCloud2& msg, size_t row, size_t col, size_t offset, T value) {
  const uint16_t probe = 1;
  const bool hostBigEndian = *reinterpret_cast<const uint8_t*>(&probe) == 0;
  auto* bytes = reinterpret_cast<uint8_t*>(&value);
  if (msg.is_bigendian != hostBigEndian) std::reverse(bytes, bytes + sizeof(T));
  std::memcpy(msg.data.data() + row * msg.row_step + col * msg.point_step + offset,
              bytes, sizeof(T));
}

sensor_msgs::msg::PointCloud2 cloud(bool bigEndian) {
  using Field = sensor_msgs::msg::PointField;
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp.sec = 10;
  msg.header.stamp.nanosec = 990000000;
  msg.width = 3;
  msg.height = 2;
  msg.point_step = 29;
  msg.row_step = msg.width * msg.point_step + 5; // Deliberately padded rows.
  msg.is_bigendian = bigEndian;
  for (const auto& spec : std::vector<std::pair<std::string, uint32_t>>{
         {"x", 0}, {"y", 4}, {"z", 8}, {"t", 16}, {"ring", 22}}) {
    Field field;
    field.name = spec.first;
    field.offset = spec.second;
    field.count = 1;
    field.datatype = field.name == "t" ? Field::UINT32
        : field.name == "ring" ? Field::UINT8 : Field::FLOAT32;
    msg.fields.push_back(field);
  }
  msg.data.resize(msg.height * msg.row_step);
  const uint32_t times[] = {30000000, 10000000, 20000000};
  for (size_t row = 0; row < msg.height; ++row) {
    for (size_t col = 0; col < msg.width; ++col) {
      write(msg, row, col, 0, float(10 * row + col));
      write(msg, row, col, 4, -1.25f);
      write(msg, row, col, 8, 2.5f);
      write(msg, row, col, 16, times[col]);
      write(msg, row, col, 22, uint8_t(row));
    }
  }
  return msg;
}

void run() {
  using Reference = okvis::pointcloud_ros::OusterTimeReference;
  for (bool bigEndian : {false, true}) {
    auto msg = cloud(bigEndian);
    okvis::LidarMeasurementBatch points;
    check(okvis::pointcloud_ros::lidar2batch(msg, points) == 0 && points.size() == 6,
          "Organized unaligned cloud lost points");
    const double expectedX[] = {1, 11, 2, 12, 0, 10};
    for (size_t i = 0; i < points.size(); ++i) {
      check(points[i].timeStamp.toNSec() == 11000000000ull + (i / 2) * 10000000ull,
            "Nanosecond offset, ordering or second rollover changed");
      check(points[i].measurement.rayMeasurement.x() == expectedX[i]
                && points[i].measurement.rayMeasurement.y() == -1.25,
            "Time sorting separated a point from its coordinates");
    }
    okvis::pointcloud_ros::lidar2batch(msg, points, Reference::ScanEnd);
    check(points.front().timeStamp.toNSec() == 10970000000ull
              && points.back().timeStamp.toNSec() == 10990000000ull,
          "Scan-end timestamp convention changed");
    okvis::pointcloud_ros::lidar2batch(msg, points, Reference::Header);
    for (const auto& point : points)
      check(point.timeStamp.toNSec() == 10990000000ull, "Header compatibility mode used offsets");
    write(msg, 0, 1, 0, std::numeric_limits<float>::quiet_NaN());
    check(okvis::pointcloud_ros::lidar2batch(msg, points) == 1 && points.size() == 5,
          "Invalid point was not counted and removed");
  }
  using Field = sensor_msgs::msg::PointField;
  for (bool bigEndian : {false, true}) {
    auto msg = cloud(bigEndian);
    msg.fields.resize(3);
    Field timestamp;
    timestamp.name = "timestamp";
    timestamp.offset = 16;
    timestamp.datatype = Field::FLOAT64;
    timestamp.count = 1;
    msg.fields.push_back(timestamp);
    for (size_t row = 0; row < msg.height; ++row)
      for (size_t col = 0; col < msg.width; ++col)
        write(msg, row, col, 16, 10.9999999999);
    okvis::LidarMeasurementBatch points;
    check(okvis::pointcloud_ros::lidar2batch(msg, points) == 0 && points.size() == 6,
          "Hesai timestamp decoding lost points");
    for (const auto& point : points)
      check(point.timeStamp == okvis::Time(11, 0), "Rounded Hesai time was not normalized");

    msg.fields.resize(3);
    for (const char* name : {"stamp_high", "stamp_low"}) {
      Field field;
      field.name = name;
      field.offset = field.name == "stamp_high" ? 16 : 20;
      field.datatype = Field::UINT32;
      field.count = 1;
      msg.fields.push_back(field);
    }
    constexpr uint64_t absoluteNs = 11000000042ull;
    for (size_t row = 0; row < msg.height; ++row) {
      for (size_t col = 0; col < msg.width; ++col) {
        write(msg, row, col, 16, uint32_t(absoluteNs >> 32));
        write(msg, row, col, 20, uint32_t(absoluteNs));
      }
    }
    check(okvis::pointcloud_ros::lidar2batch(msg, points) == 0 && points.size() == 6,
          "BLK timestamp decoding lost points");
    for (const auto& point : points)
      check(point.timeStamp.toNSec() == absoluteNs, "Split absolute nanoseconds changed");
  }

  auto doubles = cloud(false);
  doubles.width = 2;
  doubles.height = 1;
  doubles.point_step = 24;
  doubles.row_step = 48;
  doubles.data.assign(48, 0);
  doubles.fields.resize(3);
  for (size_t axis = 0; axis < 3; ++axis) {
    doubles.fields[axis].offset = uint32_t(axis * sizeof(double));
    doubles.fields[axis].datatype = Field::FLOAT64;
    for (size_t col = 0; col < doubles.width; ++col)
      write(doubles, 0, col, axis * sizeof(double), double(axis) + 0.125 * double(col));
  }
  okvis::LidarMeasurementBatch doublePoints;
  check(okvis::pointcloud_ros::lidar2batch(doubles, doublePoints) == 0 && doublePoints.size() == 2
            && doublePoints[1].measurement.rayMeasurement.z() == 2.125,
        "Generic float64 cloud coordinates changed");
  auto invalid = cloud(false);
  invalid.data.pop_back();
  okvis::LidarMeasurementBatch points;
  bool rejected = false;
  try { okvis::pointcloud_ros::lidar2batch(invalid, points); }
  catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, "Truncated PointCloud2 was read past its payload");
  invalid = cloud(false);
  invalid.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  rejected = false;
  try { okvis::pointcloud_ros::lidar2batch(invalid, points); }
  catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, "Incorrect Ouster timestamp type was silently reinterpreted");
}
} // namespace

int main() {
  try { run(); }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
  return 0;
}
