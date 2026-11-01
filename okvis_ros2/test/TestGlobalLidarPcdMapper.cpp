// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/ros2/GlobalLidarPcdMapper.hpp>
#include <okvis/ros2/GlobalLidarMapRvizInbox.hpp>
#include <okvis/ros2/LiveLidarScanBuffer.hpp>
#include <okvis/ros2/GlobalLidarScanTiming.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.hpp>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
template <typename Predicate> void wait(const std::shared_ptr<rclcpp::Node>& node, Predicate done) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (!done() && std::chrono::steady_clock::now() < end) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  check(done(), "Timed out waiting for global PCD worker/ROS data");
}
void spinFor(const std::shared_ptr<rclcpp::Node>& node, std::chrono::milliseconds duration) {
  const auto end = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < end) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
okvis::State state(uint64_t id, double time, double z = 0, bool keyframe = true) {
  okvis::State result{};
  result.id = okvis::StateId(id);
  result.timestamp = okvis::Time(time);
  result.T_WS.set(Eigen::Vector3d(time - 1.0, 0, z), Eigen::Quaterniond::Identity());
  result.v_W = Eigen::Vector3d(1, 0, 0);
  result.b_g.setZero(); result.b_a.setZero(); result.omega_S.setZero();
  result.isKeyframe = keyframe;
  if (id == 2) for (int i = 0; i <= 100; ++i) {
    result.previousImuMeasurements.emplace_back(okvis::Time(1.0 + i*.01),
        okvis::ImuSensorReadings(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 9.81)));
  }
  return result;
}
void update(okvis::GlobalLidarPcdMapper& mapper, const okvis::State& current,
            const std::optional<okvis::State>& old = {}, uint64_t reference = 0) {
  auto states = std::make_shared<okvis::GlobalLidarPcdMapper::States>();
  states->emplace(current.id, current);
  if (old) states->emplace(old->id, *old);
  okvis::TrackingState tracking{};
  tracking.id = current.id;
  tracking.currentKeyframeId = reference ? okvis::StateId(reference) : current.id;
  tracking.isKeyframe = current.isKeyframe;
  mapper.stateUpdateCallback(tracking, states);
}
sensor_msgs::msg::PointCloud2 scan(size_t count, double scanTime = 1.2, float xOffset = 0) {
  sensor_msgs::msg::PointCloud2 result;
  const okvis::Time stamp(scanTime);
  result.header.stamp.sec = stamp.sec;
  result.header.stamp.nanosec = stamp.nsec;
  result.header.frame_id = "test_pcd_lidar";
  result.height = 1; result.width = static_cast<uint32_t>(count);
  result.point_step = 20; result.row_step = result.point_step*result.width;
  result.fields.resize(4);
  const char* names[] = {"x", "y", "z", "timestamp"};
  for (size_t i = 0; i < 4; ++i) {
    result.fields[i].name = names[i]; result.fields[i].offset = static_cast<uint32_t>(4*i);
    result.fields[i].count = 1;
    result.fields[i].datatype = i == 3 ? sensor_msgs::msg::PointField::FLOAT64
                                      : sensor_msgs::msg::PointField::FLOAT32;
  }
  result.data.resize(result.row_step);
  for (size_t i = 0; i < count; ++i) {
    const float xyz[] = {xOffset + 1.f + float(i%200)*.25f, float(i/200)*.25f, float(i%5)*.25f};
    // Deliberately span the 4096-point propagation batch with identical times.
    const double time = i < 5000 ? scanTime : scanTime + .1;
    std::memcpy(result.data.data() + i*20, xyz, sizeof(xyz));
    std::memcpy(result.data.data() + i*20 + 12, &time, sizeof(time));
  }
  return result;
}

void testScanTimingPrefilter() {
  using Reference = okvis::pointcloud_ros::OusterTimeReference;
  const auto verify = [](const sensor_msgs::msg::PointCloud2& cloud, Reference reference) {
    okvis::LidarMeasurementBatch decoded;
    okvis::pointcloud_ros::lidar2batch(cloud, decoded, reference);
    const auto bounds = okvis::detail::globalLidarScanTimeBounds(cloud, reference);
    check(!decoded.empty() && bounds && bounds->first == decoded.front().timeStamp.toNSec()
        && bounds->second == decoded.back().timeStamp.toNSec(),
        "PCD prefilter time bounds differ from the full decoder");
  };
  auto absolute = scan(5003);
  verify(absolute, Reference::ScanStart);
  // Invalid XYZ with a far-future timestamp must not hold up the watermark.
  const double future = 10000;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(absolute.data.data(), &nan, sizeof(nan));
  std::memcpy(absolute.data.data() + 12, &future, sizeof(future));
  verify(absolute, Reference::ScanStart);

  using Field = sensor_msgs::msg::PointField;
  auto organised = scan(4, 10);
  organised.width = 2; organised.height = 2;
  organised.row_step = 2 * organised.point_step + 5; // unaligned row padding
  organised.data.assign(2 * organised.row_step, 0);
  organised.fields[3].name = "t"; organised.fields[3].datatype = Field::UINT32;
  auto ring = organised.fields[3]; ring.name = "ring"; ring.offset = 16; ring.datatype = Field::UINT16;
  organised.fields.push_back(ring);
  const uint32_t offsets[] = {9000000, 2000000, 6000000, 2000000};
  for (size_t i = 0; i < 4; ++i) {
    auto* point = organised.data.data() + (i / 2) * organised.row_step + (i % 2) * organised.point_step;
    const float xyz[] = {float(i + 1), 1, 2};
    std::memcpy(point, xyz, sizeof(xyz));
    std::memcpy(point + 12, &offsets[i], sizeof(uint32_t));
  }
  for (auto reference : {Reference::ScanStart, Reference::ScanEnd, Reference::Header}) verify(organised, reference);
  // ScanEnd's duration must include offsets from invalid XYZ as in lidar2batch.
  std::memcpy(organised.data.data(), &nan, sizeof(nan));
  verify(organised, Reference::ScanEnd);
  for (size_t row = 0; row < organised.height; ++row) for (size_t col = 0; col < organised.width; ++col) {
    auto* point = organised.data.data() + row * organised.row_step + col * organised.point_step;
    for (const auto& field : organised.fields) {
      const size_t bytes = field.datatype == Field::UINT16 ? 2 : 4;
      std::reverse(point + field.offset, point + field.offset + bytes);
    }
  }
  organised.is_bigendian = !organised.is_bigendian;
  for (auto reference : {Reference::ScanStart, Reference::ScanEnd, Reference::Header}) verify(organised, reference);
  auto malformed = organised;
  malformed.fields[3].offset = malformed.point_step - 1;
  bool rejected = false;
  try { okvis::detail::globalLidarScanTimeBounds(malformed, Reference::ScanStart); }
  catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, "PCD prefilter read a time field beyond its record");

  // BLK absolute nanosecond fields ignore the header/Ouster reference.
  auto blk = scan(4);
  blk.fields[3].name = "stamp_high"; blk.fields[3].datatype = Field::UINT32;
  auto low = blk.fields[3]; low.name = "stamp_low"; low.offset = 16;
  blk.fields.push_back(low);
  for (size_t i = 0; i < 4; ++i) {
    const uint64_t ns = 10000000000ull + offsets[i];
    const uint32_t words[] = {uint32_t(ns >> 32), uint32_t(ns)};
    std::memcpy(blk.data.data() + i * blk.point_step + 12, words, sizeof(words));
  }
  verify(blk, Reference::ScanStart);
  verify(blk, Reference::ScanEnd);
  auto header = scan(3);
  header.fields.pop_back();
  verify(header, Reference::Header);
}

void testLiveScanBuffer() {
  using Buffer = okvis::detail::LiveLidarScanBuffer;
  Buffer bounded;
  for (size_t i = 0; i < 20; ++i)
    bounded.accept(std::make_shared<Buffer::Cloud>(scan(1, 10.0 + i*.1)));
  auto view = bounded.view();
  check(view.pending.size() == Buffer::capacity && view.pending.front().sequence == 17
      && view.pending.back().sequence == 20, "Raw scan TF wait grew beyond its bounded history");
  bounded.consumeThrough(19);
  check(bounded.view().pending.size() == 1 && bounded.view().pending.front().sequence == 20,
      "Displaying an older TF-ready scan discarded the newer waiting scan");
  bounded.accept({});
  auto invalid = std::make_shared<Buffer::Cloud>(scan(1));
  invalid->header.frame_id.clear(); bounded.accept(invalid);
  invalid = std::make_shared<Buffer::Cloud>(scan(1));
  invalid->width = Buffer::maxPoints + 1; bounded.accept(invalid);
  check(bounded.view().rejected == 3 && bounded.view().pending.size() == 1,
      "Invalid/oversized raw input entered the waiting history");

  // Real TF interpolation with the newest scan always ahead of TF by one
  // sample. A latest-only slot never displays a scan in this situation.
  tf2_ros::Buffer transforms(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "world"; tf.child_frame_id = "test_pcd_lidar";
  tf.header.stamp.sec = 10; tf.transform.rotation.w = 1;
  tf.transform.translation.x = 1;
  check(transforms.setTransform(tf, "live_scan_test"), "Could not insert lower TF bound");
  tf.header.stamp.nanosec = 100000000; tf.transform.translation.x = 2;
  check(transforms.setTransform(tf, "live_scan_test"), "Could not insert upper TF bound");
  Buffer waiting;
  waiting.accept(std::make_shared<Buffer::Cloud>(scan(1, 10.05)));
  waiting.accept(std::make_shared<Buffer::Cloud>(scan(1, 10.15)));
  const auto ready = [&](const Buffer::Cloud& cloud) {
    return transforms.canTransform("world", cloud.header.frame_id,
        rclcpp::Time(cloud.header.stamp, RCL_ROS_TIME));
  };
  view = waiting.view();
  const auto* next = view.newestReady(ready);
  check(next && next->sequence == 1 && !ready(*view.pending.back().cloud),
      "TF-delayed newest scan starved an older transformable scan");
  const auto pose = transforms.lookupTransform("world", next->cloud->header.frame_id,
      rclcpp::Time(next->cloud->header.stamp, RCL_ROS_TIME));
  check(std::abs(pose.transform.translation.x - 1.5) < 1e-7,
      "Live scan used latest TF rather than interpolating its actual timestamp");
  waiting.consumeThrough(next->sequence);
  tf.header.stamp.nanosec = 200000000; tf.transform.translation.x = 3;
  check(transforms.setTransform(tf, "live_scan_test"), "Could not advance delayed TF");
  view = waiting.view(); next = view.newestReady(ready);
  check(next && next->sequence == 2, "Waiting scan was lost before its matching TF arrived");
  waiting.consumeThrough(next->sequence);
  check(waiting.view().pending.empty(), "Displayed scans remained in the pending queue");

  // XYZ-only decoding must preserve all finite sensor coordinates, support
  // organised row padding and both byte orders, and reject malformed layouts.
  auto cloud = scan(3);
  size_t visited = 0;
  check(Buffer::forEachPoint(cloud, [&](const std::array<float, 3>& xyz) {
    check(std::abs(xyz[0] - (1.f + .25f*visited)) < 1e-6f,
        "Raw FLOAT32 XYZ coordinates changed");
    ++visited;
  }) == 3 && visited == 3, "Live display decimated a valid raw scan");
  using Field = sensor_msgs::msg::PointField;
  cloud.width = 2; cloud.height = 2; cloud.point_step = 24; cloud.row_step = 56;
  cloud.fields.resize(3); cloud.data.assign(cloud.height*cloud.row_step, 0);
  const uint16_t endian = 1;
  cloud.is_bigendian = *reinterpret_cast<const uint8_t*>(&endian) != 0;
  for (size_t axis = 0; axis < 3; ++axis) {
    cloud.fields[axis].offset = static_cast<uint32_t>(8*axis);
    cloud.fields[axis].datatype = Field::FLOAT64;
  }
  for (size_t i = 0; i < 4; ++i) for (size_t axis = 0; axis < 3; ++axis) {
    const double coordinate = i == 2 ? std::numeric_limits<double>::quiet_NaN()
        : i == 3 ? std::numeric_limits<double>::max() : double(1 + 3*i + axis);
    std::array<uint8_t, sizeof(double)> bytes;
    std::memcpy(bytes.data(), &coordinate, sizeof(coordinate));
    std::reverse(bytes.begin(), bytes.end());
    std::memcpy(cloud.data.data() + (i/2)*cloud.row_step + (i%2)*cloud.point_step + 8*axis,
        bytes.data(), bytes.size());
  }
  visited = 0;
  check(Buffer::forEachPoint(cloud, [&](const std::array<float, 3>& xyz) {
    check(xyz[0] == float(1 + 3*visited) && xyz[1] == xyz[0] + 1 && xyz[2] == xyz[0] + 2,
        "FLOAT64 byte order or organised row padding decoded incorrectly");
    ++visited;
  }) == 2 && visited == 2, "NaN/overflow points were not rejected safely");
  const auto rejected = [](const Buffer::Cloud& value) {
    try { Buffer::forEachPoint(value, [](const auto&) {}); }
    catch (const std::invalid_argument&) { return true; }
    return false;
  };
  auto malformed = cloud; malformed.fields[0].offset = malformed.point_step;
  check(rejected(malformed), "Out-of-bounds XYZ field accepted");
  malformed = cloud; malformed.data.pop_back();
  check(rejected(malformed), "Truncated raw cloud accepted");
  malformed = cloud; malformed.row_step = 1;
  check(rejected(malformed), "Invalid row stride accepted");
  malformed = cloud; malformed.fields.pop_back();
  check(rejected(malformed), "Missing Z field accepted");
}

void testKeyframeSelection(const okvis::ViParameters& parameters, const std::filesystem::path& directory) {
  const auto file = (directory / "keyframes_rgb.pcd").string();
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("global_lidar_pcd.enabled", true),
      rclcpp::Parameter("global_lidar_pcd.output_file", file),
      rclcpp::Parameter("global_lidar_pcd.publish_interval", .1)});
  auto node = std::make_shared<rclcpp::Node>("keyframe_pcd_test", "/pcd_selection_test", options);
  okvis::GlobalLidarPcdMapper mapper(node, parameters, directory.string());
  auto input = node->create_publisher<sensor_msgs::msg::PointCloud2>("/okvis/lidar", rclcpp::SensorDataQoS());
  sensor_msgs::msg::PointCloud2::ConstSharedPtr preview;
  auto subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "global_lidar_map", rclcpp::QoS(1).best_effort().durability_volatile(),
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr value) { preview = std::move(value); });
  wait(node, [&] { return input->get_subscription_count() > 0 && preview; });
  // Scan before its state: do not discard or merge it before the KF decision.
  const auto beforeStates = mapper.statistics().receivedScans;
  input->publish(scan(3, 1.0));
  wait(node, [&] { return mapper.statistics().receivedScans > beforeStates; });
  spinFor(node, std::chrono::milliseconds(200));
  check(preview->width == 0, "A scan was merged before any keyframe state arrived");
  update(mapper, state(1, 1.0));
  update(mapper, state(2, 1.2, 0, false), {}, 1);
  wait(node, [&] { return preview->width == 3; });
  const auto unchanged = [&](sensor_msgs::msg::PointCloud2 value, size_t expected) {
    const auto before = mapper.statistics();
    input->publish(value);
    wait(node, [&] { return mapper.statistics().completedScans > before.completedScans; });
    spinFor(node, std::chrono::milliseconds(300));
    check(mapper.statistics().decodedScans == before.decodedScans,
        "Duplicate/non-keyframe scan was fully decoded before being rejected");
    check(preview->width == expected, "Non-keyframe or duplicate-keyframe scan grew the map");
  };
  unchanged(scan(5, 1.05, 40), 3); // Same KF, newer point times: no second merge.
  const auto beforeOrdinary = mapper.statistics().receivedScans;
  input->publish(scan(7, 1.2, 20)); // Ordinary frame; its matching window is not complete yet.
  wait(node, [&] { return mapper.statistics().receivedScans > beforeOrdinary; });
  spinFor(node, std::chrono::milliseconds(200));
  check(preview->width == 3, "Pending ordinary-frame scan grew the map");
  // A new KF scan can also arrive before its state; wait instead of matching
  // it backwards to the last KF or ordinary frame.
  const auto beforeNewKeyframe = mapper.statistics().receivedScans;
  input->publish(scan(4, 1.4, 60));
  wait(node, [&] { return mapper.statistics().receivedScans > beforeNewKeyframe; });
  spinFor(node, std::chrono::milliseconds(200));
  check(preview->width == 3, "A future scan was assigned before a matching SLAM state arrived");
  update(mapper, state(3, 1.4));
  update(mapper, state(4, 1.6, 0, false), {}, 3);
  update(mapper, state(5, 2.0, 0, false), {}, 3);
  wait(node, [&] { return preview->width == 7; });
  unchanged(scan(6, 1.45, 100), 7);
  unchanged(scan(9, 1.6, 80), 7);
  // Optimisation callbacks for old KFs must only move existing points.
  update(mapper, state(5, 2.0, 3, false), state(1, 1.0, 3), 3);
  update(mapper, state(5, 2.0, 3, false), state(3, 1.4, 3), 3);
  wait(node, [&] {
    if (preview->width != 7) return false;
    for (size_t i = 0; i < preview->width; ++i) {
      float z;
      std::memcpy(&z, preview->data.data() + i*16 + 8, sizeof(z));
      if (z < 2.999f) return false;
    }
    return true;
  });
  mapper.shutdown();
  check(!mapper.failed(), "Keyframe-only mapper failed");
  check(mapper.statistics().decodedScans == 2, "Keyframe prefilter decoded unnecessary scans");
  std::ifstream stream(file, std::ios::binary);
  std::string line;
  size_t exported = 0;
  while (std::getline(stream, line)) {
    if (line.rfind("POINTS ", 0) == 0) exported = std::stoull(line.substr(7));
    if (line == "DATA binary") break;
  }
  check(exported == 7, "PCD contains ordinary-frame/duplicate scans or lost a keyframe scan");
  for (size_t i = 0; i < exported; ++i) {
    float xyz[3]; uint32_t rgb;
    stream.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
    stream.read(reinterpret_cast<char*>(&rgb), sizeof(rgb));
    check(bool(stream) && xyz[2] >= 2.999f && (xyz[0] < 3 || (xyz[0] > 61 && xyz[0] < 63)),
        "PCD keyframe selection or final anchor correction is incorrect");
  }
}

void testInputExecutorIsolation(const okvis::ViParameters& parameters,
    const std::filesystem::path& directory) {
  rclcpp::NodeOptions options;
  const auto file = (directory / "isolated-input.pcd").string();
  options.parameter_overrides({rclcpp::Parameter("global_lidar_pcd.enabled", true),
      rclcpp::Parameter("global_lidar_pcd.input_reliability", "best_effort"),
      rclcpp::Parameter("global_lidar_pcd.output_file", file),
      rclcpp::Parameter("global_lidar_pcd.publish_lidar_tf", false)});
  auto node = std::make_shared<rclcpp::Node>("isolated_pcd_input_test", options);
  okvis::GlobalLidarPcdMapper mapper(node, parameters, directory.string());
  auto input = node->create_publisher<sensor_msgs::msg::PointCloud2>("/okvis/lidar", rclcpp::SensorDataQoS());
  wait(node, [&] { return input->get_subscription_count() > 0; });
  update(mapper, state(1, 1.0));
  update(mapper, state(2, 1.2, 0, false), {}, 1);
  input->publish(scan(3, 1.0));
  // Deliberately do not spin the node/default callback group. The sidecar
  // must take and integrate the cloud while the SLAM executor is unavailable.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (mapper.statistics().completedScans == 0 && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  check(mapper.statistics().completedScans == 1 && mapper.statistics().decodedScans == 1,
      "PCD input depended on the SLAM/default executor being spun");
  mapper.shutdown();
  check(!mapper.failed() && std::filesystem::exists(file), "Isolated input did not retain/export the scan");
}

void testMapInputReliability(const okvis::ViParameters& parameters,
    const std::filesystem::path& directory) {
  const std::string topic = "/pcd_auto_qos_test/input";
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "/okvis/lidar:=" + topic});
  options.parameter_overrides({rclcpp::Parameter("global_lidar_pcd.enabled", true),
      rclcpp::Parameter("global_lidar_pcd.input_reliability", "auto"),
      rclcpp::Parameter("global_lidar_pcd.save_on_shutdown", false),
      rclcpp::Parameter("global_lidar_pcd.publish_interval", .1)});
  auto node = std::make_shared<rclcpp::Node>("map_input_qos_test", "/pcd_auto_qos_test", options);
  okvis::GlobalLidarPcdMapper mapper(node, parameters, directory.string());
  const auto usingReliability = [&](rmw_qos_reliability_policy_t policy) {
    for (const auto& endpoint : node->get_subscriptions_info_by_topic(topic)) {
      const auto& qos = endpoint.qos_profile().get_rmw_qos_profile();
      if (endpoint.node_name() == node->get_name() && qos.reliability == policy
          && qos.depth == 1 && qos.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE) return true;
    }
    return false;
  };
  // The map starts BEFORE the bag/driver. Discovery must use the remapped
  // topic and switch its own reader without replacing a pending KF scan.
  wait(node, [&] { return usingReliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT); });
  auto producer = std::make_shared<rclcpp::Node>("pcd_qos_producer");
  auto reliable = producer->create_publisher<sensor_msgs::msg::PointCloud2>(topic,
      rclcpp::QoS(1).reliable().durability_volatile());
  wait(node, [&] { return usingReliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      && reliable->get_subscription_count() > 0; });
  sensor_msgs::msg::PointCloud2::ConstSharedPtr preview;
  auto output = node->create_subscription<sensor_msgs::msg::PointCloud2>("global_lidar_map",
      rclcpp::QoS(1).reliable().durability_volatile(),
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr value) { preview = std::move(value); });
  wait(node, [&] { return bool(preview); });
  reliable->publish(scan(3, 2.0));
  update(mapper, state(1, 2.0));
  update(mapper, state(2, 2.2, 0, false), {}, 1);
  wait(node, [&] { return preview->width == 3; });

  // A later Best Effort driver must remain compatible. QoS re-selection must
  // retain existing map geometry and the one-scan-per-keyframe decision.
  reliable.reset();
  auto sensor = producer->create_publisher<sensor_msgs::msg::PointCloud2>(topic,
      rclcpp::SensorDataQoS());
  wait(node, [&] { return usingReliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
      && sensor->get_subscription_count() > 0; });
  sensor->publish(scan(4, 3.0, 20));
  update(mapper, state(3, 3.0));
  update(mapper, state(4, 3.2, 0, false), {}, 3);
  wait(node, [&] { return preview->width == 7; });
  mapper.shutdown();
  check(!mapper.failed(), "Input QoS re-selection stopped keyframe mapping");
}

void testMisalignedKeyframes(const okvis::ViParameters& parameters, const std::filesystem::path& directory) {
  const auto file = (directory / "misaligned_rgb.pcd").string();
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("global_lidar_pcd.enabled", true),
      rclcpp::Parameter("global_lidar_pcd.output_file", file),
      rclcpp::Parameter("global_lidar_pcd.publish_interval", .1)});
  auto node = std::make_shared<rclcpp::Node>("misaligned_pcd_test", "/pcd_misaligned_test", options);
  okvis::GlobalLidarPcdMapper mapper(node, parameters, directory.string());
  auto input = node->create_publisher<sensor_msgs::msg::PointCloud2>("/okvis/lidar", rclcpp::SensorDataQoS());
  sensor_msgs::msg::PointCloud2::ConstSharedPtr preview;
  // RViz's default Reliable reader and existing Best Effort readers must both work.
  auto subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "global_lidar_map", rclcpp::QoS(1).reliable().durability_volatile(),
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr value) { preview = std::move(value); });
  wait(node, [&] { return input->get_subscription_count() > 0 && preview; });
  update(mapper, state(1, 1.0));
  update(mapper, state(2, 1.2, 0, false), {}, 1);
  // All three raw scans arrive before their keyframe decisions. Their MIDPOINTS
  // match ordinary frames exactly, but each scan covers a true keyframe.
  // The old nearest-frame gate rejected all of them; the single waiting slot
  // also overwrote the second scan while the first waited for state coverage.
  for (size_t s = 0; s < 3; ++s) {
    const size_t count = 3 + s;
    const double first = 1.4 + .2*s;
    auto value = scan(count, first, float(10 + 30*s));
    for (size_t i = 0; i < count; ++i) {
      const double t = first + .05*i/(count - 1);
      std::memcpy(value.data.data() + i*20 + 12, &t, sizeof(t));
    }
    const auto before = mapper.statistics().receivedScans;
    input->publish(value);
    wait(node, [&] { return mapper.statistics().receivedScans > before; });
    spinFor(node, std::chrono::milliseconds(200));
    check(preview->width == 0, "Misaligned scan merged before its delayed keyframe decision");
  }
  for (uint64_t s = 0; s < 3; ++s) {
    const uint64_t id = 3 + 2*s;
    update(mapper, state(id, 1.41 + .2*s));
    update(mapper, state(id + 1, 1.425 + .2*s, 0, false), {}, id);
  }
  update(mapper, state(9, 2.0, 0, false), {}, 7);
  wait(node, [&] { return preview->width == 12; });
  check(!mapper.failed(), "Delayed/misaligned keyframes stopped mapping");
  // A display configuration failure must not kill keyframe ingestion/export.
  check(node->set_parameter(rclcpp::Parameter("global_lidar_pcd.publish_interval", 0.0)).successful,
      "Could not inject display-only configuration failure");
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  update(mapper, state(9, 2.0, 2, false), state(7, 1.81, 2), 7);
  check(!mapper.failed(), "Display failure stopped the map worker");
  check(node->set_parameter(rclcpp::Parameter("global_lidar_pcd.publish_interval", .1)).successful,
      "Could not restore display refresh interval");
  wait(node, [&] {
    if (preview->width != 12) return false;
    for (size_t i = 0; i < preview->width; ++i) {
      float x, z;
      std::memcpy(&x, preview->data.data() + i*16, sizeof(x));
      std::memcpy(&z, preview->data.data() + i*16 + 8, sizeof(z));
      if (x > 70 && z < 1.999f) return false;
    }
    return true;
  });
  mapper.shutdown();
  check(!mapper.failed(), "Misaligned keyframe map failed during shutdown");
  std::ifstream stream(file, std::ios::binary);
  std::string line;
  size_t exported = 0;
  while (std::getline(stream, line)) {
    if (line.rfind("POINTS ", 0) == 0) exported = std::stoull(line.substr(7));
    if (line == "DATA binary") break;
  }
  check(exported == 12, "Delayed scans/midpoint normal frames caused missing keyframe PCD sections");
}

void testLiveTransformHistory(const okvis::ViParameters& parameters,
    const std::filesystem::path& directory) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("global_lidar_pcd.enabled", true),
      rclcpp::Parameter("global_lidar_pcd.save_on_shutdown", false)});
  auto node = std::make_shared<rclcpp::Node>("live_tf_history_test", "/pcd_tf_history_test", options);
  okvis::GlobalLidarPcdMapper mapper(node, parameters, (directory / "live_tf").string());
  tf2_ros::Buffer buffer(node->get_clock());
  std::set<uint64_t> stamps;
  auto subscription = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(10),
      [&](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
        for (const auto& transform : message->transforms) {
          if (transform.child_frame_id != "test_pcd_live_history") continue;
          check(buffer.setTransform(transform, "pcd_live_test"), "Invalid raw-scan TF");
          stamps.insert(uint64_t(transform.header.stamp.sec)*1000000000ull
              + transform.header.stamp.nanosec);
        }
      });
  auto input = node->create_publisher<sensor_msgs::msg::PointCloud2>("/okvis/lidar", rclcpp::SensorDataQoS());
  wait(node, [&] {
    return input->get_subscription_count() > 0 && subscription->get_publisher_count() > 0;
  });
  okvis::TrackingState tracking{};
  tracking.trackingQuality = okvis::TrackingQuality::Good;
  // Pose history arrives before the raw sensor's frame name. A latest-only
  // slot loses the first two poses and cannot transform a scan at 10.05 s.
  // No optimised/keyframe/map update is supplied: live TF is independent.
  for (uint64_t i = 0; i < 3; ++i) {
    auto value = state(10 + i, 10.0 + .1*i);
    value.timestamp.fromNSec(10000000000ull + i*100000000ull);
    mapper.frontendStateCallback(value, tracking);
  }
  auto cloud = scan(1, 10.05);
  cloud.header.frame_id = "test_pcd_live_history";
  input->publish(cloud);
  wait(node, [&] { return stamps.size() == 3; });
  const auto transform = buffer.lookupTransform("world", cloud.header.frame_id,
      rclcpp::Time(cloud.header.stamp, RCL_ROS_TIME));
  check(std::abs(transform.transform.translation.x - 9.30) < 1e-8,
      "Raw scan timestamp has no correctly calibrated/interpolated live TF");
  mapper.shutdown();
  check(!mapper.failed(), "Live TF history test stopped the independent map worker");
}

void testDisplayInbox() {
  using Inbox = okvis::detail::GlobalLidarMapRvizInbox;
  using Marker = Inbox::Marker;
  Inbox inbox;
  std::map<Inbox::Key, std::string> applied;
  const auto message = [](int id, const std::string& revision = "1", int action = Marker::ADD) {
    auto result = std::make_shared<Inbox::Array>();
    Marker marker;
    marker.ns = "test"; marker.id = id; marker.text = revision;
    marker.header.frame_id = "session/kf_" + std::to_string(id);
    marker.action = action; marker.type = Marker::POINTS;
    result->markers.push_back(std::move(marker));
    return result;
  };
  const auto consume = [&] {
    auto next = inbox.take();
    if (!next) return false;
    const auto& marker = next->message->markers[next->index];
    if (marker.action == Marker::DELETEALL) { applied.clear(); inbox.cleared(); }
    else {
      applied[Inbox::key(marker)] = marker.text;
      inbox.acknowledge(marker);
    }
    return true;
  };
  // Deliberately render four times slower than transmission. Overflow must
  // remain bounded and historical replay must eventually deliver ALL blocks.
  constexpr size_t total = 203;
  for (size_t round = 0; round < 20; ++round) {
    for (size_t i = 0; i < total; ++i) {
      inbox.accept(message(int(i)));
      check(inbox.pendingCount() <= Inbox::capacity, "RViz inbox exceeded its payload budget");
      if (i % 4 == 0) consume();
    }
    while (consume()) {}
    if (!inbox.replayWhenDrained()) break;
  }
  check(applied.size() == total, "Slow RViz overflow recovery permanently lost historical blocks");
  check(!inbox.take(), "Static replay queued already applied blocks");
  inbox.accept(message(5, "2")); inbox.accept(message(5, "3"));
  check(inbox.pendingCount() == 1, "Successive revisions duplicated queued geometry");
  consume();
  check(applied.at({"test", 5}) == "3", "Queued block did not retain its newest revision");
  inbox.accept(message(5, "4", Marker::DELETE)); consume();
  inbox.accept(message(5, "4", Marker::DELETE));
  check(!inbox.take(), "Sparse deleted blocks kept queuing during replay");
  inbox.accept(message(0, "0", Marker::DELETEALL));
  consume();
  check(applied.empty(), "New publisher session did not clear previous geometry");
  auto payload = message(0);
  std::weak_ptr<Inbox::Array> weak = payload;
  inbox.accept(payload); payload.reset(); consume();
  check(weak.expired(), "RViz retained historical marker payloads after geometry upload");
}

void testIncrementalDisplay() {
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Key = std::pair<std::string, int32_t>;
  auto node = std::make_shared<rclcpp::Node>("incremental_display_test", "/pcd_incremental_test");
  okvis::GlobalLidarMapDisplay display(node, okvis::LidarHeightColour(-2, 5), .1);
  std::map<Key, Marker> retained;
  std::map<std::string, geometry_msgs::msg::TransformStamped> frames;
  size_t clears = 0;
  auto receive = [&](const MarkerArray& message, std::map<Key, Marker>& store) {
    for (const auto& marker : message.markers) {
      const Key key{marker.ns, marker.id};
      if (marker.action == Marker::DELETEALL) { store.clear(); ++clears; }
      else if (marker.action == Marker::DELETE) store.erase(key);
      else {
        check(marker.type == Marker::POINTS && marker.frame_locked
            && marker.points.size() == marker.colors.size() && !marker.text.empty(),
            "Invalid incremental marker format/content revision");
        store[key] = marker;
      }
    }
  };
  auto subscription = node->create_subscription<MarkerArray>("global_lidar_map_markers",
      rclcpp::QoS(4).reliable().durability_volatile(),
      [&](MarkerArray::ConstSharedPtr message) { receive(*message, retained); });
  auto transforms = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",
      rclcpp::QoS(1).reliable().transient_local(), [&](tf2_msgs::msg::TFMessage::ConstSharedPtr value) {
        for (const auto& transform : value->transforms) frames[transform.child_frame_id] = transform;
      });
  wait(node, [&] { return display.subscribed(); });
  spinFor(node, std::chrono::milliseconds(100)); // Drain the matched-reader event before first replay.
  okvis::LidarPointCloudMap::Snapshot snapshot;
  for (uint64_t k = 0; k < 3; ++k) {
    auto points = std::make_shared<okvis::LidarPointCloudMap::Points>();
    for (size_t i = 0; i < 2 + k; ++i) points->emplace_back(10*k+i, .1, .1*i);
    snapshot.push_back({Eigen::Isometry3d::Identity(), points, {}, 17+k, 0});
  }
  const auto size = [](const auto& store) {
    size_t count = 0;
    for (const auto& entry : store) count += entry.second.points.size();
    return count;
  };
  auto first = display.publish(snapshot, 1, 1);
  check(first.blocks == 2 && first.points == 7, "Initial history batch did not bound upload work");
  auto second = display.publish(snapshot, 1, 1);
  check(second.blocks == 1 && second.points == 2, "Incremental replay lost an older keyframe");
  check(first.indexed == 3 && second.indexed == 0, "Replay batch unnecessarily re-indexed history");
  wait(node, [&] { return size(retained) == 9 && frames.size() >= 3; });
  const auto idle = display.publish(snapshot, 1, 1);
  check(idle.blocks == 0 && idle.indexed == 0 && idle.poses == 0,
      "An unchanged map traversed history or resent dense geometry");
  // A horizontal loop correction updates TF anchors only, with no point upload.
  for (auto& block : snapshot) block.pose.translation().x() += 3;
  auto moved = display.publish(snapshot, 1);
  check(moved.poses == 3 && moved.blocks == 0, "Pose-only correction reuploaded all historical points");
  wait(node, [&] {
    for (const auto& entry : retained) {
      const auto found = frames.find(entry.second.header.frame_id);
      if (found == frames.end() || found->second.transform.translation.x != 3) return false;
    }
    return true;
  });
  snapshot[0].pose.translation().z() = 2;
  check(display.publish(snapshot, 1).blocks == 1, "Height correction did not refresh affected colours");
  // Sampling phase is continuous across blocks (2 + 3 + 4 points): ceil(9/3).
  display.publish(snapshot, 3); display.publish(snapshot, 3);
  wait(node, [&] { return size(retained) == 3; });
  check(display.publish(snapshot, 3).blocks == 0, "Static sampled markers kept uploading");
  const auto beforeReconnect = clears;
  std::map<Key, std::string> versions;
  for (const auto& entry : retained) versions[entry.first] = entry.second.text;
  std::map<Key, Marker> late;
  auto lateSubscription = node->create_subscription<MarkerArray>("global_lidar_map_markers",
      rclcpp::QoS(4).reliable().durability_volatile(),
      [&](MarkerArray::ConstSharedPtr message) { receive(*message, late); });
  wait(node, [&] {
    display.publish(snapshot, 3);
    return size(late) == 3;
  });
  check(clears == beforeReconnect, "A new reader cleared the existing reader's global map");
  for (const auto& entry : retained)
    check(versions.at(entry.first) == entry.second.text, "History replay changed unchanged content revisions");
  display.publish(snapshot, 20); display.publish(snapshot, 20);
  wait(node, [&] { return size(retained) == 1 && size(late) == 1; });
  check(display.publish(snapshot, 20).blocks == 0, "Empty sampled blocks were not deleted stably");
  lateSubscription.reset();
  late.clear();
  lateSubscription = node->create_subscription<MarkerArray>("global_lidar_map_markers",
      rclcpp::QoS(4).reliable().durability_volatile(),
      [&](MarkerArray::ConstSharedPtr message) { receive(*message, late); });
  wait(node, [&] { display.publish(snapshot, 20); return size(late) == 1; });
  check(clears == beforeReconnect, "Rapid reconnection cleared another reader's persistent map");
}

void run() {
  namespace fs = std::filesystem;
  const auto directory = fs::temp_directory_path() / ("okvis-pcd-worker-test-" + std::to_string(::getpid()));
  fs::create_directories(directory);
  struct Cleanup { fs::path path; ~Cleanup() { std::error_code e; fs::remove_all(path, e); } } cleanup{directory};
  okvis::ViParameters parameters{};
  parameters.lidar.emplace();
  parameters.lidar->T_SL.set(Eigen::Vector3d(.25, 0, 0), Eigen::Quaterniond::Identity());
  {
    auto disabledNode = std::make_shared<rclcpp::Node>("disabled_pcd_test");
    okvis::GlobalLidarPcdMapper disabled(disabledNode, parameters, (directory / "disabled").string());
    check(!disabled.enabled() && !disabled.failed() && !fs::exists(directory / "disabled"),
        "Disabled mapper created a worker/archive or failed");
  }
  const auto file = (directory / "global_rgb.pcd").string();
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("global_lidar_pcd.enabled", true),
      rclcpp::Parameter("global_lidar_pcd.output_file", file),
      rclcpp::Parameter("global_lidar_pcd.preview_sampling_step", int64_t(1)),
      rclcpp::Parameter("global_lidar_pcd.publish_interval", .1)});
  auto node = std::make_shared<rclcpp::Node>("global_pcd_test", options);
  okvis::GlobalLidarPcdMapper mapper(node, parameters, directory.string());
  check(mapper.enabled() && !mapper.failed(), "Global PCD mapper could not start");
  update(mapper, state(1, 1));
  update(mapper, state(2, 1.25));
  update(mapper, state(3, 2, 0, false), {}, 2);
  auto input = node->create_publisher<sensor_msgs::msg::PointCloud2>("/okvis/lidar", rclcpp::SensorDataQoS());
  wait(node, [&] { return input->get_subscription_count() > 0; });
  // Oversized/malformed input only affects the sidecar; subsequent scans work.
  auto oversized = scan(1); oversized.width = 300000;
  input->publish(oversized);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  rclcpp::spin_some(node);
  check(!mapper.failed(), "Oversized scan stopped the sidecar");
  constexpr size_t count = 5003;
  input->publish(scan(count));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  sensor_msgs::msg::PointCloud2::ConstSharedPtr preview;
  size_t previewCount = 0;
  auto subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "global_lidar_map", rclcpp::QoS(1).best_effort().durability_volatile(),
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr value) { preview = std::move(value); ++previewCount; });
  wait(node, [&] { return preview && preview->width > 0; });
  check(preview->width == count && preview->point_step == 16 && preview->fields[3].name == "rgb"
      && preview->data.size() == 16 * preview->width && preview->header.frame_id == "world",
      "Sampling step 1 did not display every retained point with packed RGB");
  const auto qos = subscription->get_actual_qos();
  check(qos.depth() == 1, "Preview subscription depth changed");
  spinFor(node, std::chrono::milliseconds(200));
  const auto staticPublications = previewCount;
  spinFor(node, std::chrono::milliseconds(400));
  check(previewCount == staticPublications, "Static full map was periodically reserialized without a new reader");
  std::optional<geometry_msgs::msg::TransformStamped> liveTransform;
  auto liveSubscription = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(10),
      [&](tf2_msgs::msg::TFMessage::ConstSharedPtr value) {
        for (const auto& transform : value->transforms)
          if (transform.child_frame_id == "test_pcd_lidar" && transform.header.stamp.sec == 3)
            liveTransform = transform;
      });
  okvis::TrackingState liveTracking{};
  liveTracking.trackingQuality = okvis::TrackingQuality::Good;
  wait(node, [&] { return liveSubscription->get_publisher_count() > 0; });
  // Frontend TF bypasses the delayed backend timestamp and includes T_SL's translation.
  auto live = state(4, 3.0);
  mapper.frontendStateCallback(live, liveTracking);
  wait(node, [&] { return liveTransform.has_value(); });
  check(liveTransform->header.frame_id == "world"
      && std::abs(liveTransform->transform.translation.x - 2.25) < 1e-9,
      "Raw LiDAR live TF lost the frontend pose or calibrated LiDAR lever arm");
  update(mapper, state(3, 2, 3, false), state(2, 1.25, 3), 2);
  wait(node, [&] {
    float z;
    std::memcpy(&z, preview->data.data() + 8, sizeof(z));
    return z >= 2.99f;
  });
  // Change the density while running; an invalid factor must be rejected.
  const auto invalid = node->set_parameter(rclcpp::Parameter(
      "global_lidar_pcd.preview_sampling_step", int64_t(0)));
  check(!invalid.successful, "Zero sampling step was accepted");
  check(node->set_parameter(rclcpp::Parameter(
      "global_lidar_pcd.preview_sampling_step", int64_t(3))).successful,
      "Changing the display sampling step failed");
  const size_t sampledCount = (count + 2) / 3;
  wait(node, [&] { return preview && preview->width == sampledCount; });
  // A second volatile reader must also receive a STATIC map while the first
  // reader stays connected; it cannot rely on a new scan or retained DDS data.
  sensor_msgs::msg::PointCloud2::ConstSharedPtr late;
  auto lateSubscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "global_lidar_map", rclcpp::QoS(1).best_effort().durability_volatile(),
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr value) { late = std::move(value); });
  wait(node, [&] { return late && late->width == sampledCount; });
  mapper.shutdown(); mapper.shutdown();
  check(!mapper.failed() && fs::exists(file), "PCD worker failed or did not export during shutdown");
  std::ifstream stream(file, std::ios::binary);
  std::string line;
  size_t exported = 0;
  bool coloured = false;
  while (std::getline(stream, line)) {
    coloured |= line == "FIELDS x y z rgb";
    if (line.rfind("POINTS ", 0) == 0) exported = std::stoull(line.substr(7));
    if (line == "DATA binary") break;
  }
  check(coloured && exported == count, "Display sampling changed complete PCD export or repeated point times");
  const okvis::LidarHeightColour colour(-2, 5);
  for (size_t i = 0; i < exported; ++i) {
    float xyz[3]; uint32_t rgb;
    stream.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
    stream.read(reinterpret_cast<char*>(&rgb), sizeof(rgb));
    if (!stream || xyz[0] < 1.449f || xyz[2] < 2.999f || xyz[2] > 4.001f) {
      std::cerr << "Invalid exported point " << i << ": " << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << '\n';
      throw std::runtime_error("PCD lost LiDAR extrinsics or final pose correction");
    }
    const double elapsed = xyz[1] > 6.249f ? .3 : .2;
    const double column = (xyz[0] - 1.25 - elapsed) / .25;
    check(std::abs(column - std::round(column)) < 1e-4,
        "Individual point timestamps were not deskewed using the IMU trajectory");
    check(rgb == colour(xyz[2]), "Preview and PCD height palette are inconsistent");
  }
  {
    rclcpp::NodeOptions limitedOptions;
    limitedOptions.parameter_overrides({rclcpp::Parameter("global_lidar_pcd.enabled", true),
        rclcpp::Parameter("global_lidar_pcd.max_pending_update_bytes", int64_t(1024))});
    auto limitedNode = std::make_shared<rclcpp::Node>("global_pcd_overload_test", limitedOptions);
    okvis::GlobalLidarPcdMapper limited(limitedNode, parameters, (directory / "overload").string());
    update(limited, state(2, 2)); // Oversized IMU update: stop sidecar, do not throw into SLAM.
    wait(limitedNode, [&] { return limited.failed(); });
    limited.shutdown();
  }
  testScanTimingPrefilter();
  testInputExecutorIsolation(parameters, directory);
  testKeyframeSelection(parameters, directory);
  testMapInputReliability(parameters, directory);
  testMisalignedKeyframes(parameters, directory);
  testLiveTransformHistory(parameters, directory);
  testLiveScanBuffer();
  testDisplayInbox();
  testIncrementalDisplay();
}
} // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try { run(); std::cout << "Independent PCD worker, keyframe-only merging, RGB, deskew, final correction and fixed display sampling passed\n"; }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; result = 1; }
  rclcpp::shutdown();
  return result;
}
