// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/ros2/Publisher.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>("lidar_map_publisher_test", "/okvis_pointcloud_test");
    auto odometry = std::make_shared<okvis::ThreadedPublisher>(node);
    auto images = std::make_shared<okvis::ThreadedPublisher>(node);
    auto general = std::make_shared<okvis::ThreadedPublisher>(node);
    okvis::Publisher publisher(node, odometry, images, general);
    publisher.setMeshPublishingEnabled(false);
    general->startThread();
    constexpr size_t pointCount = 200001;
    okvis::LidarPointCloudMap::Points points;
    points.reserve(pointCount);
    for (size_t i = 0; i < pointCount; ++i)
      points.emplace_back(0.1 + 0.25 * double(i), -0.2 - 0.125 * double(i), 0.3);
    publisher.publishLidarPointCloudMap(okvis::Time(12, 345), points);
    // Subscribe after the only publication to verify transient-local durability.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    sensor_msgs::msg::PointCloud2::ConstSharedPtr received;
    auto subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>("okvis_lidar_map",
        rclcpp::QoS(1).transient_local().reliable(),
        [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) { received = cloud; });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!received && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!received) throw std::runtime_error("Late subscriber did not receive accumulated map");
    if (received->header.frame_id != "world" || received->width != pointCount || received->height != 1
        || received->header.stamp.sec != 12 || received->header.stamp.nanosec != 345) {
      throw std::runtime_error("Incorrect map frame, timestamp or point count");
    }
    if (received->point_step != 12 || received->row_step != 12 * pointCount ||
        received->data.size() != 12 * pointCount || received->fields.size() != 3 || !received->is_dense) {
      throw std::runtime_error("XYZ map payload is not packed at 12 bytes per point");
    }
    sensor_msgs::PointCloud2ConstIterator<float> x(*received, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*received, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*received, "z");
    if (std::abs(*x - float(points.front().x())) > 1e-6f ||
        std::abs(*y - float(points.front().y())) > 1e-6f ||
        std::abs(*z - float(points.front().z())) > 1e-6f) {
      throw std::runtime_error("Incorrect PointCloud2 coordinates");
    }
    x += static_cast<int>(pointCount - 1);
    y += static_cast<int>(pointCount - 1);
    z += static_cast<int>(pointCount - 1);
    if (std::abs(*x - float(points.back().x())) > 1e-6f ||
        std::abs(*y - float(points.back().y())) > 1e-6f ||
        std::abs(*z - float(points.back().z())) > 1e-6f) {
      throw std::runtime_error("Global map tail was truncated, sampled or transformed incorrectly");
    }
    general->shutdown();
    std::cout << "ROS2 packed XYZ global map above 200000 points and late-subscriber checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
