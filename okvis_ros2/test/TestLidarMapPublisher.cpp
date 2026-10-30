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
    publisher.publishLidarPointCloudMap(okvis::Time(12, 345), {{0.1, 0.2, 0.3}, {1.0, 2.0, 3.0}});
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
    if (received->header.frame_id != "world" || received->width * received->height != 2
        || received->header.stamp.sec != 12 || received->header.stamp.nanosec != 345) {
      throw std::runtime_error("Incorrect map frame, timestamp or point count");
    }
    sensor_msgs::PointCloud2ConstIterator<float> x(*received, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*received, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*received, "z");
    if (std::abs(*x - 0.1f) > 1e-6f || std::abs(*y - 0.2f) > 1e-6f || std::abs(*z - 0.3f) > 1e-6f) {
      throw std::runtime_error("Incorrect PointCloud2 coordinates");
    }
    general->shutdown();
    std::cout << "ROS2 XYZ map and late-subscriber checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
