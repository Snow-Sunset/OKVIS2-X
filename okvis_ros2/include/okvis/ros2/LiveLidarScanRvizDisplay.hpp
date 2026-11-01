// SPDX-License-Identifier: BSD-3-Clause
#ifndef OKVIS_ROS2_LIVE_LIDAR_SCAN_RVIZ_DISPLAY_HPP
#define OKVIS_ROS2_LIVE_LIDAR_SCAN_RVIZ_DISPLAY_HPP

#include <rviz_common/display.hpp>
#include <memory>

namespace okvis {
/// Live raw cloud, with an explicit timestamp-correct TF wait and automatic
/// DDS reliability selection. This plugin runs inside RViz, not inside SLAM.
class LiveLidarScanRvizDisplay : public rviz_common::Display {
 public:
  LiveLidarScanRvizDisplay();
  ~LiveLidarScanRvizDisplay() override;
  void update(float wallDt, float rosDt) override;
  void reset() override;
  void setTopic(const QString& topic, const QString& datatype) override;

 protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void fixedFrameChanged() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  void subscribe(bool clear);
  void clearCloud();
};
} // namespace okvis
#endif
