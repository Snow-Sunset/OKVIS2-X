// SPDX-License-Identifier: BSD-3-Clause
#ifndef OKVIS_ROS2_GLOBAL_LIDAR_MAP_RVIZ_DISPLAY_HPP
#define OKVIS_ROS2_GLOBAL_LIDAR_MAP_RVIZ_DISPLAY_HPP

#include <rviz_common/display.hpp>
#include <memory>

namespace okvis {

/// Persistent keyframe geometry: upload only changed blocks, move only scene
/// nodes when TF changes. All Ogre operations run on RViz's render thread.
class GlobalLidarMapRvizDisplay : public rviz_common::Display {
 public:
  GlobalLidarMapRvizDisplay();
  ~GlobalLidarMapRvizDisplay() override;
  void update(float wallDt, float rosDt) override;
  void reset() override;
  void setTopic(const QString& topic, const QString& datatype) override;

 protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  void subscribe(bool clear);
  void clearGeometry();
};
} // namespace okvis
#endif
