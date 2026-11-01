// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/ros2/GlobalLidarMapRvizDisplay.hpp>
#include <okvis/ros2/GlobalLidarMapRvizInbox.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <map>

namespace okvis {
namespace {
using Array = visualization_msgs::msg::MarkerArray;
using Marker = visualization_msgs::msg::Marker;
using Inbox = detail::GlobalLidarMapRvizInbox;
using Key = Inbox::Key;
} // namespace

struct GlobalLidarMapRvizDisplay::Impl {
  struct Anchor { Ogre::SceneNode* node = nullptr; size_t blocks = 0; bool visible = false; };
  struct Block {
    std::unique_ptr<rviz_rendering::PointCloud> cloud;
    std::string frame;
    size_t points = 0;
  };
  rviz_common::properties::RosTopicProperty* topic = nullptr;
  rviz_common::properties::FloatProperty* size = nullptr;
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<Array>::SharedPtr subscription;
  std::shared_ptr<Inbox> inbox;
  std::string subscribedTopic;
  std::map<std::string, Anchor> anchors;
  std::map<Key, Block> blocks;
  size_t points = 0;
  float pointSize = 2;
  std::chrono::steady_clock::time_point nextAnchorRefresh{};

  void erase(const Key& id, Ogre::SceneManager* manager) {
    auto block = blocks.find(id);
    if (block == blocks.end()) return;
    auto anchor = anchors.find(block->second.frame);
    anchor->second.node->detachObject(block->second.cloud.get());
    points -= block->second.points;
    blocks.erase(block);
    if (--anchor->second.blocks == 0) {
      manager->destroySceneNode(anchor->second.node);
      anchors.erase(anchor);
    }
  }
};

GlobalLidarMapRvizDisplay::GlobalLidarMapRvizDisplay() : impl_(std::make_unique<Impl>()) {
  impl_->topic = new rviz_common::properties::RosTopicProperty("Topic",
      "/okvis/global_lidar_map_markers", "visualization_msgs/msg/MarkerArray",
      "Incremental keyframe blocks, with complete historical replay on connection.", this);
  impl_->size = new rviz_common::properties::FloatProperty("Size (Pixels)", 2.0,
      "Point size in pixels. Density is controlled by global_lidar_pcd.preview_sampling_step.", this);
  impl_->size->setMin(1); impl_->size->setMax(20);
}

GlobalLidarMapRvizDisplay::~GlobalLidarMapRvizDisplay() {
  impl_->subscription.reset();
  impl_->inbox.reset();
  clearGeometry();
}

void GlobalLidarMapRvizDisplay::onInitialize() {
  impl_->topic->initialize(context_->getRosNodeAbstraction());
  if (auto abstraction = context_->getRosNodeAbstraction().lock())
    impl_->node = abstraction->get_raw_node();
}

void GlobalLidarMapRvizDisplay::clearGeometry() {
  // Destroy geometry before its scene node. No message containing the entire
  // historical map is retained in addition to RViz's point-cloud buffers.
  for (auto& block : impl_->blocks) {
    const auto anchor = impl_->anchors.find(block.second.frame);
    anchor->second.node->detachObject(block.second.cloud.get());
  }
  impl_->blocks.clear();
  for (auto& anchor : impl_->anchors) scene_manager_->destroySceneNode(anchor.second.node);
  impl_->anchors.clear();
  impl_->points = 0;
  impl_->nextAnchorRefresh = {};
}

void GlobalLidarMapRvizDisplay::subscribe(bool clear) {
  impl_->subscription.reset();
  if (clear) { clearGeometry(); impl_->inbox = std::make_shared<Inbox>(); }
  if (!impl_->inbox) impl_->inbox = std::make_shared<Inbox>();
  impl_->subscribedTopic = impl_->topic->getTopicStd();
  if (!isEnabled() || !impl_->node || impl_->subscribedTopic.empty()) return;
  const auto inbox = impl_->inbox;
  try {
    impl_->subscription = impl_->node->create_subscription<Array>(impl_->subscribedTopic,
        rclcpp::QoS(4).reliable().durability_volatile(), [inbox](Array::ConstSharedPtr message) {
      inbox->accept(std::move(message));
    });
    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "Subscribed; waiting for keyframe blocks");
  } catch (const std::exception& error) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", error.what());
  }
}

void GlobalLidarMapRvizDisplay::onEnable() { subscribe(true); }
void GlobalLidarMapRvizDisplay::onDisable() {
  impl_->subscription.reset(); impl_->inbox.reset(); clearGeometry();
}
void GlobalLidarMapRvizDisplay::reset() {
  rviz_common::Display::reset();
  subscribe(true);
}
void GlobalLidarMapRvizDisplay::setTopic(const QString& topic, const QString&) {
  impl_->topic->setValue(topic);
}

void GlobalLidarMapRvizDisplay::update(float, float) {
  if (!isEnabled()) return;
  if (impl_->topic->getTopicStd() != impl_->subscribedTopic) subscribe(true);
  if (!impl_->inbox) return;
  bool changed = false;
  const float size = impl_->size->getFloat();
  if (size != impl_->pointSize) {
    impl_->pointSize = size;
    for (auto& block : impl_->blocks) block.second.cloud->setDimensions(size, size, size);
    changed = true;
  }
  // Always make progress, but defer the second block if the first consumed
  // this tick's soft 4 ms upload budget. This never removes historical points.
  const auto uploadStarted = std::chrono::steady_clock::now();
  for (size_t work = 0; work < 2; ++work) {
    if (work && std::chrono::steady_clock::now() - uploadStarted >= std::chrono::milliseconds(4)) break;
    auto next = impl_->inbox->take();
    if (!next) break;
    const auto& marker = next->message->markers[next->index];
    if (marker.action == Marker::DELETEALL) {
      clearGeometry();
      impl_->inbox->cleared();
      changed = true; continue;
    }
    if (marker.action != Marker::ADD && marker.action != Marker::DELETE) continue;
    const Key id = Inbox::key(marker);
    try {
      if (marker.action == Marker::ADD && marker.type != Marker::POINTS) continue;
      impl_->erase(id, scene_manager_);
      if (marker.action == Marker::ADD && !marker.points.empty()) {
        std::vector<rviz_rendering::PointCloud::Point> points;
        points.reserve(marker.points.size());
        for (size_t p = 0; p < marker.points.size(); ++p) {
          const auto& xyz = marker.points[p];
          if (!std::isfinite(xyz.x) || !std::isfinite(xyz.y) || !std::isfinite(xyz.z)) continue;
          rviz_rendering::PointCloud::Point point;
          point.position = Ogre::Vector3(xyz.x, xyz.y, xyz.z);
          const auto& rgb = marker.colors.size() == marker.points.size() ? marker.colors[p] : marker.color;
          point.setColor(rgb.r, rgb.g, rgb.b, 1);
          points.push_back(point);
        }
        auto cloud = std::make_unique<rviz_rendering::PointCloud>();
        cloud->setRenderMode(rviz_rendering::PointCloud::RM_POINTS);
        cloud->setDimensions(size, size, size); cloud->setAlpha(1);
        cloud->addPoints(points.begin(), points.end());
        auto& anchor = impl_->anchors[marker.header.frame_id];
        if (!anchor.node) {
          anchor.node = scene_node_->createChildSceneNode();
          anchor.node->setVisible(false); // Never show sensor coordinates as world before TF arrives.
        }
        anchor.node->attachObject(cloud.get());
        ++anchor.blocks;
        impl_->points += points.size();
        impl_->blocks.emplace(id, Impl::Block{std::move(cloud), marker.header.frame_id, points.size()});
      }
      // Remember empty/deleted blocks as well as visible geometry.
      impl_->inbox->acknowledge(marker);
      changed = true;
    } catch (const std::exception& error) {
      setStatus(rviz_common::properties::StatusProperty::Error, "Geometry", error.what());
      impl_->inbox->requestReplay();
    }
  }
  const auto now = std::chrono::steady_clock::now();
  // Static keyframe anchors do not need a full historical TF walk at display
  // frame rate. Match the publisher's 10 Hz cadence; new geometry is immediate.
  if (changed || now >= impl_->nextAnchorRefresh) {
    impl_->nextAnchorRefresh = now + std::chrono::milliseconds(100);
    size_t missing = 0;
    for (auto& entry : impl_->anchors) {
      Ogre::Vector3 position;
      Ogre::Quaternion orientation;
      auto* node = entry.second.node;
      if (!context_->getFrameManager()->getTransform(entry.first, position, orientation)) {
        if (entry.second.visible) { node->setVisible(false); changed = true; }
        entry.second.visible = false; ++missing; continue;
      }
      if (node->getPosition() != position || node->getOrientation() != orientation) {
        node->setPosition(position); node->setOrientation(orientation); changed = true;
      }
      // setVisible affects currently attached objects, including a newly
      // replaced block attached to an already visible keyframe node.
      node->setVisible(true);
      if (!entry.second.visible) { entry.second.visible = true; changed = true; }
    }
    setStatus(rviz_common::properties::StatusProperty::Ok, "Map",
        QString("%1 points in %2 keyframes; no point cap")
            .arg(qulonglong(impl_->points)).arg(qulonglong(impl_->anchors.size())));
    setStatus(missing ? rviz_common::properties::StatusProperty::Warn : rviz_common::properties::StatusProperty::Ok,
        "TF", missing ? QString("Waiting for %1 keyframe transforms").arg(qulonglong(missing)) : "Keyframe transforms available");
  }
  if (impl_->inbox->replayWhenDrained()) subscribe(false);
  if (changed) queueRender();
}
} // namespace okvis

PLUGINLIB_EXPORT_CLASS(okvis::GlobalLidarMapRvizDisplay, rviz_common::Display)
