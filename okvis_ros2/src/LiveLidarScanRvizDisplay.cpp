// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/ros2/LiveLidarScanRvizDisplay.hpp>
#include <okvis/ros2/LiveLidarScanBuffer.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <OgreSceneNode.h>

namespace okvis {
namespace {
using Buffer = detail::LiveLidarScanBuffer;
using Status = rviz_common::properties::StatusProperty;
} // namespace

struct LiveLidarScanRvizDisplay::Impl {
  rviz_common::properties::RosTopicProperty* topic = nullptr;
  rviz_common::properties::EnumProperty* reliability = nullptr;
  rviz_common::properties::FloatProperty* size = nullptr;
  rviz_common::properties::FloatProperty* alpha = nullptr;
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<Buffer::Cloud>::SharedPtr subscription;
  std::shared_ptr<Buffer> buffer;
  std::unique_ptr<rviz_rendering::PointCloud> cloud;
  std::string subscribedTopic;
  int mode = 0;
  bool reliable = false;
  float pointSize = 2, opacity = .9;
  Buffer::Clock::time_point nextQosCheck{}, subscribedAt{}, lastShown{};

  bool chooseReliable(const std::string& topicName, int option) const {
    if (option != 0) return option == 1;
    const auto publishers = node->get_publishers_info_by_topic(topicName);
    if (publishers.empty()) return false;
    // Reliable bag playback benefits from fragment retransmission for these
    // large samples. A Best Effort driver requires a compatible subscription.
    for (const auto& publisher : publishers)
      if (publisher.qos_profile().get_rmw_qos_profile().reliability != RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        return false;
    return true;
  }
};

LiveLidarScanRvizDisplay::LiveLidarScanRvizDisplay() : impl_(std::make_unique<Impl>()) {
  impl_->topic = new rviz_common::properties::RosTopicProperty("Topic", "/ouster/points",
      "sensor_msgs/msg/PointCloud2", "Current raw scan; waits for TF at its original header timestamp.", this);
  impl_->reliability = new rviz_common::properties::EnumProperty("Reliability", "Auto",
      "Auto matches a Reliable bag publisher or a Best Effort live driver. DDS depth is one.", this);
  impl_->reliability->addOption("Auto", 0);
  impl_->reliability->addOption("Reliable", 1);
  impl_->reliability->addOption("Best Effort", 2);
  impl_->size = new rviz_common::properties::FloatProperty("Size (Pixels)", 2.0,
      "White point size in pixels.", this);
  impl_->size->setMin(1); impl_->size->setMax(20);
  impl_->alpha = new rviz_common::properties::FloatProperty("Alpha", .9,
      "Opacity of the current scan. Historical scans are never accumulated.", this);
  impl_->alpha->setMin(0); impl_->alpha->setMax(1);
}

LiveLidarScanRvizDisplay::~LiveLidarScanRvizDisplay() {
  impl_->subscription.reset(); impl_->buffer.reset(); clearCloud();
}
void LiveLidarScanRvizDisplay::onInitialize() {
  impl_->topic->initialize(context_->getRosNodeAbstraction());
  if (auto abstraction = context_->getRosNodeAbstraction().lock())
    impl_->node = abstraction->get_raw_node();
}
void LiveLidarScanRvizDisplay::clearCloud() {
  if (impl_->cloud && scene_node_) scene_node_->detachObject(impl_->cloud.get());
  impl_->cloud.reset();
  impl_->lastShown = {};
}
void LiveLidarScanRvizDisplay::subscribe(bool clear) {
  impl_->subscription.reset();
  impl_->buffer = std::make_shared<Buffer>();
  if (clear) clearCloud();
  impl_->subscribedTopic = impl_->topic->getTopicStd();
  impl_->mode = impl_->reliability->getOptionInt();
  impl_->subscribedAt = Buffer::Clock::now();
  impl_->nextQosCheck = impl_->subscribedAt + std::chrono::seconds(1);
  clearStatuses();
  if (!isEnabled()) return;
  if (!impl_->node || impl_->subscribedTopic.empty()) {
    setStatus(Status::Warn, "Topic", "Select a topic and check the RViz ROS node");
    return;
  }
  try {
    impl_->reliable = impl_->chooseReliable(impl_->subscribedTopic, impl_->mode);
    auto qos = rclcpp::QoS(1).durability_volatile();
    if (impl_->reliable) qos.reliable(); else qos.best_effort();
    const auto buffer = impl_->buffer;
    impl_->subscription = impl_->node->create_subscription<Buffer::Cloud>(impl_->subscribedTopic, qos,
        [buffer](Buffer::Cloud::ConstSharedPtr cloud) { buffer->accept(std::move(cloud)); });
    setStatus(Status::Ok, "Topic", impl_->reliable ? "Reliable, depth 1" : "Best Effort, depth 1");
  } catch (const std::exception& error) {
    setStatus(Status::Error, "Topic", error.what());
  }
}
void LiveLidarScanRvizDisplay::onEnable() { subscribe(true); }
void LiveLidarScanRvizDisplay::onDisable() {
  impl_->subscription.reset(); impl_->buffer.reset(); clearCloud();
}
void LiveLidarScanRvizDisplay::reset() {
  rviz_common::Display::reset(); subscribe(true);
}
void LiveLidarScanRvizDisplay::fixedFrameChanged() {
  // Geometry already positioned in the old fixed frame must not remain visible.
  subscribe(true);
}
void LiveLidarScanRvizDisplay::setTopic(const QString& topic, const QString&) { impl_->topic->setValue(topic); }

void LiveLidarScanRvizDisplay::update(float, float) {
  if (!isEnabled()) return;
  const auto now = Buffer::Clock::now();
  if (impl_->topic->getTopicStd() != impl_->subscribedTopic
      || impl_->reliability->getOptionInt() != impl_->mode) subscribe(true);
  if (!impl_->buffer || !impl_->node) return;
  if (!impl_->subscription) {
    if (!impl_->subscribedTopic.empty() && now >= impl_->nextQosCheck) subscribe(false);
    return; // Preserve subscription errors; do not replace them with an OK status.
  }
  if (impl_->mode == 0 && now >= impl_->nextQosCheck) {
    impl_->nextQosCheck = now + std::chrono::seconds(1);
    try {
      if (impl_->chooseReliable(impl_->subscribedTopic, 0) != impl_->reliable) subscribe(false);
    } catch (const std::exception& error) { setStatus(Status::Warn, "Topic", error.what()); }
  }
  if (!impl_->subscription) return;
  const float size = impl_->size->getFloat(), alpha = impl_->alpha->getFloat();
  if (impl_->cloud && (size != impl_->pointSize || alpha != impl_->opacity)) {
    impl_->cloud->setDimensions(size, size, size); impl_->cloud->setAlpha(alpha); queueRender();
  }
  impl_->pointSize = size; impl_->opacity = alpha;
  const auto view = impl_->buffer->view();
  const bool noInput = view.lastReceived == Buffer::Clock::time_point{}
      || now - view.lastReceived > std::chrono::seconds(1);
  setStatus(noInput ? Status::Warn : Status::Ok, "Input", noInput
      ? QString("No new valid raw scans; check bag playback/publisher (%1 rejected)")
          .arg(qulonglong(view.rejected))
      : QString("Receiving scans; %1 empty-frame/oversized scans rejected in total")
          .arg(qulonglong(view.rejected)));
  setStatus(Status::Ok, "Topic", QString("%1 received; %2; depth 1; at most 4 waiting scans")
      .arg(qulonglong(view.received)).arg(impl_->reliable ? "Reliable" : "Best Effort"));

  // Search newest-to-oldest, not just the newest scan: typically the newest
  // needs the next camera TF while the immediately preceding scan is ready.
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  const auto* next = view.newestReady([&](const Buffer::Cloud& cloud) {
    return context_->getFrameManager()->getTransform(cloud.header, position, orientation);
  });
  if (next) {
    try {
      std::vector<rviz_rendering::PointCloud::Point> points;
      points.reserve(uint64_t(next->cloud->width)*next->cloud->height);
      Buffer::forEachPoint(*next->cloud, [&](const std::array<float, 3>& xyz) {
        rviz_rendering::PointCloud::Point point;
        point.position = Ogre::Vector3(xyz[0], xyz[1], xyz[2]);
        point.setColor(1, 1, 1, 1); points.push_back(point);
      });
      // Only one current-scan geometry, replaced only after timestamp-correct
      // TF is available. Do not republish/relabel the source cloud as world.
      if (!impl_->cloud) {
        impl_->cloud = std::make_unique<rviz_rendering::PointCloud>();
        impl_->cloud->setRenderMode(rviz_rendering::PointCloud::RM_POINTS);
        scene_node_->attachObject(impl_->cloud.get());
      }
      impl_->cloud->clearAndRemoveAllPoints();
      impl_->cloud->setDimensions(size, size, size); impl_->cloud->setAlpha(alpha);
      if (!points.empty()) impl_->cloud->addPoints(points.begin(), points.end());
      scene_node_->setPosition(position); scene_node_->setOrientation(orientation);
      impl_->lastShown = now;
      impl_->buffer->consumeThrough(next->sequence);
      setStatus(Status::Ok, "Transform", "Showing newest scan with TF at its original timestamp");
      setStatus(Status::Ok, "Points", QString("%1 white points; current scan only")
          .arg(qulonglong(points.size())));
      queueRender(); return;
    } catch (const std::exception& error) {
      impl_->buffer->consumeThrough(next->sequence);
      setStatus(Status::Error, "Points", error.what()); return;
    }
  }
  if (!view.pending.empty()) {
    // A routine one-frame TF delay is not an error. Keep the most recent
    // correctly positioned scan, retry on the next render tick, and report
    // a real warning only if no scan can be transformed for a full second.
    const auto lastProgress = impl_->lastShown == Buffer::Clock::time_point{}
        ? impl_->subscribedAt : impl_->lastShown;
    const bool stalled = now - lastProgress > std::chrono::seconds(1);
    const auto& header = view.pending.back().cloud->header;
    QString description = QString("Waiting for %1 -> %2 at scan time %3.%4")
            .arg(QString::fromStdString(header.frame_id)).arg(fixed_frame_).arg(header.stamp.sec)
            .arg(qulonglong(header.stamp.nanosec), 9, 10, QLatin1Char('0'));
    if (stalled) {
      std::string reason;
      if (context_->getFrameManager()->transformHasProblems(header.frame_id,
          rclcpp::Time(header.stamp, RCL_ROS_TIME), reason))
        description += ": " + QString::fromStdString(reason);
    }
    setStatus(stalled ? Status::Warn : Status::Ok, "Transform", description);
  }
}
} // namespace okvis

PLUGINLIB_EXPORT_CLASS(okvis::LiveLidarScanRvizDisplay, rviz_common::Display)
