// SPDX-License-Identifier: BSD-3-Clause
#ifndef OKVIS_REALTIME_LIDAR_MEASUREMENT_BUFFER_HPP
#define OKVIS_REALTIME_LIDAR_MEASUREMENT_BUFFER_HPP

#include <okvis/LidarMeasurementBuffer.hpp>
#include <memory>

namespace okvis {

/// Bounded scan history for live registration, separate from mapping/spooling.
/// Packed scans avoid a deque allocation per few rays. Producers preserve the
/// active scan and older pending scans, replacing the farthest future waiting
/// scan on overflow. Only the consumer coalesces obsolete completed scans.
class RealtimeLidarMeasurementBuffer {
 public:
  using Batch = LidarMeasurementBatch;
  struct PushResult {
    size_t stored = 0, dropped = 0, droppedScans = 0;
    bool stopped = false, rejected = false;
  };
  struct Statistics : LidarMeasurementBuffer::Statistics {
    size_t residentBytes = 0;
    uint64_t coalescedPoints = 0, coalescedScans = 0, rejectedScans = 0;
  };

  void configureMemory(size_t scans = 16, size_t bytes = 64 * 1024 * 1024) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (configured_ || stopped_ || mode_ != Mode::None
        || !scans || scans > 64 || bytes < sizeof(LidarMeasurement))
      throw std::invalid_argument("Invalid realtime LiDAR scan history limits");
    legacy_.configureMemory();
    maxScans_ = scans; maxBytes_ = bytes; configured_ = true;
  }

  /// Preserve the existing lossless pointwise dataset path. A session uses
  /// either pointwise or complete-scan transport, never interleaved transports.
  bool PushBlockingIfFull(const LidarMeasurement& point, size_t limit) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopped_) return false;
      if (mode_ == Mode::Scans) throw std::logic_error("Mixed realtime LiDAR transports");
      mode_ = Mode::Points;
    }
    return legacy_.PushBlockingIfFull(point, limit);
  }

  PushResult PushBatch(const Batch& values) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopped_) { PushResult result; result.stopped = true; return result; }
      if (values.empty()) return {};
      if (mode_ == Mode::Points) throw std::logic_error("Mixed realtime LiDAR transports");
    }
    // Limits are immutable after setup. Reject oversize input BEFORE allocating
    // another payload, and never discard an active scan to make it fit.
    if (values.size() > maxBytes_ / sizeof(LidarMeasurement)) {
      std::lock_guard<std::mutex> lock(mutex_);
      PushResult result; result.stopped = stopped_; result.rejected = !stopped_;
      if (result.rejected) ++rejectedScans_;
      return result;
    }
    auto replacement = std::make_unique<Batch>(values);
    const size_t bytes = replacement->capacity() * sizeof(LidarMeasurement);
    std::vector<std::unique_ptr<Batch>> retired;
    retired.reserve(maxScans_);
    std::unique_lock<std::mutex> lock(mutex_);
    PushResult result;
    if (stopped_) { result.stopped = true; return result; }
    if (mode_ == Mode::Points) throw std::logic_error("Mixed realtime LiDAR transports");
    mode_ = Mode::Scans;
    const size_t activeBytes = active_ ? active_->capacity() * sizeof(LidarMeasurement) : 0;
    if (bytes > maxBytes_ - activeBytes) {
      result.rejected = true; ++rejectedScans_; return result;
    }
    while (!pending_.empty() && (pending_.size() >= maxScans_ || bytes > maxBytes_ - residentBytes_)) {
      auto& last = pending_.back();
      result.dropped += last->size(); ++result.droppedScans;
      queued_ -= last->size();
      residentBytes_ -= last->capacity() * sizeof(LidarMeasurement);
      retired.push_back(std::move(last)); pending_.pop_back();
    }
    // Account evictions even if allocating the small queue index throws.
    dropped_ += result.dropped; droppedScans_ += result.droppedScans;
    pending_.push_back(std::move(replacement));
    result.stored = values.size(); stored_ += values.size(); queued_ += values.size();
    residentBytes_ += bytes; newest_ = values.back().timeStamp;
    return result; // lock releases before large retired/replacement payloads.
  }

  /// Consumer-only, strict timestamp prefix, matching ThreadedSlam's old gate.
  /// Keep the newest complete scan before the image, plus any eligible prefix
  /// of its successor. Older complete scans are redundant for LIVE factors;
  /// mapping and the complete global PCD retain their independent input paths.
  size_t PopBefore(const Time& time, LidarMeasurementDeque* output) {
    std::vector<std::unique_ptr<Batch>> retired;
    retired.reserve(maxScans_ + 1);
    std::unique_lock<std::mutex> lock(mutex_);
    if (mode_ == Mode::Points) {
      lock.unlock();
      return legacy_.PopWhile([&](const LidarMeasurement& p) { return p.timeStamp < time; }, output);
    }
    promote();
    if (output) while (active_ && !pending_.empty()
        && active_->back().timeStamp < time && pending_.front()->back().timeStamp < time
        && active_->back().timeStamp <= pending_.front()->front().timeStamp) {
      const size_t count = active_->size() - offset_;
      coalescedPoints_ += count; ++coalescedScans_; queued_ -= count;
      residentBytes_ -= active_->capacity() * sizeof(LidarMeasurement);
      retired.push_back(std::move(active_)); promote();
    }
    size_t count = 0;
    while (active_ && (*active_)[offset_].timeStamp < time) {
      // Input batches are timestamp ordered. Transfer the eligible contiguous
      // range at once rather than updating the queue metadata for every ray.
      const auto first = active_->begin() + offset_;
      const auto last = std::lower_bound(first, active_->end(), time,
          [](const LidarMeasurement& point, const Time& boundary) {
            return point.timeStamp < boundary;
          });
      const size_t eligible = size_t(last - first);
      if (output) output->insert(output->end(), first, last);
      offset_ += eligible; count += eligible; queued_ -= eligible; consumed_ += eligible;
      if (offset_ == active_->size()) {
        residentBytes_ -= active_->capacity() * sizeof(LidarMeasurement);
        retired.push_back(std::move(active_)); promote();
      }
      else break; // The remaining tail starts at/after the image boundary.
    }
    return count;
  }

  Statistics statistics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Statistics result;
    if (mode_ == Mode::Points) {
      static_cast<LidarMeasurementBuffer::Statistics&>(result) = legacy_.statistics();
      result.residentBytes = result.resident * sizeof(LidarMeasurement);
      return result;
    }
    result.queued = queued_;
    result.resident = residentBytes_ / sizeof(LidarMeasurement);
    result.residentBytes = residentBytes_;
    result.stored = stored_; result.dropped = dropped_; result.consumed = consumed_;
    result.droppedScans = droppedScans_; result.pendingScans = pending_.size();
    result.coalescedPoints = coalescedPoints_; result.coalescedScans = coalescedScans_;
    result.rejectedScans = rejectedScans_; result.newest = newest_;
    result.hasFront = bool(active_) || !pending_.empty();
    if (active_) {
      result.oldest = (*active_)[offset_].timeStamp; result.frontScanEnd = active_->back().timeStamp;
    } else if (!pending_.empty()) {
      result.oldest = pending_.front()->front().timeStamp; result.frontScanEnd = pending_.front()->back().timeStamp;
    }
    return result;
  }

  void Shutdown() {
    { std::lock_guard<std::mutex> lock(mutex_); stopped_ = true; }
    legacy_.Shutdown(); // accepted scans remain drainable; no map/PCD changes.
  }

 private:
  void promote() {
    if (!active_ && !pending_.empty()) {
      active_ = std::move(pending_.front()); pending_.pop_front(); offset_ = 0;
    }
  }
  enum class Mode { None, Points, Scans };
  mutable std::mutex mutex_;
  LidarMeasurementBuffer legacy_;
  std::unique_ptr<Batch> active_;
  std::deque<std::unique_ptr<Batch>> pending_;
  size_t offset_ = 0, queued_ = 0, residentBytes_ = 0;
  size_t maxScans_ = 16, maxBytes_ = 64 * 1024 * 1024;
  uint64_t stored_ = 0, dropped_ = 0, droppedScans_ = 0, consumed_ = 0;
  uint64_t coalescedPoints_ = 0, coalescedScans_ = 0, rejectedScans_ = 0;
  Time newest_;
  Mode mode_ = Mode::None;
  bool configured_ = false, stopped_ = false;
};
} // namespace okvis
#endif
