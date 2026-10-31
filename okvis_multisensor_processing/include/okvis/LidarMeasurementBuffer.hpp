// SPDX-License-Identifier: BSD-3-Clause
#ifndef OKVIS_LIDAR_MEASUREMENT_BUFFER_HPP
#define OKVIS_LIDAR_MEASUREMENT_BUFFER_HPP

#include <okvis/Measurements.hpp>
#include <algorithm>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <stdexcept>
#include <vector>
#include <fcntl.h>
#include <unistd.h>

namespace okvis {
/// Ordered LiDAR input FIFO. It can use a lossless append-only disk spool with
/// a bounded read cache, or a bounded RAM-only queue that rejects overflow. In
/// spool mode all accepted rays remain on disk, including an unposed final
/// tail. The spool is an internal session file, not a raw-scan export format.
class LidarMeasurementBuffer {
 public:
  using Batch = LidarMeasurementBatch;

  struct PushResult {
    size_t stored = 0;
    size_t dropped = 0;
    bool stopped = false;
  };
  struct Statistics {
    size_t queued = 0;
    size_t resident = 0;
    uint64_t stored = 0;
    uint64_t dropped = 0;
    uint64_t consumed = 0;
    bool hasFront = false;
    Time oldest;
    Time newest;
  };

  void configure(const std::string& filename, size_t maxPoints) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (filename.empty() || !maxPoints || !memory_.empty() || diskWritten_ || memoryOnly_)
      throw std::invalid_argument("Invalid LiDAR spool configuration");
    filename_ = filename;
    maxPoints_ = maxPoints;
  }
  /// Configure a bounded RAM-only FIFO; no raw input file is created.
  void configureMemory(size_t maxPoints) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!maxPoints || !memory_.empty() || diskWritten_ || !filename_.empty() || memoryOnly_)
      throw std::invalid_argument("Invalid RAM-only LiDAR buffer configuration");
    maxPoints_ = maxPoints;
    memoryOnly_ = true;
  }
  bool PushBlockingIfFull(const LidarMeasurement& value, size_t) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return stopped_ || !memoryOnly_ || memory_.size() < maxPoints_; });
    if (stopped_) return false;
    return pushLocked(value);
  }
  bool PushNonBlockingDroppingIfFull(const LidarMeasurement& value, size_t) {
    bool dropped = false;
    PushNonBlockingDroppingNewestIfFull(value, &dropped);
    return dropped;
  }
  /// Push without blocking the sensor callback. RAM-only mode drops the
  /// incoming ray at capacity without modifying a prefix copied by the mapping
  /// consumer; spool mode remains lossless and appends to disk. Returns true
  /// only when the measurement was stored.
  bool PushNonBlockingDroppingNewestIfFull(
      const LidarMeasurement& value, bool* droppedNewest = nullptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (droppedNewest) *droppedNewest = false;
    if (stopped_) return false;
    if (memoryOnly_ && memory_.size() >= maxPoints_) {
      if (droppedNewest) *droppedNewest = true;
      ++droppedPoints_;
      return false;
    }
    return pushLocked(value);
  }
  /// Store an entire input batch with one lock acquisition. RAM overflow drops
  /// incoming points only; a prefix copied by the sole consumer stays valid.
  /// Spool mode appends every point and bounds only the resident read cache.
  PushResult PushBatch(const Batch& values) {
    std::lock_guard<std::mutex> lock(mutex_);
    PushResult result;
    if (stopped_) {
      result.stopped = true;
      return result;
    }
    const size_t acceptedLimit = memoryOnly_
        ? std::min(values.size(), maxPoints_ - memory_.size()) : values.size();
    result.dropped = values.size() - acceptedLimit;
    droppedPoints_ += result.dropped;
    for (size_t i = 0; i < acceptedLimit; ++i) {
      if (pushLocked(values[i], false)) ++result.stored;
    }
    if (result.stored) cv_.notify_one();
    return result;
  }
  bool getCopyOfFront(LidarMeasurement* value) {
    std::lock_guard<std::mutex> lock(mutex_);
    fill();
    if (memory_.empty()) return false;
    *value = memory_.front();
    return true;
  }
  bool PopNonBlocking(LidarMeasurement* value) {
    std::lock_guard<std::mutex> lock(mutex_);
    return pop(value);
  }
  /// Pop an ordered batch whose timestamps are no newer than maxTime.
  /// This replaces the common peek/pop loop with one lock acquisition and
  /// preserves the first newer measurement at the front of the FIFO.
  size_t PopUpTo(const Time& maxTime, size_t maxCount, Batch* values) {
    if (!values) throw std::invalid_argument("LiDAR batch output must not be null");
    values->clear();
    if (!maxCount) return 0;
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t available = memory_.size() + size_t(diskWritten_ - diskRead_);
    values->reserve(std::min<size_t>(std::min(maxCount, available), 32768));
    while (values->size() < maxCount) {
      fill();
      if (memory_.empty() || memory_.front().timeStamp > maxTime) break;
      values->push_back(std::move(memory_.front()));
      memory_.pop_front();
    }
    consumedPoints_ += values->size();
    cv_.notify_all();
    return values->size();
  }
  /// Copy an ordered batch without consuming it. This is used when pose lookup
  /// may fail: the caller can leave the unsupported prefix in the FIFO and
  /// retry it after the estimator catches up.
  size_t CopyUpTo(const Time& maxTime, size_t maxCount, Batch* values) {
    if (!values) throw std::invalid_argument("LiDAR batch output must not be null");
    values->clear();
    if (!maxCount) return 0;
    std::lock_guard<std::mutex> lock(mutex_);
    fill();
    const size_t count = std::min({maxCount, memory_.size(), size_t(32768)});
    values->reserve(count);
    for (size_t i = 0; i < count && memory_[i].timeStamp <= maxTime; ++i)
      values->push_back(memory_[i]);
    return values->size();
  }
  /// Consume exactly count entries from the front, preserving FIFO order.
  /// The count must come from a preceding CopyUpTo call; entries appended by
  /// producers cannot overtake the copied prefix.
  size_t PopCount(size_t count, Batch* values = nullptr) {
    if (values) values->clear();
    if (!count) return 0;
    std::lock_guard<std::mutex> lock(mutex_);
    size_t popped = 0;
    while (popped < count) {
      fill();
      if (memory_.empty()) break;
      if (values) values->push_back(std::move(memory_.front()));
      memory_.pop_front();
      ++popped;
    }
    consumedPoints_ += popped;
    cv_.notify_all();
    return popped;
  }
  bool PopBlocking(LidarMeasurement* value) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return stopped_ || !memory_.empty() || diskRead_ < diskWritten_; });
    if (stopped_) return false;
    return pop(value);
  }
  size_t Size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return memory_.size() + size_t(diskWritten_ - diskRead_);
  }
  bool Empty() const { return Size() == 0; }
  size_t residentPoints() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return memory_.size();
  }
  Statistics statistics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Statistics result;
    result.queued = memory_.size() + size_t(diskWritten_ - diskRead_);
    result.resident = memory_.size();
    result.stored = storedPoints_;
    result.dropped = droppedPoints_;
    result.consumed = consumedPoints_;
    result.hasFront = !memory_.empty();
    if (result.hasFront) result.oldest = memory_.front().timeStamp;
    result.newest = newestAcceptedTimestamp_;
    return result;
  }
  void Shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    cv_.notify_all();
    flush();
  }
  void Flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    flush();
  }
 private:
  struct Record { uint32_t sec, nsec; double x, y, z; int32_t sensorId; uint32_t intensity; };
  void flush() {
    if (!writer_.is_open()) return;
    writer_.flush();
    const int fd = ::open(filename_.c_str(), O_RDONLY);
    if (fd < 0) throw std::runtime_error("Cannot sync LiDAR input archive");
    const int synced = ::fsync(fd);
    ::close(fd);
    if (synced != 0) throw std::runtime_error("Cannot sync LiDAR input archive");
    const auto split = filename_.find_last_of('/');
    const std::string parent = split == std::string::npos ? "." : filename_.substr(0, split);
    const int directory = ::open(parent.empty() ? "/" : parent.c_str(), O_RDONLY | O_DIRECTORY);
    if (directory < 0) throw std::runtime_error("Cannot sync LiDAR input archive directory");
    const int directorySynced = ::fsync(directory);
    ::close(directory);
    if (directorySynced != 0) throw std::runtime_error("Cannot sync LiDAR input archive directory");
  }
  bool push(const LidarMeasurement& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_) return false;
    return pushLocked(value);
  }
  bool pushLocked(const LidarMeasurement& value, bool notify = true) {
    if (memoryOnly_ || filename_.empty()) {
      if (memoryOnly_ && memory_.size() >= maxPoints_) return false;
      memory_.push_back(value);
    } else {
      if (!writer_.is_open()) {
        writer_.exceptions(std::ios::failbit | std::ios::badbit);
        writer_.open(filename_, std::ios::binary | std::ios::trunc);
      }
      const auto& p = value.measurement.rayMeasurement;
      const Record record{value.timeStamp.sec, value.timeStamp.nsec, p.x(), p.y(), p.z(),
                          value.sensorId, value.measurement.intensity};
      // If no older record is waiting only on disk, retain a hot copy for the
      // mapping thread while appending the same lossless record to the spool.
      // diskRead_ advances because this record is already represented in RAM.
      const bool cacheHot = diskRead_ == diskWritten_ && memory_.size() < maxPoints_;
      writer_.write(reinterpret_cast<const char*>(&record), sizeof(record));
      ++diskWritten_;
      if (cacheHot) {
        memory_.push_back(value);
        ++diskRead_;
      }
    }
    ++storedPoints_;
    newestAcceptedTimestamp_ = value.timeStamp;
    if (notify) cv_.notify_one();
    return true;
  }
  void fill() {
    if (diskRead_ == diskWritten_ || memory_.size() >= maxPoints_) return;
    // Refill in useful chunks instead of issuing a seek/read for every point
    // after a producer burst. Existing hot points remain at the FIFO front.
    const size_t lowWatermark = std::max<size_t>(1, maxPoints_ / 2);
    if (!memory_.empty() && memory_.size() > lowWatermark) return;
    writer_.flush(); // Make the buffered tail visible to the independent reader.
    if (!reader_.is_open()) {
      reader_.exceptions(std::ios::failbit | std::ios::badbit);
      reader_.open(filename_, std::ios::binary);
    }
    reader_.clear();
    reader_.seekg(std::streamoff(diskRead_ * sizeof(Record)));
    constexpr size_t refillBatchPoints = 32768;
    const size_t capacity = maxPoints_ - memory_.size();
    const size_t count = std::min<size_t>(
        std::min<size_t>(capacity, refillBatchPoints), diskWritten_ - diskRead_);
    for (size_t i = 0; i < count; ++i) {
      Record record;
      reader_.read(reinterpret_cast<char*>(&record), sizeof(record));
      LidarMeasurement measurement;
      measurement.timeStamp = Time(record.sec, record.nsec);
      measurement.measurement.rayMeasurement = Eigen::Vector3d(record.x, record.y, record.z);
      measurement.sensorId = record.sensorId;
      measurement.measurement.intensity = record.intensity;
      memory_.push_back(measurement);
      ++diskRead_;
    }
  }
  bool pop(LidarMeasurement* value) {
    fill();
    if (memory_.empty()) return false;
    *value = memory_.front();
    memory_.pop_front();
    ++consumedPoints_;
    cv_.notify_all();
    return true;
  }
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<LidarMeasurement, Eigen::aligned_allocator<LidarMeasurement>> memory_;
  std::string filename_;
  size_t maxPoints_ = 200000;
  uint64_t diskRead_ = 0, diskWritten_ = 0;
  uint64_t storedPoints_ = 0, droppedPoints_ = 0, consumedPoints_ = 0;
  Time newestAcceptedTimestamp_;
  std::ofstream writer_;
  std::ifstream reader_;
  bool memoryOnly_ = false;
  bool stopped_ = false;
};
} // namespace okvis
#endif
