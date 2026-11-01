// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/LidarMeasurementBuffer.hpp>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

int main() {
  const auto directory = std::filesystem::temp_directory_path() / ("okvis-spool-test-" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    std::filesystem::create_directory(directory);
    {
      okvis::LidarMeasurementBuffer buffer;
      buffer.configure((directory / "rays.bin").string(), 8);
      for (int i = 0; i < 1000; ++i) {
        okvis::LidarMeasurement measurement;
        measurement.timeStamp = okvis::Time(42, i);
        measurement.measurement.rayMeasurement = Eigen::Vector3d(i, -i, 0.25);
        measurement.sensorId = 3;
        measurement.measurement.intensity = 100 + i;
        buffer.PushNonBlockingDroppingIfFull(measurement, 8);
      }
      if (buffer.Size() != 1000 || buffer.residentPoints() != 8)
        throw std::runtime_error("Spool dropped measurements or exceeded RAM cap");
      for (int i = 0; i < 1000; ++i) {
        okvis::LidarMeasurement front, next;
        if (!buffer.getCopyOfFront(&front) || !buffer.PopNonBlocking(&next)
            || front.timeStamp != okvis::Time(42, i) || next.timeStamp != front.timeStamp
            || next.sensorId != 3 || next.measurement.intensity != unsigned(100 + i)
            || (next.measurement.rayMeasurement - Eigen::Vector3d(i, -i, 0.25)).norm() != 0.0
            || buffer.residentPoints() > 8)
          throw std::runtime_error("Disk/RAM FIFO order or exact point/timestamp changed");
      }
      if (!buffer.Empty()) throw std::runtime_error("Spool did not drain");
      buffer.Shutdown();
    }
    {
      okvis::LidarMeasurementBuffer buffer;
      buffer.configure((directory / "batch-rays.bin").string(), 4);
      for (int i = 0; i < 12; ++i) {
        okvis::LidarMeasurement measurement;
        measurement.timeStamp = okvis::Time(7, i);
        measurement.measurement.rayMeasurement = Eigen::Vector3d(i, i + 1, i + 2);
        buffer.PushBlockingIfFull(measurement, 4);
      }
      if (buffer.residentPoints() != 4 || buffer.Size() != 12)
        throw std::runtime_error("Newest spool records did not retain a bounded hot working set");
      okvis::LidarMeasurementBuffer::Batch batch;
      if (buffer.PopUpTo(okvis::Time(7, 5), 4, &batch) != 4 || batch.front().timeStamp != okvis::Time(7, 0)
          || batch.back().timeStamp != okvis::Time(7, 3))
        throw std::runtime_error("First bounded LiDAR batch changed FIFO order");
      if (buffer.PopUpTo(okvis::Time(7, 5), 10, &batch) != 2 || batch.front().timeStamp != okvis::Time(7, 4)
          || batch.back().timeStamp != okvis::Time(7, 5))
        throw std::runtime_error("LiDAR batch did not stop at the timestamp boundary");
      okvis::LidarMeasurement next;
      if (!buffer.getCopyOfFront(&next) || next.timeStamp != okvis::Time(7, 6))
        throw std::runtime_error("LiDAR batch consumed a newer measurement");
      if (buffer.PopUpTo(okvis::Time(7, 99), 100, &batch) != 6 || !buffer.Empty())
        throw std::runtime_error("LiDAR batch did not drain the remaining spool");
      buffer.Shutdown();
    }
    {
      okvis::LidarMeasurementBuffer buffer;
      buffer.configureMemory();
      for (int i = 0; i < 12; ++i) {
        okvis::LidarMeasurement measurement;
        measurement.timeStamp = okvis::Time(8, i);
        measurement.measurement.rayMeasurement = Eigen::Vector3d(i, i + 1, i + 2);
        if (!buffer.PushBlockingIfFull(measurement, 4))
          throw std::runtime_error("RAM-only LiDAR buffer stopped while accepting data");
      }
      // Pointwise dataset callbacks cannot be treated as complete scan frames.
      if (buffer.Size() != 12 || buffer.statistics().dropped)
        throw std::runtime_error("Legacy pointwise input lost measurements");
      for (int i = 0; i < 12; ++i) {
        okvis::LidarMeasurement point;
        if (!buffer.PopNonBlocking(&point) || point.timeStamp != okvis::Time(8, i))
          throw std::runtime_error("Legacy pointwise input changed FIFO order");
      }
      buffer.Shutdown();
    }
    {
      okvis::LidarMeasurementBuffer buffer;
      buffer.configureMemory();
      okvis::LidarMeasurementBatch input;
      for (uint32_t i = 0; i < 15; ++i) {
        okvis::LidarMeasurement point;
        point.timeStamp = okvis::Time(9, i);
        point.measurement.rayMeasurement = Eigen::Vector3d(i, 0, 1);
        point.measurement.intensity = i;
        input.push_back(point);
      }
      const okvis::LidarMeasurementBatch first(input.begin(), input.begin() + 6);
      auto result = buffer.PushBatch(first);
      if (result.stored != 6 || result.dropped || result.stopped)
        throw std::runtime_error("RAM input truncated a complete scan");
      okvis::LidarMeasurementBatch copied, consumed;
      if (buffer.CopyUpTo(okvis::Time(9, 2), 99, &copied) != 3)
        throw std::runtime_error("RAM scan copy ignored per-point timestamp support");
      const okvis::LidarMeasurementBatch second(input.begin() + 6, input.begin() + 8);
      const okvis::LidarMeasurementBatch third(input.begin() + 8, input.begin() + 12);
      okvis::LidarMeasurementBuffer::PushResult replaced;
      std::thread producer([&] {
        buffer.PushBatch(second);
        replaced = buffer.PushBatch(third);
      });
      producer.join();
      if (replaced.stored != 4 || replaced.dropped != 2 || replaced.droppedScans != 1
          || buffer.PopCount(2, &consumed) != 2
          || consumed[0].timeStamp != copied[0].timeStamp
          || consumed[1].timeStamp != copied[1].timeStamp)
        throw std::runtime_error("Waiting scan replacement changed the pinned consumer prefix");
      buffer.PushBatch(okvis::LidarMeasurementBatch{});
      auto stats = buffer.statistics();
      if (stats.stored != 12 || stats.dropped != 2 || stats.droppedScans != 1
          || stats.consumed != 2 || stats.queued != 8 || stats.resident != 8
          || stats.pendingScans != 1)
        throw std::runtime_error("Partial scan or empty batch changed input accounting");
      const okvis::LidarMeasurementBatch fourth(input.begin() + 12, input.end());
      result = buffer.PushBatch(fourth);
      if (result.stored != 3 || result.dropped != 4 || result.droppedScans != 1
          || buffer.CopyUpTo(okvis::Time(9, 99), 99, &copied) != 4
          || copied.front().timeStamp != input[2].timeStamp
          || copied.back().timeStamp != input[5].timeStamp)
        throw std::runtime_error("New input evicted or reordered the active scan's remaining tail");
      buffer.Shutdown();
      result = buffer.PushBatch(fourth);
      if (!result.stopped || result.stored || result.dropped || result.droppedScans)
        throw std::runtime_error("Stopped FIFO accepted or overflow-counted a batch");
      if (buffer.PopCount(7, &consumed) != 7)
        throw std::runtime_error("Shutdown did not retain the active and latest pending scans");
      for (size_t i = 0; i < consumed.size(); ++i) {
        const auto& expected = input[i < 4 ? i + 2 : i + 8];
        if (consumed[i].timeStamp != expected.timeStamp
            || consumed[i].measurement.intensity != expected.measurement.intensity
            || (consumed[i].measurement.rayMeasurement - expected.measurement.rayMeasurement).norm() != 0.0)
          throw std::runtime_error("Active/pending scan drain changed point identity");
      }
      stats = buffer.statistics();
      if (stats.queued || stats.hasFront || stats.pendingScans || stats.droppedScans != 2
          || stats.consumed + stats.dropped != stats.stored)
        throw std::runtime_error("Latest-scan counters do not balance after drain");
    }
    {
      okvis::LidarMeasurementBuffer buffer;
      buffer.configureMemory();
      // A complete scan larger than the removed 200000-point cap must survive.
      okvis::LidarMeasurementBatch scan(200001);
      for (uint32_t i = 0; i < scan.size(); ++i) {
        scan[i].timeStamp = okvis::Time(11, i);
        scan[i].measurement.rayMeasurement = Eigen::Vector3d(i, 0, 1);
        scan[i].measurement.intensity = i;
      }
      buffer.PushBatch(scan);
      const auto replaced = buffer.PushBatch(scan);
      if (replaced.stored != scan.size() || replaced.dropped != scan.size()
          || replaced.droppedScans != 1 || buffer.Size() != scan.size())
        throw std::runtime_error("Waiting input was not replaced as one complete scan");
      okvis::LidarMeasurementBatch consumed;
      if (buffer.PopCount(scan.size(), &consumed) != scan.size()
          || consumed.back().measurement.intensity != scan.size() - 1)
        throw std::runtime_error("Large scan was truncated by a point-count limit");
    }
    {
      okvis::LidarMeasurementBuffer buffer;
      buffer.configureMemory();
      okvis::LidarMeasurementBatch scan(3);
      for (auto& point : scan) point.measurement.rayMeasurement = Eigen::Vector3d(1, 0, 1);
      scan[0].timeStamp = okvis::Time(12, 0);
      scan[1].timeStamp = okvis::Time(12, 0); // preserve repeated point times
      scan[2].timeStamp = okvis::Time(12, 1);
      buffer.PushBatch(scan);
      okvis::LidarMeasurementDeque live;
      const auto before = [](const okvis::LidarMeasurement& point) {
        return point.timeStamp < okvis::Time(12, 1);
      };
      if (buffer.PopWhile(before, &live) != 2 || live.size() != 2)
        throw std::runtime_error("Live consumer changed the strict camera-time boundary");
      scan[0].timeStamp = okvis::Time(12, 2);
      scan[1].timeStamp = okvis::Time(12, 3);
      scan[2].timeStamp = okvis::Time(12, 4);
      buffer.PushBatch(scan);
      scan[0].timeStamp = okvis::Time(12, 5);
      scan[1].timeStamp = okvis::Time(12, 6);
      scan[2].timeStamp = okvis::Time(12, 7);
      buffer.PushBatch(scan);
      if (buffer.PopWhile(before, static_cast<okvis::LidarMeasurementDeque*>(nullptr)) != 0)
        throw std::runtime_error("Pending replacement consumed the unsupported active tail");
      buffer.Shutdown();
      okvis::LidarMeasurement point;
      if (!buffer.PopBlocking(&point) || point.timeStamp != okvis::Time(12, 1))
        throw std::runtime_error("Shutdown lost the active scan tail");
      for (uint32_t i = 5; i <= 7; ++i) {
        if (!buffer.PopBlocking(&point) || point.timeStamp != okvis::Time(12, i))
          throw std::runtime_error("Shutdown lost the latest waiting scan");
      }
      if (buffer.PopBlocking(&point))
        throw std::runtime_error("Stopped empty scan input did not terminate");
    }
    {
      okvis::LidarMeasurementBuffer buffer;
      buffer.configure((directory / "bulk-rays.bin").string(), 2);
      okvis::LidarMeasurementBatch input;
      for (uint32_t i = 0; i < 9; ++i) {
        okvis::LidarMeasurement point;
        point.timeStamp = okvis::Time(10, i / 3); // repeated point timestamps are preserved
        point.measurement.rayMeasurement = Eigen::Vector3d(i, 0, 1);
        point.measurement.intensity = i;
        input.push_back(point);
      }
      const auto result = buffer.PushBatch(input);
      if (result.stored != input.size() || result.dropped || buffer.residentPoints() != 2)
        throw std::runtime_error("Spool bulk input was dropped or exceeded its read-cache cap");
      okvis::LidarMeasurementBatch consumed;
      if (buffer.PopCount(input.size(), &consumed) != input.size())
        throw std::runtime_error("Spool bulk input did not cross refill boundaries");
      for (size_t i = 0; i < input.size(); ++i) {
        if (consumed[i].timeStamp != input[i].timeStamp
            || consumed[i].measurement.intensity != i
            || (consumed[i].measurement.rayMeasurement - input[i].measurement.rayMeasurement).norm() != 0.0)
          throw std::runtime_error("Spool bulk refill changed timestamps or point identity");
      }
      const auto stats = buffer.statistics();
      if (stats.queued || stats.stored != input.size() || stats.consumed != input.size() || stats.dropped)
        throw std::runtime_error("Spool bulk accounting does not balance");
      buffer.Shutdown();
    }
    {
      okvis::LidarMeasurementBuffer buffer;
      buffer.configure((directory / "missing" / "rays.bin").string(), 1);
      okvis::LidarMeasurement point;
      bool rejected = false;
      try { buffer.PushBlockingIfFull(point, 1); } catch (const std::exception&) { rejected = true; }
      if (!rejected || buffer.Size() != 0) throw std::runtime_error("Spool disk failure silently accepted data");
    }
    std::filesystem::remove_all(directory);
    std::cout << "Latest-scan LiDAR input and lossless spool checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::filesystem::remove_all(directory);
    std::cerr << error.what() << '\n';
    return 1;
  }
}
