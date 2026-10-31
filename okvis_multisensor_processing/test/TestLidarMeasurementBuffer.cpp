// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/LidarMeasurementBuffer.hpp>
#include <chrono>
#include <filesystem>
#include <iostream>

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
      buffer.configureMemory(4);
      for (int i = 0; i < 12; ++i) {
        okvis::LidarMeasurement measurement;
        measurement.timeStamp = okvis::Time(8, i);
        measurement.measurement.rayMeasurement = Eigen::Vector3d(i, i + 1, i + 2);
        if (!buffer.PushBlockingIfFull(measurement, 4))
          throw std::runtime_error("RAM-only LiDAR buffer stopped while accepting data");
        // Keep the producer/consumer interleaving deterministic without ever
        // allowing the bounded RAM queue to grow past four records.
        if (buffer.Size() == 4) {
          okvis::LidarMeasurement consumed;
          if (!buffer.PopNonBlocking(&consumed))
            throw std::runtime_error("RAM-only LiDAR buffer could not drain");
        }
      }
      if (std::filesystem::exists(directory / "ram-only.bin") || buffer.residentPoints() > 4)
        throw std::runtime_error("RAM-only LiDAR buffer created a spool or exceeded its cap");
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
    std::cout << "Bounded lossless LiDAR spool checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::filesystem::remove_all(directory);
    std::cerr << error.what() << '\n';
    return 1;
  }
}
