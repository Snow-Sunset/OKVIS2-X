// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/RealtimeLidarMeasurementBuffer.hpp>
#include <iostream>
#include <thread>

namespace {
using Buffer = okvis::RealtimeLidarMeasurementBuffer;
using Batch = okvis::LidarMeasurementBatch;
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
Batch scan(uint32_t start, uint32_t count = 3, uint32_t repeated = 1) {
  Batch result;
  for (uint32_t i = 0; i < count; ++i) {
    okvis::LidarMeasurement point;
    point.timeStamp = okvis::Time(10, start + i / repeated);
    point.measurement.rayMeasurement = Eigen::Vector3d(start, i, 1);
    point.measurement.intensity = start + i;
    result.push_back(point);
  }
  return result;
}
void accounting(const Buffer& buffer, size_t bytes) {
  const auto stats = buffer.statistics();
  check(stats.stored == stats.queued + stats.consumed + stats.dropped + stats.coalescedPoints,
      "Packed live-scan accounting does not balance");
  check(stats.residentBytes <= bytes, "Packed live-scan payload budget exceeded");
}
void historyAndCoalescing() {
  Buffer buffer;
  const size_t budget = 64 * sizeof(okvis::LidarMeasurement);
  buffer.configureMemory(4, budget);
  buffer.PushBatch(scan(0));
  okvis::LidarMeasurementDeque output;
  buffer.PopBefore(okvis::Time(10, 1), &output); // pin the remaining active prefix
  check(output.size() == 1 && output.back().timeStamp == okvis::Time(10, 0), "Initial prefix changed");
  std::thread producer([&] {
    for (uint32_t start : {10, 20, 30, 40}) buffer.PushBatch(scan(start));
  });
  producer.join();
  auto stats = buffer.statistics();
  check(stats.pendingScans == 4 && stats.oldest == okvis::Time(10, 1) && !stats.dropped,
      "Future input replaced the active tail or needed scan history");
  const auto replaced = buffer.PushBatch(scan(50));
  check(replaced.droppedScans == 1 && replaced.dropped == 3, "History count bound did not evict one scan");
  output.clear();
  buffer.PopBefore(okvis::Time(10, 25), &output);
  check(output.size() == 3 && output.front().timeStamp == okvis::Time(10, 20),
      "Backend did not receive the newest completed scan near its image time");
  stats = buffer.statistics();
  check(stats.coalescedScans == 2 && stats.coalescedPoints == 5 && stats.pendingScans == 1,
      "Old complete scans were reprocessed or far-future eviction erased older history");
  output.clear();
  buffer.PopBefore(okvis::Time(10, 35), &output);
  check(output.size() == 3 && output.front().timeStamp == okvis::Time(10, 30),
      "Overflow kept only newest future input and lost the backend's next scan");
  accounting(buffer, budget);
  buffer.Shutdown();
  check(buffer.PushBatch(scan(60)).stopped && buffer.PushBatch({}).stopped, "Shutdown accepted input");
  output.clear();
  buffer.PopBefore(okvis::Time(11, 0), &output);
  check(output.size() == 3 && output.front().timeStamp == okvis::Time(10, 50), "Shutdown lost the retained tail");
  accounting(buffer, budget);
}
void payloadAndBoundaries() {
  Buffer buffer;
  const size_t budget = 9 * sizeof(okvis::LidarMeasurement);
  buffer.configureMemory(16, budget);
  buffer.PushBatch(scan(0));
  okvis::LidarMeasurementDeque output;
  buffer.PopBefore(okvis::Time(10, 1), &output);
  buffer.PushBatch(scan(10)); buffer.PushBatch(scan(20));
  const auto replaced = buffer.PushBatch(scan(30));
  check(replaced.droppedScans == 1 && buffer.statistics().pendingScans == 2,
      "Byte cap did not preserve active and oldest waiting scans");
  const auto rejected = buffer.PushBatch(scan(40, 10));
  check(rejected.rejected && !rejected.stopped && buffer.statistics().rejectedScans == 1,
      "Oversize input changed buffer lifetime or exceeded its allocation budget");
  accounting(buffer, budget);
  output.clear();
  buffer.PopBefore(okvis::Time(10, 1), &output);
  check(output.empty() && buffer.statistics().oldest == okvis::Time(10, 1),
      "A future scan invalidated the pinned equal-time boundary");

  Buffer repeated;
  repeated.configureMemory(2, budget);
  repeated.PushBatch(scan(100, 6, 3));
  output.clear(); repeated.PopBefore(okvis::Time(10, 101), &output);
  check(output.size() == 3, "Repeated timestamps lost scan rings or consumed equal boundary points");
  repeated.PushBatch(scan(110));
  output.clear(); repeated.PopBefore(okvis::Time(10, 102), &output);
  check(output.size() == 3 && output.back().timeStamp == okvis::Time(10, 101), "Partial scan tail changed");
  accounting(repeated, budget);
}
void completedScanAndSuccessorPrefix() {
  Buffer buffer;
  const size_t budget = 32 * sizeof(okvis::LidarMeasurement);
  buffer.configureMemory(4, budget);
  buffer.PushBatch(scan(0));
  buffer.PushBatch(scan(10));
  buffer.PushBatch(scan(20, 5));
  okvis::LidarMeasurementDeque output;
  buffer.PopBefore(okvis::Time(10, 22), &output);
  check(output.size() == 5 && output.front().timeStamp == okvis::Time(10, 10)
      && output.back().timeStamp == okvis::Time(10, 21),
      "Coalescing failed to retain the latest completed scan and successor prefix");
  auto stats = buffer.statistics();
  check(stats.coalescedScans == 1 && stats.coalescedPoints == 3
      && stats.queued == 3 && stats.oldest == okvis::Time(10, 22),
      "Coalescing consumed the equal-time boundary or lost the successor tail");
  output.clear();
  buffer.PopBefore(okvis::Time(10, 24), &output);
  check(output.size() == 2 && output.front().timeStamp == okvis::Time(10, 22)
      && buffer.statistics().oldest == okvis::Time(10, 24),
      "Bulk prefix transfer consumed the strict boundary");
  output.clear();
  buffer.PopBefore(okvis::Time(10, 25), &output);
  check(output.size() == 1 && output.front().timeStamp == okvis::Time(10, 24)
      && !buffer.statistics().hasFront && !buffer.statistics().residentBytes,
      "Final partial tail was not drained and released");
  accounting(buffer, budget);

  Buffer pinned;
  const size_t pinnedBudget = 9 * sizeof(okvis::LidarMeasurement);
  pinned.configureMemory(4, pinnedBudget);
  pinned.PushBatch(scan(0, 6));
  output.clear();
  pinned.PopBefore(okvis::Time(10, 1), &output);
  const auto rejected = pinned.PushBatch(scan(10, 4));
  check(rejected.rejected && !rejected.dropped && !rejected.stored
      && pinned.statistics().oldest == okvis::Time(10, 1)
      && pinned.statistics().queued == 5,
      "New input evicted a pinned active scan to satisfy the byte cap");
  accounting(pinned, pinnedBudget);
}
void legacy() {
  Buffer buffer;
  buffer.configureMemory(1, 1024);
  for (uint32_t i = 0; i < 40; ++i) check(buffer.PushBlockingIfFull(scan(i, 1).front(), 0), "Legacy input stopped");
  okvis::LidarMeasurementDeque output;
  check(buffer.PopBefore(okvis::Time(10, 40), &output) == 40, "Legacy pointwise input was coalesced");
  check(!buffer.statistics().dropped && !buffer.statistics().coalescedPoints, "Legacy transport was truncated");
}
} // namespace
int main() {
  try {
    historyAndCoalescing(); payloadAndBoundaries(); completedScanAndSuccessorPrefix(); legacy();
    std::cout << "Bounded timestamp-aligned live LiDAR history passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
