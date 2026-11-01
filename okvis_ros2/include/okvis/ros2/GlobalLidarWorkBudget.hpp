// SPDX-License-Identifier: BSD-3-Clause
#ifndef OKVIS_ROS2_GLOBAL_LIDAR_WORK_BUDGET_HPP
#define OKVIS_ROS2_GLOBAL_LIDAR_WORK_BUDGET_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <time.h>

namespace okvis {
namespace detail {

/// Cooperative CPU budget for nonessential sidecar work, never SLAM threads.
/// Uses THIS thread's CPU clock on Linux, so disk/DDS/condition waits are not
/// charged as computation. Checkpoints are between bounded batches. This is
/// a soft duty-cycle target, not a scheduler guarantee or a point-count cap.
class GlobalLidarWorkBudget {
 public:
  explicit GlobalLidarWorkBudget(double fraction) : fraction_(fraction) {
    if (!std::isfinite(fraction_) || fraction_ <= 0 || fraction_ > 1)
      throw std::invalid_argument("Global PCD worker CPU fraction must be in (0, 1]");
    reset();
  }
  void checkpoint(bool draining = false) {
    if (draining || fraction_ >= 1) { reset(); return; }
    const auto now = std::chrono::steady_clock::now();
    const double wall = std::chrono::duration<double>(now - wallStart_).count();
    const double cpu = std::chrono::duration<double>(cpuNow() - cpuStart_).count();
    const double debt = cpu / fraction_ - wall;
    // Short sleeps keep state updates, newest keyframes and shutdown responsive.
    if (debt > 0) std::this_thread::sleep_for(std::chrono::duration<double>(std::min(debt, .01)));
    if (wall >= .1 && debt <= 0) reset();
  }

 private:
  static std::chrono::nanoseconds cpuNow() {
#if defined(CLOCK_THREAD_CPUTIME_ID)
    timespec stamp{};
    if (::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &stamp) == 0)
      return std::chrono::seconds(stamp.tv_sec) + std::chrono::nanoseconds(stamp.tv_nsec);
#endif
    // Portable conservative fallback; this may throttle wall waits as well.
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch());
  }
  void reset() { wallStart_ = std::chrono::steady_clock::now(); cpuStart_ = cpuNow(); }
  double fraction_;
  std::chrono::steady_clock::time_point wallStart_;
  std::chrono::nanoseconds cpuStart_{};
};

} // namespace detail
} // namespace okvis
#endif
