// SPDX-License-Identifier: BSD-3-Clause
#ifndef OKVIS_LIDAR_MAP_COLOUR_HPP
#define OKVIS_LIDAR_MAP_COLOUR_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace okvis {

/// Fixed world-height palette, shared by PCD and RViz. Bounds never depend on
/// the currently visible points, so colours do not flicker as the map grows.
class LidarHeightColour {
 public:
  LidarHeightColour(double minimum, double maximum) : minimum_(minimum), maximum_(maximum) {
    if (!std::isfinite(minimum) || !std::isfinite(maximum) || maximum <= minimum
        || !std::isfinite(maximum - minimum))
      throw std::invalid_argument("LiDAR height colour requires finite increasing bounds");
  }

  uint32_t operator()(double height) const {
    if (!std::isfinite(height)) return 0;
    const double t = std::clamp((height - minimum_) / (maximum_ - minimum_), 0.0, 1.0);
    // Blue -> cyan -> green -> yellow -> red.
    const auto channel = [](double value) {
      return uint32_t(std::lround(255.0 * std::clamp(value, 0.0, 1.0)));
    };
    return (channel(4.0*t - 2.0) << 16) | (channel(2.0 - std::abs(4.0*t - 2.0)) << 8)
        | channel(2.0 - 4.0*t);
  }

 private:
  double minimum_, maximum_;
};

} // namespace okvis
#endif
