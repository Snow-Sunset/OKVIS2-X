// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <okvis/MultiFrame.hpp>
#include <vector>

namespace okvis {
struct TemporalPair {
  size_t previous = 0;
  size_t current = 0;
};

/// One-to-one correspondences by uninterrupted track identity, without descriptors.
std::vector<TemporalPair> temporalTrackPairs(const MultiFrame& previous,
    const MultiFrame& current, size_t camera, int minimumLength);

/// Bearing-space essential/rotation RANSAC, before any landmark/observation is inserted.
/// Small or degenerate sets fail closed. Works with all calibrated camera models.
std::vector<bool> temporalRansac(const MultiFrame& previous, const MultiFrame& current,
    size_t camera, const std::vector<TemporalPair>& pairs, double angularThreshold,
    bool* rotationOnly);
}  // namespace okvis
