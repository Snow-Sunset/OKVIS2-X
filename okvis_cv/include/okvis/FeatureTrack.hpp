// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>

namespace okvis {

/// How this observation acquired its landmark, rather than how its image track started.
enum class FeatureAssociation : uint8_t {
  None, TemporalTrack, BriskMap, MotionStereo, Stereo, LoopClosure, Unknown
};

/// Identity belongs to an uninterrupted image track, independently of landmark identity.
/// Zero means this observation was produced by a legacy or loaded-map frontend.
struct FeatureTrack {
  uint64_t id = 0;
  uint32_t length = 0;
  FeatureAssociation association = FeatureAssociation::None;

  bool mature(int minimumLength) const {
    return id != 0 && length >= uint32_t(minimumLength);
  }
  bool reserved(int minimumLength) const {
    return id != 0 && length > 1 && !mature(minimumLength);
  }
};

}  // namespace okvis
