// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <okvis/FeatureTrack.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/Time.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

namespace okvis {

/// Per-camera image tracking state. No backend graph or mutable MultiFrame is retained.
/// propose()/commit() bracket descriptor extraction, which may delete/reorder keypoints.
/// class_id is only a temporary index connecting the surviving keypoints to their tracks.
class TemporalFeatureTracker {
 public:
  explicit TemporalFeatureTracker(const FrontendParameters& parameters, size_t cameraIndex = 0)
      : parameters_(parameters), cameraIndex_(cameraIndex) {}
  std::vector<cv::KeyPoint> propose(const cv::Mat& image, const Time& timestamp);
  std::vector<FeatureTrack> commit(const cv::Mat& image, const Time& timestamp,
                                 const std::vector<cv::KeyPoint>& describedKeypoints);
  void reset();

 private:
  FrontendParameters parameters_;
  size_t cameraIndex_;
  cv::Mat previousImage_;
  Time previousTime_;
  std::vector<cv::KeyPoint> previousKeypoints_;
  std::vector<FeatureTrack> previousTracks_;
};

}  // namespace okvis
