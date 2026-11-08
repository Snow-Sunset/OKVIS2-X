// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/TemporalFeatureTracker.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <condition_variable>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <stdexcept>

namespace okvis {
namespace {
std::atomic<uint64_t> nextTrackId{1};
bool inImage(const cv::Point2f& point, const cv::Mat& image) {
  constexpr float border = 8.f;
  return std::isfinite(point.x) && std::isfinite(point.y)
      && point.x >= border && point.y >= border
      && point.x < image.cols - border && point.y < image.rows - border;
}

bool temporalMatchVisualizationEnabled() {
  static const bool enabled = [] {
    // Keep batch/headless runs safe. An explicit environment setting can force
    // the viewer on or off; otherwise show it when a desktop display is present.
    if (const char* setting = std::getenv("OKVIS_SHOW_TEMPORAL_MATCHES"))
      return std::string(setting) != "0" && std::string(setting) != "false";
    return std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
  }();
  return enabled;
}

class TemporalMatchViewer {
 public:
  static TemporalMatchViewer& instance() {
    static TemporalMatchViewer viewer;
    return viewer;
  }

  void submit(size_t cameraIndex, const cv::Mat& image) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) return;
      pending_[cameraIndex] = image.clone();
    }
    condition_.notify_one();
  }

  ~TemporalMatchViewer() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_one();
    if (worker_.joinable()) worker_.join();
  }

 private:
  TemporalMatchViewer() : worker_(&TemporalMatchViewer::run, this) {}

  void run() {
    std::set<size_t> openWindows;
    while (true) {
      std::map<size_t, cv::Mat> frames;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_for(lock, std::chrono::milliseconds(20),
                            [&] { return stopping_ || !pending_.empty(); });
        if (stopping_) break;
        frames.swap(pending_);
      }

      try {
        for (const auto& entry : frames) {
          const std::string name = "OKVIS temporal matches camera " + std::to_string(entry.first);
          if (openWindows.insert(entry.first).second) {
            cv::namedWindow(name, cv::WINDOW_NORMAL);
            cv::resizeWindow(name, entry.second.cols, entry.second.rows);
          }
          cv::imshow(name, entry.second);
        }
        // HighGUI event processing and all window operations stay on this one
        // thread instead of racing the two camera feature workers.
        cv::waitKey(1);
      } catch (const cv::Exception& error) {
        std::cerr << "OpenCV temporal match visualization stopped: " << error.what() << std::endl;
        break;
      }
    }

    for (const size_t cameraIndex : openWindows) {
      try {
        cv::destroyWindow("OKVIS temporal matches camera " + std::to_string(cameraIndex));
      } catch (const cv::Exception&) {
      }
    }
  }

  std::mutex mutex_;
  std::condition_variable condition_;
  std::map<size_t, cv::Mat> pending_;
  bool stopping_ = false;
  std::thread worker_;
};

void showTemporalMatches(const cv::Mat& previousImage,
                         const std::vector<cv::KeyPoint>& previousKeypoints,
                         const std::vector<FeatureTrack>& previousTracks,
                         const cv::Mat& currentImage,
                         const std::vector<cv::KeyPoint>& matches,
                         size_t cameraIndex) {
  if (!temporalMatchVisualizationEnabled()) return;

  cv::Mat previousColor, currentColor;
  cv::cvtColor(previousImage, previousColor, cv::COLOR_GRAY2BGR);
  cv::cvtColor(currentImage, currentColor, cv::COLOR_GRAY2BGR);
  cv::Mat display;
  cv::hconcat(previousColor, currentColor, display);

  size_t drawn = 0;
  size_t shortTracks = 0;
  size_t mediumTracks = 0;
  size_t matureTracks = 0;
  uint32_t longestTrack = 1;
  for (const auto& key : matches) {
    if (key.class_id < 0 || size_t(key.class_id) >= previousKeypoints.size()
        || size_t(key.class_id) >= previousTracks.size()) continue;
    const size_t oldIndex = size_t(key.class_id);
    const uint32_t length = previousTracks[oldIndex].length == std::numeric_limits<uint32_t>::max()
        ? previousTracks[oldIndex].length : previousTracks[oldIndex].length + 1;
    longestTrack = std::max(longestTrack, length);
    if (length < 10) ++shortTracks;
    else if (length < 30) ++mediumTracks;
    else ++matureTracks;

    // Short tracks are red, mature tracks become green. Both endpoints and
    // their connecting line use the same color to make each association clear.
    const double maturity = std::min(1.0, double(length > 2 ? length - 2 : 0) / 28.0);
    const cv::Scalar color(0, 255.0 * maturity, 255.0 * (1.0 - maturity));
    const cv::Point2f oldPoint = previousKeypoints[oldIndex].pt;
    const cv::Point2f currentPoint(key.pt.x + float(previousImage.cols), key.pt.y);
    cv::line(display, oldPoint, currentPoint, color, 1, cv::LINE_AA);
    cv::circle(display, oldPoint, 3, color, cv::FILLED, cv::LINE_AA);
    cv::circle(display, currentPoint, 3, color, cv::FILLED, cv::LINE_AA);
    ++drawn;
  }

  // Keep the two-camera montage within a typical desktop window size.
  const double scale = std::min({1.0, 1400.0 / display.cols, 900.0 / display.rows});
  if (scale < 1.0) cv::resize(display, display, cv::Size(), scale, scale, cv::INTER_AREA);

  cv::putText(display, "Previous", cv::Point(12, 26), cv::FONT_HERSHEY_SIMPLEX,
              0.7, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
  cv::putText(display, "Current", cv::Point(display.cols / 2 + 12, 26),
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
  const std::string counts = "Camera " + std::to_string(cameraIndex) + " | LK kept: "
      + std::to_string(drawn) + "/" + std::to_string(previousKeypoints.size())
      + " | age <10: " + std::to_string(shortTracks) + " | 10-29: "
      + std::to_string(mediumTracks) + " | >=30: " + std::to_string(matureTracks)
      + " | longest: " + std::to_string(longestTrack);
  cv::putText(display, counts, cv::Point(12, display.rows - 34),
              cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
  cv::putText(display, "red: short-lived | green: >=30 frames",
              cv::Point(12, display.rows - 10), cv::FONT_HERSHEY_SIMPLEX,
              0.5, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

  TemporalMatchViewer::instance().submit(cameraIndex, display);
}
}

void TemporalFeatureTracker::reset() {
  previousImage_.release();
  previousKeypoints_.clear();
  previousTracks_.clear();
}

std::vector<cv::KeyPoint> TemporalFeatureTracker::propose(
    const cv::Mat& image, const Time& timestamp) {
  if (image.type() != CV_8UC1 || image.empty())
    throw std::invalid_argument("Temporal tracking requires a nonempty CV_8UC1 image");
  if (!previousImage_.empty() && (image.size() != previousImage_.size()
      || timestamp <= previousTime_
      || (timestamp - previousTime_).toSec() > parameters_.track_max_gap)) reset();
  if (previousImage_.empty()) return {};
  if (previousKeypoints_.empty()) {
    showTemporalMatches(previousImage_, previousKeypoints_, previousTracks_, image, {}, cameraIndex_);
    return {};
  }

  std::vector<cv::Point2f> oldPoints, newPoints;
  for (const auto& key : previousKeypoints_) oldPoints.push_back(key.pt);
  std::vector<uchar> status;
  std::vector<float> error;
  const cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, .01);
  cv::calcOpticalFlowPyrLK(previousImage_, image, oldPoints, newPoints, status, error,
                          cv::Size(21, 21), 3, criteria);
  std::vector<size_t> indices;
  std::vector<cv::Point2f> forward, backward;
  for (size_t i = 0; i < status.size(); ++i) {
    if (!status[i] || !inImage(newPoints[i], image)) continue;
    indices.push_back(i);
    forward.push_back(newPoints[i]);
  }
  if (indices.empty()) {
    showTemporalMatches(previousImage_, previousKeypoints_, previousTracks_, image, {}, cameraIndex_);
    return {};
  }
  cv::calcOpticalFlowPyrLK(image, previousImage_, forward, backward, status, error,
                          cv::Size(21, 21), 3, criteria);
  std::vector<cv::KeyPoint> keys;
  for (size_t j = 0; j < indices.size(); ++j) {
    const size_t i = indices[j];
    if (!status[j] || !inImage(backward[j], previousImage_)
        || cv::norm(backward[j] - oldPoints[i]) > parameters_.track_max_fb_error) continue;
    cv::KeyPoint key = previousKeypoints_[i];
    key.pt = forward[j];
    key.class_id = int(i);
    keys.push_back(key);
  }
  std::stable_sort(keys.begin(), keys.end(), [&](const cv::KeyPoint& a, const cv::KeyPoint& b) {
    return previousTracks_.at(a.class_id).length > previousTracks_.at(b.class_id).length;
  });
  showTemporalMatches(previousImage_, previousKeypoints_, previousTracks_, image, keys, cameraIndex_);
  return keys;
}

std::vector<FeatureTrack> TemporalFeatureTracker::commit(
    const cv::Mat& image, const Time& timestamp, const std::vector<cv::KeyPoint>& keys) {
  std::vector<FeatureTrack> tracks;
  for (const auto& key : keys) {
    FeatureTrack track;
    if (key.class_id >= 0) {
      track = previousTracks_.at(size_t(key.class_id));
      if (track.length != std::numeric_limits<uint32_t>::max()) ++track.length;
    } else {
      track.id = nextTrackId.fetch_add(1);
      track.length = 1;
    }
    tracks.push_back(track);
    tracks.back().association = FeatureAssociation::None;
  }
  previousImage_ = image;
  previousTime_ = timestamp;
  previousKeypoints_ = keys;
  previousTracks_ = tracks;
  return tracks;
}

}  // namespace okvis
