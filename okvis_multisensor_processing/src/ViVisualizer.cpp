/**
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense 
 * Depth or LiDAR, and GNSS
 *
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich 
 * and ETH Zurich
 *
 * SPDX-License-Identifier: BSD-3-Clause, see LICENESE file for details
 */

/**
 * @file ViVisualizer.cpp
 * @brief Source file for the ViVisualizer class.
 * @author Pascal Gohl
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */


#include <algorithm>
#include <cmath>
#include <string>
#include <okvis/kinematics/Transformation.hpp>

#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/FrameTypedefs.hpp>

#include "okvis/ViVisualizer.hpp"

// cameras and distortions
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>
#include <okvis/cameras/EucmCamera.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
namespace {

cv::Scalar trackAgeColor(uint32_t age) {
  if (age <= 3) return cv::Scalar(40, 40, 255);       // red: 1-3 frames
  if (age <= 8) return cv::Scalar(0, 165, 255);       // orange: 4-8 frames
  if (age <= 15) return cv::Scalar(0, 255, 255);      // yellow: 9-15 frames
  if (age <= 30) return cv::Scalar(0, 255, 0);        // green: 16-30 frames
  if (age <= 60) return cv::Scalar(255, 255, 0);      // cyan: 31-60 frames
  return cv::Scalar(255, 0, 255);                     // magenta: 61+ frames
}

void drawTrackAgeLegend(cv::Mat& image) {
  if (image.cols < 220 || image.rows < 52) return;

  const int panelWidth = std::min(370, image.cols - 12);
  const cv::Rect panel(6, 6, panelWidth, 40);
  cv::Mat background = image(panel).clone();
  cv::rectangle(background, cv::Rect(0, 0, panel.width, panel.height),
      cv::Scalar(10, 10, 10), cv::FILLED);
  cv::addWeighted(background, 0.68, image(panel), 0.32, 0.0, image(panel));

  const char* labels[] = {"1-3", "4-8", "9-15", "16-30", "31-60", "61+"};
  const uint32_t representativeAges[] = {2, 6, 12, 20, 45, 61};
  const int cellWidth = panel.width / 3;
  for (int i = 0; i < 6; ++i) {
    const int column = i % 3;
    const int row = i / 3;
    const cv::Point swatch(panel.x + column * cellWidth + 9,
        panel.y + 11 + row * 17);
    cv::circle(image, swatch, 3, trackAgeColor(representativeAges[i]), cv::FILLED, cv::LINE_AA);
    cv::putText(image, labels[i], cv::Point(swatch.x + 7, swatch.y + 4),
        cv::FONT_HERSHEY_SIMPLEX, 0.30, cv::Scalar(245, 245, 245), 1, cv::LINE_AA);
  }
}

}  // namespace

ViVisualizer::ViVisualizer(ViParameters &parameters)
    : parameters_(parameters) {
  if (parameters.nCameraSystem.numCameras() > 0) {
    init(parameters);
  }
}

ViVisualizer::~ViVisualizer() {
}

void ViVisualizer::init(ViParameters &parameters) {
  parameters_ = parameters;
}

cv::Mat ViVisualizer::drawMatches(VisualizationData::Ptr& data, size_t image_number) {
  const auto& image = data->currentFrames->image(image_number);
  if (image.empty()) return {};
  cv::Mat output;
  if (image.channels() == 1) cv::cvtColor(image, output, cv::COLOR_GRAY2BGR);
  else output = image.clone();

  constexpr double fontScale = .35;
  for (const auto& observation : data->observations) {
    if (observation.cameraIdx != image_number) continue;
    const auto& pixel = observation.keypointMeasurement;
    if (!pixel.allFinite() || pixel.x() < 0 || pixel.y() < 0
        || pixel.x() >= image.cols || pixel.y() >= image.rows) continue;
    const cv::Scalar color = trackAgeColor(observation.track.length);
    const cv::Point point(cvRound(pixel.x()), cvRound(pixel.y()));
    cv::circle(output, point, 2, color, cv::FILLED, cv::LINE_AA);
    const std::string label = std::to_string(observation.track.length);
    int baseline;
    const auto textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, fontScale, 1, &baseline);
    const cv::Point origin(std::max(0, std::min(point.x + 4, image.cols - textSize.width - 2)),
        std::min(image.rows - 3, std::max(textSize.height + 2, point.y - 4)));
    cv::putText(output, label, origin, cv::FONT_HERSHEY_SIMPLEX, fontScale,
        cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
    cv::putText(output, label, origin, cv::FONT_HERSHEY_SIMPLEX, fontScale, color, 1, cv::LINE_AA);
  }
  drawTrackAgeLegend(output);
  return output;
}


cv::Mat ViVisualizer::drawKeypoints(VisualizationData::Ptr& data,
                                     size_t cameraIndex) {

  std::shared_ptr<okvis::MultiFrame> currentFrames = data->currentFrames;
  cv::Mat currentImage = currentFrames->image(cameraIndex);

  cv::Mat outimg;
  cv::cvtColor(currentImage, outimg, cv::COLOR_GRAY2BGR);
  cv::Scalar greenColor(0, 255, 0);  // green

  cv::KeyPoint keypoint;
  for (size_t k = 0; k < currentFrames->numKeypoints(cameraIndex); ++k) {
    currentFrames->getCvKeypoint(cameraIndex, k, keypoint);

    const float radius = keypoint.size;
    const float angle = keypoint.angle / 180.0f * float(M_PI);

    cv::circle(outimg, keypoint.pt, int(radius), greenColor);
    cv::line(
        outimg,
        keypoint.pt,
        cv::Point2f(keypoint.pt.x + radius * cos(angle),
                    keypoint.pt.y - radius * sin(angle)),
        greenColor);
  }

  return outimg;
}

} /* namespace okvis */
