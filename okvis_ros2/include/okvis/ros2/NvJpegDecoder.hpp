// SPDX-License-Identifier: BSD-3-Clause
#ifndef INCLUDE_OKVIS_ROS2_NVJPEGDECODER_HPP_
#define INCLUDE_OKVIS_ROS2_NVJPEGDECODER_HPP_

#include <cstddef>
#include <mutex>

#include <cuda_runtime_api.h>
#include <nvjpeg.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace okvis {

/// GPU JPEG decoder backed by NVIDIA nvJPEG. Returned images are CV_8UC1 for
/// monochrome cameras and CV_8UC3 in RGB order for colour cameras.
class NvJpegDecoder {
 public:
  NvJpegDecoder();
  ~NvJpegDecoder();

  NvJpegDecoder(const NvJpegDecoder&) = delete;
  NvJpegDecoder& operator=(const NvJpegDecoder&) = delete;

  cv::Mat decode(const sensor_msgs::msg::CompressedImage& message, bool colour);

 private:
  void release() noexcept;

  nvjpegHandle_t handle_ = nullptr;
  nvjpegJpegState_t state_ = nullptr;
  cudaStream_t stream_ = nullptr;
  unsigned char* deviceImage_ = nullptr;
  size_t devicePitch_ = 0;
  int bufferWidth_ = 0;
  int bufferHeight_ = 0;
  int bufferChannels_ = 0;
  std::mutex mutex_;
};

}  // namespace okvis

#endif  // INCLUDE_OKVIS_ROS2_NVJPEGDECODER_HPP_
