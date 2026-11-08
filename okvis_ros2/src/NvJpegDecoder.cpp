// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/ros2/NvJpegDecoder.hpp>

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>
#include <opencv2/imgproc.hpp>

namespace okvis {
namespace {

void checkNvjpeg(nvjpegStatus_t status, const char* operation) {
  if (status != NVJPEG_STATUS_SUCCESS) {
    throw std::runtime_error(std::string(operation) + " failed with nvJPEG status "
        + std::to_string(static_cast<int>(status)));
  }
}

void checkCuda(cudaError_t status, const char* operation) {
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + " failed: " + cudaGetErrorString(status));
  }
}

bool containsJpeg(const std::string& format) {
  std::string lower = format;
  std::transform(lower.begin(), lower.end(), lower.begin(),
      [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
  return lower.find("jpeg") != std::string::npos || lower.find("jpg") != std::string::npos;
}

}  // namespace

NvJpegDecoder::NvJpegDecoder() {
  try {
    checkCuda(cudaSetDevice(0), "cudaSetDevice");
    checkNvjpeg(nvjpegCreateSimple(&handle_), "nvjpegCreateSimple");
    checkNvjpeg(nvjpegJpegStateCreate(handle_, &state_), "nvjpegJpegStateCreate");
    checkCuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "cudaStreamCreate");
  } catch (...) {
    release();
    throw;
  }
}

NvJpegDecoder::~NvJpegDecoder() {
  release();
}

cv::Mat NvJpegDecoder::decode(const sensor_msgs::msg::CompressedImage& message, bool colour) {
  std::lock_guard<std::mutex> lock(mutex_);
  checkCuda(cudaSetDevice(0), "cudaSetDevice");
  if (message.data.empty()) throw std::invalid_argument("Compressed image payload is empty");
  if (!containsJpeg(message.format)) {
    throw std::invalid_argument("nvJPEG requires a JPEG payload; received format '" + message.format + "'");
  }

  int components = 0;
  nvjpegChromaSubsampling_t subsampling = NVJPEG_CSS_UNKNOWN;
  int widths[NVJPEG_MAX_COMPONENT] = {};
  int heights[NVJPEG_MAX_COMPONENT] = {};
  checkNvjpeg(nvjpegGetImageInfo(handle_, message.data.data(), message.data.size(),
      &components, &subsampling, widths, heights), "nvjpegGetImageInfo");
  const int width = widths[0];
  const int height = heights[0];
  if (width <= 0 || height <= 0 || (components != 1 && components != 3)) {
    throw std::runtime_error("nvJPEG returned invalid image dimensions or component count");
  }

  const int channels = colour && components == 3 ? 3 : 1;
  const size_t rowBytes = static_cast<size_t>(width) * channels;
  if (bufferWidth_ != width || bufferHeight_ != height || bufferChannels_ != channels) {
    if (deviceImage_) {
      checkCuda(cudaFree(deviceImage_), "cudaFree");
      deviceImage_ = nullptr;
    }
    devicePitch_ = 0;
    checkCuda(cudaMallocPitch(reinterpret_cast<void**>(&deviceImage_), &devicePitch_,
        rowBytes, static_cast<size_t>(height)), "cudaMallocPitch");
    bufferWidth_ = width;
    bufferHeight_ = height;
    bufferChannels_ = channels;
  }

  nvjpegImage_t destination{};
  destination.channel[0] = deviceImage_;
  destination.pitch[0] = devicePitch_;
  const nvjpegOutputFormat_t outputFormat = channels == 3
      ? NVJPEG_OUTPUT_RGBI : NVJPEG_OUTPUT_Y;
  checkNvjpeg(nvjpegDecode(handle_, state_, message.data.data(), message.data.size(),
      outputFormat, &destination, stream_), "nvjpegDecode");
  checkCuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");

  cv::Mat decoded(height, width, channels == 3 ? CV_8UC3 : CV_8UC1);
  checkCuda(cudaMemcpy2D(decoded.data, decoded.step, deviceImage_, devicePitch_, rowBytes,
      static_cast<size_t>(height), cudaMemcpyDeviceToHost), "cudaMemcpy2D");
  if (colour && components == 1) {
    cv::Mat rgb;
    cv::cvtColor(decoded, rgb, cv::COLOR_GRAY2RGB);
    return rgb;
  }
  return decoded;
}

void NvJpegDecoder::release() noexcept {
  if (deviceImage_) cudaFree(deviceImage_);
  deviceImage_ = nullptr;
  if (stream_) cudaStreamDestroy(stream_);
  stream_ = nullptr;
  if (state_) nvjpegJpegStateDestroy(state_);
  state_ = nullptr;
  if (handle_) nvjpegDestroy(handle_);
  handle_ = nullptr;
}

}  // namespace okvis
