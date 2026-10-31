/**
 * Disk-backed frame payloads for bounded online SLAM working sets.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <okvis/Frame.hpp>

#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>

namespace okvis {
namespace {
constexpr uint64_t kMagic = 0x4f4b5646524d3031ULL; // OKVFRM01; same-build session format.
constexpr uint64_t kMaxArrayBytes = 512ULL * 1024 * 1024;

template<class T> void put(std::ostream& out, const T& value) {
  out.write(reinterpret_cast<const char*>(&value), sizeof(value));
}
template<class T> void get(std::istream& in, T& value) {
  in.read(reinterpret_cast<char*>(&value), sizeof(value));
}
void putMat(std::ostream& out, const cv::Mat& mat) {
  const int rows = mat.rows, cols = mat.cols, type = mat.type();
  put(out, rows); put(out, cols); put(out, type);
  if (!mat.empty()) {
    for (int row = 0; row < rows; ++row)
      out.write(reinterpret_cast<const char*>(mat.ptr(row)), cols * mat.elemSize());
  }
}
void checkRemaining(std::istream& in, uint64_t bytes) {
  const auto position = in.tellg();
  in.seekg(0, std::ios::end);
  const auto end = in.tellg();
  in.seekg(position);
  if (position < 0 || end < position || bytes > uint64_t(end - position))
    throw std::runtime_error("Truncated frame archive payload");
}
void getMat(std::istream& in, cv::Mat& mat) {
  int rows, cols, type;
  get(in, rows); get(in, cols); get(in, type);
  if (rows < 0 || cols < 0 || type < 0 || type > CV_MAKETYPE(CV_64F, CV_CN_MAX))
    throw std::runtime_error("Invalid matrix in frame archive");
  // Bound the element count before multiplying by the channel/element size.
  // Corrupt dimensions can otherwise wrap the byte count before allocation.
  const uint64_t elements = uint64_t(rows) * uint64_t(cols);
  if (elements > kMaxArrayBytes / CV_ELEM_SIZE(type))
    throw std::runtime_error("Oversized matrix in frame archive");
  const uint64_t bytes = elements * CV_ELEM_SIZE(type);
  checkRemaining(in, bytes);
  if (!rows || !cols) { mat.release(); return; }
  mat.create(rows, cols, type);
  in.read(reinterpret_cast<char*>(mat.data), bytes);
}
template<class V> void putVector(std::ostream& out, const V& values) {
  const uint64_t count = values.size();
  put(out, count);
  for (const auto& value : values) put(out, value);
}
template<class V> void getVector(std::istream& in, V& values) {
  uint64_t count;
  get(in, count);
  if (count > kMaxArrayBytes / sizeof(typename V::value_type))
    throw std::runtime_error("Oversized vector in frame archive");
  checkRemaining(in, count * sizeof(typename V::value_type));
  values.resize(count);
  for (auto& value : values) get(in, value);
}
void putBools(std::ostream& out, const std::vector<bool>& values) {
  put(out, uint64_t(values.size()));
  for (bool value : values) put(out, uint8_t(value));
}
void getBools(std::istream& in, std::vector<bool>& values) {
  uint64_t count;
  get(in, count);
  if (count > kMaxArrayBytes) throw std::runtime_error("Oversized bit vector in frame archive");
  checkRemaining(in, count);
  values.resize(count);
  for (uint64_t i = 0; i < count; ++i) {
    uint8_t value; get(in, value);
    if (value > 1) throw std::runtime_error("Invalid boolean in frame archive");
    values[i] = value != 0;
  }
}
void syncFile(const std::string& filename) {
  const int fd = ::open(filename.c_str(), O_RDONLY);
  if (fd < 0) throw std::runtime_error("Cannot open frame archive for sync: " + filename);
  const int result = ::fsync(fd);
  ::close(fd);
  if (result != 0) throw std::runtime_error("Cannot sync frame archive: " + filename);
}
}

bool Frame::releaseImages() {
  std::lock_guard<std::mutex> lock(payloadMutex_);
  // A network can be attached even when use_cnn is false. Only an active worker
  // pins pixels here; the frontend joins pending workers before frames turn cold.
  if (classificationInProgress_) return false;
  if (!payloadResident_) return false;
  image_.release();
  depthImage_.release();
  payloadDirty_ = true;
  return true;
}

size_t Frame::residentPayloadBytes() const {
  if (!payloadResident_) return 0;
  return image_.total() * image_.elemSize() + depthImage_.total() * depthImage_.elemSize() +
      descriptors_.total() * descriptors_.elemSize() + classifications_.total() * classifications_.elemSize() +
      keypoints_.capacity() * sizeof(cv::KeyPoint) +
      backProjections_.capacity() * sizeof(Eigen::Vector3d) + backProjectionsValid_.capacity() / 8 +
      landmarkIds_.capacity() * sizeof(uint64_t) + landmarks_.capacity() * sizeof(Eigen::Vector4d) +
      landmarkInitialisations_.capacity() / 8;
}

bool Frame::archivePayload(const std::string& filename, bool release) {
  std::lock_guard<std::mutex> lock(payloadMutex_);
  // A CNN worker may still own this frame. It is joined before the next keyframe frontend work.
  if (classificationInProgress_ || (network_ && !isClassified_)) return false;
  if (!payloadResident_) return true;
  if (payloadDirty_ || payloadArchivePath_ != filename) {
    const std::string temporary = filename + ".tmp";
    try {
      std::ofstream out;
      out.exceptions(std::ios::failbit | std::ios::badbit);
      out.open(temporary, std::ios::binary | std::ios::trunc);
      put(out, kMagic);
      putMat(out, image_); putMat(out, depthImage_); putMat(out, descriptors_); putMat(out, classifications_);
      putVector(out, keypoints_); putVector(out, backProjections_); putBools(out, backProjectionsValid_);
      putVector(out, landmarkIds_); putVector(out, landmarks_); putBools(out, landmarkInitialisations_);
      put(out, uint8_t(isClassified_));
      put(out, kMagic);
      out.flush(); out.close();
      syncFile(temporary);
      std::filesystem::rename(temporary, filename);
      // Persist rename before releasing the only in-memory copy.
      const auto parent = std::filesystem::path(filename).parent_path();
      syncFile(parent.empty() ? "." : parent.string());
    } catch (const std::exception& error) {
      std::error_code ignored;
      std::filesystem::remove(temporary, ignored);
      throw std::runtime_error("Cannot archive frame payload " + filename + ": " + error.what());
    }
    payloadArchivePath_ = filename;
    payloadDirty_ = false;
    archivedNumKeypoints_ = keypoints_.size();
  }
  if (release) {
    image_.release(); depthImage_.release(); descriptors_.release(); classifications_.release();
    decltype(keypoints_)().swap(keypoints_);
    decltype(backProjections_)().swap(backProjections_);
    decltype(backProjectionsValid_)().swap(backProjectionsValid_);
    decltype(landmarkIds_)().swap(landmarkIds_);
    decltype(landmarks_)().swap(landmarks_);
    decltype(landmarkInitialisations_)().swap(landmarkInitialisations_);
    payloadResident_ = false;
  }
  return true;
}

void Frame::ensurePayload() const {
  if (payloadResident_) return;
  std::lock_guard<std::mutex> lock(payloadMutex_);
  if (payloadResident_) return;
  // Load transactionally: incomplete/corrupt input never leaves half-initialised feature vectors.
  Frame loaded;
  try {
    std::ifstream in;
    in.exceptions(std::ios::failbit | std::ios::badbit);
    in.open(payloadArchivePath_, std::ios::binary);
    uint64_t magic;
    get(in, magic);
    if (magic != kMagic) throw std::runtime_error("Invalid frame archive header");
    getMat(in, loaded.image_); getMat(in, loaded.depthImage_);
    getMat(in, loaded.descriptors_); getMat(in, loaded.classifications_);
    getVector(in, loaded.keypoints_); getVector(in, loaded.backProjections_);
    getBools(in, loaded.backProjectionsValid_); getVector(in, loaded.landmarkIds_);
    getVector(in, loaded.landmarks_); getBools(in, loaded.landmarkInitialisations_);
    uint8_t classified; get(in, classified); get(in, magic);
    const size_t n = loaded.keypoints_.size();
    if (magic != kMagic || classified > 1 || bool(classified) != isClassified_.load() ||
        n != archivedNumKeypoints_ ||
        loaded.landmarkIds_.size() != n || loaded.landmarks_.size() != n ||
        loaded.landmarkInitialisations_.size() != n ||
        loaded.backProjections_.size() != loaded.backProjectionsValid_.size() ||
        (!loaded.backProjections_.empty() && loaded.backProjections_.size() != n) ||
        (!loaded.descriptors_.empty() &&
         (size_t(loaded.descriptors_.rows) != n || loaded.descriptors_.type() != CV_8UC1)) ||
        (classified && n &&
         (size_t(loaded.classifications_.rows) != n || loaded.classifications_.cols != 19 ||
          loaded.classifications_.type() != CV_32FC1)))
      throw std::runtime_error("Inconsistent frame archive payload");
    if (in.rdbuf()->sgetc() != std::char_traits<char>::eof())
      throw std::runtime_error("Trailing data in frame archive");
    image_ = std::move(loaded.image_); depthImage_ = std::move(loaded.depthImage_);
    descriptors_ = std::move(loaded.descriptors_); classifications_ = std::move(loaded.classifications_);
    keypoints_.swap(loaded.keypoints_); backProjections_.swap(loaded.backProjections_);
    backProjectionsValid_.swap(loaded.backProjectionsValid_); landmarkIds_.swap(loaded.landmarkIds_);
    landmarks_.swap(loaded.landmarks_); landmarkInitialisations_.swap(loaded.landmarkInitialisations_);
    payloadResident_ = true;
    if (payloadLoadedCallback_) payloadLoadedCallback_();
  } catch (const std::exception& error) {
    throw std::runtime_error("Cannot reload historical frame " + payloadArchivePath_ + ": " + error.what());
  }
}
} // namespace okvis
