// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/ceres/TwoPoseGraphError.hpp>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <limits>
#include <fcntl.h>
#include <unistd.h>

namespace okvis { namespace ceres {
namespace {
constexpr uint64_t kMagic = 0x4f4b565450473031ULL; // OKVTPG01, same-build session archive.
constexpr uint64_t kMaxItems = 10000000;
template<class T> void put(std::ostream& out, const T& value) {
  out.write(reinterpret_cast<const char*>(&value), sizeof(value));
}
template<class T> void get(std::istream& in, T& value) {
  in.read(reinterpret_cast<char*>(&value), sizeof(value));
}
template<class M> void putEigen(std::ostream& out, const M& matrix) {
  out.write(reinterpret_cast<const char*>(matrix.data()), matrix.size() * sizeof(double));
}
template<class M> void getEigen(std::istream& in, M& matrix) {
  in.read(reinterpret_cast<char*>(matrix.data()), matrix.size() * sizeof(double));
}
uint64_t count(std::istream& in) {
  uint64_t value; get(in, value);
  if (value > kMaxItems) throw std::runtime_error("Oversized pose-graph recovery payload");
  return value;
}
void sync(const std::string& file) {
  const int fd = ::open(file.c_str(), O_RDONLY);
  if (fd < 0) throw std::runtime_error("Cannot open pose-graph archive for sync");
  const int result = ::fsync(fd); ::close(fd);
  if (result != 0) throw std::runtime_error("Cannot sync pose-graph archive");
}
}

size_t TwoPoseGraphError::residentObservationBytes() const {
  if (!observationsResident_) return 0;
  size_t result = landmarkParameterBlockInfos_.capacity() * sizeof(ParameterBlockInfo<4>) +
      landmarks_.size() * (sizeof(Eigen::Vector4d) + sizeof(uint64_t) + 4 * sizeof(void*));
  for (const auto& group : observations_)
    result += group.second.capacity() * (sizeof(Observation) + 256) + 4 * sizeof(void*);
  return result;
}

bool TwoPoseGraphError::archiveObservations(const std::string& filename) {
  std::lock_guard<std::mutex> lock(observationArchiveMutex_);
  if (!isComputed_) return false;
  if (!observationsResident_) return true;
  if (observations_.empty()) return true;
  std::vector<std::shared_ptr<ReprojectionError2dBase>> prototypes;
  std::vector<::ceres::LossFunction*> losses;
  std::map<uint64_t, std::shared_ptr<HomogeneousPointParameterBlock>> points;
  for (const auto& group : observations_) {
    for (const auto& observation : group.second) points.emplace(observation.hPoint->id(), observation.hPoint);
  }
  for (const auto& info : landmarkParameterBlockInfos_) {
    auto point = std::static_pointer_cast<HomogeneousPointParameterBlock>(info.parameterBlock);
    points.emplace(point->id(), point);
  }
  const std::string temporary = filename + ".tmp";
  try {
    std::ofstream out;
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out.open(temporary, std::ios::binary | std::ios::trunc);
    put(out, kMagic);
    put(out, referencePoseId_.value()); put(out, otherPoseId_.value());
    put(out, uint64_t(points.size()));
    for (const auto& point : points) {
      put(out, point.first); putEigen(out, point.second->estimate());
      put(out, uint8_t(point.second->initialized())); put(out, uint8_t(point.second->fixed()));
    }
    put(out, uint64_t(landmarkParameterBlockInfos_.size()));
    for (const auto& info : landmarkParameterBlockInfos_) {
      put(out, info.parameterBlock->id()); put(out, uint64_t(info.idx)); putEigen(out, info.parameters);
    }
    put(out, uint64_t(landmarks_.size()));
    for (const auto& point : landmarks_) { put(out, point.first); putEigen(out, point.second); }
    put(out, uint64_t(landmarkParameterBlockId2idx_.size()));
    for (const auto& entry : landmarkParameterBlockId2idx_) {
      put(out, entry.first); put(out, uint64_t(entry.second));
    }
    put(out, uint64_t(observations_.size()));
    for (const auto& group : observations_) {
      put(out, group.first); put(out, uint64_t(group.second.size()));
      for (const auto& observation : group.second) {
        const auto& kid = observation.keypointIdentifier;
        put(out, kid.frameId); put(out, uint64_t(kid.cameraIndex)); put(out, uint64_t(kid.keypointIndex));
        size_t side = 0;
        if (observation.pose == poseParameterBlockInfos_[1].parameterBlock) side = 1;
        else if (observation.pose != poseParameterBlockInfos_[0].parameterBlock)
          throw std::runtime_error("Unknown pose identity in pose-graph observation");
        put(out, uint64_t(side));
        size_t extrinsic = 0;
        while (extrinsic < extrinsicsParameterBlockInfos_.size() &&
            extrinsicsParameterBlockInfos_[extrinsic].parameterBlock != observation.extrinsics) ++extrinsic;
        if (extrinsic == extrinsicsParameterBlockInfos_.size())
          throw std::runtime_error("Unknown extrinsics identity in pose-graph observation");
        put(out, uint64_t(extrinsic)); put(out, observation.hPoint->id());
        auto prototype = std::find_if(prototypes.begin(), prototypes.end(), [&](const auto& candidate) {
          return candidate->cameraId() == observation.reprojectionError->cameraId();
        });
        if (prototype == prototypes.end()) {
          prototypes.push_back(observation.reprojectionError->clone()); prototype = prototypes.end() - 1;
        }
        put(out, uint64_t(prototype - prototypes.begin()));
        putEigen(out, observation.reprojectionError->measurement());
        putEigen(out, observation.reprojectionError->information());
        auto loss = std::find(losses.begin(), losses.end(), observation.lossFunction);
        if (loss == losses.end()) { losses.push_back(observation.lossFunction); loss = losses.end() - 1; }
        put(out, uint64_t(loss - losses.begin()));
        put(out, uint8_t(observation.isMarginalised)); put(out, uint8_t(observation.isDuplication));
      }
    }
    put(out, kMagic); out.flush(); out.close(); sync(temporary);
    std::filesystem::rename(temporary, filename);
    const auto parent = std::filesystem::path(filename).parent_path();
    sync(parent.empty() ? "." : parent.string());
  } catch (const std::exception& error) {
    std::error_code ignored; std::filesystem::remove(temporary, ignored);
    throw std::runtime_error("Cannot archive visual pose-graph recovery data " + filename + ": " + error.what());
  }
  observationPrototypes_.swap(prototypes);
  observationLossFunctions_.swap(losses);
  observationArchivePath_ = filename;
  // Pose/extrinsics and the linearised factor matrices are retained. Everything required to
  // reactivate original visual observations is recoverable from this completed archive.
  decltype(observations_)().swap(observations_);
  decltype(landmarks_)().swap(landmarks_);
  decltype(landmarkParameterBlockInfos_)().swap(landmarkParameterBlockInfos_);
  decltype(landmarkParameterBlockId2idx_)().swap(landmarkParameterBlockId2idx_);
  observationsResident_ = false;
  return true;
}

void TwoPoseGraphError::restoreObservations() {
  if (observationsResident_) return;
  std::lock_guard<std::mutex> lock(observationArchiveMutex_);
  if (observationsResident_) return;
  decltype(observations_) observations;
  decltype(landmarks_) landmarks;
  decltype(landmarkParameterBlockInfos_) infos;
  decltype(landmarkParameterBlockId2idx_) indices;
  std::map<uint64_t, std::shared_ptr<HomogeneousPointParameterBlock>> points;
  try {
    std::ifstream in;
    in.exceptions(std::ios::failbit | std::ios::badbit);
    in.open(observationArchivePath_, std::ios::binary);
    uint64_t magic, reference, other;
    get(in, magic); get(in, reference); get(in, other);
    if (magic != kMagic || reference != referencePoseId_.value() || other != otherPoseId_.value())
      throw std::runtime_error("Mismatched pose-graph archive header");
    for (uint64_t n = count(in); n; --n) {
      uint64_t id; uint8_t initialised, fixed; Eigen::Vector4d estimate;
      get(in, id); getEigen(in, estimate); get(in, initialised); get(in, fixed);
      auto point = std::make_shared<HomogeneousPointParameterBlock>(estimate, id, initialised != 0);
      point->setFixed(fixed != 0);
      if (!points.emplace(id, point).second) throw std::runtime_error("Duplicate archived landmark ID");
    }
    for (uint64_t n = count(in); n; --n) {
      uint64_t id, index; ParameterBlockInfo<4> info;
      get(in, id); get(in, index); getEigen(in, info.parameters);
      info.parameterBlock = points.at(id); info.idx = index; infos.push_back(info);
    }
    for (uint64_t n = count(in); n; --n) {
      uint64_t id; Eigen::Vector4d position; get(in, id); getEigen(in, position); landmarks.emplace(id, position);
    }
    for (uint64_t n = count(in); n; --n) {
      uint64_t id, index; get(in, id); get(in, index);
      if (index >= infos.size()) throw std::runtime_error("Invalid archived landmark parameter index");
      indices.emplace(id, index);
    }
    uint64_t total = 0;
    for (uint64_t n = count(in); n; --n) {
      uint64_t group; get(in, group);
      auto& values = observations[group];
      for (uint64_t m = count(in); m; --m) {
        if (++total > kMaxItems) throw std::runtime_error("Oversized archived observations");
        uint64_t frame, camera, keypoint, side, extrinsic, point, prototype, loss;
        uint8_t marginalised, duplicate;
        Eigen::Vector2d measurement; Eigen::Matrix2d information;
        get(in, frame); get(in, camera); get(in, keypoint); get(in, side); get(in, extrinsic);
        get(in, point); get(in, prototype); getEigen(in, measurement); getEigen(in, information);
        get(in, loss); get(in, marginalised); get(in, duplicate);
        if (side > 1 || extrinsic >= extrinsicsParameterBlockInfos_.size() || group != point ||
            prototype >= observationPrototypes_.size() || loss >= observationLossFunctions_.size())
          throw std::runtime_error("Invalid archived observation reference");
        auto error = observationPrototypes_.at(prototype)->clone();
        if (error->cameraId() != camera) throw std::runtime_error("Mismatched archived observation camera");
        error->setMeasurement(measurement); error->setInformation(information);
        values.push_back(Observation{
            KeypointIdentifier(frame, camera, keypoint),
            std::static_pointer_cast<PoseParameterBlock>(poseParameterBlockInfos_[side].parameterBlock),
            points.at(point),
            std::static_pointer_cast<PoseParameterBlock>(extrinsicsParameterBlockInfos_[extrinsic].parameterBlock),
            error, observationLossFunctions_.at(loss), marginalised != 0, duplicate != 0});
      }
    }
    get(in, magic);
    if (magic != kMagic) throw std::runtime_error("Truncated pose-graph archive");
  } catch (const std::exception& error) {
    throw std::runtime_error("Cannot restore visual pose-graph recovery data " + observationArchivePath_ + ": " + error.what());
  }
  observations_.swap(observations); landmarks_.swap(landmarks);
  landmarkParameterBlockInfos_.swap(infos); landmarkParameterBlockId2idx_.swap(indices);
  observationsResident_ = true;
}
}} // namespace okvis::ceres
