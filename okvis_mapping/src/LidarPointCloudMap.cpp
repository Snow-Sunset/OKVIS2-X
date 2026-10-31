// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/LidarPointCloudMap.hpp>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace okvis {
namespace {
namespace fs = std::filesystem;
constexpr size_t kArchiveHeaderBytes = 16;
constexpr size_t kArchivePointBytes = 3 * sizeof(float);
const char kArchiveMagic[8] = {'O','K','L','X','Y','Z','0','1'};
std::atomic<uint64_t> serial{0};

std::string uniqueName(const std::string& prefix) {
  return prefix + std::to_string(::getpid()) + "-" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "-" +
      std::to_string(serial.fetch_add(1));
}

[[noreturn]] void ioError(const std::string& operation, const std::string& filename) {
  throw std::runtime_error(operation + ": " + filename);
}

void syncFile(const fs::path& filename) {
  const int fd = ::open(filename.c_str(), O_RDONLY);
  if (fd < 0) ioError("Cannot open archive for synchronization", filename.string());
  const int result = ::fsync(fd);
  ::close(fd);
  if (result != 0) ioError("Cannot synchronize archive", filename.string());
}

void atomicWrite(const fs::path& filename, const std::function<void(std::ofstream&)>& write) {
  const fs::path temporary = filename.string() + "." + uniqueName("tmp-");
  try {
    std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
    if (!stream) ioError("Cannot create file", temporary.string());
    write(stream);
    stream.flush();
    if (!stream) ioError("Cannot write file", temporary.string());
    stream.close();
    if (!stream) ioError("Cannot close file", temporary.string());
    syncFile(temporary);
    fs::rename(temporary, filename);
    // Make the directory entry durable as well as the payload.
    const fs::path parent = filename.has_parent_path() ? filename.parent_path() : fs::path(".");
    const int directoryFd = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY);
    if (directoryFd < 0) ioError("Cannot open archive directory", parent.string());
    const int result = ::fsync(directoryFd);
    ::close(directoryFd);
    if (result != 0) ioError("Cannot synchronize archive directory", parent.string());
  } catch (...) {
    std::error_code ignored;
    fs::remove(temporary, ignored);
    throw;
  }
}

uint64_t pointCount(const LidarPointCloudMap::SubmapSnapshot& submap) {
  return submap.archive ? submap.archive->pointCount : (submap.points ? submap.points->size() : 0);
}

class MappedChunk {
 public:
  explicit MappedChunk(const LidarPointCloudMap::ArchiveChunk& chunk) : filename_(chunk.filename) {
    fd_ = ::open(filename_.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd_ < 0) ioError("Cannot open LiDAR archive", filename_);
    struct stat status {};
    if (::fstat(fd_, &status) != 0 || status.st_size < std::streamoff(kArchiveHeaderBytes)) {
      close();
      ioError("Cannot stat LiDAR archive", filename_);
    }
    size_ = static_cast<size_t>(status.st_size);
    data_ = ::mmap(nullptr, size_, PROT_READ, MAP_PRIVATE, fd_, 0);
    if (data_ == MAP_FAILED) {
      data_ = nullptr;
      close();
      ioError("Cannot map LiDAR archive", filename_);
    }
    uint64_t count = 0;
    std::memcpy(&count, bytes() + sizeof(kArchiveMagic), sizeof(count));
    const bool valid = std::memcmp(bytes(), kArchiveMagic, sizeof(kArchiveMagic)) == 0 &&
        count == chunk.pointCount &&
        count <= (std::numeric_limits<uint64_t>::max() - kArchiveHeaderBytes) / kArchivePointBytes &&
        size_ == kArchiveHeaderBytes + count * kArchivePointBytes;
    if (!valid) {
      unmap();
      close();
      ioError("Invalid or incomplete LiDAR archive", filename_);
    }
  }

  ~MappedChunk() { unmap(); close(); }
  MappedChunk(const MappedChunk&) = delete;
  MappedChunk& operator=(const MappedChunk&) = delete;

  Eigen::Vector3d point(uint64_t index) const {
    float xyz[3];
    std::memcpy(xyz, bytes() + kArchiveHeaderBytes + index * kArchivePointBytes, sizeof(xyz));
    return Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
  }

 private:
  const unsigned char* bytes() const { return static_cast<const unsigned char*>(data_); }
  void unmap() {
    if (data_) {
      ::munmap(data_, size_);
      data_ = nullptr;
    }
  }
  void close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }
  std::string filename_;
  int fd_ = -1;
  void* data_ = nullptr;
  size_t size_ = 0;
};

/// Memory-map immutable chunks so strided display sampling does not issue one
/// seek/read pair per displayed point. The kernel still pages only the touched
/// regions, while complete export remains a sequential traversal.
void visitPoints(const LidarPointCloudMap::Snapshot& snapshot, size_t stride,
                 const std::function<void(const Eigen::Vector3d&)>& visit) {
  uint64_t offset = 0, next = 0;
  for (const auto& submap : snapshot) {
    const uint64_t count = pointCount(submap);
    if (count > std::numeric_limits<uint64_t>::max() - offset) {
      throw std::overflow_error("LiDAR point count overflow");
    }
    const uint64_t end = offset + count;
    if (next < end) {
      if (submap.archive) {
        MappedChunk chunk(*submap.archive);
        while (next < end) {
          const uint64_t index = next - offset;
          visit(submap.pose * chunk.point(index));
          if (std::numeric_limits<uint64_t>::max() - next < stride) return;
          next += stride;
        }
      } else if (submap.points) {
        while (next < end) {
          visit(submap.pose * submap.points->at(next - offset));
          if (std::numeric_limits<uint64_t>::max() - next < stride) return;
          next += stride;
        }
      }
    }
    offset = end;
  }
}

struct ExportRecord {
  int64_t key[3];
  double xyz[3];
};
bool before(const ExportRecord& a, const ExportRecord& b) {
  return std::lexicographical_compare(a.key, a.key + 3, b.key, b.key + 3);
}
bool sameKey(const ExportRecord& a, const ExportRecord& b) {
  return std::equal(a.key, a.key + 3, b.key);
}
double centerDistance(const ExportRecord& record, double voxelSize) {
  double distance = 0;
  for (int axis = 0; axis < 3; ++axis) {
    const double delta = record.xyz[axis] - voxelSize * (double(record.key[axis]) + 0.5);
    distance += delta * delta;
  }
  return distance;
}
ExportRecord preferred(const ExportRecord& a, const ExportRecord& b, double voxelSize) {
  return centerDistance(b, voxelSize) < centerDistance(a, voxelSize) ? b : a;
}
bool readRecord(std::ifstream& stream, ExportRecord& record) {
  stream.read(reinterpret_cast<char*>(&record), sizeof(record));
  if (stream.gcount() == 0 && stream.eof()) return false;
  if (!stream) throw std::runtime_error("Truncated LiDAR export sort run");
  return true;
}
void writeRecord(std::ofstream& stream, const ExportRecord& record) {
  stream.write(reinterpret_cast<const char*>(&record), sizeof(record));
  if (!stream) throw std::runtime_error("Cannot write LiDAR export sort run");
}
} // namespace

LidarPointCloudMap::LidarPointCloudMap(double voxelSize) : voxelSize_(voxelSize) {
  if (!(voxelSize > 0.0) || !std::isfinite(voxelSize)) {
    throw std::invalid_argument("pointcloud_map_voxel_size must be finite and positive");
  }
}

LidarPointCloudMap::LidarPointCloudMap(double voxelSize, size_t memoryBudgetBytes)
    : LidarPointCloudMap(voxelSize) {
  if (memoryBudgetBytes < 4096) {
    throw std::invalid_argument("RAM-only LiDAR map requires >=4096 resident bytes");
  }
  memoryBudgetBytes_ = memoryBudgetBytes;
}

LidarPointCloudMap::LidarPointCloudMap(double voxelSize, const ArchiveOptions& archive)
    : LidarPointCloudMap(voxelSize) {
  if (archive.directory.empty() || archive.memoryBudgetBytes < 4096 || archive.chunkPointLimit == 0) {
    throw std::invalid_argument("LiDAR archive requires a directory, >=4096 resident bytes and positive chunk size");
  }
  memoryBudgetBytes_ = archive.memoryBudgetBytes;
  // Includes unordered-map nodes, buckets and the immutable snapshot copy.
  chunkPointLimit_ = std::min(archive.chunkPointLimit, std::max(size_t(1), memoryBudgetBytes_ / 256));
  fs::create_directories(archive.directory);
  archiveDirectory_ = (fs::path(archive.directory) / uniqueName("lidar-cloud-")).string();
  if (!fs::create_directory(archiveDirectory_)) ioError("Cannot create LiDAR archive", archiveDirectory_);
  writeManifest();
}

void LidarPointCloudMap::enableGlobalPreview(double voxelSize, size_t maxPoints) {
  if (!(voxelSize > 0.0) || !std::isfinite(voxelSize) || !maxPoints) {
    throw std::invalid_argument("Global LiDAR preview requires a finite positive voxel size and point cap");
  }
  if (!submaps_.empty() || !previewSubmaps_.empty()) {
    throw std::logic_error("Global LiDAR preview must be enabled before inserting points");
  }
  globalPreviewEnabled_ = true;
  globalPreviewVoxelSize_ = voxelSize;
  globalPreviewMaxPoints_ = maxPoints;
}

size_t LidarPointCloudMap::Hash::operator()(const Key& key) const {
  size_t seed = 0;
  for (const auto value : key) {
    seed ^= std::hash<int64_t>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }
  return seed;
}

LidarPointCloudMap::Key LidarPointCloudMap::key(const Eigen::Vector3d& point) const {
  return key(point, voxelSize_);
}

LidarPointCloudMap::Key LidarPointCloudMap::key(
    const Eigen::Vector3d& point, double voxelSize) const {
  return {{int64_t(std::floor(point.x() / voxelSize)),
           int64_t(std::floor(point.y() / voxelSize)),
           int64_t(std::floor(point.z() / voxelSize))}};
}

bool LidarPointCloudMap::insideLocalWindow(const Eigen::Vector3d& world) const {
  return localWindowEnabled_ && world.allFinite()
      && (world - localWindowCenter_).cwiseAbs().maxCoeff() <= localWindowHalfExtent_;
}

bool LidarPointCloudMap::insert(Voxels& voxels, const Eigen::Vector3d& point) const {
  return insert(voxels, point, voxelSize_);
}

bool LidarPointCloudMap::insert(
    Voxels& voxels, const Eigen::Vector3d& point, double voxelSize) const {
  if (!point.allFinite()) return false;
  for (int axis = 0; axis < 3; ++axis) {
    const double cell = std::floor(point[axis] / voxelSize);
    if (!std::isfinite(cell) || cell < double(std::numeric_limits<int64_t>::min()) ||
        cell >= double(std::numeric_limits<int64_t>::max())) return false;
  }
  const auto index = key(point, voxelSize);
  const auto result = voxels.emplace(index, point);
  if (result.second) return true;
  const Eigen::Vector3d center = voxelSize * Eigen::Vector3d(
      double(index[0]) + 0.5, double(index[1]) + 0.5, double(index[2]) + 0.5);
  if ((point - center).squaredNorm() < (result.first->second - center).squaredNorm()) {
    result.first->second = point;
    return true;
  }
  return false;
}

void LidarPointCloudMap::updatePose(uint64_t id, const Eigen::Isometry3d& pose,
                                   bool fromEstimator, uint64_t estimatorSequence) {
  if (!pose.matrix().allFinite()) return;
  auto local = submaps_.find(id);
  auto preview = previewSubmaps_.find(id);
  // Preserve the historical behaviour for new anchors, but do not recreate a
  // high-resolution local submap when only its global preview remains.
  if (local == submaps_.end() && preview == previewSubmaps_.end()) {
    local = submaps_.emplace(id, Submap()).first;
  }
  auto apply = [&](Submap& submap, bool localMap) {
    if (submap.estimatorPose && !fromEstimator) return false;
    if (fromEstimator && estimatorSequence < submap.estimatorSequence) return false;
    if (fromEstimator) submap.estimatorSequence = estimatorSequence;
    submap.estimatorPose = submap.estimatorPose || fromEstimator;
    if ((submap.pose.matrix() - pose.matrix()).cwiseAbs().maxCoeff() <= 1e-12) return false;
    const Eigen::Isometry3d previousPose = submap.pose;
    submap.pose = pose;
    // A large pose correction can move every local voxel. Small odometry
    // updates are absorbed by the normal window trigger so the map is not
    // rescanned once per LiDAR batch.
    if (localMap && localWindowEnabled_ && archiveDirectory_.empty()) {
      const double poseShift = (previousPose.translation() - pose.translation()).norm()
          + (previousPose.linear() - pose.linear()).norm() * localWindowHalfExtent_;
      const double sweepTrigger = std::max(voxelSize_, std::min(2.0, 0.1 * localWindowHalfExtent_));
      if (poseShift >= sweepTrigger) localWindowSweepInitialized_ = false;
    }
    return !submap.voxels.empty() || !submap.chunks.empty();
  };
  if (local != submaps_.end() && apply(local->second, true)) ++revision_;
  if (preview != previewSubmaps_.end() && apply(preview->second, false)) ++previewRevision_;
}

void LidarPointCloudMap::setLocalWindow(const Eigen::Vector3d& center, double halfExtent) {
  if (!center.allFinite() || !std::isfinite(halfExtent) || halfExtent <= 0.0) {
    throw std::invalid_argument("LiDAR local window requires a finite positive extent");
  }
  localWindowEnabled_ = true;
  localWindowCenter_ = center;
  localWindowHalfExtent_ = halfExtent;
  // Window centers advance continuously with the platform. Avoid rescanning
  // every voxel for sub-voxel motion; new insertions are filtered immediately,
  // while a bounded sweep margin amortizes cleanup over several updates.
  const double centerShift = localWindowSweepInitialized_
      ? (center - localWindowSweepCenter_).cwiseAbs().maxCoeff()
      : std::numeric_limits<double>::infinity();
  const double extentShift = localWindowSweepInitialized_
      ? std::abs(halfExtent - localWindowSweepHalfExtent_)
      : std::numeric_limits<double>::infinity();
  const double sweepTrigger = std::max(voxelSize_, std::min(2.0, 0.1 * halfExtent));
  const bool windowShrank = localWindowSweepInitialized_ && halfExtent < localWindowSweepHalfExtent_;
  if (localWindowSweepInitialized_ && centerShift < sweepTrigger
      && extentShift < voxelSize_ && !windowShrank) return;

  bool changed = false;
  // Persistent chunks are intentionally left intact: enabling lossless replay
  // must keep the historical source needed for delayed pose corrections. The
  // default RAM-only map has no chunks, so old voxels are reclaimed here.
  for (auto it = submaps_.begin(); it != submaps_.end();) {
    auto& submap = it->second;
    if (!submap.chunks.empty()) {
      ++it;
      continue;
    }
    bool submapChanged = false;
    for (auto voxel = submap.voxels.begin(); voxel != submap.voxels.end();) {
      const Eigen::Vector3d world = submap.pose * voxel->second;
      if (!insideLocalWindow(world)) {
        voxel = submap.voxels.erase(voxel);
        submapChanged = true;
        changed = true;
      } else {
        ++voxel;
      }
    }
    if (submap.voxels.empty()) {
      submap.cachedPoints.reset();
      // An empty anchor is recreated by addPoints/updatePose when it becomes
      // active again; removing it avoids retaining a growing submap index.
      it = submaps_.erase(it);
      changed = true;
    } else {
      if (submapChanged) submap.cachedPoints.reset();
      ++it;
    }
  }
  localWindowSweepCenter_ = center;
  localWindowSweepHalfExtent_ = halfExtent;
  localWindowSweepInitialized_ = true;
  if (changed) ++revision_;
}

void LidarPointCloudMap::addPreviewPoints(
    uint64_t id, const Submap& poseSource, const Points& points) {
  if (!globalPreviewEnabled_) return;
  auto inserted = previewSubmaps_.emplace(id, Submap());
  auto& preview = inserted.first->second;
  if (inserted.second) {
    preview.pose = poseSource.pose;
    preview.estimatorPose = poseSource.estimatorPose;
    preview.estimatorSequence = poseSource.estimatorSequence;
  }
  bool changed = false;
  for (const auto& point : points) {
    const size_t before = preview.voxels.size();
    changed = insert(preview.voxels, point, globalPreviewVoxelSize_) || changed;
    if (preview.voxels.size() > before) ++globalPreviewPointCount_;
  }
  if (changed) {
    preview.cachedPoints.reset();
    ++previewRevision_;
  }
  enforcePreviewLimit();
}

void LidarPointCloudMap::enforcePreviewLimit() {
  if (!globalPreviewEnabled_ || globalPreviewPointCount_ <= globalPreviewMaxPoints_) return;
  const size_t targetPointCount = std::max<size_t>(
      1, globalPreviewMaxPoints_ - globalPreviewMaxPoints_ / 5);
  while (globalPreviewPointCount_ > targetPointCount) {
    const size_t previousPointCount = globalPreviewPointCount_;
    const double coarserVoxelSize = 2.0 * globalPreviewVoxelSize_;
    if (!std::isfinite(coarserVoxelSize)) {
      throw std::overflow_error("Global LiDAR preview voxel size overflow");
    }
    size_t coarsenedPointCount = 0;
    for (auto& entry : previewSubmaps_) {
      Voxels coarsened;
      coarsened.reserve(std::max<size_t>(1, entry.second.voxels.size() / 2));
      for (const auto& voxel : entry.second.voxels) {
        insert(coarsened, voxel.second, coarserVoxelSize);
      }
      entry.second.voxels.swap(coarsened);
      entry.second.cachedPoints.reset();
      coarsenedPointCount += entry.second.voxels.size();
    }
    globalPreviewVoxelSize_ = coarserVoxelSize;
    globalPreviewPointCount_ = coarsenedPointCount;
    // Extremely sparse submaps may not merge even after a voxel-size step.
    // Preserve at least one sample per historical submap when possible, then
    // trim only the excess needed to maintain the hard visualization cap.
    if (globalPreviewPointCount_ >= previousPointCount
        && globalPreviewPointCount_ > targetPointCount) {
      const size_t minimumPerSubmap = previewSubmaps_.size() <= targetPointCount ? 1 : 0;
      for (auto& entry : previewSubmaps_) {
        while (globalPreviewPointCount_ > targetPointCount
            && entry.second.voxels.size() > minimumPerSubmap) {
          entry.second.voxels.erase(entry.second.voxels.begin());
          --globalPreviewPointCount_;
        }
        entry.second.cachedPoints.reset();
        if (globalPreviewPointCount_ <= targetPointCount) break;
      }
    }
    ++previewRevision_;
  }
}

void LidarPointCloudMap::addPoints(uint64_t id, const Eigen::Isometry3d& initialPose,
                                  const Points& points) {
  if (!initialPose.matrix().allFinite()) return;
  if (submaps_.count(id) == 0) {
    const auto preview = previewSubmaps_.find(id);
    if (preview != previewSubmaps_.end()) {
      Submap local;
      local.pose = preview->second.pose;
      local.estimatorPose = preview->second.estimatorPose;
      local.estimatorSequence = preview->second.estimatorSequence;
      submaps_.emplace(id, std::move(local));
    }
  }
  updatePose(id, initialPose);
  auto& submap = submaps_.at(id);
  addPreviewPoints(id, submap, points);
  bool changed = false;
  try {
    for (const auto& point : points) {
      if (localWindowEnabled_ && archiveDirectory_.empty()
          && !insideLocalWindow(submap.pose * point)) continue;
      changed = insert(submap.voxels, point) || changed;
      if (!archiveDirectory_.empty() && submap.voxels.size() >= chunkPointLimit_) {
        archiveSubmap(id, submap);
      }
    }
  } catch (...) {
    // Insertion may already have succeeded when archiving fails. Invalidate
    // snapshots and retain those points so the caller can recover/retry.
    if (changed) { submap.cachedPoints.reset(); ++revision_; }
    throw;
  }
  if (changed) {
    submap.cachedPoints.reset();
    ++revision_;
  }
  enforceBudget(id);
  if (localWindowEnabled_ && archiveDirectory_.empty()) {
    const auto empty = submaps_.find(id);
    if (empty != submaps_.end() && empty->second.voxels.empty() && empty->second.chunks.empty()) {
      submaps_.erase(empty);
    }
  }
}

size_t LidarPointCloudMap::residentBytes() const {
  size_t bytes = 0;
  const auto addSubmaps = [&](const std::map<uint64_t, Submap>& submaps) {
    for (const auto& entry : submaps) {
      const auto& submap = entry.second;
      // Reserve the immutable snapshot copy even before it has been requested,
      // excluding small history metadata and snapshots retained by callers after
      // the corresponding resident cache is released.
      bytes += submap.voxels.size() * 128 + submap.voxels.bucket_count() * sizeof(void*);
      bytes += (submap.cachedPoints ? submap.cachedPoints->capacity() : submap.voxels.size())
          * sizeof(Eigen::Vector3d);
    }
  };
  addSubmaps(submaps_);
  addSubmaps(previewSubmaps_);
  return bytes;
}

uint64_t LidarPointCloudMap::archivedPointCount() const {
  uint64_t count = 0;
  for (const auto& entry : submaps_) for (const auto& chunk : entry.second.chunks) count += chunk->pointCount;
  return count;
}

void LidarPointCloudMap::writeManifest() const {
  if (archiveDirectory_.empty()) return;
  atomicWrite(fs::path(archiveDirectory_) / "manifest.txt", [&](std::ofstream& out) {
    out << "OKVIS_LIDAR_ARCHIVE 1\nvoxel_size " << std::setprecision(17) << voxelSize_ << '\n';
    for (const auto& entry : submaps_) {
      out << "submap " << entry.first << ' ' << entry.second.estimatorPose << ' ' << entry.second.estimatorSequence;
      for (int row = 0; row < 4; ++row) for (int col = 0; col < 4; ++col) out << ' ' << entry.second.pose.matrix()(row, col);
      out << '\n';
      for (const auto& chunk : entry.second.chunks) {
        out << "chunk " << entry.first << ' ' << fs::path(chunk->filename).filename().string() << ' ' << chunk->pointCount << '\n';
      }
    }
  });
}

void LidarPointCloudMap::writeSubmapManifest(uint64_t id, const Submap& submap) const {
  if (archiveDirectory_.empty()) return;
  atomicWrite(fs::path(archiveDirectory_) / ("submap-" + std::to_string(id) + ".txt"), [&](std::ofstream& out) {
    // These per-anchor manifests are authoritative between complete checkpoints.
    // New submaps do not rewrite an ever-growing global index on every chunk.
    out << "OKVIS_LIDAR_SUBMAP 1\nvoxel_size " << std::setprecision(17) << voxelSize_ << '\n';
    out << "submap " << id << ' ' << submap.estimatorPose << ' ' << submap.estimatorSequence;
    for (int row = 0; row < 4; ++row) for (int col = 0; col < 4; ++col) out << ' ' << submap.pose.matrix()(row, col);
    out << '\n';
    for (const auto& chunk : submap.chunks) {
      out << "chunk " << id << ' ' << fs::path(chunk->filename).filename().string() << ' ' << chunk->pointCount << '\n';
    }
  });
}

void LidarPointCloudMap::archiveSubmap(uint64_t id, Submap& submap) {
  if (archiveDirectory_.empty() || submap.voxels.empty()) return;
  auto chunk = std::make_shared<ArchiveChunk>();
  chunk->filename = (fs::path(archiveDirectory_) / (std::to_string(id) + "-" + std::to_string(nextChunk_++) + ".xyz")).string();
  chunk->pointCount = submap.voxels.size();
  atomicWrite(chunk->filename, [&](std::ofstream& out) {
    out.write(kArchiveMagic, sizeof(kArchiveMagic));
    out.write(reinterpret_cast<const char*>(&chunk->pointCount), sizeof(chunk->pointCount));
    for (const auto& point : submap.voxels) {
      const Eigen::Vector3f xyz = point.second.cast<float>();
      if (!xyz.allFinite()) throw std::runtime_error("LiDAR point cannot be represented by local float XYZ");
      out.write(reinterpret_cast<const char*>(xyz.data()), kArchivePointBytes);
    }
  });
  submap.chunks.push_back(chunk);
  try {
    // The manifest and chunk are durable before the only resident copy is freed.
    writeSubmapManifest(id, submap);
  } catch (...) {
    submap.chunks.pop_back();
    // Keep the durable chunk as well as resident data: a rename may have
    // succeeded before a directory fsync failed. A manifest must never refer
    // to a chunk that rollback has removed.
    throw;
  }
  submap.cachedPoints.reset();
  Voxels().swap(submap.voxels);
}

void LidarPointCloudMap::enforceBudget(uint64_t activeId) {
  if (archiveDirectory_.empty()) {
    enforceResidentBudget(activeId);
    return;
  }
  if (residentBytes() <= memoryBudgetBytes_) return;
  for (auto& entry : submaps_) {
    if (entry.first == activeId || entry.second.voxels.empty()) continue;
    archiveSubmap(entry.first, entry.second);
    if (residentBytes() <= memoryBudgetBytes_) return;
  }
  const auto active = submaps_.find(activeId);
  if (active != submaps_.end()) archiveSubmap(activeId, active->second);
}

void LidarPointCloudMap::enforceResidentBudget(uint64_t activeId) {
  if (!memoryBudgetBytes_) return;
  // A cached immutable copy is only a convenience for snapshots. Release it
  // before trimming so the resident budget reflects the actual map payload.
  for (auto& entry : submaps_) entry.second.cachedPoints.reset();
  for (auto& entry : previewSubmaps_) entry.second.cachedPoints.reset();
  if (residentBytes() <= memoryBudgetBytes_) return;

  // Prefer dropping complete old submaps. This keeps the active local map
  // useful and avoids repeatedly rehashing a large unordered_map.
  for (auto& entry : submaps_) {
    if (entry.first == activeId || entry.second.voxels.empty()) continue;
    Voxels().swap(entry.second.voxels);
    entry.second.cachedPoints.reset();
    if (residentBytes() <= memoryBudgetBytes_) return;
  }

  auto active = submaps_.find(activeId);
  if (active == submaps_.end() || active->second.voxels.empty()) return;
  // Include an estimate for the hash bucket/pointer overhead as well as the
  // node and cached XYZ payload. The final check below handles allocator
  // rounding for very small budgets.
  constexpr size_t estimatedVoxelBytes = 128 + sizeof(Eigen::Vector3d) + sizeof(void*);
  const size_t target = std::max<size_t>(1, memoryBudgetBytes_ / estimatedVoxelBytes);
  if (active->second.voxels.size() > target) {
    size_t remove = active->second.voxels.size() - target;
    auto voxel = active->second.voxels.begin();
    while (remove-- && voxel != active->second.voxels.end())
      voxel = active->second.voxels.erase(voxel);
    active->second.cachedPoints.reset();
  }
  while (residentBytes() > memoryBudgetBytes_ && !active->second.voxels.empty()) {
    active->second.voxels.erase(active->second.voxels.begin());
  }
  if (active->second.voxels.empty()) {
    Voxels().swap(active->second.voxels);
  } else if (active->second.voxels.bucket_count() > 2 * active->second.voxels.size()) {
    active->second.voxels.rehash(0);
  }
}

void LidarPointCloudMap::finalizeSubmap(uint64_t id) {
  auto entry = submaps_.find(id);
  if (entry != submaps_.end()) {
    const size_t chunksBefore = entry->second.chunks.size();
    archiveSubmap(id, entry->second);
    // archiveSubmap already committed the updated manifest. Only refresh it
    // when there was no resident payload to archive (for example a pose-only
    // finalisation).
    if (entry->second.chunks.size() == chunksBefore) writeSubmapManifest(id, entry->second);
  }
}

void LidarPointCloudMap::flush() {
  for (auto& entry : submaps_) archiveSubmap(entry.first, entry.second);
  writeManifest();
}

LidarPointCloudMap::Snapshot LidarPointCloudMap::snapshot(
    const std::map<uint64_t, Submap>& submaps) const {
  Snapshot result;
  for (const auto& entry : submaps) {
    const auto& submap = entry.second;
    for (const auto& chunk : submap.chunks) result.push_back({submap.pose, {}, chunk});
    if (submap.voxels.empty()) continue;
    if (!submap.cachedPoints) {
      auto points = std::make_shared<Points>();
      points->reserve(submap.voxels.size());
      for (const auto& voxel : submap.voxels) points->push_back(voxel.second);
      submap.cachedPoints = std::move(points);
    }
    result.push_back({submap.pose, submap.cachedPoints, {}});
  }
  return result;
}

LidarPointCloudMap::Snapshot LidarPointCloudMap::snapshot() const {
  return snapshot(submaps_);
}

LidarPointCloudMap::Snapshot LidarPointCloudMap::visualizationSnapshot() const {
  return globalPreviewEnabled_ ? snapshot(previewSubmaps_) : snapshot(submaps_);
}

LidarPointCloudMap::Points LidarPointCloudMap::assemble(const Snapshot& snapshot,
    double voxelSize, size_t maxPoints) {
  LidarPointCloudMap sampler(voxelSize);
  uint64_t total = 0;
  for (const auto& submap : snapshot) {
    if (pointCount(submap) > std::numeric_limits<uint64_t>::max() - total) throw std::overflow_error("LiDAR point count overflow");
    total += pointCount(submap);
  }
  const size_t stride = maxPoints && total > maxPoints ? 1 + (total - 1) / maxPoints : 1;
  Voxels world;
  world.reserve(stride == 1 ? total : 1 + (total - 1) / stride);
  visitPoints(snapshot, stride, [&](const Eigen::Vector3d& point) { sampler.insert(world, point); });
  Points points;
  points.reserve(world.size());
  for (const auto& voxel : world) points.push_back(voxel.second);
  return points;
}

LidarPointCloudMap::Points LidarPointCloudMap::assembleUnfiltered(const Snapshot& snapshot) {
  Points points;
  size_t total = 0;
  for (const auto& submap : snapshot) {
    const uint64_t count = pointCount(submap);
    if (count > points.max_size() - total)
      throw std::length_error("LiDAR display point count exceeds vector capacity");
    total += static_cast<size_t>(count);
  }
  points.reserve(total);
  visitPoints(snapshot, 1, [&](const Eigen::Vector3d& point) { points.push_back(point); });
  return points;
}

bool LidarPointCloudMap::savePcd(const Snapshot& snapshot, double voxelSize,
                               const std::string& filename, size_t workspaceBytes) {
  LidarPointCloudMap sampler(voxelSize);
  if (filename.empty() || workspaceBytes < 4096) throw std::invalid_argument("PCD export requires a filename and >=4096 workspace bytes");
  bool any = false;
  for (const auto& submap : snapshot) any = any || pointCount(submap) != 0;
  if (!any) return false;
  const fs::path output(filename);
  const fs::path parent = output.has_parent_path() ? output.parent_path() : fs::path(".");
  const fs::path work = parent / uniqueName(".okvis-pcd-");
  if (!fs::create_directory(work)) ioError("Cannot create PCD export workspace", work.string());
  struct Cleanup { fs::path path; ~Cleanup() { std::error_code ignored; fs::remove_all(path, ignored); } } cleanup{work};
  const size_t capacity = std::max(size_t(1), workspaceBytes / (2 * sizeof(ExportRecord)));
  std::vector<ExportRecord> records;
  records.reserve(capacity);
  std::vector<fs::path> runs;
  uint64_t runIndex = 0;
  auto newRun = [&] { return work / (std::to_string(runIndex++) + ".run"); };
  auto mergeRuns = [&](const fs::path& leftPath, const fs::path& rightPath) {
    std::ifstream left(leftPath, std::ios::binary), right(rightPath, std::ios::binary);
    if (!left || !right) throw std::runtime_error("Cannot reopen PCD sort run");
    const fs::path result = newRun();
    std::ofstream out(result, std::ios::binary);
    if (!out) ioError("Cannot create merged PCD run", result.string());
    ExportRecord a, b;
    bool haveA = readRecord(left, a), haveB = readRecord(right, b);
    while (haveA || haveB) {
      if (!haveB || (haveA && before(a, b))) { writeRecord(out, a); haveA = readRecord(left, a); }
      else if (!haveA || before(b, a)) { writeRecord(out, b); haveB = readRecord(right, b); }
      else { writeRecord(out, preferred(a, b, voxelSize)); haveA = readRecord(left, a); haveB = readRecord(right, b); }
    }
    out.close();
    if (!out) ioError("Cannot close merged PCD run", result.string());
    left.close(); right.close();
    fs::remove(leftPath); fs::remove(rightPath);
    return result;
  };
  auto flushRun = [&] {
    if (records.empty()) return;
    std::sort(records.begin(), records.end(), before);
    fs::path run = newRun();
    std::ofstream out(run, std::ios::binary);
    if (!out) ioError("Cannot create PCD sort run", run.string());
    ExportRecord best = records.front();
    for (size_t i = 1; i < records.size(); ++i) {
      if (sameKey(best, records[i])) best = preferred(best, records[i], voxelSize);
      else { writeRecord(out, best); best = records[i]; }
    }
    writeRecord(out, best);
    out.close();
    if (!out) ioError("Cannot close PCD sort run", run.string());
    records.clear();
    // Binary compaction bounds the run catalog to O(log(number of points)),
    // including when export is configured with a very small sort workspace.
    size_t level = 0;
    while (level < runs.size() && !runs[level].empty()) {
      run = mergeRuns(runs[level], run);
      runs[level].clear();
      ++level;
    }
    if (level == runs.size()) runs.emplace_back();
    runs[level] = std::move(run);
  };
  visitPoints(snapshot, 1, [&](const Eigen::Vector3d& point) {
    if (!point.allFinite()) return;
    for (int axis = 0; axis < 3; ++axis) {
      const double cell = std::floor(point[axis] / voxelSize);
      if (!std::isfinite(cell) || cell < double(std::numeric_limits<int64_t>::min()) ||
          cell >= double(std::numeric_limits<int64_t>::max())) return;
    }
    const Key index = sampler.key(point);
    ExportRecord record;
    for (int axis = 0; axis < 3; ++axis) { record.key[axis] = index[axis]; record.xyz[axis] = point[axis]; }
    records.push_back(record);
    if (records.size() == capacity) flushRun();
  });
  flushRun();
  std::vector<ExportRecord>().swap(records);
  fs::path finalRun;
  for (const auto& run : runs) {
    if (run.empty()) continue;
    finalRun = finalRun.empty() ? run : mergeRuns(finalRun, run);
  }
  if (finalRun.empty()) return false;
  const uint64_t count = fs::file_size(finalRun) / sizeof(ExportRecord);
  atomicWrite(output, [&](std::ofstream& out) {
    out << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "
        << count << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << count << "\nDATA binary\n";
    std::ifstream in(finalRun, std::ios::binary);
    if (!in) throw std::runtime_error("Cannot reopen final PCD sort run");
    ExportRecord record;
    while (readRecord(in, record)) {
      const float xyz[3] = {float(record.xyz[0]), float(record.xyz[1]), float(record.xyz[2])};
      if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2])) {
        throw std::runtime_error("World point cannot be represented by PCD float XYZ");
      }
      out.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
    }
  });
  return true;
}

LidarPointCloudMap::Points LidarPointCloudMap::pointCloud() const {
  return assemble(snapshot(), voxelSize_);
}

} // namespace okvis
