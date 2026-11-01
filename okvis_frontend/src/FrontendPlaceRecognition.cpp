// SPDX-License-Identifier: BSD-3-Clause
#include "PlaceRecognitionDb.hpp"
#include <condition_variable>
#include <cstring>
#include <glog/logging.h>
#include <deque>
#include <exception>
#include <limits>

namespace okvis {

// Only this actor accesses the DBoW databases after setup. Requests own deep
// feature/geometry snapshots; neither an Estimator nor live frames enter it.
class Frontend::PlaceRecognition {
 public:
  struct Request {
    ViParameters params;
    MultiFramePtr query;
    std::vector<MultiFramePtr> historicalFrames;
    double matchingThreshold;
    uint64_t sequence = 0;
  };
  explicit PlaceRecognition(Frontend& owner, bool filterClassification) : owner_(owner) {
    for (const auto& component : owner.components_) {
      std::map<StateId, MultiFramePtr> frames;
      for (const auto& frame : component.multiFrames_)
        frames.emplace(frame.first, snapshotPlaceRecognitionFrame(*frame.second, true, filterClassification));
      components_.push_back(std::move(frames));
    }
    thread_ = std::thread([this] { run(); });
  }
  ~PlaceRecognition() { stop(false); }
  bool submit(Request request) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failure_) std::rethrow_exception(failure_);
    if (stopping_ || requests_.size() >= kMaxPending) {
      ++statistics_.skipped;
      return false; // Optional queries may be skipped; camera processing never waits.
    }
    request.sequence = ++sequence_;
    requests_.push_back(std::move(request));
    ++statistics_.submitted;
    cv_.notify_one();
    return true;
  }
  AlignedVector<VerifiedPlace> takeResults() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failure_) std::rethrow_exception(failure_);
    AlignedVector<VerifiedPlace> result;
    result.swap(results_);
    return result;
  }
  void exclude(StateId id) {
    std::lock_guard<std::mutex> lock(mutex_);
    unavailable_[id] = sequence_;
  }
  void stop(bool drain) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      if (!drain) requests_.clear();
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
  }
  PlaceRecognitionStatistics statistics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failure_) std::rethrow_exception(failure_);
    auto result = statistics_;
    result.pending = requests_.size();
    return result;
  }
 private:
  static constexpr size_t kMaxPending = 4;
  static constexpr size_t kMaxResults = 8;
  void run() {
    try {
      while (true) {
        Request request;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          cv_.wait(lock, [&] { return stopping_ || !requests_.empty(); });
          if (requests_.empty()) break;
          request = std::move(requests_.front());
          requests_.pop_front();
          statistics_.active = true;
        }
        process(request);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          ++statistics_.processed;
          statistics_.active = false;
        }
      }
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      failure_ = std::current_exception();
      statistics_.active = false;
      stopping_ = true;
      requests_.clear();
    }
  }
  void process(const Request& request) {
    for (const auto& frame : request.historicalFrames) {
      const StateId id(frame->id());
      history_[id] = frame;
      std::lock_guard<std::mutex> lock(mutex_);
      const auto blocked = unavailable_.find(id);
      // Do not let a snapshot queued before exclusion re-enable a stale frame.
      if (blocked != unavailable_.end() && blocked->second < request.sequence)
        unavailable_.erase(blocked);
    }
    std::vector<std::vector<uchar>> features;
    features.reserve(request.query->numKeypoints());
    AlignedVector<kinematics::Transformation> extrinsics;
    for (size_t im = 0; im < request.query->numFrames(); ++im) {
      extrinsics.push_back(*request.query->T_SC(im));
      for (size_t k = 0; k < request.query->numKeypoints(im); ++k) {
        const auto descriptor = request.query->keypointDescriptor(im, k);
        features.emplace_back(descriptor, descriptor + 48);
      }
    }
    const auto queryDatabase = [&](const std::unique_ptr<DBoW>& database,
        const std::map<StateId, MultiFramePtr>& history, int component, double threshold, int minInliers) {
      std::vector<std::pair<StateId, double>> candidates;
      const auto queryStart = std::chrono::steady_clock::now();
      owner_.getFilteredDBoWResult(database, features, candidates);
      const double queryMs = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - queryStart).count();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        statistics_.queryMaxMs = std::max(statistics_.queryMaxMs, queryMs);
      }
      size_t attempts = 0;
      for (const auto& candidate : candidates) {
        if (attempts >= std::max<size_t>(10, database->poseIds.size() / 20)) break;
        if (candidate.second <= threshold || (component < 0 && candidate.first == StateId(request.query->id()))) continue;
        if (component < 0) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (unavailable_.count(candidate.first)) continue;
        }
        const auto reference = history.find(candidate.first);
        if (reference == history.end()) continue; // Not a completed, stable historical frame yet.
        ++attempts;
        VerifiedPlace result;
        result.queryId = StateId(request.query->id());
        result.candidateId = candidate.first;
        result.component = component;
        result.score = candidate.second;
        const auto verificationStart = std::chrono::steady_clock::now();
        const bool verified = owner_.verifyPlaceRecognitionSnapshot(request.params, request.query, reference->second,
              extrinsics, request.matchingThreshold, result.relativePose, result.information, minInliers, false);
        const double verificationMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - verificationStart).count();
        std::lock_guard<std::mutex> lock(mutex_);
        statistics_.verificationMaxMs = std::max(statistics_.verificationMaxMs, verificationMs);
        if (!verified) continue;
        // These are optional proposals, not accepted graph constraints.
        if (results_.size() == kMaxResults) results_.erase(results_.begin());
        results_.push_back(std::move(result));
        ++statistics_.verified;
        break;
      }
    };
    for (size_t c = 0; c < components_.size(); ++c)
      queryDatabase(owner_.componentDBows_.at(c), components_[c], int(c), 0.4, 40);
    if (request.params.estimator.do_loop_closures)
      queryDatabase(owner_.dBow_, history_, -1, request.params.estimator.p_dbow, 10);
    // Query before insertion: a keyframe can never retrieve itself.
    if (request.params.estimator.do_loop_closures) {
      owner_.dBow_->database.add(features);
      owner_.dBow_->poseIds.push_back(request.query->id());
    }
  }
  Frontend& owner_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Request> requests_;
  AlignedVector<VerifiedPlace> results_;
  std::map<StateId, MultiFramePtr> history_;
  std::vector<std::map<StateId, MultiFramePtr>> components_;
  PlaceRecognitionStatistics statistics_;
  bool stopping_ = false;
  uint64_t sequence_ = 0;
  std::map<StateId, uint64_t> unavailable_;
  std::exception_ptr failure_;
  std::thread thread_;
};

MultiFramePtr Frontend::snapshotPlaceRecognitionFrame(const MultiFrame& source, bool landmarks,
    bool filterClassification) {
  (void)filterClassification;
  auto snapshot = std::make_shared<MultiFrame>(source.cameraSystem(), source.timestamp(), source.id());
  for (size_t im = 0; im < source.numFrames(); ++im) {
    snapshot->setExtrinsics(im, *source.T_SC(im));
    snapshot->setGeometry(im, source.geometry(im));
    std::vector<cv::KeyPoint> keypoints(source.numKeypoints(im));
    cv::Mat descriptors(int(keypoints.size()), 48, CV_8UC1);
    for (size_t k = 0; k < keypoints.size(); ++k) {
      source.getCvKeypoint(im, k, keypoints[k]);
      std::memcpy(descriptors.ptr(int(k)), source.keypointDescriptor(im, k), 48);
    }
    snapshot->resetKeypoints(im, keypoints);
    snapshot->resetDescriptors(im, descriptors);
    snapshot->computeBackProjections(im);
    if (!landmarks) continue;
    for (size_t k = 0; k < keypoints.size(); ++k) {
      Eigen::Vector4d point = Eigen::Vector4d::Zero();
      bool initialized = false;
      source.getLandmark(im, k, point, initialized);
#ifdef OKVIS_USE_NN
      if (filterClassification && source.isClassified(im)) {
        cv::Mat classification;
        source.getClassification(im, k, classification);
        if (classification.at<float>(10) > 3.5f || classification.at<float>(11) > 53.5f)
          initialized = false;
      }
#endif
      snapshot->setLandmarkId(im, k, source.landmarkId(im, k));
      snapshot->setLandmark(im, k, point, initialized);
    }
  }
  return snapshot;
}

bool Frontend::enqueuePlaceRecognition(const ViParameters& params, MultiFramePtr query,
    std::vector<MultiFramePtr> historicalFrames) {
  if (!placeRecognition_) placeRecognition_ = std::make_shared<PlaceRecognition>(*this, params.frontend.use_cnn);
  return placeRecognition_->submit({params, std::move(query), std::move(historicalFrames), briskMatchingThreshold_});
}

void Frontend::submitPlaceRecognitionKeyframe(Estimator& estimator, const ViParameters& params,
    const MultiFramePtr& frame) {
  std::vector<MultiFramePtr> historicalFrames;
  std::vector<StateId> completed;
  // Only recently submitted keyframes remain here; completed frames are copied
  // once, after conversion stored their landmarks in their sensor frame.
  for (const auto id : pendingPlaceRecognitionFrames_) {
    if (!estimator.isPlaceRecognitionFrame(id)) { completed.push_back(id); continue; }
    if (!estimator.isPoseGraphFrame(id) || estimator.isLoopClosureFrame(id)
        || estimator.isRecentLoopClosureFrame(id)) continue;
    historicalFrames.push_back(snapshotPlaceRecognitionFrame(*estimator.multiFrame(id), true, params.frontend.use_cnn));
    completed.push_back(id);
  }
  auto query = snapshotPlaceRecognitionFrame(*frame, false);
  for (size_t im = 0; im < query->numFrames(); ++im)
    query->setExtrinsics(im, estimator.extrinsics(StateId(frame->id()), uchar(im)));
  if (enqueuePlaceRecognition(params, std::move(query), std::move(historicalFrames))) {
    for (const auto id : completed) pendingPlaceRecognitionFrames_.erase(id);
    pendingPlaceRecognitionFrames_.insert(StateId(frame->id()));
    const auto stats = placeRecognitionStatistics();
    LOG_EVERY_N(INFO, 100) << "Place recognition worker: submitted=" << stats.submitted
        << ", processed=" << stats.processed << ", skipped=" << stats.skipped
        << ", pending=" << stats.pending << ", verified=" << stats.verified
        << ", query_max_ms=" << stats.queryMaxMs << ", verification_max_ms=" << stats.verificationMaxMs;
  } else {
    LOG_EVERY_N(WARNING, 40) << "Place recognition worker busy; skipped optional keyframe query " << frame->id();
  }
}

AlignedVector<Frontend::VerifiedPlace> Frontend::takePlaceRecognitionResults() {
  return placeRecognition_ ? placeRecognition_->takeResults() : AlignedVector<VerifiedPlace>{};
}
void Frontend::stopPlaceRecognition(bool drain) {
  if (placeRecognition_) placeRecognition_->stop(drain);
}
void Frontend::finishPlaceRecognition() { stopPlaceRecognition(true); }
void Frontend::excludePlaceRecognitionFrame(StateId id) {
  if (placeRecognition_) placeRecognition_->exclude(id);
}
Frontend::PlaceRecognitionStatistics Frontend::placeRecognitionStatistics() const {
  return placeRecognition_ ? placeRecognition_->statistics() : PlaceRecognitionStatistics{};
}
} // namespace okvis
