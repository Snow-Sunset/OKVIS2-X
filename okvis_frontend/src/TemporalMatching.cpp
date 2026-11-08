// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/TemporalMatching.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/RotationOnlySacProblem.hpp>
#include <cmath>
#include <map>
#include <set>

namespace okvis {
std::vector<TemporalPair> temporalTrackPairs(const MultiFrame& previous,
    const MultiFrame& current, size_t camera, int minimumLength) {
  std::map<uint64_t, size_t> indices;
  std::set<uint64_t> duplicates;
  for (size_t k = 0; k < previous.numKeypoints(camera); ++k) {
    const auto track = previous.featureTrack(camera, k);
    if (track.id && !indices.emplace(track.id, k).second) duplicates.insert(track.id);
  }
  std::vector<TemporalPair> pairs;
  std::set<uint64_t> used;
  for (size_t k = 0; k < current.numKeypoints(camera); ++k) {
    const auto track = current.featureTrack(camera, k);
    const auto old = indices.find(track.id);
    if (!track.mature(minimumLength) || old == indices.end() || duplicates.count(track.id)
        || !used.insert(track.id).second) continue;
    pairs.push_back({old->second, k});
  }
  return pairs;
}

std::vector<bool> temporalRansac(const MultiFrame& previous, const MultiFrame& current,
    size_t camera, const std::vector<TemporalPair>& pairs, double angularThreshold,
    bool* rotationOnly) {
  *rotationOnly = true;
  std::vector<bool> inliers(pairs.size(), false);
  opengv::bearingVectors_t a, b;
  std::vector<size_t> indices;
  for (size_t i = 0; i < pairs.size(); ++i) {
    Eigen::Vector3d rayA, rayB;
    if (!previous.getBackProjection(camera, pairs[i].previous, rayA)
        || !current.getBackProjection(camera, pairs[i].current, rayB)
        || !rayA.allFinite() || !rayB.allFinite()
        || rayA.norm() < 1.e-12 || rayB.norm() < 1.e-12) continue;
    a.push_back(rayA.normalized()); b.push_back(rayB.normalized()); indices.push_back(i);
  }
  if (indices.size() < 8) return inliers;
  opengv::relative_pose::CentralRelativeAdapter adapter(a, b);
  using Rotation = opengv::sac_problems::relative_pose::RotationOnlySacProblem;
  opengv::sac::Ransac<Rotation> rotation;
  rotation.sac_model_ = std::make_shared<Rotation>(adapter, false);
  rotation.threshold_ = 1.0 - std::cos(angularThreshold);
  rotation.max_iterations_ = 100;
  const bool haveRotation = rotation.computeModel();
  using Relative = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
  opengv::sac::Ransac<Relative> relative;
  relative.sac_model_ = std::make_shared<Relative>(adapter, Relative::STEWENIUS, false);
  relative.threshold_ = 2.0 * (1.0 - std::cos(angularThreshold));
  relative.max_iterations_ = 100;
  const bool haveRelative = relative.computeModel();
  const size_t rotationCount = haveRotation ? rotation.inliers_.size() : 0;
  const size_t relativeCount = haveRelative ? relative.inliers_.size() : 0;
  const bool useRotation = rotationCount >= 8 &&
      (rotationCount >= relativeCount || double(rotationCount) / indices.size() > .8);
  const auto& selected = useRotation ? rotation.inliers_ : relative.inliers_;
  if ((!useRotation && !haveRelative) || selected.size() < 8
      || selected.size() * 2 < indices.size()) return inliers;
  *rotationOnly = useRotation;
  for (const auto index : selected) inliers.at(indices.at(size_t(index))) = true;
  return inliers;
}
}  // namespace okvis
