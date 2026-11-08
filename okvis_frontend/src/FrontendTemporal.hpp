// SPDX-License-Identifier: BSD-3-Clause
// Template implementation included by Frontend.cpp inside namespace okvis.
template<class CAMERA_GEOMETRY>
int Frontend::matchTemporalTracks(Estimator& estimator, const ViParameters& params,
                                  uint64_t currentFrameId, bool& rotationOnly) {
  rotationOnly = true;
  int count = 0;
  const StateId currentId(currentFrameId);
  const auto current = estimator.multiFrame(currentId);
  std::set<StateId> candidates(estimator.keyFrames().begin(), estimator.keyFrames().end());
  const StateId previousId = estimator.stateIdByAge(1);
  candidates.insert(previousId);
  candidates.erase(currentId);
  std::vector<std::pair<double, StateId>> ranked;
  for (const auto& id : candidates) {
    const double score = id == previousId ? 2.0 : estimator.overlapFraction(
        estimator.multiFrame(previousId), estimator.multiFrame(id));
    ranked.emplace_back(score, id);
  }
  std::sort(ranked.rbegin(), ranked.rend());
  for (const auto& candidate : ranked) {
    const auto older = estimator.multiFrame(candidate.second);
    for (size_t im = 0; im < current->numFrames(); ++im) {
      const auto camera0 = older->geometryAs<CAMERA_GEOMETRY>(im);
      const auto camera1 = current->geometryAs<CAMERA_GEOMETRY>(im);
      const double f0 = .5 * (camera0->focalLengthU() + camera0->focalLengthV());
      const double f1 = .5 * (camera1->focalLengthU() + camera1->focalLengthV());
      const auto pairs = temporalTrackPairs(*older, *current, im, params.frontend.track_min_length);
      bool pairRotationOnly = true;
      const auto inliers = temporalRansac(*older, *current, im, pairs,
                                         2.5 / std::min(f0, f1), &pairRotationOnly);
      const auto T_WC0 = estimator.pose(candidate.second) * estimator.extrinsics(candidate.second, im);
      const auto T_WC1 = estimator.pose(currentId) * estimator.extrinsics(currentId, im);
      for (size_t i = 0; i < pairs.size(); ++i) {
        const auto& pair = pairs[i];
        if (!inliers[i] || current->landmarkId(im, pair.current)) continue;
        uint64_t landmarkId = older->landmarkId(im, pair.previous);
        if (landmarkId && (!estimator.isLandmarkAdded(LandmarkId(landmarkId))
            || estimator.isLandmarkInitialised(LandmarkId(landmarkId)))) continue;
        Eigen::Vector3d e0, e1;
        if (!older->getBackProjection(im, pair.previous, e0)
            || !current->getBackProjection(im, pair.current, e1)) continue;
        e0 = (T_WC0.C() * e0).normalized(); e1 = (T_WC1.C() * e1).normalized();
        if (e0.dot(e1) < .8) continue;
        double size0, size1;
        older->getKeypointSize(im, pair.previous, size0);
        current->getKeypointSize(im, pair.current, size1);
        bool valid = false, parallel = false;
        Eigen::Vector4d point = triangulation::triangulateFast(T_WC0.r(), e0, T_WC1.r(), e1,
            .125 * std::max(size0 / f0, size1 / f1), valid, parallel);
        if (!valid || parallel || !point.allFinite() || std::abs(point[3]) < 1.e-12) continue;
        point /= point[3];
        const Eigen::Vector4d p0 = T_WC0.inverse() * point, p1 = T_WC1.inverse() * point;
        if (p0.z() < .2 || p1.z() < .2) continue;
        Eigen::Vector2d measured0, measured1, projected0, projected1;
        older->getKeypoint(im, pair.previous, measured0);
        current->getKeypoint(im, pair.current, measured1);
        if (camera0->projectHomogeneous(p0, &projected0) != cameras::ProjectionStatus::Successful
            || camera1->projectHomogeneous(p1, &projected1) != cameras::ProjectionStatus::Successful
            || !projected0.allFinite() || !projected1.allFinite()
            || (measured0 - projected0).norm() >= 4 || (measured1 - projected1).norm() >= 4) continue;
        const double quality = std::acos(std::max(-1.0, std::min(1.0, e0.dot(e1))));
        const MatchInfo match{point, pair.current, true, true, quality};
        if (landmarkId) {
          // Updating an existing immature point must also respect all of its observations.
          MapPoint2 existing;
          estimator.getLandmark(LandmarkId(landmarkId), existing);
          bool consistent = true;
          for (const auto& obs : existing.observations) {
            const auto observed = estimator.multiFrame(StateId(obs.frameId));
            const auto T_CW = (estimator.pose(StateId(obs.frameId))
                * estimator.extrinsics(StateId(obs.frameId), obs.cameraIndex)).inverse();
            Eigen::Vector2d pixel, projection;
            observed->getKeypoint(obs.cameraIndex, obs.keypointIndex, pixel);
            if (observed->geometry(obs.cameraIndex)->projectHomogeneous(T_CW * point, &projection)
                    != cameras::ProjectionStatus::Successful
                || !projection.allFinite() || (pixel - projection).norm() >= 4) { consistent = false; break; }
          }
          if (!consistent) continue;
          estimator.setLandmark(LandmarkId(landmarkId), match.hp_W, match.initialisable);
        } else {
          landmarkId = estimator.addLandmark(match.hp_W, match.initialisable).value();
          older->setLandmarkId(im, pair.previous, landmarkId, FeatureAssociation::MotionStereo);
        }
        if (!estimator.isObserved(KeypointIdentifier{candidate.second.value(), im, pair.previous}))
          estimator.addObservation<CAMERA_GEOMETRY>(LandmarkId(landmarkId), candidate.second, im, pair.previous);
        current->setLandmarkId(im, match.k1, landmarkId, FeatureAssociation::MotionStereo);
        estimator.addObservation<CAMERA_GEOMETRY>(LandmarkId(landmarkId), currentId, im, match.k1);
        // A finite, non-parallel triangulation with both reprojections checked is
        // sufficient parallax even if rotation RANSAC also explains this pair.
        rotationOnly = false;
        ++count;
      }
    }
  }
  return count;
}
