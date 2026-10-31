// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/ViInterface.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

okvis::State makeState(uint64_t id, double timestamp, double x, bool keyframe = true) {
  okvis::State state{};
  state.id = okvis::StateId(id);
  state.timestamp = okvis::Time(timestamp);
  state.T_WS = okvis::kinematics::Transformation::Identity();
  state.T_WS.set(Eigen::Vector3d(x, 0.0, 0.0), Eigen::Quaterniond::Identity());
  state.v_W.setZero();
  state.b_g.setZero();
  state.b_a.setZero();
  state.omega_S.setZero();
  state.isKeyframe = keyframe;
  return state;
}

okvis::ImuMeasurementDeque interval(double start, double end) {
  okvis::ImuMeasurementDeque measurements;
  for (double timestamp = start; timestamp < end + 0.0001; timestamp += 0.01) {
    measurements.emplace_back(okvis::Time(timestamp),
        okvis::ImuSensorReadings(Eigen::Vector3d(0.01, 0.02, 0.03),
                                Eigen::Vector3d(0.1, 0.2, 9.81)));
  }
  return measurements;
}

void update(okvis::Trajectory& trajectory, const okvis::State& state,
            bool keyframe, okvis::StateId currentKeyframe) {
  auto states = std::make_shared<okvis::AlignedMap<okvis::StateId, okvis::State>>();
  states->emplace(state.id, state);
  okvis::TrackingState tracking{};
  tracking.id = state.id;
  tracking.isKeyframe = keyframe;
  tracking.currentKeyframeId = currentKeyframe;
  std::set<okvis::StateId> affected;
  trajectory.update(tracking, states, affected);
}

using Poses = std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>;
using LookupStatus = okvis::Trajectory::PoseLookupStatus;

void addMeasurements(okvis::Trajectory& trajectory, const okvis::ImuMeasurementDeque& measurements) {
  for (const auto& measurement : measurements) {
    okvis::State unused;
    trajectory.addImuMeasurement(measurement.timeStamp,
        measurement.measurement.accelerometers, measurement.measurement.gyroscopes, unused);
  }
}

void seedThreeStates(okvis::Trajectory& trajectory) {
  for (uint64_t id = 1; id <= 3; ++id) {
    auto state = makeState(id, double(id), double(id - 1));
    if (id > 1) state.previousImuMeasurements = interval(double(id - 1), double(id));
    update(trajectory, state, true, state.id);
  }
}

void runPosePrefixes() {
  okvis::Trajectory trajectory;
  seedThreeStates(trajectory);
  const std::vector<okvis::Time> times{
      okvis::Time(1.0), okvis::Time(1.0), okvis::Time(1.2), okvis::Time(1.2),
      okvis::Time(1.999), okvis::Time(2.0), okvis::Time(2.0), okvis::Time(2.2),
      okvis::Time(2.999), okvis::Time(3.0), okvis::Time(3.0)};
  Poses poses{Eigen::Matrix4d::Zero()};
  auto result = trajectory.getStatePosePrefix(times, poses);
  check(result.status == LookupStatus::Complete && result.supportedCount == times.size()
            && result.expiredCount == 0 && poses.size() == times.size(),
        "Cross-interval prefix lost duplicate timestamps or state boundaries");
  for (size_t i = 0; i < times.size(); ++i) {
    okvis::State expected;
    check(trajectory.getState(times[i], expected), "Cross-interval scalar reference unavailable");
    check((poses[i] - expected.T_WS.T()).norm() < 1e-9,
          "Cross-interval propagation differs from the corresponding scalar query");
    if (i && times[i] == times[i - 1]) {
      check((poses[i] - poses[i - 1]).isZero(), "Duplicate timestamp changed its pose");
    }
  }

  result = trajectory.getStatePosePrefix({okvis::Time(3.0), okvis::Time(3.1)}, poses);
  check(result.status == LookupStatus::WaitingForState && result.supportedCount == 1
            && result.expiredCount == 0 && poses.size() == 1,
        "Future state query did not preserve its valid exact-state prefix");
  Poses untouched{Eigen::Matrix4d::Identity()};
  check(!trajectory.getStatePoses({okvis::Time(3.0), okvis::Time(3.1)}, untouched)
            && untouched.size() == 1 && untouched.front().isIdentity(),
        "All-or-nothing wrapper appended a partial result");
  check(trajectory.getStatePoses(times, untouched) && untouched.size() == times.size() + 1
            && untouched.front().isIdentity(),
        "Successful all-or-nothing wrapper did not append to existing output");
  result = trajectory.getStatePosePrefix({}, poses);
  check(result.status == LookupStatus::Complete && result.supportedCount == 0 && poses.empty(),
        "Empty prefix query retained stale output");
}

void runMissingImuPrefixes() {
  okvis::Trajectory trajectory;
  auto first = makeState(1, 1.0, 0.0);
  auto second = makeState(2, 2.0, 1.0);
  update(trajectory, first, true, first.id);
  second.previousImuMeasurements = interval(1.0, 1.5);
  update(trajectory, second, true, second.id);
  Poses poses;
  auto result = trajectory.getStatePosePrefix(
      {okvis::Time(1.25), okvis::Time(1.5), okvis::Time(1.5), okvis::Time(1.75)}, poses);
  check(result.status == LookupStatus::WaitingForImu && result.supportedCount == 3
            && result.expiredCount == 0 && poses.size() == 3,
        "Missing trailing IMU lost the supported prefix or its exact last-IMU endpoint");
  check((poses[1] - poses[2]).isZero(), "Last-IMU endpoint duplicate changed pose");
  addMeasurements(trajectory, interval(1.5, 2.0));
  result = trajectory.getStatePosePrefix({okvis::Time(1.75), okvis::Time(2.0)}, poses);
  check(result.status == LookupStatus::Complete && result.supportedCount == 2,
        "Delayed trailing IMU did not unblock a waiting prefix");

  // Pruning did occur, but the missing starting sample is still inside the
  // retained horizon and can arrive late. The current front alone is no proof
  // of expiry.
  okvis::Trajectory delayed;
  addMeasurements(delayed, interval(4.0, 4.1));
  first = makeState(1, 5.5, 0.0);
  second = makeState(2, 7.0, 1.0);
  update(delayed, first, true, first.id);
  update(delayed, second, true, second.id);
  addMeasurements(delayed, interval(10.0, 10.0));
  result = delayed.getStatePosePrefix({okvis::Time(5.5), okvis::Time(6.0)}, poses);
  check(result.status == LookupStatus::WaitingForImu && result.supportedCount == 1
            && result.expiredCount == 0,
        "Temporary front gap inside the retention horizon was misclassified as expired");
  addMeasurements(delayed, interval(5.5, 6.0));
  result = delayed.getStatePosePrefix({okvis::Time(6.0)}, poses);
  check(result.status == LookupStatus::Complete && result.supportedCount == 1,
        "Out-of-order starting IMU could not repair a temporary front gap");
}

void runExpiredImuPrefixes() {
  okvis::Trajectory trajectory;
  seedThreeStates(trajectory);
  // One cached timestamp must remain usable after its raw IMU interval expires.
  okvis::State cached;
  check(trajectory.getState(okvis::Time(1.4), cached), "Could not seed a cached IMU propagation");
  addMeasurements(trajectory, interval(10.0, 10.0));
  Poses poses;
  auto result = trajectory.getStatePosePrefix(
      {okvis::Time(1.0), okvis::Time(1.0), okvis::Time(1.25), okvis::Time(1.25),
       okvis::Time(1.3), okvis::Time(1.4), okvis::Time(1.6), okvis::Time(2.0)}, poses);
  check(result.status == LookupStatus::ExpiredImu && result.supportedCount == 2
            && result.expiredCount == 3 && poses.size() == 2,
        "Expired run did not follow the valid prefix or skipped a cached timestamp");
  result = trajectory.getStatePosePrefix(
      {okvis::Time(1.4), okvis::Time(1.6), okvis::Time(1.8), okvis::Time(2.0)}, poses);
  check(result.status == LookupStatus::ExpiredImu && result.supportedCount == 1
            && result.expiredCount == 2 && poses.size() == 1
            && (poses.front() - cached.T_WS.T()).norm() < 1e-12,
        "Expired run discarded a cached pose or crossed an exact state boundary");
  result = trajectory.getStatePosePrefix({okvis::Time(2.0), okvis::Time(3.0)}, poses);
  check(result.status == LookupStatus::Complete && result.supportedCount == 2,
        "Exact historical states incorrectly required expired IMU samples");

  // A point can itself lie inside the five-second horizon while the earlier
  // camera state needed to integrate its pose has already fallen outside it.
  okvis::Trajectory olderStart;
  auto first = makeState(1, 4.0, 0.0);
  auto second = makeState(2, 6.0, 1.0);
  update(olderStart, first, true, first.id);
  second.previousImuMeasurements = interval(4.0, 6.0);
  update(olderStart, second, true, second.id);
  addMeasurements(olderStart, interval(10.0, 10.0));
  result = olderStart.getStatePosePrefix({okvis::Time(5.5), okvis::Time(6.0)}, poses);
  check(result.status == LookupStatus::ExpiredImu && result.supportedCount == 0
            && result.expiredCount == 1 && poses.empty(),
        "Expiry was based on the point time instead of its required preceding state");
}

void runInvalidPosePrefix() {
  okvis::Trajectory trajectory;
  auto state = makeState(1, 1.0, 0.0);
  state.T_WS.set(Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
                 Eigen::Quaterniond::Identity());
  update(trajectory, state, true, state.id);
  Poses poses{Eigen::Matrix4d::Identity()};
  const auto result = trajectory.getStatePosePrefix({okvis::Time(1.0)}, poses);
  check(result.status == LookupStatus::Invalid && result.supportedCount == 0
            && result.expiredCount == 0 && poses.empty(),
        "Nonfinite exact-state pose entered a valid prefix or was mislabeled expired");
}

void runBatchedInputGuards() {
  const auto identity = okvis::kinematics::Transformation::Identity();
  okvis::SpeedAndBias sb = okvis::SpeedAndBias::Zero();
  okvis::SpeedAndBias outputSb;
  okvis::kinematics::Transformation pose;
  Eigen::Vector3d omega;

  const okvis::ImuMeasurementDeque empty;
  okvis::BatchedLidarPropagator emptyPropagator(okvis::Time(1.0), empty);
  check(!emptyPropagator.appendTo(empty, identity, sb, okvis::Time(1.1))
            && !emptyPropagator.getState(identity, sb, pose, outputSb, omega),
        "Empty IMU input was dereferenced or exposed an uninitialized result");
  const auto one = interval(1.0, 1.0);
  okvis::BatchedLidarPropagator onePropagator(okvis::Time(1.0), one);
  check(!onePropagator.appendTo(one, identity, sb, okvis::Time(1.1))
            && !onePropagator.getState(identity, sb, pose, outputSb, omega),
        "Single IMU input was treated as a supported integration interval");

  const auto measurements = interval(1.0, 2.0);
  okvis::BatchedLidarPropagator missingStart(okvis::Time(0.9), measurements);
  check(!missingStart.appendTo(measurements, identity, sb, okvis::Time(1.1)),
        "Batched propagation accepted input newer than its starting state");
  auto unordered = measurements;
  std::swap(unordered[1], unordered[2]);
  okvis::BatchedLidarPropagator unorderedPropagator(okvis::Time(1.0), unordered);
  check(!unorderedPropagator.appendTo(unordered, identity, sb, okvis::Time(1.1)),
        "Batched propagation accepted an unordered IMU cursor");

  okvis::BatchedLidarPropagator propagator(okvis::Time(1.0), measurements);
  check(!propagator.getState(identity, sb, pose, outputSb, omega)
            && !propagator.appendTo(measurements, identity, sb, okvis::Time(1.0))
            && !propagator.appendTo(measurements, identity, sb, okvis::Time(0.99))
            && !propagator.appendTo(measurements, identity, sb, okvis::Time(2.1)),
        "Uninitialized, zero-length or out-of-coverage query was accepted");
  check(propagator.appendTo(measurements, identity, sb, okvis::Time(1.234))
            && propagator.getState(identity, sb, pose, outputSb, omega),
        "Valid query failed after rejected requests");
  const Eigen::Matrix4d firstPose = pose.T();
  const auto otherMeasurements = measurements;
  check(!propagator.appendTo(otherMeasurements, identity, sb, okvis::Time(1.237)),
        "Batched cursor accepted a different deque with identical values");
  // Both times lie in the same IMU interval. Checking only t_end_ (the last
  // completed IMU boundary, 1.23 here) would incorrectly permit this reversal.
  check(!propagator.appendTo(measurements, identity, sb, okvis::Time(1.233))
            && propagator.getState(identity, sb, pose, outputSb, omega)
            && (pose.T() - firstPose).norm() < 1e-12,
        "Backward interpolation request changed a previously valid result");
  check(propagator.appendTo(measurements, identity, sb, okvis::Time(1.234))
            && propagator.getState(identity, sb, pose, outputSb, omega)
            && (pose.T() - firstPose).norm() < 1e-12,
        "Repeated timestamp did not preserve the batched interpolation result");
  check(propagator.appendTo(measurements, identity, sb, okvis::Time(1.237))
            && propagator.getState(identity, sb, pose, outputSb, omega),
        "Forward query within the same IMU interval failed");
  okvis::Propagator reference(okvis::Time(1.0));
  okvis::kinematics::Transformation expected;
  check(reference.appendTo(measurements, identity, sb, okvis::Time(1.237))
            && reference.getState(identity, sb, expected, outputSb, omega)
            && (pose.T() - expected.T()).norm() < 1e-9,
        "Cursor guards changed the normal integration result");

  check(propagator.appendTo(measurements, identity, sb, okvis::Time(2.0))
            && propagator.getState(identity, sb, pose, outputSb, omega) && pose.T().allFinite(),
        "Query at the final IMU sample overran the cursor or lost its result");
  const Eigen::Matrix4d lastPose = pose.T();
  check(!propagator.appendTo(measurements, identity, sb, okvis::Time(2.001))
            && propagator.appendTo(measurements, identity, sb, okvis::Time(2.0))
            && propagator.getState(identity, sb, pose, outputSb, omega)
            && (pose.T() - lastPose).norm() < 1e-12,
        "Rejected tail query corrupted the last supported IMU endpoint");
}

void run() {
  okvis::Trajectory trajectory;
  auto first = makeState(1, 1.0, 0.0);
  update(trajectory, first, true, first.id);

  auto second = makeState(2, 2.0, 1.0);
  second.previousImuMeasurements = interval(1.0, 2.0);
  update(trajectory, second, true, second.id);

  okvis::State stored;
  check(trajectory.getState(second.id, stored), "Stored state missing");
  check(stored.previousImuMeasurements.empty(), "Persistent state retained interval IMU");
  check(trajectory.bufferedImuMeasurementCount() <= 102,
        "Rolling trajectory duplicated the interval IMU");

  okvis::State interpolated;
  check(trajectory.getState(okvis::Time(1.5), interpolated),
        "Rolling IMU could not interpolate inside the active interval");
  check(interpolated.previousImuMeasurements.empty(),
        "Interpolated state exposed rolling IMU payload");

  auto third = makeState(3, 3.0, 2.0);
  third.previousImuMeasurements = interval(2.0, 3.0);
  update(trajectory, third, true, third.id);

  auto checkBatch = [&](const std::vector<okvis::Time>& timestamps) {
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> poses;
    check(trajectory.getStatePoses(timestamps, poses), "Rolling batched query failed");
    check(poses.size() == timestamps.size(), "Batched query lost a timestamp");
    for (size_t i = 0; i < timestamps.size(); ++i) {
      okvis::State expected;
      check(trajectory.getState(timestamps[i], expected), "Scalar rolling query failed");
      check((poses[i] - expected.T_WS.T()).norm() < 1e-9,
            "Batched rolling query changed order or pose");
    }
  };
  checkBatch({okvis::Time(1.2), okvis::Time(1.2), okvis::Time(1.7)});
  checkBatch({okvis::Time(1.2), okvis::Time(2.0), okvis::Time(2.2), okvis::Time(3.0)});
  checkBatch({okvis::Time(3.0), okvis::Time(1.0), okvis::Time(2.2)});

  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> untouched{
      Eigen::Matrix4d::Identity()};
  check(!trajectory.getStatePoses({okvis::Time(1.2), okvis::Time(4.0)}, untouched)
            && untouched.size() == 1 && untouched.front().isIdentity(),
        "Failed rolling batch modified caller output");

  // Once an IMU interval has expired, do not silently deskew it with the
  // constant-velocity fallback used by genuine camera-only trajectories.
  okvis::State unused;
  trajectory.addImuMeasurement(okvis::Time(10.0), Eigen::Vector3d(0.1, 0.2, 9.81),
                               Eigen::Vector3d(0.01, 0.02, 0.03), unused);
  check(!trajectory.getState(okvis::Time(1.55), unused),
        "Expired IMU interval silently used constant-velocity propagation");

  // Relative non-keyframe history remains available without retaining IMU.
  okvis::Trajectory relative;
  auto keyframe = makeState(10, 10.0, 0.0);
  update(relative, keyframe, true, keyframe.id);
  auto nonKeyframe = makeState(11, 11.0, 1.0, false);
  nonKeyframe.previousImuMeasurements = interval(10.0, 11.0);
  update(relative, nonKeyframe, false, keyframe.id);
  keyframe.T_WS.set(Eigen::Vector3d(10.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
  update(relative, keyframe, true, keyframe.id);
  check(relative.getState(nonKeyframe.id, stored), "Relative non-keyframe history missing");
  check(std::abs(stored.T_WS.r().x() - 11.0) < 1e-10,
        "Loop correction did not update relative non-keyframe");
  check(stored.previousImuMeasurements.empty(), "Non-keyframe retained historical IMU");

  // Long streams retain only the fixed five-second propagation horizon.
  okvis::Trajectory rolling;
  for (uint32_t i = 0; i <= 2000; ++i) {
    rolling.addImuMeasurement(okvis::Time(0.01 * i), Eigen::Vector3d(0.1, 0.2, 9.81),
                              Eigen::Vector3d(0.01, 0.02, 0.03), unused);
  }
  check(rolling.bufferedImuMeasurementCount() <= 502,
        "Rolling IMU exceeded its five-second horizon");
}
}  // namespace

int main() {
  try {
    run();
    runPosePrefixes();
    runMissingImuPrefixes();
    runExpiredImuPrefixes();
    runInvalidPosePrefix();
    runBatchedInputGuards();
    std::cout << "Rolling IMU lifetime and trajectory propagation checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
