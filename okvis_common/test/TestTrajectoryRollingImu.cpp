// SPDX-License-Identifier: BSD-3-Clause
#include <okvis/ViInterface.hpp>
#include <cmath>
#include <iostream>
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
    std::cout << "Rolling IMU lifetime and trajectory propagation checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
