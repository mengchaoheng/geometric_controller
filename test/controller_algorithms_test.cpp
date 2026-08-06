// Copyright 2026 Chaoheng Meng
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include "geometric_controller/controllers/controller_factory.hpp"
#include "geometric_controller/controllers/wrench_normalization.hpp"
#include "geometric_controller/reference_trajectory.hpp"

namespace
{

geometric_controller::VehicleState hoverState()
{
  geometric_controller::VehicleState state;
  state.position = Eigen::Vector3d(0.0, 0.0, -3.0);
  state.attitude = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  return state;
}

geometric_controller::FlatReference hoverReference()
{
  geometric_controller::FlatReference reference;
  reference.position = Eigen::Vector3d(0.0, 0.0, -3.0);
  return reference;
}

TEST(ControllerFactory, ExposesOnlySupportedSelectors)
{
  const auto & names = geometric_controller::supportedControllerTypes();
  ASSERT_EQ(names.size(), 7U);
  for (int id = 0; id <= 6; ++id) {
    const auto type = geometric_controller::controllerTypeFromId(id);
    EXPECT_EQ(geometric_controller::controllerTypeName(type), names[static_cast<size_t>(id)]);
  }
  EXPECT_EQ(geometric_controller::controllerTypeFromId(-1),
    geometric_controller::ControllerType::LEGACY_GEOMETRIC);
  EXPECT_EQ(geometric_controller::controllerTypeFromId(99),
    geometric_controller::ControllerType::PX4_DIRECT);
  EXPECT_EQ(geometric_controller::makeController(
    geometric_controller::ControllerType::PX4_DIRECT), nullptr);
}

TEST(FullWrenchControllers, ProducePhysicalHoverWrench)
{
  geometric_controller::ControllerParams params;
  auto state = hoverState();
  const auto reference = hoverReference();

  for (int id = 0; id < 6; ++id) {
    const auto type = geometric_controller::controllerTypeFromId(id);
    state.applied_thrust_axis_force.setZero();
    if (type == geometric_controller::ControllerType::MAIN_GEOMETRIC_INDI) {
      state.applied_thrust_axis_force = params.mass * params.gravity;
    }
    auto controller = geometric_controller::makeController(type);
    ASSERT_NE(controller, nullptr) << "controller id " << id;
    controller->reset(state);
    const auto command = controller->update(state, reference, params, 0.004);
    EXPECT_TRUE(command.torque.allFinite()) << controller->name();
    EXPECT_TRUE(std::isfinite(command.collective_thrust)) << controller->name();
    EXPECT_NEAR(command.collective_thrust, params.mass * params.gravity.z(), 1e-5)
      << controller->name();
    EXPECT_LT(command.torque.norm(), 1e-5) << controller->name();
  }
}

TEST(FullWrenchControllers, RepeatedUpdatesRemainFinite)
{
  geometric_controller::ControllerParams params;
  auto state = hoverState();
  auto reference = hoverReference();
  reference.velocity = Eigen::Vector3d(0.2, -0.1, 0.05);
  reference.acceleration = Eigen::Vector3d(0.1, 0.05, -0.1);
  reference.jerk = Eigen::Vector3d(0.01, -0.02, 0.03);
  reference.snap = Eigen::Vector3d(-0.01, 0.01, 0.0);

  for (int id = 0; id < 6; ++id) {
    auto controller = geometric_controller::makeController(
      geometric_controller::controllerTypeFromId(id));
    controller->reset(state);
    for (int iteration = 0; iteration < 20; ++iteration) {
      state.velocity += Eigen::Vector3d(1e-4, -2e-4, 1e-4);
      state.body_rate += Eigen::Vector3d(1e-5, -2e-5, 1e-5);
      const auto command = controller->update(state, reference, params, 0.004);
      EXPECT_TRUE(command.torque.allFinite()) << controller->name();
      EXPECT_TRUE(std::isfinite(command.collective_thrust)) << controller->name();
    }
  }
}

TEST(LegacyGeometricController, TracksMovingYawReferenceInTorqueMode)
{
  geometric_controller::ControllerParams params;
  auto controller = geometric_controller::makeController(
    geometric_controller::ControllerType::LEGACY_GEOMETRIC);
  const auto state = hoverState();
  auto reference = hoverReference();
  reference.yaw_rate = 1.0;

  controller->reset(state);
  const auto command = controller->update(state, reference, params, 0.004);

  EXPECT_NEAR(command.desired_body_rate.z(), 1.0, 1e-9);
  EXPECT_GT(command.torque.z(), 0.0);
  EXPECT_NEAR(
    command.torque.z(), params.inertia(2, 2) * params.KOmega.z(), 1e-9);
}

TEST(ReferenceTrajectoryYaw, LockHasAbsolutePriority)
{
  geometric_controller::TrajectoryParameters params;
  params.traj_name = "figure8_horizontal";
  params.trajectory_yaw_lock = true;
  params.trajectory_yaw_fixed = 0.7;
  const geometric_controller::ReferenceTrajectory trajectory(params);

  for (const double time : {0.0, 0.7, 3.0, 12.0}) {
    const auto sample = trajectory.sample(time);
    EXPECT_NEAR(sample.yaw, 0.7, 1e-12);
    EXPECT_DOUBLE_EQ(sample.yaw_rate, 0.0);
    EXPECT_DOUBLE_EQ(sample.yaw_acceleration, 0.0);
  }
}

TEST(ReferenceTrajectoryYaw, UnlockedUsesTrajectoryHeadingDerivatives)
{
  geometric_controller::TrajectoryParameters params;
  params.traj_name = "figure8_horizontal";
  params.trajectory_yaw_lock = false;
  const geometric_controller::ReferenceTrajectory trajectory(params);
  const auto sample = trajectory.sample(0.7);

  const double speed_squared =
    sample.velocity[0] * sample.velocity[0] +
    sample.velocity[1] * sample.velocity[1];
  const double expected_yaw =
    std::atan2(sample.velocity[1], sample.velocity[0]);
  const double expected_yaw_rate =
    (sample.velocity[0] * sample.acceleration[1] -
    sample.velocity[1] * sample.acceleration[0]) / speed_squared;

  EXPECT_NEAR(sample.yaw, expected_yaw, 1e-12);
  EXPECT_NEAR(sample.yaw_rate, expected_yaw_rate, 1e-12);
  EXPECT_NE(sample.yaw_rate, 0.0);
  EXPECT_NE(sample.yaw_acceleration, 0.0);
}

TEST(ReferenceTrajectory, OmegaTransitionPreservesReferenceDerivatives)
{
  geometric_controller::TrajectoryParameters initial;
  initial.omega_value = 0.5;
  geometric_controller::ReferenceTrajectory trajectory(initial);
  constexpr double transition_time = 4.0;
  const auto before = trajectory.sample(transition_time);

  auto updated = initial;
  updated.omega_value = 2.0;
  trajectory.setParametersWithOmegaTransition(updated, transition_time, 1.0);
  const auto after = trajectory.sample(transition_time);

  for (size_t axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(after.position[axis], before.position[axis], 1e-12);
    EXPECT_NEAR(after.velocity[axis], before.velocity[axis], 1e-12);
    EXPECT_NEAR(after.acceleration[axis], before.acceleration[axis], 1e-12);
    EXPECT_NEAR(after.jerk[axis], before.jerk[axis], 1e-12);
    EXPECT_NEAR(after.snap[axis], before.snap[axis], 1e-12);
  }
  EXPECT_NEAR(after.yaw, before.yaw, 1e-12);
  EXPECT_NEAR(after.yaw_rate, before.yaw_rate, 1e-12);
  EXPECT_NEAR(after.yaw_acceleration, before.yaw_acceleration, 1e-12);
}

TEST(ReferenceTrajectory, PreviewIsClosedAfterAnOmegaTransition)
{
  geometric_controller::TrajectoryParameters parameters;
  parameters.traj_name = "figure8_horizontal";
  parameters.omega_value = 0.15;
  geometric_controller::ReferenceTrajectory trajectory(parameters);

  auto updated = parameters;
  updated.omega_value = 0.5;
  constexpr double transition_time = 60.0;
  trajectory.setParametersWithOmegaTransition(updated, transition_time, 1.0);

  const double preview_start = transition_time + 2.0;
  const double duration = trajectory.previewDuration();
  const auto first = trajectory.sample(preview_start);
  const auto last = trajectory.sample(preview_start + duration);
  for (size_t axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(last.position[axis], first.position[axis], 1e-12);
  }
}

TEST(MainSunDFBCController, UsesPreviousCollectiveThrustForFlatnessRates)
{
  geometric_controller::ControllerParams params;
  auto state = hoverState();
  auto reference = hoverReference();
  auto controller = geometric_controller::makeController(
    geometric_controller::ControllerType::MAIN_SUN_DFBC);

  controller->reset(state);
  (void)controller->update(state, reference, params, 0.004);

  reference.acceleration.z() = 3.0;
  reference.jerk.x() = 1.0;
  const auto command = controller->update(state, reference, params, 0.004);
  EXPECT_NEAR(command.desired_body_rate.y(), -1.0 / params.gravity.z(), 1e-12);
}

TEST(ReferenceTrajectory, ZeroOmegaFreezesTrajectoryAtInitialPhase)
{
  geometric_controller::TrajectoryParameters parameters;
  parameters.traj_name = "figure8_horizontal";
  parameters.omega_value = 0.0;
  geometric_controller::ReferenceTrajectory trajectory(parameters);

  const auto initial = trajectory.sample(0.0);
  const auto later = trajectory.sample(10.0);
  EXPECT_NEAR(trajectory.theta(10.0), trajectory.theta(0.0), 1e-12);
  for (size_t axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(later.position[axis], initial.position[axis], 1e-12);
    EXPECT_NEAR(later.velocity[axis], 0.0, 1e-12);
    EXPECT_NEAR(later.acceleration[axis], 0.0, 1e-12);
  }
}

TEST(FullWrenchControllers, GeometricIndiImplementsRotationalEquations)
{
  geometric_controller::ControllerParams params;
  auto state = hoverState();
  state.applied_thrust_axis_force =
    params.mass * params.gravity;
  state.body_rate = Eigen::Vector3d(0.0, 0.0, 0.05);
  state.angular_acceleration = Eigen::Vector3d(0.4, -0.2, 0.1);
  state.applied_torque = Eigen::Vector3d(0.03, -0.02, 0.01);
  const auto reference = hoverReference();

  auto controller = geometric_controller::makeController(
    geometric_controller::ControllerType::MAIN_GEOMETRIC_INDI);
  controller->reset(state);
  const auto first = controller->update(state, reference, params, 0.004);
  const Eigen::Vector3d expected_acceleration =
    -(params.KOmega.asDiagonal() * state.body_rate);
  EXPECT_TRUE(first.torque.isApprox(
      state.applied_torque + params.inertia *
      (expected_acceleration - state.angular_acceleration), 1e-12));
  EXPECT_FALSE(first.indi_torque_feedback_valid);

  const auto command = controller->update(state, reference, params, 0.004);
  EXPECT_TRUE(command.desired_angular_acceleration.isApprox(
      expected_acceleration, 1e-12));
  EXPECT_TRUE(command.torque.isApprox(
      state.applied_torque + params.inertia *
      (expected_acceleration - state.angular_acceleration), 1e-12));
  EXPECT_FALSE(command.indi_torque_feedback_valid);
}

TEST(FullWrenchControllers, GeometricIndiRunsForceAtFixedOuterLoopRate)
{
  geometric_controller::ControllerParams params;
  params.indi_acceleration_enabled = true;
  auto state = hoverState();
  state.applied_thrust_axis_force =
    params.mass * params.gravity;
  const auto reference = hoverReference();
  auto controller = geometric_controller::makeController(
    geometric_controller::ControllerType::MAIN_GEOMETRIC_INDI);
  controller->reset(state);

  const auto initial = controller->update(state, reference, params, 0.004);
  state.acceleration.z() = 2.0;
  const auto held = controller->update(state, reference, params, 0.004);
  EXPECT_NEAR(held.collective_thrust, initial.collective_thrust, 1e-12);

  const auto updated = controller->update(state, reference, params, 0.004);
  EXPECT_NEAR(
    updated.collective_thrust - initial.collective_thrust,
    params.mass * state.acceleration.z(), 1e-12);
}

TEST(FullWrenchControllers, GeometricIndiUsesSharedControllerGains)
{
  geometric_controller::ControllerParams params;
  params.Kp = Eigen::Vector3d(2.0, 3.0, 4.0);
  params.Kv = Eigen::Vector3d(5.0, 6.0, 7.0);
  params.indi_acceleration_enabled = false;

  auto state = hoverState();
  state.position += Eigen::Vector3d(0.3, -0.2, 0.1);
  state.velocity = Eigen::Vector3d(0.4, -0.3, 0.2);
  state.applied_thrust_axis_force = params.mass * params.gravity;

  auto reference = hoverReference();
  reference.velocity = Eigen::Vector3d(-0.1, 0.2, -0.05);
  auto indi = geometric_controller::makeController(
    geometric_controller::ControllerType::MAIN_GEOMETRIC_INDI);
  indi->reset(state);
  const auto indi_command = indi->update(state, reference, params, 0.004);
  const Eigen::Vector3d expected_acceleration =
    params.Kp.asDiagonal() * (reference.position - state.position) +
    params.Kv.asDiagonal() * (reference.velocity - state.velocity);
  EXPECT_TRUE(indi_command.desired_acceleration.isApprox(expected_acceleration, 1e-12));
  EXPECT_NEAR(
    indi_command.collective_thrust,
    params.mass * (params.gravity - expected_acceleration).norm(), 1e-12);
}

TEST(FullWrenchControllers, GeometricIndiClosesPhysicalWrenchLoop)
{
  constexpr double dt = 0.005;
  geometric_controller::ControllerParams params;
  params.Kp = Eigen::Vector3d(10.0, 10.0, 10.0);
  params.Kv = Eigen::Vector3d(6.0, 6.0, 6.0);
  params.KR = Eigen::Vector3d(150.0, 150.0, 3.0);
  params.KOmega = Eigen::Vector3d(20.0, 20.0, 8.0);
  // This ideal plant has no actuator dynamics; the test isolates the INDI
  // equations and multi-rate sample-and-hold behavior.
  params.indi_acceleration_enabled = true;
  const auto reference = hoverReference();

  {
    auto state = hoverState();
    state.position += Eigen::Vector3d(0.25, -0.2, 0.15);
    state.velocity = Eigen::Vector3d(0.1, -0.05, 0.02);
    Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitX()).toRotationMatrix() *
      Eigen::AngleAxisd(-0.06, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Quaterniond attitude(rotation);
    state.attitude =
      Eigen::Vector4d(attitude.w(), attitude.x(), attitude.y(), attitude.z());
    state.applied_thrust_axis_force =
      params.mass * params.gravity;
    auto controller = geometric_controller::makeController(
      geometric_controller::ControllerType::MAIN_GEOMETRIC_INDI);
    controller->reset(state);
    for (int iteration = 0; iteration < 1250; ++iteration) {
      auto command = controller->update(state, reference, params, dt);
      state.applied_torque = command.torque;
      state.acceleration =
        params.gravity -
        (command.collective_thrust / params.mass) * rotation.col(2);
      state.applied_thrust_axis_force =
        command.collective_thrust * rotation.col(2);
      const Eigen::Vector3d actual_angular_acceleration = params.inertia.inverse() *
        (command.torque -
        state.body_rate.cross(params.inertia * state.body_rate));
      state.velocity += dt * state.acceleration;
      state.position += dt * state.velocity;
      state.body_rate += dt * actual_angular_acceleration;
      const Eigen::Vector3d rotation_increment = dt * state.body_rate;
      if (rotation_increment.norm() > 1e-12) {
        rotation = rotation * Eigen::AngleAxisd(
          rotation_increment.norm(), rotation_increment.normalized()).toRotationMatrix();
      }
      attitude = Eigen::Quaterniond(rotation);
      attitude.normalize();
      state.attitude =
        Eigen::Vector4d(attitude.w(), attitude.x(), attitude.y(), attitude.z());
      state.angular_acceleration = actual_angular_acceleration;
      ASSERT_TRUE(state.position.allFinite()) << controller->name();
      ASSERT_TRUE(state.body_rate.allFinite()) << controller->name();
    }

    EXPECT_LT((reference.position - state.position).norm(), 0.1)
      << controller->name() << " final_position=" << state.position.transpose();
    EXPECT_LT(state.body_rate.norm(), 0.5)
      << controller->name() << " final_position=" << state.position.transpose()
      << " final_velocity=" << state.velocity.transpose()
      << " final_body_rate=" << state.body_rate.transpose();
  }
}

TEST(WrenchNormalization, UsesExplicitIrisConstants)
{
  constexpr double mass = 0.75;
  constexpr double total_max_thrust = 4.0 * 8.5;
  geometric_controller::ControllerCommand command;
  command.collective_thrust = mass * 9.81;
  command.torque = Eigen::Vector3d(0.05, -0.05, 0.025);
  const Eigen::Vector3d torque_constants(
    0.319957823650, 0.319957823650, 1.962568474088);

  const auto normalized = geometric_controller::normalizeWrench(
    command, mass, mass / total_max_thrust, 0.0, torque_constants);

  EXPECT_NEAR(normalized.thrust, command.collective_thrust / total_max_thrust, 1e-12);
  EXPECT_TRUE(normalized.torque.isApprox(
    torque_constants.asDiagonal() * command.torque, 1e-12));
  EXPECT_FALSE(normalized.saturated);
}

TEST(WrenchNormalization, PreservesIndiFeedbackFractionDuringTorqueClipping)
{
  geometric_controller::ControllerCommand command;
  command.collective_thrust = 7.5;
  command.torque = Eigen::Vector3d(4.0, -2.0, 0.25);
  command.indi_torque_feedback = Eigen::Vector3d(1.0, -0.5, 0.1);
  command.indi_torque_feedback_valid = true;
  const Eigen::Vector3d torque_constants(0.5, 0.5, 2.0);

  const auto normalized = geometric_controller::normalizeWrench(
    command, 1.0, 0.025, 0.0, torque_constants);

  EXPECT_TRUE(normalized.indi_torque_feedback_valid);
  EXPECT_TRUE(normalized.torque.isApprox(Eigen::Vector3d(1.0, -1.0, 0.5)));
  EXPECT_TRUE(normalized.indi_torque_feedback.isApprox(
    Eigen::Vector3d(0.25, -0.25, 0.2)));
}

TEST(WrenchNormalization, UsesPx4FloatEpsilonForPriorityRatio)
{
  geometric_controller::ControllerCommand command;
  command.collective_thrust = 7.5;
  command.torque =
    Eigen::Vector3d(std::numeric_limits<float>::epsilon() * 0.5, 0.0, 0.0);
  command.indi_torque_feedback = Eigen::Vector3d::Ones();
  command.indi_torque_feedback_valid = true;

  const auto normalized = geometric_controller::normalizeWrench(
    command, 1.0, 0.025, 0.0, Eigen::Vector3d::Ones());

  EXPECT_TRUE(normalized.indi_torque_feedback_valid);
  EXPECT_TRUE(normalized.indi_torque_feedback.isZero());
}

TEST(WrenchNormalization, ClampsOnlyThePublishedCommand)
{
  geometric_controller::ControllerCommand command;
  command.collective_thrust = 100.0;
  command.torque = Eigen::Vector3d(10.0, -10.0, 10.0);

  const auto normalized = geometric_controller::normalizeWrench(
    command, 0.75, 0.022058823529, 0.0, Eigen::Vector3d::Ones());

  EXPECT_DOUBLE_EQ(normalized.thrust, 1.0);
  EXPECT_TRUE(normalized.torque.isApprox(Eigen::Vector3d(1.0, -1.0, 1.0)));
  EXPECT_TRUE(normalized.saturated);
  EXPECT_DOUBLE_EQ(command.collective_thrust, 100.0);
  EXPECT_TRUE(command.torque.isApprox(Eigen::Vector3d(10.0, -10.0, 10.0)));
}

}  // namespace
