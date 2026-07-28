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

#include "geometric_controller/controllers/controller_factory.hpp"
#include "geometric_controller/controllers/velocity_acceleration_filter.hpp"
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

TEST(ControllerFactory, PreservesRos1SelectorNumbers)
{
  const auto & names = geometric_controller::supportedControllerTypes();
  ASSERT_EQ(names.size(), 9U);
  for (int id = 0; id <= 8; ++id) {
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
  const auto state = hoverState();
  const auto reference = hoverReference();

  for (int id = 0; id < 8; ++id) {
    const auto type = geometric_controller::controllerTypeFromId(id);
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

  for (int id = 0; id < 8; ++id) {
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

TEST(FullWrenchControllers, IndiResetRejectsPreSwitchDerivativeSpikes)
{
  geometric_controller::ControllerParams params;
  auto clean_state = hoverState();
  auto spiked_state = clean_state;
  clean_state.acceleration_valid = true;
  clean_state.angular_acceleration_valid = true;
  spiked_state.acceleration_valid = true;
  spiked_state.angular_acceleration_valid = true;
  spiked_state.acceleration = Eigen::Vector3d(80.0, -60.0, 40.0);
  spiked_state.angular_acceleration = Eigen::Vector3d(500.0, -400.0, 300.0);

  auto reference = hoverReference();
  reference.velocity = Eigen::Vector3d(0.1, -0.05, 0.0);
  reference.acceleration = Eigen::Vector3d(0.05, 0.02, -0.03);

  for (const auto type : {
      geometric_controller::ControllerType::MAIN_SUN_DFBC_INDI,
      geometric_controller::ControllerType::MAIN_TAL,
      geometric_controller::ControllerType::MAIN_GEOMETRIC_INDI})
  {
    auto clean = geometric_controller::makeController(type);
    auto spiked = geometric_controller::makeController(type);
    clean->reset(clean_state);
    spiked->reset(spiked_state);

    const auto clean_first = clean->update(clean_state, reference, params, 0.004);
    const auto spiked_first = spiked->update(spiked_state, reference, params, 0.004);
    EXPECT_TRUE(clean_first.torque.isApprox(spiked_first.torque, 1e-10))
      << clean->name();
    EXPECT_NEAR(
      clean_first.collective_thrust, spiked_first.collective_thrust, 1e-10)
      << clean->name();

    // The second update must also be independent of derivatives that existed
    // only before the controller owned the wrench output.
    const auto clean_second = clean->update(clean_state, reference, params, 0.004);
    const auto spiked_second = spiked->update(clean_state, reference, params, 0.004);
    EXPECT_TRUE(clean_second.torque.isApprox(spiked_second.torque, 1e-10))
      << clean->name();
    EXPECT_NEAR(
      clean_second.collective_thrust, spiked_second.collective_thrust, 1e-10)
      << clean->name();
  }
}

TEST(WrenchNormalization, UsesExplicitIrisConstants)
{
  constexpr double mass = 0.75;
  constexpr double total_max_thrust = 4.0 * 8.5;
  geometric_controller::ControllerCommand command;
  command.collective_thrust = mass * 9.81;
  command.torque = Eigen::Vector3d(0.5, -0.5, 0.25);
  const Eigen::Vector3d torque_constants(
    0.319957823650, 0.319957823650, 1.962568474088);

  const auto normalized = geometric_controller::normalizeWrench(
    command, mass, mass / total_max_thrust, 0.0, torque_constants);

  EXPECT_NEAR(normalized.thrust, command.collective_thrust / total_max_thrust, 1e-12);
  EXPECT_TRUE(normalized.torque.isApprox(
    torque_constants.asDiagonal() * command.torque, 1e-12));
  EXPECT_FALSE(normalized.saturated);
}

TEST(VelocityAccelerationFilter, MatchesPx4AlphaDerivativeFilter)
{
  geometric_controller::VelocityAccelerationFilter filter;
  geometric_controller::VelocityAccelerationFilterParams params;
  params.velocity_lpf_hz = 0.0;
  params.velocity_notch_hz = 0.0;
  params.derivative_lpf_hz = 5.0;
  constexpr double dt = 0.008;

  EXPECT_TRUE(filter.update(Eigen::Vector3d::Zero(), dt, params).isZero());
  const Eigen::Vector3d acceleration = filter.update(
    Eigen::Vector3d(dt, -2.0 * dt, 0.5 * dt), dt, params);
  const double time_constant = 1.0 / (2.0 * M_PI * params.derivative_lpf_hz);
  const double alpha = dt / (time_constant + dt);
  EXPECT_TRUE(acceleration.isApprox(
    alpha * Eigen::Vector3d(1.0, -2.0, 0.5), 1e-12));
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
