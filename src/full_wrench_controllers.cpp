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

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

#include "geometric_controller/controllers/legacy_geometric_controller.hpp"
#include "geometric_controller/controllers/main_controller_math.hpp"
#include "geometric_controller/controllers/main_geometric_controller.hpp"
#include "geometric_controller/controllers/main_geometric_indi_controller.hpp"
#include "geometric_controller/controllers/main_johnson_controller.hpp"
#include "geometric_controller/controllers/main_lee_controller.hpp"
#include "geometric_controller/controllers/main_sun_dfbc_controller.hpp"

namespace geometric_controller
{
namespace
{

constexpr double kEpsilon = 1e-8;

struct DesiredAttitudeKinematics
{
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector4d quaternion{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration{Eigen::Vector3d::Zero()};
};

Eigen::Vector3d accelerationCommand(
  const VehicleState & state, const FlatReference & reference,
  const Eigen::Vector3d & position_gain,
  const Eigen::Vector3d & velocity_gain)
{
  const Eigen::Vector3d feedback =
    position_gain.asDiagonal() * (reference.position - state.position) +
    velocity_gain.asDiagonal() * (reference.velocity - state.velocity);
  return reference.acceleration + feedback;
}

Eigen::Vector3d accelerationCommand(
  const VehicleState & state, const FlatReference & reference,
  const ControllerParams & params)
{
  return accelerationCommand(state, reference, params.Kp, params.Kv);
}

Eigen::Matrix3d safeAttitudeFromBodyZ(
  const Eigen::Vector3d & body_z_vector, double yaw, const Eigen::Matrix3d & fallback)
{
  if (!body_z_vector.allFinite() || body_z_vector.norm() < kEpsilon) {
    return fallback;
  }
  const Eigen::Vector3d b3 = body_z_vector.normalized();
  Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
  if (b3.cross(heading).norm() < kEpsilon) {
    heading = Eigen::Vector3d::UnitY();
  }
  return main_math::attitudeFromUnitBodyZAndHeading(b3, heading);
}

DesiredAttitudeKinematics desiredAttitudeKinematics(
  const Eigen::Vector3d & body_z_vector,
  const Eigen::Vector3d & body_z_vector_dot,
  const Eigen::Vector3d & body_z_vector_ddot,
  const FlatReference & reference,
  const Eigen::Matrix3d & fallback)
{
  DesiredAttitudeKinematics result;
  result.rotation = safeAttitudeFromBodyZ(body_z_vector, reference.yaw, fallback);
  result.quaternion = main_math::matrixToQuaternion(result.rotation);
  if (body_z_vector.norm() < kEpsilon) {
    return result;
  }

  const auto heading = main_math::headingAxisFromYaw(reference);
  Eigen::Vector3d x_c = heading.xC;
  Eigen::Vector3d x_c_dot = heading.xCDot;
  Eigen::Vector3d x_c_ddot = heading.xCDDot;
  const auto b3 = main_math::unitVectorDerivativesFromVector(
    body_z_vector, body_z_vector_dot, body_z_vector_ddot);
  if (b3.b.cross(x_c).norm() < kEpsilon) {
    x_c = Eigen::Vector3d::UnitY();
    x_c_dot.setZero();
    x_c_ddot.setZero();
  }
  const auto derivatives = main_math::attitudeDerivativesFromUnitBodyZAndHeading(
    b3.b, b3.bDot, b3.bDDot, x_c, x_c_dot, x_c_ddot);
  result.rotation = main_math::attitudeFromUnitBodyZAndHeading(b3.b, x_c);
  result.quaternion = main_math::matrixToQuaternion(result.rotation);
  result.body_rate = main_math::vee(result.rotation.transpose() * derivatives.RDot);
  result.angular_acceleration = main_math::vee(
    result.rotation.transpose() * derivatives.RDDot -
    main_math::hat(result.body_rate) * main_math::hat(result.body_rate));
  if (!result.body_rate.allFinite()) {
    result.body_rate.setZero();
  }
  if (!result.angular_acceleration.allFinite()) {
    result.angular_acceleration.setZero();
  }
  return result;
}

DesiredAttitudeKinematics closedLoopDesiredAttitudeKinematics(
  const VehicleState & state, const FlatReference & reference,
  const ControllerParams & params,
  const Eigen::Vector3d & body_z_acceleration)
{
  const Eigen::Matrix3d rotation = quat2RotMatrix(state.attitude);
  const double specific_thrust = body_z_acceleration.norm();
  const Eigen::Vector3d b3 = rotation.col(2);
  const Eigen::Vector3d b3_dot =
    rotation * main_math::hat(state.body_rate) * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d actual_acceleration =
    params.gravity - specific_thrust * b3;
  const Eigen::Vector3d body_z_acceleration_dot =
    params.Kp.asDiagonal() * (state.velocity - reference.velocity) +
    params.Kv.asDiagonal() * (actual_acceleration - reference.acceleration) -
    reference.jerk;
  const double thrust_dot = specific_thrust > kEpsilon ?
    body_z_acceleration.normalized().dot(body_z_acceleration_dot) : 0.0;
  const Eigen::Vector3d actual_acceleration_dot =
    -thrust_dot * b3 - specific_thrust * b3_dot;
  const Eigen::Vector3d body_z_acceleration_ddot =
    params.Kp.asDiagonal() * (actual_acceleration - reference.acceleration) +
    params.Kv.asDiagonal() * (actual_acceleration_dot - reference.jerk) -
    reference.snap;
  return desiredAttitudeKinematics(
    body_z_acceleration, body_z_acceleration_dot,
    body_z_acceleration_ddot, reference, rotation);
}

ControllerCommand commandFromWrench(
  const FlatReference & reference, const Eigen::Vector3d & acceleration,
  const DesiredAttitudeKinematics & desired, double thrust,
  const Eigen::Vector3d & torque)
{
  ControllerCommand command;
  command.torque = torque;
  command.attitude = desired.quaternion;
  command.reference_position = reference.position;
  command.desired_acceleration = acceleration;
  command.desired_body_rate = desired.body_rate;
  command.desired_angular_acceleration = desired.angular_acceleration;
  command.collective_thrust = thrust;
  return command;
}

Eigen::Vector3d angularAccelerationCommand(
  const VehicleState & state, const ControllerParams & params,
  const DesiredAttitudeKinematics & desired)
{
  const Eigen::Matrix3d rotation = quat2RotMatrix(state.attitude);
  const Eigen::Matrix3d relative = rotation.transpose() * desired.rotation;
  const Eigen::Vector3d rate_in_body = relative * desired.body_rate;
  const Eigen::Vector3d acceleration_in_body =
    relative * desired.angular_acceleration -
    main_math::hat(state.body_rate) * rate_in_body;
  return params.KR.asDiagonal() * main_math::logSO3(relative) +
         params.KOmega.asDiagonal() * (rate_in_body - state.body_rate) +
         acceleration_in_body;
}

Eigen::Vector3d rigidBodyTorque(
  const VehicleState & state, const ControllerParams & params,
  const Eigen::Vector3d & angular_acceleration)
{
  return state.body_rate.cross(params.inertia * state.body_rate) +
         params.inertia * angular_acceleration;
}

DesiredAttitudeKinematics sunDesiredCommand(
  const VehicleState & state, const FlatReference & reference,
  const Eigen::Vector3d & body_z_force, double thrust_for_rates,
  const ControllerParams & params)
{
  const Eigen::Matrix3d rotation = quat2RotMatrix(state.attitude);
  DesiredAttitudeKinematics desired;
  desired.rotation = safeAttitudeFromBodyZ(body_z_force, reference.yaw, rotation);
  desired.quaternion = main_math::matrixToQuaternion(desired.rotation);
  if (thrust_for_rates > kEpsilon) {
    const auto rates = main_math::sunFlatnessReferenceRates(
      state, reference, thrust_for_rates / params.mass);
    desired.body_rate = rates.omega;
    desired.angular_acceleration = rates.alpha;
  }
  return desired;
}

Eigen::Vector3d sunAngularAcceleration(
  const VehicleState & state, const DesiredAttitudeKinematics & desired,
  const ControllerParams & params)
{
  Eigen::Vector4d q = state.attitude;
  q.normalize();
  const Eigen::Vector4d q_inverse(q[0], -q[1], -q[2], -q[3]);
  Eigen::Vector4d q_error =
    main_math::quaternionMultiply(q_inverse, desired.quaternion);
  q_error.normalize();
  const double denominator =
    std::sqrt(q_error[0] * q_error[0] + q_error[3] * q_error[3]);
  Eigen::Vector3d reduced = Eigen::Vector3d::Zero();
  Eigen::Vector3d yaw = Eigen::Vector3d::Zero();
  if (denominator > kEpsilon) {
    reduced << (q_error[0] * q_error[1] - q_error[2] * q_error[3]) / denominator,
      (q_error[0] * q_error[2] + q_error[1] * q_error[3]) / denominator, 0.0;
    yaw.z() = q_error[3] / denominator;
  }
  Eigen::Vector3d feedback;
  feedback <<
    2.0 * params.KR.x() * reduced.x(),
    2.0 * params.KR.y() * reduced.y(),
    params.KR.z() * (q_error[0] < 0.0 ? -1.0 : 1.0) * yaw.z();
  return feedback +
         params.KOmega.asDiagonal() * (desired.body_rate - state.body_rate) +
         desired.angular_acceleration;
}

}  // namespace

ControllerCommand LegacyGeometricController::update(
  const VehicleState & state, const FlatReference & reference,
  const ControllerParams & params, double /* dt */)
{
  const Eigen::Matrix3d rotation = quat2RotMatrix(state.attitude);
  const Eigen::Vector3d feedback =
    params.Kp.asDiagonal() * (reference.position - state.position) +
    params.Kv.asDiagonal() * (reference.velocity - state.velocity);
  const Eigen::Vector3d acceleration =
    reference.acceleration + feedback;
  const Eigen::Vector3d body_z_acceleration = params.gravity - acceleration;
  const double specific_thrust = body_z_acceleration.norm();
  const Eigen::Vector3d b3 = rotation.col(2);
  const Eigen::Vector3d b3_dot =
    rotation * main_math::hat(state.body_rate) * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d actual_acceleration =
    params.gravity - specific_thrust * b3;
  const Eigen::Vector3d body_z_acceleration_dot =
    params.Kp.asDiagonal() * (state.velocity - reference.velocity) +
    params.Kv.asDiagonal() * (actual_acceleration - reference.acceleration) -
    reference.jerk;
  const double thrust_dot = specific_thrust > kEpsilon ?
    body_z_acceleration.normalized().dot(body_z_acceleration_dot) : 0.0;
  const Eigen::Vector3d actual_acceleration_dot =
    -thrust_dot * b3 - specific_thrust * b3_dot;
  const Eigen::Vector3d body_z_acceleration_ddot =
    params.Kp.asDiagonal() * (actual_acceleration - reference.acceleration) +
    params.Kv.asDiagonal() * (actual_acceleration_dot - reference.jerk) -
    reference.snap;
  auto desired = desiredAttitudeKinematics(
    body_z_acceleration, body_z_acceleration_dot,
    body_z_acceleration_ddot, reference, rotation);

  const Eigen::Matrix3d relative = rotation.transpose() * desired.rotation;
  Eigen::Vector3d attitude_error;
  if (params.ctrl_mode == kErrorGeometric) {
    attitude_error = main_math::logSO3(relative);
  } else {
    attitude_error =
      main_math::quaternionAttitudeError(state.attitude, desired.quaternion);
  }
  const Eigen::Vector3d desired_rate_in_body =
    relative * desired.body_rate;
  const Eigen::Vector3d desired_acceleration_in_body =
    relative * desired.angular_acceleration -
    main_math::hat(state.body_rate) * desired_rate_in_body;
  const Eigen::Vector3d angular_acceleration =
    params.KR.asDiagonal() * attitude_error +
    params.KOmega.asDiagonal() * (desired_rate_in_body - state.body_rate) +
    desired_acceleration_in_body;
  const Eigen::Vector3d torque =
    rigidBodyTorque(state, params, angular_acceleration);
  auto command = commandFromWrench(
    reference, acceleration, desired, params.mass * specific_thrust, torque);
  command.desired_body_rate = desired_rate_in_body;
  command.desired_angular_acceleration = desired_acceleration_in_body;
  return command;
}

ControllerCommand MainGeometricController::update(
  const VehicleState & state, const FlatReference & reference,
  const ControllerParams & params, double /* dt */)
{
  const Eigen::Matrix3d rotation = quat2RotMatrix(state.attitude);
  const Eigen::Vector3d acceleration =
    accelerationCommand(state, reference, params);
  const Eigen::Vector3d body_z_acceleration = params.gravity - acceleration;
  const double specific_thrust = body_z_acceleration.norm();
  const auto desired = closedLoopDesiredAttitudeKinematics(
    state, reference, params, body_z_acceleration);
  const Eigen::Vector3d angular_acceleration =
    angularAccelerationCommand(state, params, desired);
  const Eigen::Vector3d torque =
    rigidBodyTorque(state, params, angular_acceleration);
  auto command = commandFromWrench(
    reference, acceleration, desired, params.mass * specific_thrust, torque);
  command.desired_body_rate =
    rotation.transpose() * desired.rotation * desired.body_rate;
  command.desired_angular_acceleration = angular_acceleration;
  return command;
}

ControllerCommand MainLeeController::update(
  const VehicleState & state, const FlatReference & reference,
  const ControllerParams & params, double /* dt */)
{
  const Eigen::Matrix3d rotation = quat2RotMatrix(state.attitude);
  const Eigen::Vector3d position_error = state.position - reference.position;
  const Eigen::Vector3d velocity_error = state.velocity - reference.velocity;
  const Eigen::Vector3d body_z_force =
    params.mass * (
    params.Kp.asDiagonal() * position_error +
    params.Kv.asDiagonal() * velocity_error +
    params.gravity - reference.acceleration);
  const double thrust = body_z_force.dot(rotation.col(2));
  const Eigen::Vector3d b3_dot =
    rotation * main_math::hat(state.body_rate) * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d actual_acceleration =
    params.gravity - (thrust / params.mass) * rotation.col(2);
  const Eigen::Vector3d force_dot =
    params.mass * (
    params.Kp.asDiagonal() * velocity_error +
    params.Kv.asDiagonal() * (actual_acceleration - reference.acceleration) -
    reference.jerk);
  const double thrust_dot =
    force_dot.dot(rotation.col(2)) + body_z_force.dot(b3_dot);
  const Eigen::Vector3d actual_acceleration_dot =
    -(thrust_dot / params.mass) * rotation.col(2) -
    (thrust / params.mass) * b3_dot;
  const Eigen::Vector3d force_ddot =
    params.mass * (
    params.Kp.asDiagonal() * (actual_acceleration - reference.acceleration) +
    params.Kv.asDiagonal() * (actual_acceleration_dot - reference.jerk) -
    reference.snap);
  const auto desired = desiredAttitudeKinematics(
    body_z_force, force_dot, force_ddot, reference, rotation);
  const Eigen::Vector3d attitude_error =
    main_math::leeSO3Error(rotation, desired.rotation);
  const Eigen::Vector3d desired_rate_in_body =
    rotation.transpose() * desired.rotation * desired.body_rate;
  const Eigen::Vector3d rate_error = state.body_rate - desired_rate_in_body;
  const Eigen::Vector3d torque =
    -params.inertia * params.KR.asDiagonal() * attitude_error -
    params.inertia * params.KOmega.asDiagonal() * rate_error +
    state.body_rate.cross(params.inertia * state.body_rate) -
    params.inertia * (
    main_math::hat(state.body_rate) * desired_rate_in_body -
    rotation.transpose() * desired.rotation * desired.angular_acceleration);
  return commandFromWrench(reference, actual_acceleration, desired, thrust, torque);
}

void MainJohnsonController::reset(const VehicleState & /* state */)
{
  integral_error_.setZero();
  initialized_ = false;
}

ControllerCommand MainJohnsonController::update(
  const VehicleState & state, const FlatReference & reference,
  const ControllerParams & params, double dt)
{
  const Eigen::Matrix3d rotation = quat2RotMatrix(state.attitude);
  const Eigen::Vector3d position_error = state.position - reference.position;
  const Eigen::Vector3d velocity_error = state.velocity - reference.velocity;
  if (!initialized_ || dt <= 0.0) {
    integral_error_.setZero();
    initialized_ = true;
  } else {
    integral_error_ += dt * position_error;
  }
  const Eigen::Vector3d force =
    params.mass * (
    -(params.Kp.asDiagonal() * position_error) -
    (params.Kv.asDiagonal() * velocity_error) +
    reference.acceleration - params.gravity);
  const Eigen::Vector3d body_z_force = -force;
  const double thrust = force.norm();
  const Eigen::Vector3d actual_acceleration =
    params.gravity - (thrust / params.mass) * rotation.col(2);
  const Eigen::Vector3d force_dot =
    params.mass * (
    -(params.Kp.asDiagonal() * velocity_error) -
    (params.Kv.asDiagonal() * (actual_acceleration - reference.acceleration)) +
    reference.jerk);
  const Eigen::Vector3d body_z_force_dot = -force_dot;
  const double thrust_dot = body_z_force.norm() > kEpsilon ?
    body_z_force.normalized().dot(body_z_force_dot) : 0.0;
  const Eigen::Vector3d b3_dot =
    rotation * main_math::hat(state.body_rate) * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d actual_acceleration_dot =
    -(thrust_dot / params.mass) * rotation.col(2) -
    (thrust / params.mass) * b3_dot;
  const Eigen::Vector3d force_ddot =
    params.mass * (
    -(params.Kp.asDiagonal() * (actual_acceleration - reference.acceleration)) -
    (params.Kv.asDiagonal() * (actual_acceleration_dot - reference.jerk)) +
    reference.snap);
  const auto desired = desiredAttitudeKinematics(
    body_z_force, body_z_force_dot, -force_ddot, reference, rotation);
  const Eigen::Matrix3d relative = rotation.transpose() * desired.rotation;
  const Eigen::Vector3d attitude_error = main_math::johnsonLogSO3(relative);
  const Eigen::Vector3d desired_rate_in_body = relative * desired.body_rate;
  const Eigen::Vector3d rate_error = desired_rate_in_body - state.body_rate;
  const Eigen::Vector3d acceleration_in_body =
    relative * desired.angular_acceleration -
    main_math::hat(state.body_rate) * desired_rate_in_body;
  const Eigen::Matrix3d jacobian =
    main_math::johnsonLeftJacobianSO3(attitude_error);
  const Eigen::Vector3d torque =
    state.body_rate.cross(params.inertia * state.body_rate) +
    params.inertia * acceleration_in_body +
    jacobian.transpose() * params.inertia *
    params.KR.asDiagonal() * attitude_error +
    params.inertia * params.KOmega.asDiagonal() * rate_error;
  auto command = commandFromWrench(
    reference, actual_acceleration, desired, thrust, torque);
  command.desired_body_rate = desired_rate_in_body;
  command.desired_angular_acceleration = acceleration_in_body;
  return command;
}

void MainSunDFBCController::reset(const VehicleState & /* state */)
{
  thrust_feedback_valid_ = false;
  previous_collective_thrust_ = 0.0;
}

ControllerCommand MainSunDFBCController::update(
  const VehicleState & state, const FlatReference & reference,
  const ControllerParams & params, double /* dt */)
{
  const Eigen::Vector3d acceleration =
    accelerationCommand(state, reference, params);
  const Eigen::Vector3d body_z_force =
    params.mass * (params.gravity - acceleration);
  const double thrust = body_z_force.norm();
  const double thrust_for_rates = thrust_feedback_valid_ ?
    previous_collective_thrust_ : thrust;
  auto desired = sunDesiredCommand(
    state, reference, body_z_force, thrust_for_rates, params);
  const Eigen::Vector3d desired_angular_acceleration =
    sunAngularAcceleration(state, desired, params);
  desired.angular_acceleration = desired_angular_acceleration;
  const Eigen::Vector3d torque =
    rigidBodyTorque(state, params, desired_angular_acceleration);

  previous_collective_thrust_ = thrust;
  thrust_feedback_valid_ = true;

  return commandFromWrench(
    reference, acceleration, desired, thrust, torque);
}

void MainGeometricINDIController::reset(const VehicleState & state)
{
  (void)state;
  commanded_body_z_force_.setZero();
  force_command_valid_ = false;
  outer_loop_elapsed_s_ = 0.0;
}

ControllerCommand MainGeometricINDIController::update(
  const VehicleState & state, const FlatReference & reference,
  const ControllerParams & params, double dt)
{
  const Eigen::Matrix3d rotation = quat2RotMatrix(state.attitude);
  const Eigen::Vector3d acceleration_command =
    accelerationCommand(state, reference, params);
  const Eigen::Vector3d direct_body_z_force =
    params.mass * (params.gravity - acceleration_command);

  // main.tex Eq. (55) runs at the configured outer-loop frequency. The
  // angular-rate-driven inner loop holds the most recent thrust-vector command.
  const double outer_period_s = 1.0 / params.outer_loop_rate_hz;
  outer_loop_elapsed_s_ += dt;
  const bool run_outer_loop =
    !force_command_valid_ || outer_loop_elapsed_s_ + 1e-12 >= outer_period_s;
  if (run_outer_loop) {
    commanded_body_z_force_ = params.indi_acceleration_enabled ?
      state.applied_thrust_axis_force -
      params.mass * (acceleration_command - state.acceleration) :
      direct_body_z_force;
    force_command_valid_ = true;
    outer_loop_elapsed_s_ = std::fmod(outer_loop_elapsed_s_, outer_period_s);
  }

  const double thrust = commanded_body_z_force_.norm();
  const Eigen::Vector3d body_z_force_for_attitude =
    params.indi_acceleration_enabled ?
    commanded_body_z_force_ : direct_body_z_force;
  // main.m uses the previous allocated thrust magnitude T_0 in the
  // flatness-rate map, while the attitude itself follows the command T*b3.
  const double thrust_for_rates = params.indi_acceleration_enabled ?
    state.applied_thrust_axis_force.norm() : direct_body_z_force.norm();
  auto desired = sunDesiredCommand(
    state, reference, body_z_force_for_attitude,
    thrust_for_rates, params);
  const Eigen::Vector3d attitude_error =
    main_math::logSO3(rotation.transpose() * desired.rotation);
  // main.tex Eq. (61).
  const Eigen::Vector3d angular_acceleration_command =
    params.KR.asDiagonal() * attitude_error +
    params.KOmega.asDiagonal() * (desired.body_rate - state.body_rate) +
    desired.angular_acceleration;
  // main.tex Eq. (60): the PX4 allocator publishes tau_0 continuously before
  // offboard starts, so every ROS INDI update uses the same feedback form.
  const Eigen::Vector3d torque = state.applied_torque +
    params.inertia * (angular_acceleration_command - state.angular_acceleration);
  auto command = commandFromWrench(
    reference, acceleration_command, desired, thrust, torque);
  command.desired_angular_acceleration = angular_acceleration_command;
  // PCA priority decomposition is intentionally disabled while validating the
  // base INDI loop. The transport publishes only the total torque above.
  return command;
}

}  // namespace geometric_controller
