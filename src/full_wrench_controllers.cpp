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
#include "geometric_controller/controllers/main_tal_controller.hpp"

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

Eigen::Vector3d saturateFeedback(
  const Eigen::Vector3d & value, const ControllerParams & params)
{
  if (params.max_feedback_acc > 0.0 && value.norm() > params.max_feedback_acc) {
    return value * params.max_feedback_acc / value.norm();
  }
  return value;
}

Eigen::Vector3d accelerationCommand(
  const VehicleState & state, const FlatReference & reference,
  const ControllerParams & params)
{
  const Eigen::Vector3d feedback =
    params.Kp.asDiagonal() * (reference.position - state.position) +
    params.Kv.asDiagonal() * (reference.velocity - state.velocity);
  return reference.acceleration + saturateFeedback(feedback, params);
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

void initializeFilter(SecondOrderFilterState & state, const Eigen::Vector3d & raw)
{
  state.x1 = raw;
  state.x2 = raw;
  state.y1 = raw;
  state.y2 = raw;
  state.initialized = true;
}

Eigen::Vector3d filterValue(
  SecondOrderFilterState & state, const Eigen::Vector3d & raw,
  double dt, double cutoff_hz)
{
  if (!state.initialized || cutoff_hz <= 0.0 || dt <= 0.0) {
    initializeFilter(state, raw);
    return raw;
  }
  const double sample_rate = 1.0 / dt;
  const double cutoff = std::min(cutoff_hz, 0.45 * sample_rate);
  const double k = std::tan(M_PI * cutoff / sample_rate);
  const double norm = 1.0 / (1.0 + std::sqrt(2.0) * k + k * k);
  const double b0 = k * k * norm;
  const double b1 = 2.0 * b0;
  const double b2 = b0;
  const double a1 = 2.0 * (k * k - 1.0) * norm;
  const double a2 = (1.0 - std::sqrt(2.0) * k + k * k) * norm;
  const Eigen::Vector3d output =
    b0 * raw + b1 * state.x1 + b2 * state.x2 -
    a1 * state.y1 - a2 * state.y2;
  state.x2 = state.x1;
  state.x1 = raw;
  state.y2 = state.y1;
  state.y1 = output;
  return output;
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

Eigen::Vector3d quaternionLogVector(const Eigen::Matrix3d & relative)
{
  Eigen::Vector4d q = main_math::matrixToQuaternion(relative);
  if (q[0] < 0.0) {
    q = -q;
  }
  const double vector_norm = q.tail<3>().norm();
  if (vector_norm < kEpsilon) {
    return 2.0 * q.tail<3>();
  }
  return 2.0 * std::atan2(vector_norm, std::clamp(q[0], -1.0, 1.0)) *
         q.tail<3>() / vector_norm;
}

Eigen::Vector3d talYawRow(const Eigen::Matrix3d & rotation)
{
  const Eigen::Vector3d body_x = rotation.col(0);
  const double denominator =
    body_x.x() * body_x.x() + body_x.y() * body_x.y();
  if (denominator < kEpsilon) {
    return Eigen::Vector3d::UnitZ();
  }
  Eigen::Vector3d row;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d basis = Eigen::Vector3d::Zero();
    basis[axis] = 1.0;
    const Eigen::Vector3d derivative =
      rotation * main_math::hat(basis) * Eigen::Vector3d::UnitX();
    row[axis] =
      (body_x.x() * derivative.y() - body_x.y() * derivative.x()) /
      denominator;
  }
  return row;
}

double talYawRowDerivativeTimesRate(
  const Eigen::Matrix3d & rotation, const Eigen::Vector3d & omega)
{
  const Eigen::Vector3d body_x = rotation.col(0);
  const Eigen::Vector3d body_x_dot =
    rotation * main_math::hat(omega) * Eigen::Vector3d::UnitX();
  const double denominator =
    body_x.x() * body_x.x() + body_x.y() * body_x.y();
  if (denominator < kEpsilon) {
    return 0.0;
  }
  const double denominator_dot =
    2.0 * body_x.x() * body_x_dot.x() +
    2.0 * body_x.y() * body_x_dot.y();
  Eigen::Vector3d row_dot;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d basis = Eigen::Vector3d::Zero();
    basis[axis] = 1.0;
    const Eigen::Vector3d directional =
      rotation * main_math::hat(basis) * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d directional_dot =
      rotation * main_math::hat(omega) * main_math::hat(basis) *
      Eigen::Vector3d::UnitX();
    const double numerator =
      body_x.x() * directional.y() - body_x.y() * directional.x();
    const double numerator_dot =
      body_x_dot.x() * directional.y() +
      body_x.x() * directional_dot.y() -
      body_x_dot.y() * directional.x() -
      body_x.y() * directional_dot.x();
    row_dot[axis] =
      (numerator_dot * denominator - numerator * denominator_dot) /
      (denominator * denominator);
  }
  return row_dot.dot(omega);
}

void talFlatnessFeedforward(
  const Eigen::Matrix3d & rotation, double signed_specific_thrust,
  const FlatReference & reference, Eigen::Vector3d & omega,
  Eigen::Vector3d & angular_acceleration)
{
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Zero();
  matrix.block<3, 3>(0, 0) =
    -signed_specific_thrust * rotation *
    main_math::hat(Eigen::Vector3d::UnitZ());
  matrix.block<3, 1>(0, 3) = rotation.col(2);
  matrix.block<1, 3>(3, 0) = talYawRow(rotation).transpose();

  Eigen::Vector4d jerk_rhs;
  jerk_rhs << reference.jerk, reference.yaw_rate;
  const Eigen::Vector4d jerk_solution =
    matrix.completeOrthogonalDecomposition().solve(jerk_rhs);
  omega = jerk_solution.head<3>();
  const double thrust_dot = jerk_solution[3];

  const Eigen::Matrix3d omega_hat = main_math::hat(omega);
  const Eigen::Vector3d known_snap =
    rotation *
    (2.0 * thrust_dot * omega_hat +
    signed_specific_thrust * omega_hat * omega_hat) *
    Eigen::Vector3d::UnitZ();
  Eigen::Vector4d snap_rhs;
  snap_rhs << reference.snap - known_snap,
    reference.yaw_accel - talYawRowDerivativeTimesRate(rotation, omega);
  const Eigen::Vector4d snap_solution =
    matrix.completeOrthogonalDecomposition().solve(snap_rhs);
  angular_acceleration = snap_solution.head<3>();
  if (!omega.allFinite() || !angular_acceleration.allFinite()) {
    omega.setZero();
    angular_acceleration.setZero();
  }
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
    reference.acceleration + saturateFeedback(feedback, params);
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
  const Eigen::Vector3d b3 = rotation.col(2);
  const Eigen::Vector3d b3_dot =
    rotation * main_math::hat(state.body_rate) * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d actual_acceleration =
    params.gravity - specific_thrust * b3;
  const Eigen::Vector3d vector_dot =
    params.Kp.asDiagonal() * (state.velocity - reference.velocity) +
    params.Kv.asDiagonal() * (actual_acceleration - reference.acceleration) -
    reference.jerk;
  const double thrust_dot = specific_thrust > kEpsilon ?
    body_z_acceleration.normalized().dot(vector_dot) : 0.0;
  const Eigen::Vector3d actual_acceleration_dot =
    -thrust_dot * b3 - specific_thrust * b3_dot;
  const Eigen::Vector3d vector_ddot =
    params.Kp.asDiagonal() * (actual_acceleration - reference.acceleration) +
    params.Kv.asDiagonal() * (actual_acceleration_dot - reference.jerk) -
    reference.snap;
  const auto desired = desiredAttitudeKinematics(
    body_z_acceleration, vector_dot, vector_ddot, reference, rotation);
  const Eigen::Vector3d angular_acceleration =
    angularAccelerationCommand(state, params, desired);
  const Eigen::Vector3d torque =
    rigidBodyTorque(state, params, angular_acceleration);
  auto command = commandFromWrench(
    reference, acceleration, desired, params.mass * specific_thrust, torque);
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

void MainSunDFBCController::reset(const VehicleState & state)
{
  initialized_ = false;
  previous_omega_ = state.body_rate;
  previous_torque_.setZero();
  previous_thrust_ = 0.0;
  thrust_filter_ = {};
  torque_filter_ = {};
}

ControllerCommand MainSunDFBCController::update(
  const VehicleState & state, const FlatReference & reference,
  const ControllerParams & params, double dt)
{
  const Eigen::Vector3d acceleration =
    accelerationCommand(state, reference, params);
  const Eigen::Vector3d body_z_force =
    params.mass * (params.gravity - acceleration);
  const double thrust = body_z_force.norm();
  const bool first_update = !initialized_;
  double thrust_for_rates = thrust;
  if (indi_variant_) {
    if (first_update || dt <= 0.0) {
      initializeFilter(
        thrust_filter_, Eigen::Vector3d(thrust, 0.0, 0.0));
    } else {
      // The previous requested collective force is an INDI input feedback,
      // so filter it just like the previous requested moment. This is
      // separate from PX4's already-filtered angular-acceleration feedback.
      thrust_for_rates = filterValue(
        thrust_filter_, Eigen::Vector3d(previous_thrust_, 0.0, 0.0),
        dt, params.indi_filter_cutoff_hz).x();
    }
  } else if (!first_update) {
    thrust_for_rates = previous_thrust_;
  }
  auto desired = sunDesiredCommand(
    state, reference, body_z_force, thrust_for_rates, params);
  const Eigen::Vector3d desired_angular_acceleration =
    sunAngularAcceleration(state, desired, params);
  desired.angular_acceleration = desired_angular_acceleration;
  Eigen::Vector3d torque =
    rigidBodyTorque(state, params, desired_angular_acceleration);

  if (indi_variant_) {
    if (first_update || dt <= 0.0) {
      previous_omega_ = state.body_rate;
      initializeFilter(torque_filter_, torque);
    } else {
      // VehicleAngularVelocity.xyz_derivative is already differentiated and
      // second-order low-pass filtered by PX4 (IMU_DGYRO_CUTOFF). Do not add
      // another ROS-side LPF and its phase delay.
      const Eigen::Vector3d omega_dot = state.angular_acceleration_valid ?
        state.angular_acceleration : (state.body_rate - previous_omega_) / dt;
      const Eigen::Vector3d torque_filtered = filterValue(
        torque_filter_, previous_torque_, dt, params.indi_filter_cutoff_hz);
      torque = torque_filtered +
        params.inertia * (desired_angular_acceleration - omega_dot);
    }
  }
  initialized_ = true;
  previous_omega_ = state.body_rate;
  previous_torque_ = torque;
  previous_thrust_ = thrust;

  return commandFromWrench(
    reference, acceleration, desired, thrust, torque);
}

void MainTalController::reset(const VehicleState & state)
{
  initialized_ = false;
  previous_velocity_ = state.velocity;
  previous_omega_ = state.body_rate;
  previous_torque_.setZero();
  previous_thrust_ = 0.0;
  acceleration_filter_ = {};
  omega_filter_ = {};
  thrust_acceleration_filter_ = {};
  torque_filter_ = {};
}

ControllerCommand MainTalController::update(
  const VehicleState & state, const FlatReference & reference,
  const ControllerParams & params, double dt)
{
  const Eigen::Matrix3d rotation = quat2RotMatrix(state.attitude);
  const double h = std::max(dt, 1e-4);
  // main.m controllerTal starts Eq. (20) from the reference acceleration,
  // not from an arbitrary estimator sample captured while changing control
  // modes. The measured values enter through the LPFs from the next sample.
  Eigen::Vector3d acceleration_filtered = reference.acceleration;
  Eigen::Vector3d omega_filtered = state.body_rate;
  Eigen::Vector3d omega_dot_filtered = Eigen::Vector3d::Zero();
  Eigen::Vector3d thrust_acceleration_filtered =
    reference.acceleration - params.gravity;
  Eigen::Vector3d torque_filtered = Eigen::Vector3d::Zero();

  if (!initialized_) {
    initializeFilter(acceleration_filter_, acceleration_filtered);
    initializeFilter(omega_filter_, omega_filtered);
    initializeFilter(
      thrust_acceleration_filter_, thrust_acceleration_filtered);
    initializeFilter(torque_filter_, torque_filtered);
    initialized_ = true;
  } else {
    const Eigen::Vector3d raw_acceleration = state.acceleration_valid ?
      state.acceleration : (state.velocity - previous_velocity_) / h;
    omega_dot_filtered = state.angular_acceleration_valid ?
      state.angular_acceleration : (state.body_rate - previous_omega_) / h;
    // The measurement source reproduces PX4's NED velocity-derivative
    // pipeline. This controller LPF remains part of Tal's Fig. 4 control
    // law, independently of how the measurement was obtained.
    acceleration_filtered = filterValue(
      acceleration_filter_, raw_acceleration, h, params.indi_filter_cutoff_hz);
    omega_filtered = filterValue(
      omega_filter_, state.body_rate, h, params.indi_filter_cutoff_hz);
    const Eigen::Vector3d previous_thrust_acceleration =
      -(previous_thrust_ / params.mass) * rotation.col(2);
    thrust_acceleration_filtered = filterValue(
      thrust_acceleration_filter_, previous_thrust_acceleration,
      h, params.indi_filter_cutoff_hz);
    torque_filtered = filterValue(
      torque_filter_, previous_torque_, h, params.indi_filter_cutoff_hz);
  }

  const Eigen::Vector3d acceleration =
    accelerationCommand(state, reference, params);
  const Eigen::Vector3d thrust_acceleration =
    thrust_acceleration_filtered + acceleration - acceleration_filtered;
  const Eigen::Vector3d body_z_force = -params.mass * thrust_acceleration;
  const double thrust = body_z_force.norm();
  auto desired = sunDesiredCommand(
    state, reference, body_z_force, thrust, params);
  const Eigen::Vector3d attitude_error =
    quaternionLogVector(rotation.transpose() * desired.rotation);

  const Eigen::Vector3d feedforward_body_z =
    params.gravity - reference.acceleration;
  const double signed_specific_thrust = -feedforward_body_z.norm();
  const Eigen::Matrix3d feedforward_rotation =
    safeAttitudeFromBodyZ(feedforward_body_z, reference.yaw, rotation);
  Eigen::Vector3d omega_reference;
  Eigen::Vector3d alpha_reference;
  talFlatnessFeedforward(
    feedforward_rotation, signed_specific_thrust, reference,
    omega_reference, alpha_reference);
  const Eigen::Vector3d angular_acceleration =
    params.KR.asDiagonal() * attitude_error +
    params.KOmega.asDiagonal() * (omega_reference - omega_filtered) +
    alpha_reference;
  const Eigen::Vector3d torque =
    torque_filtered +
    params.inertia * (angular_acceleration - omega_dot_filtered);

  desired.body_rate = omega_reference;
  desired.angular_acceleration = alpha_reference;
  previous_velocity_ = state.velocity;
  previous_omega_ = state.body_rate;
  previous_torque_ = torque;
  previous_thrust_ = thrust;
  return commandFromWrench(
    reference, acceleration, desired, thrust, torque);
}

void MainGeometricINDIController::reset(const VehicleState & state)
{
  initialized_ = false;
  previous_velocity_ = state.velocity;
  previous_omega_ = state.body_rate;
  previous_torque_.setZero();
  previous_thrust_ = 0.0;
  acceleration_filter_ = {};
  omega_filter_ = {};
  thrust_axis_filter_ = {};
  torque_filter_ = {};
}

ControllerCommand MainGeometricINDIController::update(
  const VehicleState & state, const FlatReference & reference,
  const ControllerParams & params, double dt)
{
  const Eigen::Matrix3d rotation = quat2RotMatrix(state.attitude);
  const double h = std::max(dt, 1e-4);
  const bool first_update = !initialized_;
  const Eigen::Vector3d acceleration =
    accelerationCommand(state, reference, params);
  Eigen::Vector3d acceleration_feedback = state.acceleration_valid ?
    state.acceleration : acceleration;
  Eigen::Vector3d omega_feedback = state.body_rate;
  Eigen::Vector3d omega_dot_feedback = state.angular_acceleration_valid ?
    state.angular_acceleration : Eigen::Vector3d::Zero();
  Eigen::Vector3d body_z_force_feedback =
    params.mass * (params.gravity - acceleration);
  Eigen::Vector3d torque_feedback = Eigen::Vector3d::Zero();

  if (first_update) {
    // The first GINDI sample is direct rigid-body inversion.  Seed the
    // incremental feedback filters with the response of that applied
    // command below, as initGeometricINDIState() does in main.m, instead of
    // seeding them with unrelated pre-switch estimator derivatives.
    initializeFilter(acceleration_filter_, acceleration_feedback);
    initializeFilter(omega_filter_, omega_feedback);
    initializeFilter(thrust_axis_filter_, body_z_force_feedback);
    initializeFilter(torque_filter_, torque_feedback);
    initialized_ = true;
  } else {
    const Eigen::Vector3d raw_acceleration = state.acceleration_valid ?
      state.acceleration : (state.velocity - previous_velocity_) / h;
    omega_dot_feedback = state.angular_acceleration_valid ?
      state.angular_acceleration : (state.body_rate - previous_omega_) / h;
    // Keep main.m's outer-loop Butterworth feedback filter. The node-level
    // velocity derivative defines the measurement source; it does not
    // replace the filtering that belongs to the GINDI control law.
    acceleration_feedback = filterValue(
      acceleration_filter_, raw_acceleration, h, params.indi_filter_cutoff_hz);
    omega_feedback = filterValue(
      omega_filter_, state.body_rate, h, params.indi_filter_cutoff_hz);
    body_z_force_feedback = filterValue(
      thrust_axis_filter_, previous_thrust_ * rotation.col(2),
      h, params.indi_filter_cutoff_hz);
    torque_feedback = filterValue(
      torque_filter_, previous_torque_, h, params.indi_filter_cutoff_hz);
  }

  Eigen::Vector3d body_z_force;
  if (first_update) {
    body_z_force = params.mass * (params.gravity - acceleration);
  } else {
    body_z_force = body_z_force_feedback -
      params.mass * (acceleration - acceleration_feedback);
  }
  const double thrust = body_z_force.norm();
  auto desired = sunDesiredCommand(
    state, reference, body_z_force, body_z_force_feedback.norm(), params);
  const Eigen::Vector3d attitude_error =
    main_math::logSO3(rotation.transpose() * desired.rotation);
  const Eigen::Vector3d angular_acceleration =
    params.KR.asDiagonal() * attitude_error +
    params.KOmega.asDiagonal() * (desired.body_rate - omega_feedback) +
    desired.angular_acceleration;
  const Eigen::Vector3d torque = first_update ?
    rigidBodyTorque(state, params, angular_acceleration) :
    torque_feedback +
    params.inertia * (angular_acceleration - omega_dot_feedback);

  if (first_update) {
    const Eigen::Vector3d applied_acceleration =
      params.gravity - (thrust / params.mass) * rotation.col(2);
    initializeFilter(acceleration_filter_, applied_acceleration);
    initializeFilter(omega_filter_, state.body_rate);
    initializeFilter(thrust_axis_filter_, thrust * rotation.col(2));
    initializeFilter(torque_filter_, torque);
  }

  previous_velocity_ = state.velocity;
  previous_omega_ = state.body_rate;
  previous_torque_ = torque;
  previous_thrust_ = thrust;
  auto command = commandFromWrench(
    reference, acceleration, desired, thrust, torque);
  command.desired_angular_acceleration = angular_acceleration;
  return command;
}

}  // namespace geometric_controller
