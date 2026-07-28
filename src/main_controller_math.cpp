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

#include "geometric_controller/controllers/main_controller_math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace geometric_controller
{
namespace main_math
{

Eigen::Matrix3d hat(const Eigen::Vector3d & w)
{
  Eigen::Matrix3d S;
  S << 0.0, -w.z(), w.y(), w.z(), 0.0, -w.x(), -w.y(), w.x(), 0.0;
  return S;
}

Eigen::Vector3d vee(const Eigen::Matrix3d & S)
{
  return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0));
}

Eigen::Vector3d logSO3(const Eigen::Matrix3d & R)
{
  const double cos_angle = std::max(-1.0, std::min(1.0, 0.5 * (R.trace() - 1.0)));
  const double angle = std::acos(cos_angle);
  const Eigen::Vector3d v = vee(0.5 * (R - R.transpose()));
  if (angle < 1e-6) {
    return v;
  }
  const double sin_angle = std::sin(angle);
  if (std::abs(sin_angle) < 1e-6) {
    return v;
  }
  return angle / sin_angle * v;
}

Eigen::Vector3d johnsonLogSO3(const Eigen::Matrix3d & R)
{
  const double cos_angle = std::max(-1.0, std::min(1.0, 0.5 * (R.trace() - 1.0)));
  const double angle = std::acos(cos_angle);

  if (std::abs(std::abs(angle) - M_PI) < 1e-6) {
    Eigen::EigenSolver<Eigen::Matrix3d> solver(R);
    int axis_index = 0;
    double min_distance_to_one = std::numeric_limits<double>::max();
    for (int i = 0; i < 3; ++i) {
      const double distance_to_one = std::abs(solver.eigenvalues()(i).real() - 1.0) +
                                     std::abs(solver.eigenvalues()(i).imag());
      if (distance_to_one < min_distance_to_one) {
            min_distance_to_one = distance_to_one;
            axis_index = i;
      }
    }

    const Eigen::Vector3d axis = solver.eigenvectors().col(axis_index).real();
    if (axis.norm() > 1e-9 && axis.allFinite()) {
      return angle * axis.normalized();
    }
  }

  return logSO3(R);
}

Eigen::Matrix3d johnsonLeftJacobianSO3(const Eigen::Vector3d & phi)
{
  const double angle = phi.norm();
  if (angle < 1e-8) {
    return Eigen::Matrix3d::Identity();
  }

  const Eigen::Vector3d axis = phi / angle;
  const Eigen::Matrix3d axis_hat = hat(axis);
  const double sinc_half = std::sin(0.5 * angle) / (0.5 * angle);
  const double sinc_full = std::sin(angle) / angle;
  return Eigen::Matrix3d::Identity() + std::sin(0.5 * angle) * sinc_half * axis_hat +
         (1.0 - sinc_full) * axis_hat * axis_hat;
}

Eigen::Vector3d leeSO3Error(const Eigen::Matrix3d & R, const Eigen::Matrix3d & Rd)
{
  return vee(0.5 * (Rd.transpose() * R - R.transpose() * Rd));
}

Eigen::Vector4d quaternionMultiply(const Eigen::Vector4d & q, const Eigen::Vector4d & p)
{
  Eigen::Vector4d quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3),
    p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
    p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1),
    p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}

Eigen::Vector3d quaternionAttitudeError(const Eigen::Vector4d & q, const Eigen::Vector4d & qd)
{
  const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
  const Eigen::Vector4d qe = quaternionMultiply(inverse.asDiagonal() * q, qd);
  return Eigen::Vector3d(2.0 * std::copysign(1.0, qe(0)) * qe(1),
                         2.0 * std::copysign(1.0, qe(0)) * qe(2),
                         2.0 * std::copysign(1.0, qe(0)) * qe(3));
}

Eigen::Vector4d matrixToQuaternion(const Eigen::Matrix3d & R)
{
  return rot2Quaternion(R);
}

Eigen::Vector4d attitudeFromSpecificForce(const Eigen::Vector3d & specific_force, double yaw)
{
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;
  zb_des = specific_force / specific_force.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2),
    zb_des(2);
  return matrixToQuaternion(rotmat);
}

Eigen::Matrix3d attitudeFromUnitBodyZAndHeading(
  const Eigen::Vector3d & b3d,
  const Eigen::Vector3d & xC)
{
  const Eigen::Vector3d C = b3d.cross(xC);
  const Eigen::Vector3d b2d = C / C.norm();
  const Eigen::Vector3d b1d = b2d.cross(b3d);
  Eigen::Matrix3d Rd;
  Rd.col(0) = b1d;
  Rd.col(1) = b2d;
  Rd.col(2) = b3d;
  return Rd;
}

HeadingDerivatives headingAxisFromYaw(const FlatReference & reference)
{
  HeadingDerivatives heading;
  const double psi = reference.yaw;
  const double psi_dot = reference.yaw_rate;
  const double psi_ddot = reference.yaw_accel;

  heading.xC << std::cos(psi), std::sin(psi), 0.0;
  heading.xCDot = psi_dot * Eigen::Vector3d(-std::sin(psi), std::cos(psi), 0.0);
  heading.xCDDot = psi_ddot * Eigen::Vector3d(-std::sin(psi), std::cos(psi), 0.0) -
    psi_dot * psi_dot * heading.xC;
  return heading;
}

UnitVectorDerivatives unitVectorDerivativesFromVector(
  const Eigen::Vector3d & v, const Eigen::Vector3d & vDot,
  const Eigen::Vector3d & vDDot)
{
  UnitVectorDerivatives result;
  const double rho = v.norm();
  if (rho < 1e-9) {
    return result;
  }

  result.b = v / rho;
  const Eigen::Matrix3d P = Eigen::Matrix3d::Identity() - result.b * result.b.transpose();
  result.bDot = P * vDot / rho;
  result.bDDot = P * vDDot / rho - 2.0 * result.b.dot(vDot) / rho * result.bDot -
    result.bDot.dot(result.bDot) * result.b;
  return result;
}

AttitudeDerivatives attitudeDerivativesFromUnitBodyZAndHeading(
  const Eigen::Vector3d & b3d,
  const Eigen::Vector3d & b3dDot,
  const Eigen::Vector3d & b3dDDot,
  const Eigen::Vector3d & xC,
  const Eigen::Vector3d & xCDot,
  const Eigen::Vector3d & xCDDot)
{
  const Eigen::Vector3d C = b3d.cross(xC);
  const Eigen::Vector3d CDot = b3dDot.cross(xC) + b3d.cross(xCDot);
  const Eigen::Vector3d CDDot = b3dDDot.cross(xC) + 2.0 * b3dDot.cross(xCDot) + b3d.cross(xCDDot);

  const UnitVectorDerivatives b2 = unitVectorDerivativesFromVector(C, CDot, CDDot);
  const Eigen::Vector3d b1dDot = b2.bDot.cross(b3d) + b2.b.cross(b3dDot);
  const Eigen::Vector3d b1dDDot = b2.bDDot.cross(b3d) + 2.0 * b2.bDot.cross(b3dDot) +
    b2.b.cross(b3dDDot);

  AttitudeDerivatives result;
  result.RDot.col(0) = b1dDot;
  result.RDot.col(1) = b2.bDot;
  result.RDot.col(2) = b3dDot;
  result.RDDot.col(0) = b1dDDot;
  result.RDDot.col(1) = b2.bDDot;
  result.RDDot.col(2) = b3dDDot;
  return result;
}

Eigen::Vector3d saturatedFeedbackAcceleration(
  const Eigen::Vector3d & feedback,
  double max_feedback_acc)
{
  if (feedback.norm() > max_feedback_acc) {
    return (max_feedback_acc / feedback.norm()) * feedback;
  }
  return feedback;
}

Eigen::Vector3d outerLoopAcceleration(
  const VehicleState & state, const FlatReference & reference,
  const ControllerParams & params)
{
  const Eigen::Vector3d ep = reference.position - state.position;
  const Eigen::Vector3d ev = reference.velocity - state.velocity;
  const Eigen::Vector3d a_fb =
    saturatedFeedbackAcceleration(params.Kp.asDiagonal() * ep + params.Kv.asDiagonal() * ev,
                                    params.max_feedback_acc);
  return reference.acceleration + a_fb;
}

Eigen::Vector3d closedLoopSpecificForceDerivative(
  const VehicleState & state, const FlatReference & reference,
  const ControllerParams & params, double thrust_accel)
{
  if (!std::isfinite(thrust_accel) || !state.attitude.allFinite() || state.attitude.norm() < 1e-6) {
    return reference.jerk;
  }

  const Eigen::Matrix3d R = quat2RotMatrix(state.attitude / state.attitude.norm());
  const Eigen::Vector3d predicted_acc = params.gravity + thrust_accel * R.col(2);
  return params.Kp.asDiagonal() * (reference.velocity - state.velocity) +
         params.Kv.asDiagonal() * (reference.acceleration - predicted_acc) + reference.jerk;
}

Eigen::Vector3d closedLoopSpecificForceSecondDerivative(
  const VehicleState & state, const FlatReference & reference,
  const ControllerParams & params, double thrust_accel,
  const Eigen::Vector3d & specific_force_dot)
{
  const Eigen::Matrix3d R = quat2RotMatrix(state.attitude / state.attitude.norm());
  const Eigen::Vector3d b3 = R.col(2);
  const Eigen::Vector3d b3_dot = R * hat(state.body_rate) * Eigen::Vector3d::UnitZ();
  const double thrust_accel_dot = b3.dot(specific_force_dot);
  const Eigen::Vector3d predicted_acc = params.gravity + thrust_accel * b3;
  const Eigen::Vector3d predicted_acc_dot = thrust_accel_dot * b3 + thrust_accel * b3_dot;
  return params.Kp.asDiagonal() * (reference.acceleration - predicted_acc) +
         params.Kv.asDiagonal() * (reference.jerk - predicted_acc_dot) + reference.snap;
}

Eigen::Vector3d referenceBodyRateFromForce(
  const Eigen::Vector3d & specific_force,
  const Eigen::Vector3d & specific_force_dot, double yaw,
  double yaw_rate)
{
  if (!specific_force.allFinite() || !specific_force_dot.allFinite()) {
    return Eigen::Vector3d::Zero();
  }

  const double force_norm = specific_force.norm();
  if (force_norm < 1e-6) {
    return Eigen::Vector3d::Zero();
  }

  const Eigen::Vector3d b3d = specific_force / force_norm;
  const Eigen::Vector3d b3d_dot =
    (Eigen::Matrix3d::Identity() - b3d * b3d.transpose()) * specific_force_dot / force_norm;
  const Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
  const Eigen::Vector3d heading_dot = yaw_rate * Eigen::Vector3d(-std::sin(yaw), std::cos(yaw),
        0.0);
  const Eigen::Vector3d b2_raw = b3d.cross(heading);
  const double b2_raw_norm = b2_raw.norm();
  if (b2_raw_norm < 1e-6) {
    return Eigen::Vector3d::Zero();
  }

  const Eigen::Vector3d b2d = b2_raw / b2_raw_norm;
  const Eigen::Vector3d b2_raw_dot = b3d_dot.cross(heading) + b3d.cross(heading_dot);
  const Eigen::Vector3d b2d_dot =
    (Eigen::Matrix3d::Identity() - b2d * b2d.transpose()) * b2_raw_dot / b2_raw_norm;
  const Eigen::Vector3d b1d = b2d.cross(b3d);
  const Eigen::Vector3d b1d_dot = b2d_dot.cross(b3d) + b2d.cross(b3d_dot);

  Eigen::Matrix3d Rd;
  Rd.col(0) = b1d;
  Rd.col(1) = b2d;
  Rd.col(2) = b3d;

  Eigen::Matrix3d Rd_dot;
  Rd_dot.col(0) = b1d_dot;
  Rd_dot.col(1) = b2d_dot;
  Rd_dot.col(2) = b3d_dot;

  const Eigen::Matrix3d omega_hat = 0.5 * (Rd.transpose() * Rd_dot - Rd_dot.transpose() * Rd);
  const Eigen::Vector3d omega_ref = vee(omega_hat);
  return omega_ref.allFinite() ? omega_ref : Eigen::Vector3d::Zero();
}

SunReferenceRates sunFlatnessReferenceRates(
  const VehicleState & state, const FlatReference & reference,
  double thrust_accel)
{
  SunReferenceRates rates;
  if (thrust_accel < 1e-6 || !state.attitude.allFinite() || state.attitude.norm() < 1e-6) {
    return rates;
  }

  const Eigen::Matrix3d R = quat2RotMatrix(state.attitude / state.attitude.norm());
  const Eigen::Vector3d b1 = R.col(0);
  const Eigen::Vector3d b2 = R.col(1);
  const Eigen::Vector3d b3 = R.col(2);
  const Eigen::Vector3d omega_world = R * state.body_rate;

  const double thrust_accel_dot = -reference.jerk.dot(b3);
  const Eigen::Vector3d hOmega = (-reference.jerk - thrust_accel_dot * b3) / thrust_accel;
  rates.omega << -hOmega.dot(b2), hOmega.dot(b1),
    reference.yaw_rate * Eigen::Vector3d::UnitZ().dot(b3);

  const double thrust_accel_ddot =
    -reference.snap.dot(b3) - (omega_world.cross(b3)).dot(reference.jerk);
  const Eigen::Vector3d hAlpha = -reference.snap / thrust_accel - omega_world.cross(hOmega) -
    2.0 * (thrust_accel_dot / thrust_accel) * hOmega -
    (thrust_accel_ddot / thrust_accel) * b3;
  rates.alpha << -hAlpha.dot(b2), hAlpha.dot(b1),
    reference.yaw_accel * Eigen::Vector3d::UnitZ().dot(b3);
  return rates;
}

Eigen::Vector3d attitudeRateFeedback(
  const Eigen::Vector3d & attitude_error,
  const ControllerParams & params)
{
  return params.KR.asDiagonal() * attitude_error;
}

}  // namespace main_math
}  // namespace geometric_controller
