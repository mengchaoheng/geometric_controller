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

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>

namespace geometric_controller
{
namespace main_math
{
namespace
{

Eigen::Matrix3d projectSO3(const Eigen::Matrix3d & matrix)
{
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix3d u = svd.matrixU();
  const Eigen::Matrix3d v = svd.matrixV();
  Eigen::Vector3d orientation(1.0, 1.0, (u * v.transpose()).determinant());
  return u * orientation.asDiagonal() * v.transpose();
}

Eigen::Vector4d canonicalQuaternionCoefficients(Eigen::Quaterniond quaternion)
{
  quaternion.normalize();
  Eigen::Vector4d coefficients(
    quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());

  // Match PX4 Quaternion::canonical(): q and -q encode the same rotation.
  // The first significant component also fixes the axis sign at theta=pi.
  constexpr double canonical_epsilon = std::numeric_limits<float>::epsilon();
  for (int index = 0; index < coefficients.size(); ++index) {
    if (std::abs(coefficients[index]) > canonical_epsilon) {
      if (coefficients[index] < 0.0) {
        coefficients = -coefficients;
      }
      break;
    }
  }

  return coefficients;
}

Eigen::Vector3d quaternionPrincipalLog(const Eigen::Quaterniond & quaternion)
{
  const Eigen::Vector4d coefficients = canonicalQuaternionCoefficients(quaternion);

  const double scalar = std::clamp(coefficients[0], 0.0, 1.0);
  const Eigen::Vector3d vector = coefficients.tail<3>();
  const double vector_norm_squared = vector.squaredNorm();

  if (vector_norm_squared < 1e-12) {
    if (scalar > 1e-6) {
      const double scalar_squared = scalar * scalar;
      const double coefficient = 2.0 / scalar -
        (2.0 / 3.0) * vector_norm_squared /
        (scalar * scalar_squared);
      return coefficient * vector;
    }

    return 2.0 * vector;
  }

  const double vector_norm = std::sqrt(vector_norm_squared);
  const double angle = 2.0 * std::atan2(vector_norm, scalar);
  return (angle / vector_norm) * vector;
}

}  // namespace

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
  // Principal SO(3) Log through a canonical unit quaternion. This matches
  // UAV_Algorithm_Benchmark::LogSO3 and PX4 AttitudeControl::logMapSO3,
  // including the small-angle limit and deterministic theta=pi axis sign.
  return quaternionPrincipalLog(Eigen::Quaterniond(projectSO3(R)));
}

Eigen::Vector3d canonicalQuaternionImaginaryError(const Eigen::Vector4d & quaternion)
{
  const Eigen::Quaterniond q(
    quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
  return 2.0 * canonicalQuaternionCoefficients(q).tail<3>();
}

Eigen::Vector3d johnsonLogSO3(const Eigen::Matrix3d & R)
{
  // Preserve Johnson's full-angle Log error while using the canonical
  // principal branch from PX4 AttitudeControl at theta=pi. Eigenvectors do
  // not provide a deterministic axis sign at the repeated pi branch.
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

Eigen::Vector4d matrixToQuaternion(const Eigen::Matrix3d & R)
{
  return rot2Quaternion(R);
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

SunReferenceRates sunFlatnessReferenceRates(
  const VehicleState & state, const FlatReference & reference,
  double thrust_accel)
{
  SunReferenceRates rates;
  if (thrust_accel < 1e-6 || !state.attitude.allFinite() || state.attitude.norm() < 1e-6) {
    return rates;
  }

  const Eigen::Matrix3d R = quat2RotMatrix(state.attitude);
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

}  // namespace main_math
}  // namespace geometric_controller
