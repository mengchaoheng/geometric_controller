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

#ifndef GEOMETRIC_CONTROLLER__CONTROLLERS__MAIN_CONTROLLER_MATH_HPP_
#define GEOMETRIC_CONTROLLER__CONTROLLERS__MAIN_CONTROLLER_MATH_HPP_

#include "geometric_controller/controllers/common.hpp"
#include "geometric_controller/controllers/controller_types.hpp"

namespace geometric_controller
{
namespace main_math
{

struct HeadingDerivatives
{
  Eigen::Vector3d xC = Eigen::Vector3d::UnitX();
  Eigen::Vector3d xCDot = Eigen::Vector3d::Zero();
  Eigen::Vector3d xCDDot = Eigen::Vector3d::Zero();
};

struct UnitVectorDerivatives
{
  Eigen::Vector3d b = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d bDot = Eigen::Vector3d::Zero();
  Eigen::Vector3d bDDot = Eigen::Vector3d::Zero();
};

struct AttitudeDerivatives
{
  Eigen::Matrix3d RDot = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d RDDot = Eigen::Matrix3d::Zero();
};

struct SunReferenceRates
{
  Eigen::Vector3d omega = Eigen::Vector3d::Zero();
  Eigen::Vector3d alpha = Eigen::Vector3d::Zero();
};

Eigen::Matrix3d hat(const Eigen::Vector3d & w);
Eigen::Vector3d vee(const Eigen::Matrix3d & S);
Eigen::Vector3d logSO3(const Eigen::Matrix3d & R);
Eigen::Vector3d johnsonLogSO3(const Eigen::Matrix3d & R);
Eigen::Matrix3d johnsonLeftJacobianSO3(const Eigen::Vector3d & phi);
Eigen::Vector3d leeSO3Error(const Eigen::Matrix3d & R, const Eigen::Matrix3d & Rd);
Eigen::Vector3d quaternionAttitudeError(const Eigen::Vector4d & q, const Eigen::Vector4d & qd);
Eigen::Vector4d quaternionMultiply(const Eigen::Vector4d & q, const Eigen::Vector4d & p);
Eigen::Vector4d matrixToQuaternion(const Eigen::Matrix3d & R);
Eigen::Matrix3d attitudeFromUnitBodyZAndHeading(
  const Eigen::Vector3d & b3d,
  const Eigen::Vector3d & xC);
HeadingDerivatives headingAxisFromYaw(const FlatReference & reference);
UnitVectorDerivatives unitVectorDerivativesFromVector(
  const Eigen::Vector3d & v, const Eigen::Vector3d & vDot,
  const Eigen::Vector3d & vDDot);
AttitudeDerivatives attitudeDerivativesFromUnitBodyZAndHeading(
  const Eigen::Vector3d & b3d,
  const Eigen::Vector3d & b3dDot,
  const Eigen::Vector3d & b3dDDot,
  const Eigen::Vector3d & xC,
  const Eigen::Vector3d & xCDot,
  const Eigen::Vector3d & xCDDot);
SunReferenceRates sunFlatnessReferenceRates(
  const VehicleState & state, const FlatReference & reference,
  double thrust_accel);

}  // namespace main_math
}  // namespace geometric_controller

#endif  // GEOMETRIC_CONTROLLER__CONTROLLERS__MAIN_CONTROLLER_MATH_HPP_
