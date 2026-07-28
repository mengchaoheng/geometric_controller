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

#ifndef GEOMETRIC_CONTROLLER__CONTROLLERS__WRENCH_NORMALIZATION_HPP_
#define GEOMETRIC_CONTROLLER__CONTROLLERS__WRENCH_NORMALIZATION_HPP_

#include <Eigen/Dense>

#include <algorithm>

#include "geometric_controller/controllers/controller_types.hpp"

namespace geometric_controller
{

struct NormalizedWrench
{
  Eigen::Vector3d torque = Eigen::Vector3d::Zero();
  double thrust = 0.0;
  double requested_thrust = 0.0;
  bool saturated = false;
};

inline NormalizedWrench normalizeWrench(
  const ControllerCommand & command, double mass,
  double normalizedthrust_constant, double normalizedthrust_offset,
  const Eigen::Vector3d & normalizedtorque_constant)
{
  NormalizedWrench result;
  result.torque = normalizedtorque_constant.asDiagonal() * command.torque;
  result.requested_thrust =
    normalizedthrust_constant * command.collective_thrust / mass +
    normalizedthrust_offset;
  result.saturated =
    (result.torque.array().abs() > 1.0).any() ||
    result.requested_thrust<0.0 || result.requested_thrust>1.0;
  result.torque = result.torque.cwiseMax(-1.0).cwiseMin(1.0);
  result.thrust = std::clamp(result.requested_thrust, 0.0, 1.0);
  return result;
}

}  // namespace geometric_controller

#endif  // GEOMETRIC_CONTROLLER__CONTROLLERS__WRENCH_NORMALIZATION_HPP_
