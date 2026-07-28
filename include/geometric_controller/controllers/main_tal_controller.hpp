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

#ifndef GEOMETRIC_CONTROLLER__CONTROLLERS__MAIN_TAL_CONTROLLER_HPP_
#define GEOMETRIC_CONTROLLER__CONTROLLERS__MAIN_TAL_CONTROLLER_HPP_

#include <string>

#include "geometric_controller/controllers/controller_base.hpp"
#include "geometric_controller/controllers/main_controller_math.hpp"

namespace geometric_controller
{

class MainTalController : public ControllerBase {
public:
  MainTalController() = default;
  ~MainTalController() override = default;

  std::string name() const override {return "main_tal";}

  ControllerCommand update(
    const VehicleState & state, const FlatReference & reference,
    const ControllerParams & params, double dt) override;
  void reset(const VehicleState & state) override;

private:
  bool initialized_{false};
  Eigen::Vector3d previous_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_omega_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_torque_{Eigen::Vector3d::Zero()};
  double previous_thrust_{0.0};
  SecondOrderFilterState acceleration_filter_;
  SecondOrderFilterState omega_filter_;
  SecondOrderFilterState thrust_acceleration_filter_;
  SecondOrderFilterState torque_filter_;
};

}  // namespace geometric_controller

#endif  // GEOMETRIC_CONTROLLER__CONTROLLERS__MAIN_TAL_CONTROLLER_HPP_
