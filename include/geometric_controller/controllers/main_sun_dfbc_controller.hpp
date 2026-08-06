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

#ifndef GEOMETRIC_CONTROLLER__CONTROLLERS__MAIN_SUN_DFBC_CONTROLLER_HPP_
#define GEOMETRIC_CONTROLLER__CONTROLLERS__MAIN_SUN_DFBC_CONTROLLER_HPP_

#include <string>

#include "geometric_controller/controllers/controller_base.hpp"
#include "geometric_controller/controllers/main_controller_math.hpp"

namespace geometric_controller
{

class MainSunDFBCController : public ControllerBase {
public:
  MainSunDFBCController() = default;
  ~MainSunDFBCController() override = default;

  std::string name() const override {return "main_sun_dfbc";}

  ControllerCommand update(
    const VehicleState & state, const FlatReference & reference,
    const ControllerParams & params, double dt) override;

  void reset(const VehicleState & state) override;

private:
  // Sun Eq. (18)--(24) uses the collective thrust that is already acting on
  // the vehicle.  The direct-wrench transport has no rotor-thrust feedback
  // for this non-INDI controller, so retain the preceding commanded value.
  bool thrust_feedback_valid_{false};
  double previous_collective_thrust_{0.0};
};

}  // namespace geometric_controller

#endif  // GEOMETRIC_CONTROLLER__CONTROLLERS__MAIN_SUN_DFBC_CONTROLLER_HPP_
