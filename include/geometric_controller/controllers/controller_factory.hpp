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

#ifndef GEOMETRIC_CONTROLLER__CONTROLLERS__CONTROLLER_FACTORY_HPP_
#define GEOMETRIC_CONTROLLER__CONTROLLERS__CONTROLLER_FACTORY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometric_controller/controllers/controller_base.hpp"

namespace geometric_controller
{

std::shared_ptr<ControllerBase> makeController(ControllerType type);
std::string controllerTypeName(ControllerType type);
ControllerType controllerTypeFromId(int id);
bool isRosController(ControllerType type);
const std::vector<std::string> & supportedControllerTypes();

}  // namespace geometric_controller

#endif  // GEOMETRIC_CONTROLLER__CONTROLLERS__CONTROLLER_FACTORY_HPP_
