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

#include "geometric_controller/controllers/controller_factory.hpp"

#include <algorithm>
#include <memory>

#include "geometric_controller/controllers/legacy_geometric_controller.hpp"
#include "geometric_controller/controllers/main_geometric_controller.hpp"
#include "geometric_controller/controllers/main_geometric_indi_controller.hpp"
#include "geometric_controller/controllers/main_johnson_controller.hpp"
#include "geometric_controller/controllers/main_lee_controller.hpp"
#include "geometric_controller/controllers/main_sun_dfbc_controller.hpp"
#include "geometric_controller/controllers/main_tal_controller.hpp"

namespace geometric_controller
{

ControllerType controllerTypeFromId(int id)
{
  const int minimum = static_cast<int>(ControllerType::LEGACY_GEOMETRIC);
  const int maximum = static_cast<int>(ControllerType::PX4_DIRECT);
  return static_cast<ControllerType>(std::clamp(id, minimum, maximum));
}

std::string controllerTypeName(ControllerType type)
{
  switch (type) {
    case ControllerType::LEGACY_GEOMETRIC:
      return "legacy_geometric";
    case ControllerType::MAIN_GEOMETRIC:
      return "main_geometric";
    case ControllerType::MAIN_LEE:
      return "main_lee";
    case ControllerType::MAIN_JOHNSON:
      return "main_johnson";
    case ControllerType::MAIN_SUN_DFBC:
      return "main_sun_dfbc";
    case ControllerType::MAIN_SUN_DFBC_INDI:
      return "main_sun_dfbc_indi";
    case ControllerType::MAIN_TAL:
      return "main_tal";
    case ControllerType::MAIN_GEOMETRIC_INDI:
      return "main_geometric_indi";
    case ControllerType::PX4_DIRECT:
      return "px4_direct";
  }
  return "px4_direct";
}

bool isRosController(ControllerType type)
{
  return type != ControllerType::PX4_DIRECT;
}

std::shared_ptr<ControllerBase> makeController(ControllerType type)
{
  switch (type) {
    case ControllerType::LEGACY_GEOMETRIC:
      return std::make_shared<LegacyGeometricController>();
    case ControllerType::MAIN_GEOMETRIC:
      return std::make_shared<MainGeometricController>();
    case ControllerType::MAIN_LEE:
      return std::make_shared<MainLeeController>();
    case ControllerType::MAIN_JOHNSON:
      return std::make_shared<MainJohnsonController>();
    case ControllerType::MAIN_SUN_DFBC:
      return std::make_shared<MainSunDFBCController>(false);
    case ControllerType::MAIN_SUN_DFBC_INDI:
      return std::make_shared<MainSunDFBCController>(true);
    case ControllerType::MAIN_TAL:
      return std::make_shared<MainTalController>();
    case ControllerType::MAIN_GEOMETRIC_INDI:
      return std::make_shared<MainGeometricINDIController>();
    case ControllerType::PX4_DIRECT:
      return nullptr;
  }
  return nullptr;
}

const std::vector<std::string> & supportedControllerTypes()
{
  static const std::vector<std::string> names{
    "legacy_geometric",
    "main_geometric",
    "main_lee",
    "main_johnson",
    "main_sun_dfbc",
    "main_sun_dfbc_indi",
    "main_tal",
    "main_geometric_indi",
    "px4_direct",
  };
  return names;
}

}  // namespace geometric_controller
