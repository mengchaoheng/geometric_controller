// Copyright 2026 Chaoheng Meng
// SPDX-License-Identifier: Apache-2.0

#ifndef GEOMETRIC_CONTROLLER__CONTROLLERS__LU_OMMPC_CONTROLLER_HPP_
#define GEOMETRIC_CONTROLLER__CONTROLLERS__LU_OMMPC_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometric_controller/controllers/controller_base.hpp"
#include "lu_ommpc/core.hpp"

namespace geometric_controller
{

class LuOMMPCController final : public ControllerBase
{
public:
  std::string name() const override {return "lu_ommpc";}

  ControllerCommand update(
    const VehicleState & state, const FlatReference & reference,
    const ControllerParams & params, double dt) override;

  void reset(const VehicleState & state) override;

private:
  static lu_ommpc::MpcConfig makeConfig(const ControllerParams & params);
  const lu_ommpc::ReferenceHorizon & makeHorizon(
    const FlatReference & reference, const lu_ommpc::MpcConfig & config);
  static lu_ommpc::State makeState(const VehicleState & state);
  void configure(const ControllerParams & params);

  std::unique_ptr<lu_ommpc::OMMPCController> controller_;
  lu_ommpc::MpcConfig config_;
  ControllerCommand last_command_;
  std::string solver_name_;
  lu_ommpc::ReferenceHorizon horizon_;
};

}  // namespace geometric_controller

#endif  // GEOMETRIC_CONTROLLER__CONTROLLERS__LU_OMMPC_CONTROLLER_HPP_
