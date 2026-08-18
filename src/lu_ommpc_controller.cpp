// Copyright 2026 Chaoheng Meng
// SPDX-License-Identifier: Apache-2.0

#include "geometric_controller/controllers/lu_ommpc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace geometric_controller
{
namespace
{

bool different(double lhs, double rhs)
{
  return std::abs(lhs - rhs) > 1e-12;
}

bool configurationChanged(
  const lu_ommpc::MpcConfig & config, const ControllerParams & params,
  const std::string & solver_name)
{
  return solver_name != params.ommpc_solver ||
         config.horizon_steps != params.ommpc_horizon_steps ||
         different(config.horizon_dt, params.ommpc_horizon_dt) ||
         different(config.gravity, std::abs(params.gravity.z())) ||
         different(config.thrust_acceleration_min, params.ommpc_thrust_acceleration_min) ||
         different(config.thrust_acceleration_max, params.ommpc_thrust_acceleration_max) ||
         !config.body_rate_max.isApprox(params.ommpc_body_rate_max) ||
         config.solver_max_iterations != params.ommpc_max_iterations ||
         different(config.solver_tolerance, params.ommpc_tolerance) ||
         different(config.admm_rho, params.ommpc_admm_rho) ||
         different(config.Q(0, 0), 15000.0 * params.ommpc_position_weight_scale) ||
         different(config.Q(3, 3), 40.0 * params.ommpc_velocity_weight_scale) ||
         different(config.Q(6, 6), 80.0 * params.ommpc_attitude_weight_scale) ||
         different(config.R(0, 0), 0.5 * params.ommpc_input_weight_scale) ||
         config.warm_start != params.ommpc_warm_start ||
         config.dataset_path != params.ommpc_dataset_path;
}

}  // namespace

lu_ommpc::MpcConfig LuOMMPCController::makeConfig(const ControllerParams & params)
{
  lu_ommpc::MpcConfig config;
  config.horizon_steps = params.ommpc_horizon_steps;
  config.horizon_dt = params.ommpc_horizon_dt;
  config.gravity = std::abs(params.gravity.z());
  config.thrust_acceleration_min = params.ommpc_thrust_acceleration_min;
  config.thrust_acceleration_max = params.ommpc_thrust_acceleration_max;
  config.body_rate_max = params.ommpc_body_rate_max;
  config.Q.block<3, 3>(0, 0) *= params.ommpc_position_weight_scale;
  config.Q.block<3, 3>(3, 3) *= params.ommpc_velocity_weight_scale;
  config.Q.block<3, 3>(6, 6) *= params.ommpc_attitude_weight_scale;
  config.P = config.Q;
  config.R *= params.ommpc_input_weight_scale;
  config.solver_max_iterations = params.ommpc_max_iterations;
  config.solver_tolerance = params.ommpc_tolerance;
  config.admm_rho = params.ommpc_admm_rho;
  config.warm_start = params.ommpc_warm_start;
  // Flight comparisons must exercise the selected backend itself. A hidden
  // solver substitution would make tracking results meaningless. Failed or
  // non-finite solves are rejected by the command-valid path
  // and reported by the node.
  config.verify_solution = false;
  config.dataset_path = params.ommpc_dataset_path;
  return config;
}

void LuOMMPCController::configure(const ControllerParams & params)
{
  if (!controller_ || configurationChanged(config_, params, solver_name_)) {
    config_ = makeConfig(params);
    solver_name_ = params.ommpc_solver;
    controller_ = std::make_unique<lu_ommpc::OMMPCController>(config_, solver_name_);
    horizon_.resize(static_cast<std::size_t>(config_.horizon_steps + 1));
  }
}

lu_ommpc::State LuOMMPCController::makeState(const VehicleState & state)
{
  lu_ommpc::State result;
  result.position = state.position;
  result.velocity = state.velocity;
  Eigen::Quaterniond attitude(
    state.attitude[0], state.attitude[1], state.attitude[2], state.attitude[3]);
  if (!attitude.coeffs().allFinite() || attitude.norm() < 1e-9) {
    throw std::invalid_argument("invalid vehicle attitude for Lu OMMPC");
  }
  attitude.normalize();
  result.rotation = attitude.toRotationMatrix();
  return result;
}

const lu_ommpc::ReferenceHorizon & LuOMMPCController::makeHorizon(
  const FlatReference & reference, const lu_ommpc::MpcConfig & config)
{
  if (horizon_.size() != static_cast<std::size_t>(config.horizon_steps + 1)) {
    horizon_.resize(static_cast<std::size_t>(config.horizon_steps + 1));
  }
  const bool exact_horizon =
    reference.horizon.size() == static_cast<std::size_t>(config.horizon_steps + 1);
  for (int k = 0; k <= config.horizon_steps; ++k) {
    if (exact_horizon) {
      const auto & knot = reference.horizon[static_cast<std::size_t>(k)];
      lu_ommpc::FlatOutput flat;
      flat.position = knot.position;
      flat.velocity = knot.velocity;
      flat.acceleration = knot.acceleration;
      flat.jerk = knot.jerk;
      flat.snap = knot.snap;
      flat.yaw = knot.yaw;
      flat.yaw_rate = knot.yaw_rate;
      flat.yaw_acceleration = knot.yaw_accel;
      horizon_[static_cast<std::size_t>(k)] =
        lu_ommpc::flatnessReference(flat, config.gravity);
      continue;
    }
    const double t = static_cast<double>(k) * config.horizon_dt;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    lu_ommpc::FlatOutput flat;
    flat.position = reference.position + t * reference.velocity +
      0.5 * t2 * reference.acceleration + (t3 / 6.0) * reference.jerk +
      (t4 / 24.0) * reference.snap;
    flat.velocity = reference.velocity + t * reference.acceleration +
      0.5 * t2 * reference.jerk + (t3 / 6.0) * reference.snap;
    flat.acceleration = reference.acceleration + t * reference.jerk +
      0.5 * t2 * reference.snap;
    flat.jerk = reference.jerk + t * reference.snap;
    flat.snap = reference.snap;
    flat.yaw = reference.yaw + t * reference.yaw_rate +
      0.5 * t2 * reference.yaw_accel;
    flat.yaw_rate = reference.yaw_rate + t * reference.yaw_accel;
    flat.yaw_acceleration = reference.yaw_accel;
    horizon_[static_cast<std::size_t>(k)] = lu_ommpc::flatnessReference(flat, config.gravity);
  }
  return horizon_;
}

ControllerCommand LuOMMPCController::update(
  const VehicleState & state, const FlatReference & reference,
  const ControllerParams & params, double /* dt */)
{
  const auto controller_start = std::chrono::steady_clock::now();
  configure(params);
  const auto result = controller_->solve(makeState(state), makeHorizon(reference, config_));
  ControllerCommand command = last_command_;
  command.body_rate_control = true;
  command.valid = result.command_valid;
  command.qp_build_time_us =
    result.manifold_us + result.linearization_us + result.qp_build_us;
  command.solver_time_us = result.solver.update_us + result.solver.solve_us;
  command.solver_iterations = result.solver.iterations;
  command.solver_status = static_cast<int>(result.solver.status);
  command.solver_fallback_used = result.fallback_used;
  command.candidate_solver_status = static_cast<int>(result.candidate_status);
  command.candidate_primal_residual = result.candidate_primal_residual;
  command.candidate_kkt_scaled = result.candidate_kkt_scaled;
  command.solution_updated = true;
  if (result.command_valid) {
    command.collective_thrust = params.mass * result.command.thrust_acceleration;
    command.desired_body_rate = result.command.body_rate;
    const Eigen::Quaterniond desired_attitude(
      lu_ommpc::flatnessReference(
        lu_ommpc::FlatOutput{
          reference.position, reference.velocity, reference.acceleration,
          reference.jerk, reference.snap, reference.yaw, reference.yaw_rate,
          reference.yaw_accel}, config_.gravity).state.rotation);
    command.attitude = Eigen::Vector4d(
      desired_attitude.w(), desired_attitude.x(), desired_attitude.y(), desired_attitude.z());
    last_command_ = command;
  } else if (!last_command_.body_rate_control) {
    // A first-cycle solver failure must never become zero thrust.
    command.collective_thrust = params.mass * std::abs(params.gravity.z());
    command.desired_body_rate.setZero();
  }
  command.solve_time_us = std::chrono::duration<double, std::micro>(
    std::chrono::steady_clock::now() - controller_start).count();
  return command;
}

void LuOMMPCController::reset(const VehicleState & /* state */)
{
  if (controller_) {
    controller_->reset();
  }
  last_command_ = ControllerCommand{};
}

}  // namespace geometric_controller
