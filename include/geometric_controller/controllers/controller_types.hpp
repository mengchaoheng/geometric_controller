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

#ifndef GEOMETRIC_CONTROLLER__CONTROLLERS__CONTROLLER_TYPES_HPP_
#define GEOMETRIC_CONTROLLER__CONTROLLERS__CONTROLLER_TYPES_HPP_

#include <Eigen/Dense>

#include <cstdint>
#include <string>

namespace geometric_controller
{

enum class ControllerType
{
  MAIN_GEOMETRIC = 1,
  MAIN_LEE = 2,
  MAIN_JOHNSON = 3,
  MAIN_SUN_DFBC = 4,
  MAIN_GEOMETRIC_INDI = 5,
  LU_OMMPC = 6,
  PX4_DIRECT = 7,
};

struct VehicleState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  // PX4 hrt timestamp of the VehicleLocalPosition sample that supplied the
  // acceleration feedback. Acceleration INDI updates once per new sample and
  // holds its thrust-vector command between samples.
  uint64_t acceleration_sample_timestamp = 0;
  // Filtered allocated force feedback, converted to the paper's positive
  // T*b_z convention in NED.
  Eigen::Vector3d applied_thrust_axis_force = Eigen::Vector3d::Zero();
  // Filtered allocated physical torque feedback in body FRD [N m].
  Eigen::Vector3d applied_torque = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector4d attitude = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  double yaw = 0.0;
};

struct FlatReferenceKnot
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk = Eigen::Vector3d::Zero();
  Eigen::Vector3d snap = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double yaw_rate = 0.0;
  double yaw_accel = 0.0;
};

struct FlatReference : public FlatReferenceKnot
{
  // Optional exact future samples, including k=0. Lu OMMPC consumes these
  // instead of Taylor-extrapolating one current knot over a long horizon.
  std::vector<FlatReferenceKnot> horizon;
};

struct ControllerParams
{
  // All controller laws use PX4's NED/FRD convention, matching main.m.
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, 9.81);
  Eigen::Vector3d Kp = Eigen::Vector3d(10.0, 10.0, 10.0);
  Eigen::Vector3d Kv = Eigen::Vector3d(6.0, 6.0, 6.0);
  // Shared numeric defaults for the separate geometric and geometric_indi
  // gain namespaces in main.m. In particular, yaw stiffness is
  // intentionally much smaller than roll/pitch stiffness; replacing it with
  // a large symmetric value excites yaw/tilt coupling on aggressive paths.
  Eigen::Vector3d KR = Eigen::Vector3d(150.0, 150.0, 3.0);
  Eigen::Vector3d KOmega = Eigen::Vector3d(20.0, 20.0, 8.0);
  Eigen::Matrix3d inertia =
    (Eigen::Vector3d(0.0025, 0.0021, 0.0043)).asDiagonal();
  // The two diagnostic switches bypass only the corresponding incremental
  // law. main_geometric_indi always retains its own Sun reference-rate path.
  bool indi_rate_enabled = true;
  bool indi_acceleration_enabled = true;
  double mass = 0.75;

  // Lu et al. on-manifold MPC. Q/P/R start from the paper values and these
  // scales make controlled comparisons possible without hiding the matrices.
  std::string ommpc_solver = "qpdunes";
  int ommpc_horizon_steps = 20;
  double ommpc_horizon_dt = 0.05;
  double ommpc_thrust_acceleration_min = 0.0;
  double ommpc_thrust_acceleration_max = 45.3333333333;
  Eigen::Vector3d ommpc_body_rate_max = Eigen::Vector3d::Constant(6.0);
  double ommpc_position_weight_scale = 1.0;
  double ommpc_velocity_weight_scale = 1.0;
  double ommpc_attitude_weight_scale = 1.0;
  double ommpc_input_weight_scale = 1.0;
  int ommpc_max_iterations = 4000;
  double ommpc_tolerance = 1e-9;
  double ommpc_admm_rho = 1.0;
  bool ommpc_warm_start = true;
  std::string ommpc_dataset_path;
};

struct ControllerCommand
{
  Eigen::Vector3d torque = Eigen::Vector3d::Zero();
  // Higher-priority INDI feedback component of torque, in physical N*m.
  // The transport layer converts it with the same processing as torque.
  Eigen::Vector3d indi_torque_feedback = Eigen::Vector3d::Zero();
  bool indi_torque_feedback_valid = false;
  Eigen::Vector4d attitude = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector3d desired_body_rate = Eigen::Vector3d::Zero();
  Eigen::Vector3d desired_angular_acceleration = Eigen::Vector3d::Zero();
  double collective_thrust = 0.0;
  // Lu OMMPC optimizes body rate directly. When true, the transport layer
  // publishes desired_body_rate + collective_thrust as VehicleRatesSetpoint.
  bool body_rate_control = false;
  bool valid = true;
  double solve_time_us = 0.0;
  double solver_time_us = 0.0;
  double qp_build_time_us = 0.0;
  int solver_iterations = 0;
  int solver_status = 0;
  bool solver_fallback_used = false;
  int candidate_solver_status = 0;
  double candidate_primal_residual = 0.0;
  double candidate_kkt_scaled = 0.0;
  bool solution_updated = false;
};

}  // namespace geometric_controller

#endif  // GEOMETRIC_CONTROLLER__CONTROLLERS__CONTROLLER_TYPES_HPP_
