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

#include <algorithm>
#include <cmath>

namespace geometric_controller
{

constexpr int kErrorQuaternion = 1;
constexpr int kErrorGeometric = 2;

enum class ControllerType
{
  LEGACY_GEOMETRIC = 0,
  MAIN_GEOMETRIC = 1,
  MAIN_LEE = 2,
  MAIN_JOHNSON = 3,
  MAIN_SUN_DFBC = 4,
  MAIN_SUN_DFBC_INDI = 5,
  MAIN_TAL = 6,
  MAIN_GEOMETRIC_INDI = 7,
  PX4_DIRECT = 8,
};

struct VehicleState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector4d attitude = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  bool acceleration_valid = false;
  bool angular_acceleration_valid = false;
  double yaw = 0.0;
};

struct FlatReference
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

struct ControllerParams
{
  int ctrl_mode = kErrorQuaternion;
  // All controller laws use PX4's NED/FRD convention, matching main.m.
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, 9.81);
  Eigen::Vector3d drag = Eigen::Vector3d::Zero();
  Eigen::Vector3d Kp = Eigen::Vector3d(10.0, 10.0, 10.0);
  Eigen::Vector3d Kv = Eigen::Vector3d(6.0, 6.0, 6.0);
  // PX4/Gazebo includes allocation, motor dynamics, and DDS feedback latency
  // that are absent from main.m's ideal rigid body. The yaw gains are tuned
  // separately for the df-main Iris direct-wrench path.
  Eigen::Vector3d KR = Eigen::Vector3d(150.0, 150.0, 80.0);
  Eigen::Vector3d KOmega = Eigen::Vector3d(50.0, 50.0, 3.0);
  Eigen::Matrix3d inertia =
    (Eigen::Vector3d(0.0025, 0.0021, 0.0043)).asDiagonal();
  double max_feedback_acc = 45.0;
  double mass = 0.75;
  double indi_filter_cutoff_hz = 30.0;
};

struct ControllerCommand
{
  Eigen::Vector3d torque = Eigen::Vector3d::Zero();
  Eigen::Vector4d attitude = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector3d reference_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d desired_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d desired_body_rate = Eigen::Vector3d::Zero();
  Eigen::Vector3d desired_angular_acceleration = Eigen::Vector3d::Zero();
  double collective_thrust = 0.0;
};

struct SecondOrderFilterState
{
  Eigen::Vector3d x1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d x2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d y1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d y2 = Eigen::Vector3d::Zero();
  bool initialized = false;
};

inline double clampUnit(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

}  // namespace geometric_controller

#endif  // GEOMETRIC_CONTROLLER__CONTROLLERS__CONTROLLER_TYPES_HPP_
