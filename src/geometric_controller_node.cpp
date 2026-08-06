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

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/allocation_value.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include "geometric_controller/controllers/controller_factory.hpp"
#include "geometric_controller/controllers/wrench_normalization.hpp"
#include "geometric_controller/reference_trajectory.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr float kMavModeFlagCustomModeEnabled = 1.0F;
constexpr float kPx4CustomMainModeOffboard = 6.0F;
constexpr float kVehicleCommandArm = 1.0F;
constexpr float kVehicleCommandParamUnused = 0.0F;
constexpr double kSetpointRateHzDefault = 250.0;
constexpr double kSetpointRateHzMin = 50.0;
constexpr double kSetpointRateHzMax = 250.0;
constexpr double kInnerLoopRateHzDefault = 250.0;
constexpr double kInnerLoopRateHzMin = 50.0;
constexpr double kInnerLoopRateHzMax = 250.0;
constexpr double kOuterLoopRateHzDefault = 100.0;
constexpr double kOuterLoopRateHzMin = 10.0;
constexpr double kOuterLoopRateHzMax = 250.0;
constexpr double kHeartbeatRateHzDefault = 5.0;
constexpr double kHeartbeatRateHzMin = 3.0;
constexpr double kHeartbeatRateHzMax = 10.0;
constexpr double kStatusTopicWarningDelayS = 5.0;
constexpr double kStatusTopicWarningPeriodS = 10.0;

struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

class VectorButterworthLowPass2p
{
public:
  void reset(const Eigen::Vector3d & value)
  {
    input_1_ = value;
    input_2_ = value;
    output_1_ = value;
    output_2_ = value;
    initialized_ = value.allFinite();
  }

  void clear()
  {
    initialized_ = false;
  }

  Eigen::Vector3d update(const Eigen::Vector3d & input, double dt, double cutoff_hz)
  {
    if (!input.allFinite() || !std::isfinite(dt) || dt <= 0.0) {
      clear();
      return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    if (!initialized_) {
      reset(input);
      return input;
    }
    if (!std::isfinite(cutoff_hz) || cutoff_hz <= 0.0) {
      reset(input);
      return input;
    }

    const double sample_hz = 1.0 / dt;
    const double limited_cutoff_hz = std::min(cutoff_hz, 0.45 * sample_hz);
    const double k = std::tan(kPi * limited_cutoff_hz / sample_hz);
    const double k_squared = k * k;
    const double norm = 1.0 / (1.0 + std::sqrt(2.0) * k + k_squared);
    const double b0 = k_squared * norm;
    const double b1 = 2.0 * b0;
    const double b2 = b0;
    const double a1 = 2.0 * (k_squared - 1.0) * norm;
    const double a2 = (1.0 - std::sqrt(2.0) * k + k_squared) * norm;
    const Eigen::Vector3d output =
      b0 * input + b1 * input_1_ + b2 * input_2_ -
      a1 * output_1_ - a2 * output_2_;
    input_2_ = input_1_;
    input_1_ = input;
    output_2_ = output_1_;
    output_1_ = output;
    return output;
  }

private:
  Eigen::Vector3d input_1_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d input_2_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d output_1_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d output_2_{Eigen::Vector3d::Zero()};
  bool initialized_{false};
};

bool startsWith(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

template<typename MessageT>
std::string px4VersionedTopic(const std::string & base_topic)
{
  if constexpr (MessageT::MESSAGE_VERSION == 0U) {
    return base_topic;
  } else {
    return base_topic + "_v" + std::to_string(MessageT::MESSAGE_VERSION);
  }
}

float finiteFloatOrNan(double value)
{
  if (!std::isfinite(value)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return static_cast<float>(value);
}

std::array<float, 3> toFloatArray(const geometric_controller::Vector3 & value)
{
  return {
    finiteFloatOrNan(value[0]),
    finiteFloatOrNan(value[1]),
    finiteFloatOrNan(value[2]),
  };
}

std::array<float, 3> nanVector()
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  return {nan, nan, nan};
}

double wrapPi(double value)
{
  while (value > kPi) {
    value -= 2.0 * kPi;
  }
  while (value < -kPi) {
    value += 2.0 * kPi;
  }
  return value;
}

double yawNedToEnu(double yaw_ned)
{
  return wrapPi(kPi / 2.0 - yaw_ned);
}

Quaternion multiply(const Quaternion & lhs, const Quaternion & rhs)
{
  return {
    lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
    lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
    lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
    lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
  };
}

Quaternion normalized(Quaternion q)
{
  const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (!std::isfinite(norm) || norm < 1e-9) {
    return {};
  }
  q.w /= norm;
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  return q;
}

Quaternion yawToQuaternion(double yaw)
{
  return {std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
}

Quaternion px4NedFrdToRosEnuFlu(const std::array<float, 4> & q_px4)
{
  if (!std::isfinite(q_px4[0])) {
    return {};
  }
  const Quaternion enu_from_ned{0.0, std::sqrt(0.5), std::sqrt(0.5), 0.0};
  const Quaternion frd_from_flu{0.0, 1.0, 0.0, 0.0};
  const Quaternion ned_from_frd{
    static_cast<double>(q_px4[0]), static_cast<double>(q_px4[1]),
    static_cast<double>(q_px4[2]), static_cast<double>(q_px4[3])};
  return normalized(multiply(multiply(enu_from_ned, ned_from_frd), frd_from_flu));
}

rcl_interfaces::msg::ParameterDescriptor describeParameter(const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describeDouble(
  const std::string & description, double /* min */, double /* max */, double /* step */)
{
  return describeParameter(description);
}

rcl_interfaces::msg::ParameterDescriptor describeInteger(
  const std::string & description, int64_t /* min */, int64_t /* max */, uint64_t /* step */)
{
  return describeParameter(description);
}

std::array<double, 6> computeQuinticCoefficients(
  double p0, double v0, double a0, double pf, double vf, double af, double duration)
{
  const double t = std::max(duration, 1e-3);
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;

  return {
    p0,
    v0,
    0.5 * a0,
    (20.0 * (pf - p0) - (8.0 * vf + 12.0 * v0) * t - (3.0 * a0 - af) * t2) /
    (2.0 * t3),
    (30.0 * (p0 - pf) + (14.0 * vf + 16.0 * v0) * t + (3.0 * a0 - 2.0 * af) * t2) /
    (2.0 * t4),
    (12.0 * (pf - p0) - (6.0 * vf + 6.0 * v0) * t - (a0 - af) * t2) /
    (2.0 * t5),
  };
}

}  // namespace

class TrajectoryOffboardNode : public rclcpp::Node
{
public:
  TrajectoryOffboardNode()
  : Node("trajectory_offboard_node")
  {
    declareParameters();
    loadParameters();
    syncSelectorParameters();
    reference_trajectory_.setParameters(trajectory_parameters_);
    configureActiveController(false);
    configureRosInterfaces();
    configureTimers();

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&TrajectoryOffboardNode::onSetParameters, this, std::placeholders::_1));

    start_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "Trajectory offboard ready: controller=%s trajName=%s omega_value=%.3f setpoint.level=%s "
      "setpoint_rate=%.1f Hz outer_loop=%.1f Hz inner_loop=%.1f Hz heartbeat_rate=%.1f Hz",
      geometric_controller::controllerTypeName(active_controller_type_).c_str(),
      trajectory_parameters_.traj_name.c_str(), trajectory_parameters_.omega_value,
      setpoint_level_.c_str(), setpoint_rate_hz_, outer_loop_rate_hz_,
      inner_loop_rate_hz_, heartbeat_rate_hz_);
    RCLCPP_INFO(
      get_logger(),
      "PX4 normalization constants: thrust=%.9f kg/N offset=%.6f "
      "torque=[%.6f, %.6f, %.6f] 1/(N*m)",
      normalizedthrust_constant_, normalizedthrust_offset_,
      normalizedtorque_constant_.x(), normalizedtorque_constant_.y(),
      normalizedtorque_constant_.z());
    if (!auto_start_) {
      RCLCPP_WARN(
        get_logger(),
        "auto_start is false: streaming OffboardControlMode heartbeat only until PX4 is manually "
        "switched to Offboard. TrajectorySetpoint publication is gated to Offboard mode.");
    }
    if (takeoff_before_trajectory_) {
      const auto start = reference_trajectory_.sample(0.0);
      RCLCPP_INFO(
        get_logger(),
        "takeoff_before_trajectory is true: holding trajectory start setpoint at NED "
        "[%.2f, %.2f, %.2f] until the vehicle reaches it.",
        start.position[0], start.position[1], start.position[2]);
    }
  }

private:
  void declareParameters()
  {
    declare_parameter<bool>(
      "offboard.enabled", true,
      describeParameter(
        "Publish PX4 offboard heartbeat and trajectory setpoints after local position is valid."));
    declare_parameter<bool>(
      "offboard.auto_start", true,
      describeParameter("Automatically request PX4 Offboard mode and Arm after heartbeat warmup."));
    declare_parameter<bool>(
      "offboard.arm_on_start", true, describeParameter("Arm the vehicle during automatic start."));
    declare_parameter<int>(
      "offboard.prearm_setpoints", 10,
      describeInteger(
        "Number of OffboardControlMode heartbeat cycles before requesting Offboard/Arm.", 0, 200,
        1));
    declare_parameter<double>(
      "offboard.setpoint_rate_hz", kSetpointRateHzDefault,
      describeDouble(
        "TrajectorySetpoint publish rate after local position is valid [Hz].",
        kSetpointRateHzMin, kSetpointRateHzMax, 1.0));
    declare_parameter<double>(
      "offboard.heartbeat_rate_hz", kHeartbeatRateHzDefault,
      describeDouble(
        "OffboardControlMode heartbeat publish rate [Hz].", kHeartbeatRateHzMin,
        kHeartbeatRateHzMax, 1.0));
    declare_parameter<double>(
      "offboard.publish_rate_hz", 0.0,
      describeDouble(
        "Deprecated alias for offboard.setpoint_rate_hz. Use 0 to disable the alias.",
        0.0, kSetpointRateHzMax, 1.0));
    declare_parameter<double>(
      "outer_loop_rate_hz", kOuterLoopRateHzDefault,
      describeDouble(
        "Position/reference outer-loop update rate for ROS-side controllers [Hz].",
        kOuterLoopRateHzMin, kOuterLoopRateHzMax, 1.0));
    declare_parameter<double>(
      "inner_loop_rate_hz", kInnerLoopRateHzDefault,
      describeDouble(
        "Angular-rate feedback and wrench publication rate for ROS-side controllers [Hz].",
        kInnerLoopRateHzMin, kInnerLoopRateHzMax, 1.0));
    declare_parameter<double>(
      "offboard.auto_start_retry_period_s", 1.0,
      describeDouble("Retry period for automatic Offboard/Arm commands [s].", 0.2, 10.0, 0.1));
    declare_parameter<bool>(
      "offboard.takeoff_before_trajectory", true,
      describeParameter(
        "Hold current position before Offboard/Arm, then transition to the trajectory start."));
    declare_parameter<bool>(
      "offboard.use_start_transition", true,
      describeParameter(
      "Use a quintic transition from current vehicle position to the trajectory start point."));
    declare_parameter<double>(
      "offboard.start_transition_duration_s", 4.0,
      describeDouble("Nominal quintic transition duration to the trajectory start point [s].", 0.5,
      20.0, 0.1));
    declare_parameter<double>(
      "offboard.takeoff_position_tolerance", 0.25,
      describeDouble("Position tolerance for releasing the trajectory from its start point [m].",
      0.02, 2.0, 0.01));
    declare_parameter<double>(
      "offboard.takeoff_velocity_tolerance", 0.5,
      describeDouble("Velocity tolerance for releasing the trajectory from its start point [m/s].",
      0.02, 3.0, 0.01));

    declare_parameter<std::string>(
      "setpoint.level", "position",
      describeParameter("PX4 setpoint level: position, velocity, or acceleration."));
    declare_parameter<bool>(
      "setpoint.velocity_feedforward", true,
      describeParameter("Include velocity feedforward when setpoint.level is position."));
    declare_parameter<bool>(
      "setpoint.acceleration_feedforward", true,
      describeParameter(
      "Include acceleration feedforward when setpoint.level is position or velocity."));
    declare_parameter<int>(
      "controller_type", 6,
      describeInteger(
        "Controller selector: 0 legacy_geometric, 1 main_geometric, 2 main_lee, "
        "3 main_johnson, 4 main_sun_dfbc, 5 main_geometric_indi, 6 px4_direct.",
        0, 6, 1));
    declare_parameter<int>(
      "ctrl_mode", geometric_controller::kErrorQuaternion,
      describeInteger("Legacy attitude error: 1 quaternion, 2 geometric.", 1, 2, 1));
    declare_parameter<double>(
      "gravity", 9.81, describeDouble("Gravity magnitude used by ROS-side controllers [m/s^2].",
      1.0, 20.0, 0.01));
    declare_parameter<double>("drag_dx", 0.0, describeDouble("Body x linear drag coefficient.", 0.0,
      10.0, 0.01));
    declare_parameter<double>("drag_dy", 0.0, describeDouble("Body y linear drag coefficient.", 0.0,
      10.0, 0.01));
    declare_parameter<double>("drag_dz", 0.0, describeDouble("Body z linear drag coefficient.", 0.0,
      10.0, 0.01));
    declare_parameter<double>("mass", 0.75, describeDouble("Vehicle mass [kg].", 0.05, 50.0,
      0.001));
    declare_parameter<double>("inertia_x", 0.0025, describeDouble("Body roll inertia [kg m^2].",
      0.00001, 10.0, 0.00001));
    declare_parameter<double>("inertia_y", 0.0021, describeDouble("Body pitch inertia [kg m^2].",
      0.00001, 10.0, 0.00001));
    declare_parameter<double>("inertia_z", 0.0043, describeDouble("Body yaw inertia [kg m^2].",
      0.00001, 10.0, 0.00001));
    declare_parameter<bool>("indi_acceleration_enabled", true);
    declare_parameter<double>(
      "indi_acceleration_cutoff_hz", 8.0,
      describeDouble(
        "Second-order low-pass cutoff for VehicleLocalPosition EKF acceleration [Hz].",
        0.0, 50.0, 0.5));
    declare_parameter<double>(
      "indi_rate_feedback_cutoff_hz", 30.0,
      describeDouble(
        "Second-order low-pass cutoff for geometric INDI omega/alpha/torque feedback [Hz].",
        0.0, 100.0, 0.5));
    declare_parameter<double>("Kp_x", 10.0, describeDouble("Position gain x.", 0.0, 40.0, 0.1));
    declare_parameter<double>("Kp_y", 10.0, describeDouble("Position gain y.", 0.0, 40.0, 0.1));
    declare_parameter<double>("Kp_z", 10.0, describeDouble("Position gain z.", 0.0, 40.0, 0.1));
    declare_parameter<double>("Kv_x", 6.0, describeDouble("Velocity gain x.", 0.0, 40.0, 0.1));
    declare_parameter<double>("Kv_y", 6.0, describeDouble("Velocity gain y.", 0.0, 40.0, 0.1));
    declare_parameter<double>("Kv_z", 6.0, describeDouble("Velocity gain z.", 0.0, 40.0, 0.1));
    declare_parameter<double>("KR_r", 150.0, describeDouble("Roll attitude acceleration gain.",
      0.0, 500.0, 1.0));
    declare_parameter<double>("KR_p", 150.0, describeDouble("Pitch attitude acceleration gain.",
      0.0, 500.0, 1.0));
    declare_parameter<double>("KR_y", 80.0, describeDouble("Yaw attitude acceleration gain.",
      0.0, 100.0, 0.1));
    declare_parameter<double>("KOmega_r", 50.0, describeDouble("Roll angular-rate gain.", 0.0,
      100.0, 0.1));
    declare_parameter<double>("KOmega_p", 50.0, describeDouble("Pitch angular-rate gain.", 0.0,
      100.0, 0.1));
    declare_parameter<double>("KOmega_y", 3.0, describeDouble("Yaw angular-rate gain.", 0.0,
      100.0, 0.1));
    declare_parameter<double>(
      "normalizedthrust_constant", 0.022058823529,
      describeDouble(
        "PX4 normalized thrust coefficient m/T_max [kg/N].", 0.0, 1.0, 0.000001));
    declare_parameter<double>(
      "normalizedthrust_offset", 0.0,
      describeDouble("PX4 normalized collective-thrust offset.", -1.0, 1.0, 0.0001));
    declare_parameter<double>(
      "normalizedtorque_constant_r", 0.319957823650,
      describeDouble("Physical roll moment to PX4 normalized torque scale [1/(N m)].",
      0.0, 1000.0, 0.0001));
    declare_parameter<double>(
      "normalizedtorque_constant_p", 0.319957823650,
      describeDouble("Physical pitch moment to PX4 normalized torque scale [1/(N m)].",
      0.0, 1000.0, 0.0001));
    declare_parameter<double>(
      "normalizedtorque_constant_y", 1.962568474088,
      describeDouble("Physical yaw moment to PX4 normalized torque scale [1/(N m)].",
      0.0, 1000.0, 0.0001));

    declare_parameter<std::string>("px4.offboard_control_mode_topic",
      "/fmu/in/offboard_control_mode");
    declare_parameter<std::string>("px4.trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
    declare_parameter<std::string>(
      "px4.vehicle_thrust_setpoint_topic", "/fmu/in/vehicle_thrust_setpoint");
    declare_parameter<std::string>(
      "px4.vehicle_torque_setpoint_topic", "/fmu/in/vehicle_torque_setpoint");
    declare_parameter<std::string>("px4.vehicle_command_topic", "/fmu/in/vehicle_command");
    declare_parameter<std::string>("px4.vehicle_status_topic",
      px4VersionedTopic<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status"));
    declare_parameter<std::string>("px4.vehicle_local_position_topic",
      px4VersionedTopic<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position"));
    declare_parameter<std::string>(
      "px4.vehicle_attitude_topic", "/fmu/out/vehicle_attitude");
    declare_parameter<std::string>(
      "px4.vehicle_angular_velocity_topic", "/fmu/out/vehicle_angular_velocity");
    declare_parameter<std::string>(
      "px4.allocation_value_topic", "/fmu/out/allocation_value");
    declare_parameter<int>("px4.target_system", 1);
    declare_parameter<int>("px4.target_component", 1);
    declare_parameter<int>("px4.source_system", 1);
    declare_parameter<int>("px4.source_component", 1);

    declare_parameter<std::string>("visualization.path_topic", "reference/trajectory");
    declare_parameter<std::string>("visualization.current_pose_topic", "reference/current_pose");
    declare_parameter<std::string>("visualization.vehicle_pose_topic", "vehicle/current_pose");
    declare_parameter<std::string>("visualization.vehicle_path_topic", "vehicle/path");
    declare_parameter<std::string>("visualization.frame_id", "map");
    declare_parameter<bool>("visualization.ned_to_enu", true);
    declare_parameter<int>("visualization.preview_points", 240);
    declare_parameter<double>("visualization.path_publish_period_s", 0.5);
    declare_parameter<int>("visualization.vehicle_path_max_points", 600);
    declare_parameter<double>("visualization.vehicle_path_min_distance_m", 0.05);

    declare_parameter<std::string>(
      "trajName", "figure8_horizontal",
      describeParameter(
        "Trajectory name: figure8_horizontal, figure8_vertical, helix_flip, helix_flip_y, "
        "flip_loop_sine, fast_circle."));
    declare_parameter<int>(
      "trajectory_type", 1,
      describeInteger(
        "ROS 1-style trajectory selector: 1 figure8_horizontal, 2 figure8_vertical, "
        "3 helix_flip, 4 helix_flip_y, 5 flip_loop_sine, 6 fast_circle.",
        1, 6, 1));
    declare_parameter<double>(
      "omega_value", 0.5,
      describeDouble("Fixed trajectory angular rate omega.value [rad/s].", 0.01, 4.0, 0.01));
    declare_parameter<double>(
      "path_preview_cycles", 10.0,
      describeDouble("RViz path preview length for periodic trajectories [cycles].", 1.0, 50.0,
      1.0));
    declare_parameter<bool>(
      "trajectory_yaw_lock", false,
      describeParameter("Use trajectory_yaw_fixed instead of trajectory heading."));
    declare_parameter<double>(
      "trajectory_yaw_fixed", 0.0,
      describeDouble("Fixed yaw when trajectory_yaw_lock is true [rad].", -kPi, kPi, 0.01));
    declare_parameter<double>(
      "origin_x", 0.0,
      describeDouble("Trajectory origin x in PX4 NED frame [m].", -20.0, 20.0, 0.1));
    declare_parameter<double>(
      "origin_y", 0.0,
      describeDouble("Trajectory origin y in PX4 NED frame [m].", -20.0, 20.0, 0.1));

    declare_parameter<double>(
      "figure8_horizontal_Ax", 3.0, describeDouble("figure8_horizontal.Ax [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "figure8_horizontal_Ay", 3.0, describeDouble("figure8_horizontal.Ay [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "figure8_horizontal_Hc", 6.0, describeDouble("figure8_horizontal.Hc [m].", 0.5, 10.0, 0.1));
    declare_parameter<double>(
      "figure8_horizontal_theta0", 0.0,
      describeDouble("figure8_horizontal.theta0 [rad].", -kPi, kPi, 0.01));

    declare_parameter<double>(
      "figure8_vertical_Ay", 3.0, describeDouble("figure8_vertical.Ay [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "figure8_vertical_Az", 3.0, describeDouble("figure8_vertical.Az [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "figure8_vertical_Hc", 6.0, describeDouble("figure8_vertical.Hc [m].", 0.5, 10.0, 0.1));
    declare_parameter<double>(
      "figure8_vertical_theta0", -0.7853981633974483,
      describeDouble("figure8_vertical.theta0 [rad].", -kPi, kPi, 0.01));

    declare_parameter<double>(
      "helix_flip_Ay", 3.0, describeDouble("helix_flip.Ay [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "helix_flip_Az", 3.0, describeDouble("helix_flip.Az [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "helix_flip_Hc", 6.0, describeDouble("helix_flip.Hc [m].", 0.5, 10.0, 0.1));
    declare_parameter<double>(
      "helix_flip_Vx", 0.30, describeDouble("helix_flip.Vx [m/s].", -5.0, 5.0, 0.1));
    declare_parameter<double>(
      "helix_flip_theta0", 0.0, describeDouble("helix_flip.theta0 [rad].", -kPi, kPi, 0.01));

    declare_parameter<double>(
      "helix_flip_y_Ax", 3.0, describeDouble("helix_flip_y.Ax [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "helix_flip_y_Az", 3.0, describeDouble("helix_flip_y.Az [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "helix_flip_y_Hc", 6.0, describeDouble("helix_flip_y.Hc [m].", 0.5, 10.0, 0.1));
    declare_parameter<double>(
      "helix_flip_y_Vy", 0.30, describeDouble("helix_flip_y.Vy [m/s].", -5.0, 5.0, 0.1));
    declare_parameter<double>(
      "helix_flip_y_theta0", 0.0, describeDouble("helix_flip_y.theta0 [rad].", -kPi, kPi, 0.01));

    declare_parameter<double>(
      "flip_loop_sine_Ay", 3.0, describeDouble("flip_loop_sine.Ay [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "flip_loop_sine_Az", 3.0, describeDouble("flip_loop_sine.Az [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "flip_loop_sine_Hc", 6.0, describeDouble("flip_loop_sine.Hc [m].", 0.5, 10.0, 0.1));
    declare_parameter<double>(
      "flip_loop_sine_Vx", 0.0, describeDouble("flip_loop_sine.Vx [m/s].", -5.0, 5.0, 0.1));
    declare_parameter<double>(
      "flip_loop_sine_theta0", 0.0,
      describeDouble("flip_loop_sine.theta0 [rad].", -kPi, kPi, 0.01));

    declare_parameter<double>(
      "fast_circle_Ax", 3.0, describeDouble("fast_circle.Ax [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "fast_circle_Ay", 3.0, describeDouble("fast_circle.Ay [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "fast_circle_Hc", 6.0, describeDouble("fast_circle.Hc [m].", 0.5, 10.0, 0.1));
    declare_parameter<double>(
      "fast_circle_theta0", 0.0, describeDouble("fast_circle.theta0 [rad].", -kPi, kPi, 0.01));
  }

  void loadParameters()
  {
    offboard_enabled_ = get_parameter("offboard.enabled").as_bool();
    auto_start_ = get_parameter("offboard.auto_start").as_bool();
    arm_on_start_ = get_parameter("offboard.arm_on_start").as_bool();
    prearm_setpoints_ =
      static_cast<int>(get_parameter("offboard.prearm_setpoints").as_int());
    const double legacy_publish_rate_hz = get_parameter("offboard.publish_rate_hz").as_double();
    const double requested_setpoint_rate_hz = legacy_publish_rate_hz > 0.0 ?
      legacy_publish_rate_hz : get_parameter("offboard.setpoint_rate_hz").as_double();
    setpoint_rate_hz_ = requested_setpoint_rate_hz;
    outer_loop_rate_hz_ = get_parameter("outer_loop_rate_hz").as_double();
    inner_loop_rate_hz_ = get_parameter("inner_loop_rate_hz").as_double();
    heartbeat_rate_hz_ = get_parameter("offboard.heartbeat_rate_hz").as_double();
    auto_start_retry_period_s_ =
      get_parameter("offboard.auto_start_retry_period_s").as_double();
    takeoff_before_trajectory_ = get_parameter("offboard.takeoff_before_trajectory").as_bool();
    use_start_transition_ = get_parameter("offboard.use_start_transition").as_bool();
    start_transition_duration_s_ =
      get_parameter("offboard.start_transition_duration_s").as_double();
    takeoff_position_tolerance_ =
      get_parameter("offboard.takeoff_position_tolerance").as_double();
    takeoff_velocity_tolerance_ =
      get_parameter("offboard.takeoff_velocity_tolerance").as_double();

    setpoint_level_ = normalizeSetpointLevel(get_parameter("setpoint.level").as_string());
    velocity_feedforward_ = get_parameter("setpoint.velocity_feedforward").as_bool();
    acceleration_feedforward_ = get_parameter("setpoint.acceleration_feedforward").as_bool();
    active_controller_type_ = geometric_controller::controllerTypeFromId(
      static_cast<int>(get_parameter("controller_type").as_int()));
    controller_params_.ctrl_mode = static_cast<int>(get_parameter("ctrl_mode").as_int());
    controller_params_.gravity =
      Eigen::Vector3d(0.0, 0.0, std::abs(get_parameter("gravity").as_double()));
    controller_params_.drag = Eigen::Vector3d(
      get_parameter("drag_dx").as_double(),
      get_parameter("drag_dy").as_double(),
      get_parameter("drag_dz").as_double());
    controller_params_.Kp = Eigen::Vector3d(
      get_parameter("Kp_x").as_double(),
      get_parameter("Kp_y").as_double(),
      get_parameter("Kp_z").as_double());
    controller_params_.Kv = Eigen::Vector3d(
      get_parameter("Kv_x").as_double(),
      get_parameter("Kv_y").as_double(),
      get_parameter("Kv_z").as_double());
    controller_params_.KR = Eigen::Vector3d(
      get_parameter("KR_r").as_double(),
      get_parameter("KR_p").as_double(),
      get_parameter("KR_y").as_double());
    controller_params_.KOmega = Eigen::Vector3d(
      get_parameter("KOmega_r").as_double(),
      get_parameter("KOmega_p").as_double(),
      get_parameter("KOmega_y").as_double());
    controller_params_.mass = get_parameter("mass").as_double();
    controller_params_.inertia = Eigen::Vector3d(
      get_parameter("inertia_x").as_double(),
      get_parameter("inertia_y").as_double(),
      get_parameter("inertia_z").as_double()).asDiagonal();
    controller_params_.indi_acceleration_enabled =
      get_parameter("indi_acceleration_enabled").as_bool();
    indi_acceleration_cutoff_hz_ = get_parameter("indi_acceleration_cutoff_hz").as_double();
    indi_rate_feedback_cutoff_hz_ =
      get_parameter("indi_rate_feedback_cutoff_hz").as_double();
    controller_params_.outer_loop_rate_hz = outer_loop_rate_hz_;
    normalizedthrust_constant_ = get_parameter("normalizedthrust_constant").as_double();
    normalizedthrust_offset_ = get_parameter("normalizedthrust_offset").as_double();
    normalizedtorque_constant_ = Eigen::Vector3d(
      get_parameter("normalizedtorque_constant_r").as_double(),
      get_parameter("normalizedtorque_constant_p").as_double(),
      get_parameter("normalizedtorque_constant_y").as_double());

    offboard_control_mode_topic_ = get_parameter("px4.offboard_control_mode_topic").as_string();
    trajectory_setpoint_topic_ = get_parameter("px4.trajectory_setpoint_topic").as_string();
    vehicle_thrust_setpoint_topic_ =
      get_parameter("px4.vehicle_thrust_setpoint_topic").as_string();
    vehicle_torque_setpoint_topic_ =
      get_parameter("px4.vehicle_torque_setpoint_topic").as_string();
    vehicle_command_topic_ = get_parameter("px4.vehicle_command_topic").as_string();
    vehicle_status_topic_ = get_parameter("px4.vehicle_status_topic").as_string();
    vehicle_local_position_topic_ = get_parameter("px4.vehicle_local_position_topic").as_string();
    vehicle_attitude_topic_ = get_parameter("px4.vehicle_attitude_topic").as_string();
    vehicle_angular_velocity_topic_ =
      get_parameter("px4.vehicle_angular_velocity_topic").as_string();
    allocation_value_topic_ = get_parameter("px4.allocation_value_topic").as_string();
    target_system_ = static_cast<uint8_t>(get_parameter("px4.target_system").as_int());
    target_component_ = static_cast<uint8_t>(get_parameter("px4.target_component").as_int());
    source_system_ = static_cast<uint8_t>(get_parameter("px4.source_system").as_int());
    source_component_ = static_cast<uint8_t>(get_parameter("px4.source_component").as_int());

    visualization_path_topic_ = get_parameter("visualization.path_topic").as_string();
    visualization_pose_topic_ = get_parameter("visualization.current_pose_topic").as_string();
    visualization_vehicle_pose_topic_ =
      get_parameter("visualization.vehicle_pose_topic").as_string();
    visualization_vehicle_path_topic_ =
      get_parameter("visualization.vehicle_path_topic").as_string();
    visualization_frame_id_ = get_parameter("visualization.frame_id").as_string();
    visualization_ned_to_enu_ = get_parameter("visualization.ned_to_enu").as_bool();
    preview_points_ =
      static_cast<int>(get_parameter("visualization.preview_points").as_int());
    path_publish_period_s_ =
      get_parameter("visualization.path_publish_period_s").as_double();
    vehicle_path_max_points_ =
      static_cast<int>(get_parameter("visualization.vehicle_path_max_points").as_int());
    vehicle_path_min_distance_m_ =
      get_parameter("visualization.vehicle_path_min_distance_m").as_double();
    trimVehiclePath();

    const auto traj_name_from_param =
      geometric_controller::normalizeTrajectoryType(get_parameter("trajName").as_string());
    const int trajectory_type_from_param =
      static_cast<int>(get_parameter("trajectory_type").as_int());
    if (prefer_trajectory_type_) {
      trajectory_type_ = trajectory_type_from_param;
      trajectory_parameters_.traj_name =
        geometric_controller::trajectoryTypeNameFromId(trajectory_type_);
    } else {
      trajectory_parameters_.traj_name = traj_name_from_param;
      trajectory_type_ =
        geometric_controller::trajectoryTypeIdFromName(trajectory_parameters_.traj_name);
    }
    trajectory_parameters_.omega_value = get_parameter("omega_value").as_double();
    trajectory_parameters_.path_preview_cycles = get_parameter("path_preview_cycles").as_double();
    trajectory_parameters_.trajectory_yaw_lock = get_parameter("trajectory_yaw_lock").as_bool();
    trajectory_parameters_.trajectory_yaw_fixed = get_parameter("trajectory_yaw_fixed").as_double();
    trajectory_parameters_.origin_x = get_parameter("origin_x").as_double();
    trajectory_parameters_.origin_y = get_parameter("origin_y").as_double();

    trajectory_parameters_.figure8_horizontal_Ax =
      get_parameter("figure8_horizontal_Ax").as_double();
    trajectory_parameters_.figure8_horizontal_Ay =
      get_parameter("figure8_horizontal_Ay").as_double();
    trajectory_parameters_.figure8_horizontal_Hc =
      get_parameter("figure8_horizontal_Hc").as_double();
    trajectory_parameters_.figure8_horizontal_theta0 =
      get_parameter("figure8_horizontal_theta0").as_double();
    trajectory_parameters_.figure8_vertical_Ay = get_parameter("figure8_vertical_Ay").as_double();
    trajectory_parameters_.figure8_vertical_Az = get_parameter("figure8_vertical_Az").as_double();
    trajectory_parameters_.figure8_vertical_Hc = get_parameter("figure8_vertical_Hc").as_double();
    trajectory_parameters_.figure8_vertical_theta0 =
      get_parameter("figure8_vertical_theta0").as_double();
    trajectory_parameters_.helix_flip_Ay = get_parameter("helix_flip_Ay").as_double();
    trajectory_parameters_.helix_flip_Az = get_parameter("helix_flip_Az").as_double();
    trajectory_parameters_.helix_flip_Hc = get_parameter("helix_flip_Hc").as_double();
    trajectory_parameters_.helix_flip_Vx = get_parameter("helix_flip_Vx").as_double();
    trajectory_parameters_.helix_flip_theta0 = get_parameter("helix_flip_theta0").as_double();
    trajectory_parameters_.helix_flip_y_Ax = get_parameter("helix_flip_y_Ax").as_double();
    trajectory_parameters_.helix_flip_y_Az = get_parameter("helix_flip_y_Az").as_double();
    trajectory_parameters_.helix_flip_y_Hc = get_parameter("helix_flip_y_Hc").as_double();
    trajectory_parameters_.helix_flip_y_Vy = get_parameter("helix_flip_y_Vy").as_double();
    trajectory_parameters_.helix_flip_y_theta0 = get_parameter("helix_flip_y_theta0").as_double();
    trajectory_parameters_.flip_loop_sine_Ay = get_parameter("flip_loop_sine_Ay").as_double();
    trajectory_parameters_.flip_loop_sine_Az = get_parameter("flip_loop_sine_Az").as_double();
    trajectory_parameters_.flip_loop_sine_Hc = get_parameter("flip_loop_sine_Hc").as_double();
    trajectory_parameters_.flip_loop_sine_Vx = get_parameter("flip_loop_sine_Vx").as_double();
    trajectory_parameters_.flip_loop_sine_theta0 =
      get_parameter("flip_loop_sine_theta0").as_double();
    trajectory_parameters_.fast_circle_Ax = get_parameter("fast_circle_Ax").as_double();
    trajectory_parameters_.fast_circle_Ay = get_parameter("fast_circle_Ay").as_double();
    trajectory_parameters_.fast_circle_Hc = get_parameter("fast_circle_Hc").as_double();
    trajectory_parameters_.fast_circle_theta0 = get_parameter("fast_circle_theta0").as_double();
  }

  void configureRosInterfaces()
  {
    auto px4_pub_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    px4_pub_qos.reliable();
    px4_pub_qos.durability_volatile();

    // The angular-rate INDI loop must never work through a backlog of stale
    // samples. Keep only the newest high-rate PX4 sample and use the same
    // best-effort semantics as sensor data.
    auto px4_sub_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    px4_sub_qos.best_effort();
    px4_sub_qos.durability_volatile();

    // Thrust/torque are streaming setpoints, not commands that should be
    // retransmitted later. A reliable KeepLast(10) writer can deliver a burst
    // of stale torque setpoints after a short transport stall, which adds
    // destabilizing delay to the incremental feedback loop.
    auto px4_wrench_pub_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    px4_wrench_pub_qos.best_effort();
    px4_wrench_pub_qos.durability_volatile();

    offboard_control_mode_publisher_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>(offboard_control_mode_topic_,
      px4_pub_qos);
    trajectory_setpoint_publisher_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(trajectory_setpoint_topic_, px4_pub_qos);
    vehicle_thrust_setpoint_publisher_ =
      create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
      vehicle_thrust_setpoint_topic_, px4_wrench_pub_qos);
    vehicle_torque_setpoint_publisher_ =
      create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
      vehicle_torque_setpoint_topic_, px4_wrench_pub_qos);
    vehicle_command_publisher_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(vehicle_command_topic_, px4_pub_qos);

    if (!vehicle_status_topic_.empty()) {
      vehicle_status_subscriber_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        vehicle_status_topic_, px4_sub_qos,
        std::bind(&TrajectoryOffboardNode::vehicleStatusCallback, this, std::placeholders::_1));
    }
    if (!vehicle_local_position_topic_.empty()) {
      vehicle_local_position_subscriber_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        vehicle_local_position_topic_, px4_sub_qos,
        std::bind(&TrajectoryOffboardNode::vehicleLocalPositionCallback, this,
        std::placeholders::_1));
    }
    if (!vehicle_attitude_topic_.empty()) {
      vehicle_attitude_subscriber_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        vehicle_attitude_topic_, px4_sub_qos,
        std::bind(&TrajectoryOffboardNode::vehicleAttitudeCallback, this, std::placeholders::_1));
    }
    if (!vehicle_angular_velocity_topic_.empty()) {
      vehicle_angular_velocity_subscriber_ =
        create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
        vehicle_angular_velocity_topic_, px4_sub_qos,
        std::bind(
          &TrajectoryOffboardNode::vehicleAngularVelocityCallback, this,
          std::placeholders::_1));
    }
    if (!allocation_value_topic_.empty()) {
      allocation_value_subscriber_ = create_subscription<px4_msgs::msg::AllocationValue>(
        allocation_value_topic_, px4_sub_qos,
        std::bind(
          &TrajectoryOffboardNode::allocationValueCallback, this,
          std::placeholders::_1));
    }
    auto reference_path_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    reference_path_qos.reliable();
    reference_path_qos.transient_local();
    reference_path_publisher_ =
      create_publisher<nav_msgs::msg::Path>(visualization_path_topic_, reference_path_qos);
    reference_pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(visualization_pose_topic_, 10);
    vehicle_pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(visualization_vehicle_pose_topic_, 10);
    vehicle_path_publisher_ =
      create_publisher<nav_msgs::msg::Path>(visualization_vehicle_path_topic_, 10);
  }

  void configureTimers()
  {
    configureSetpointTimer();
    configureHeartbeatTimer();
  }

  void configureActiveController(bool log_change)
  {
    active_controller_ = geometric_controller::makeController(active_controller_type_);
    last_controller_sample_timestamp_ = 0;
    next_controller_sample_timestamp_ = 0;
    controller_allocation_feedback_valid_ = false;
    if (active_controller_ && controllerFeedbackValid()) {
      active_controller_->reset(controllerState());
    }
    if (log_change) {
      RCLCPP_INFO(
        get_logger(), "Controller switched to %s (%s output).",
        geometric_controller::controllerTypeName(active_controller_type_).c_str(),
        geometric_controller::isRosController(active_controller_type_) ?
        "VehicleThrustSetpoint + VehicleTorqueSetpoint" : "TrajectorySetpoint");
    }
  }

  void configureSetpointTimer()
  {
    if (setpoint_timer_) {
      setpoint_timer_->cancel();
    }
    const double timer_rate_hz =
      geometric_controller::isRosController(active_controller_type_) ?
      outer_loop_rate_hz_ : setpoint_rate_hz_;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / timer_rate_hz));
    setpoint_timer_ =
      create_wall_timer(period, std::bind(&TrajectoryOffboardNode::setpointTimerCallback, this));
  }

  void configureHeartbeatTimer()
  {
    if (heartbeat_timer_) {
      heartbeat_timer_->cancel();
    }
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / heartbeat_rate_hz_));
    heartbeat_timer_ =
      create_wall_timer(period, std::bind(&TrajectoryOffboardNode::heartbeatTimerCallback, this));
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    if (syncing_selector_parameters_) {
      return result;
    }

    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();

      if (parameterAffectsTrajectoryRestart(name)) {
        trajectory_reset_pending_ = true;
      }
      if (parameterAffectsReferenceTrajectory(name)) {
        reference_parameters_pending_ = true;
      }
      if (parameterAffectsControllerReset(name)) {
        controller_reset_pending_ = true;
      }
      if (name == "trajectory_type") {
        prefer_trajectory_type_ = true;
      } else if (name == "trajName") {
        prefer_trajectory_type_ = false;
      }
    }

    parameters_pending_ = true;
    return result;
  }

  void syncSelectorParameters()
  {
    syncing_selector_parameters_ = true;
    set_parameter(rclcpp::Parameter("trajName", trajectory_parameters_.traj_name));
    set_parameter(rclcpp::Parameter("trajectory_type", trajectory_type_));
    syncing_selector_parameters_ = false;
  }

  bool parameterAffectsTrajectoryRestart(const std::string & name) const
  {
    if (name == "trajName" || name == "trajectory_type" || name == "origin_x" ||
      name == "origin_y" ||
      name == "offboard.takeoff_before_trajectory" || name == "offboard.use_start_transition" ||
      name == "offboard.start_transition_duration_s")
    {
      return true;
    }
    return startsWith(name, "figure8_") || startsWith(name, "helix_") ||
           startsWith(name, "flip_loop_") || startsWith(name, "fast_circle_");
  }

  bool parameterAffectsReferenceTrajectory(const std::string & name) const
  {
    return parameterAffectsTrajectoryRestart(name) ||
           name == "omega_value" || name == "path_preview_cycles" ||
           name == "trajectory_yaw_lock" || name == "trajectory_yaw_fixed";
  }

  bool parameterAffectsControllerReset(const std::string & name) const
  {
    return name == "ctrl_mode" || name == "gravity" ||
           name == "mass" ||
           name == "indi_acceleration_enabled" ||
           startsWith(name, "drag_") ||
           startsWith(name, "inertia_") || startsWith(name, "Kp_") ||
           startsWith(name, "Kv_") || startsWith(name, "KR_") ||
           startsWith(name, "KOmega_") || startsWith(name, "indi_");
  }

  static std::string normalizeSetpointLevel(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    std::replace(value.begin(), value.end(), '-', '_');
    if (value == "position" || value == "pos") {
      return "position";
    }
    if (value == "velocity" || value == "vel") {
      return "velocity";
    }
    if (value == "acceleration" || value == "accel") {
      return "acceleration";
    }
    return "invalid";
  }

  void applyPendingParameters()
  {
    if (!parameters_pending_) {
      return;
    }

    const double previous_setpoint_rate = setpoint_rate_hz_;
    const double previous_outer_loop_rate = outer_loop_rate_hz_;
    const double previous_inner_loop_rate = inner_loop_rate_hz_;
    const double previous_heartbeat_rate = heartbeat_rate_hz_;
    const double previous_omega_value = trajectory_parameters_.omega_value;
    const auto previous_controller_type = active_controller_type_;
    const std::string previous_offboard_topic = offboard_control_mode_topic_;
    const std::string previous_trajectory_topic = trajectory_setpoint_topic_;
    const std::string previous_thrust_topic = vehicle_thrust_setpoint_topic_;
    const std::string previous_torque_topic = vehicle_torque_setpoint_topic_;
    const std::string previous_command_topic = vehicle_command_topic_;
    const std::string previous_status_topic = vehicle_status_topic_;
    const std::string previous_local_position_topic = vehicle_local_position_topic_;
    const std::string previous_attitude_topic = vehicle_attitude_topic_;
    const std::string previous_angular_velocity_topic = vehicle_angular_velocity_topic_;
    const std::string previous_allocation_value_topic = allocation_value_topic_;
    const std::string previous_path_topic = visualization_path_topic_;
    const std::string previous_pose_topic = visualization_pose_topic_;
    const std::string previous_vehicle_pose_topic = visualization_vehicle_pose_topic_;
    const std::string previous_vehicle_path_topic = visualization_vehicle_path_topic_;
    const std::string previous_frame_id = visualization_frame_id_;
    const bool previous_ned_to_enu = visualization_ned_to_enu_;
    const bool preserve_phase =
      !trajectory_reset_pending_ && (trajectory_started_ || !takeoff_before_trajectory_);
    const double elapsed_time_s = std::max(0.0, (now() - start_time_).seconds());

    loadParameters();
    syncSelectorParameters();
    if (previous_controller_type != active_controller_type_) {
      configureActiveController(true);
      if (offboard_enabled_) {
        publishOffboardControlMode(now());
      }
    } else if (controller_reset_pending_ && active_controller_ && controllerFeedbackValid()) {
      controller_allocation_feedback_valid_ = false;
      active_controller_->reset(controllerState());
      RCLCPP_INFO(
        get_logger(), "Reset %s state after controller parameter update.",
        active_controller_->name().c_str());
    }

    const bool omega_changed =
      std::abs(previous_omega_value - trajectory_parameters_.omega_value) > 1e-9;
    if (reference_parameters_pending_ && preserve_phase && omega_changed) {
      constexpr double kOmegaTransitionDurationS = 1.0;
      reference_trajectory_.setParametersWithOmegaTransition(
        trajectory_parameters_, elapsed_time_s, kOmegaTransitionDurationS);
      RCLCPP_INFO(
        get_logger(),
        "Trajectory omega transition: trajName=%s %.3f -> %.3f rad/s in %.1f s.",
        trajectory_parameters_.traj_name.c_str(), previous_omega_value,
        trajectory_parameters_.omega_value, kOmegaTransitionDurationS);
    } else if (reference_parameters_pending_) {
      reference_trajectory_.setParameters(trajectory_parameters_);
    }

    const bool topics_changed =
      previous_offboard_topic != offboard_control_mode_topic_ ||
      previous_trajectory_topic != trajectory_setpoint_topic_ ||
      previous_thrust_topic != vehicle_thrust_setpoint_topic_ ||
      previous_torque_topic != vehicle_torque_setpoint_topic_ ||
      previous_command_topic != vehicle_command_topic_ ||
      previous_status_topic != vehicle_status_topic_ ||
      previous_local_position_topic != vehicle_local_position_topic_ ||
      previous_attitude_topic != vehicle_attitude_topic_ ||
      previous_angular_velocity_topic != vehicle_angular_velocity_topic_ ||
      previous_allocation_value_topic != allocation_value_topic_ ||
      previous_path_topic != visualization_path_topic_ ||
      previous_pose_topic != visualization_pose_topic_ ||
      previous_vehicle_pose_topic != visualization_vehicle_pose_topic_ ||
      previous_vehicle_path_topic != visualization_vehicle_path_topic_;

    if (topics_changed) {
      configureRosInterfaces();
    }
    if (previous_frame_id != visualization_frame_id_ ||
      previous_ned_to_enu != visualization_ned_to_enu_)
    {
      vehicle_path_.clear();
    }
    if (previous_controller_type != active_controller_type_ ||
      std::abs(previous_setpoint_rate - setpoint_rate_hz_) > 1e-6 ||
      std::abs(previous_outer_loop_rate - outer_loop_rate_hz_) > 1e-6)
    {
      configureSetpointTimer();
    }
    if (std::abs(previous_inner_loop_rate - inner_loop_rate_hz_) > 1e-6) {
      last_controller_sample_timestamp_ = 0;
      next_controller_sample_timestamp_ = 0;
    }
    if (std::abs(previous_heartbeat_rate - heartbeat_rate_hz_) > 1e-6) {
      configureHeartbeatTimer();
    }
    if (trajectory_reset_pending_) {
      resetTrajectoryStart("trajectory parameters changed");
    }

    parameters_pending_ = false;
    trajectory_reset_pending_ = false;
    reference_parameters_pending_ = false;
    controller_reset_pending_ = false;
    path_publish_time_s_ = -std::numeric_limits<double>::infinity();

    RCLCPP_INFO(
      get_logger(), "Updated trajectory parameters: trajName=%s omega_value=%.3f",
      trajectory_parameters_.traj_name.c_str(), trajectory_parameters_.omega_value);
  }

  void resetTrajectoryStart(const std::string & reason)
  {
    start_time_ = now();
    trajectory_parameters_.phase_shift = 0.0;
    reference_trajectory_.setParameters(trajectory_parameters_);
    trajectory_started_ = false;
    start_transition_active_ = false;
    start_transition_finished_ = false;
    offboard_heartbeat_counter_ = 0;
    trajectory_setpoint_gate_open_ = false;
    auto_start_ready_logged_ = false;
    auto_start_requested_ = false;
    controller_reference_received_ = false;
    controller_allocation_feedback_valid_ = false;
    if (active_controller_ && controllerFeedbackValid()) {
      active_controller_->reset(controllerState());
    }
    last_auto_start_command_time_s_ = -std::numeric_limits<double>::infinity();
    last_auto_start_wait_log_time_s_ = -std::numeric_limits<double>::infinity();
    RCLCPP_INFO(get_logger(), "Trajectory reset: %s", reason.c_str());
  }

  void setpointTimerCallback()
  {
    applyPendingParameters();

    const auto stamp = now();
    const auto setpoint = currentReference(stamp);
    publishVisualization(setpoint, stamp);
    if (geometric_controller::isRosController(active_controller_type_)) {
      // Do not publish a start-target sample cached before EKF position became
      // valid. The angular-rate callback can run between the first valid PX4
      // state message and this timer; keep the ROS controller gated until its
      // reference has been refreshed from a valid/hold/transition sample.
      if (localPositionValid()) {
        controller_reference_sample_ = setpoint;
        controller_reference_received_ = true;
      } else {
        controller_reference_received_ = false;
      }
    }

    if (!offboard_enabled_) {
      return;
    }

    if (!shouldPublishSetpoint()) {
      if (active_controller_type_ ==
        geometric_controller::ControllerType::MAIN_GEOMETRIC_INDI)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "INDI NOT ACTIVE: %s; no VehicleThrustSetpoint/VehicleTorqueSetpoint is published.",
          indiFeedbackBlocker().c_str());
      }
      if (trajectory_setpoint_gate_open_) {
        trajectory_setpoint_gate_open_ = false;
        RCLCPP_WARN(
          get_logger(),
          "Withholding PX4 setpoints until the selected controller feedback is valid.");
      }
      return;
    }

    if (!trajectory_setpoint_gate_open_) {
      trajectory_setpoint_gate_open_ = true;
      if (active_controller_type_ ==
        geometric_controller::ControllerType::MAIN_GEOMETRIC_INDI)
      {
        RCLCPP_INFO(
          get_logger(),
          "INDI ACTIVE: PX4 AllocationValue %s feedback is fresh; publishing "
          "VehicleThrustSetpoint + VehicleTorqueSetpoint at %.1f Hz.",
          controller_params_.indi_acceleration_enabled ? "force/torque" : "torque",
          inner_loop_rate_hz_);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Controller feedback is valid; publishing %s at %.1f Hz for Offboard.",
          geometric_controller::isRosController(active_controller_type_) ?
          "VehicleThrustSetpoint + VehicleTorqueSetpoint" : "TrajectorySetpoint",
          geometric_controller::isRosController(active_controller_type_) ?
          inner_loop_rate_hz_ : setpoint_rate_hz_);
      }
    }
    if (!geometric_controller::isRosController(active_controller_type_)) {
      publishTrajectorySetpoint(setpoint, stamp);
    }
  }

  void heartbeatTimerCallback()
  {
    applyPendingParameters();

    if (!offboard_enabled_) {
      return;
    }

    const auto stamp = now();
    warnIfVehicleStatusMissing(stamp);
    publishOffboardControlMode(stamp);
    maybeSendAutoStartCommands(stamp);
    if (offboard_heartbeat_counter_ < static_cast<uint64_t>(prearm_setpoints_ + 1)) {
      ++offboard_heartbeat_counter_;
    }
  }

  void warnIfVehicleStatusMissing(const rclcpp::Time & stamp)
  {
    if (status_received_) {
      return;
    }

    const double elapsed_s = std::max(0.0, (stamp - start_time_).seconds());
    if (elapsed_s < kStatusTopicWarningDelayS ||
      stamp.seconds() - last_status_topic_warning_time_s_ < kStatusTopicWarningPeriodS)
    {
      return;
    }
    last_status_topic_warning_time_s_ = stamp.seconds();

    RCLCPP_WARN(
      get_logger(),
      "No PX4 VehicleStatus received on '%s' after %.1f s. Check that PX4 SITL, "
      "Micro XRCE-DDS Agent, and QGroundControl are running.",
      vehicle_status_topic_.c_str(), elapsed_s);
  }

  void maybeSendAutoStartCommands(const rclcpp::Time & stamp)
  {
    if (!auto_start_ || offboard_heartbeat_counter_ < static_cast<uint64_t>(prearm_setpoints_)) {
      return;
    }
    if (isOffboardAndArmed()) {
      return;
    }
    const auto blocker = autoStartReadinessBlocker();
    if (!blocker.empty()) {
      logAutoStartWait(stamp, blocker);
      return;
    }
    if (!auto_start_ready_logged_) {
      auto_start_ready_logged_ = true;
      RCLCPP_INFO(
        get_logger(), "Ready to request takeoff: PX4 local position is valid.");
    }
    if (stamp.seconds() - last_auto_start_command_time_s_ < auto_start_retry_period_s_) {
      return;
    }

    if (!isOffboard()) {
      engageOffboardMode();
    }
    if (arm_on_start_ && !isArmed()) {
      arm();
    }
    auto_start_requested_ = true;
    last_auto_start_command_time_s_ = stamp.seconds();
  }

  geometric_controller::TrajectorySample currentReference(const rclcpp::Time & stamp)
  {
    // Visualization-only mode must animate without PX4 feedback. Otherwise the
    // takeoff gate would keep the reference pose at the first sample forever.
    if (!offboard_enabled_ || !takeoff_before_trajectory_) {
      const double t = (stamp - start_time_).seconds();
      return reference_trajectory_.sample(t);
    }

    if (!trajectory_started_) {
      auto start_sample = reference_trajectory_.sample(0.0);
      start_sample.velocity = {0.0, 0.0, 0.0};
      start_sample.acceleration = {0.0, 0.0, 0.0};
      start_sample.jerk = {0.0, 0.0, 0.0};
      start_sample.snap = {0.0, 0.0, 0.0};
      start_sample.yaw_rate = 0.0;
      start_sample.yaw_acceleration = 0.0;
      const auto start_target_position = start_sample.position;

      if (offboard_enabled_ && !readyForStartTransition()) {
        logTakeoffProgress(
          start_target_position, stamp,
          auto_start_requested_ ? "waiting for Offboard/Arm" : "waiting to request Offboard/Arm");
        if (localPositionValid()) {
          return currentPositionHoldSample(start_sample);
        }
        return start_sample;
      }

      if (use_start_transition_ && localPositionValid() && !start_transition_finished_) {
        if (!start_transition_active_) {
          startStartTransition(start_sample, stamp);
        }
        start_sample = evaluateStartTransition(start_sample, stamp);
      } else if (use_start_transition_ && !localPositionValid()) {
        logTakeoffProgress(start_sample.position, stamp, "waiting for local position feedback");
      }

      if (takeoffTargetReached(start_target_position)) {
        trajectory_started_ = true;
        start_transition_active_ = false;
        start_time_ = stamp;
        RCLCPP_INFO(get_logger(), "Takeoff/start reference reached; starting %s",
          trajectory_parameters_.traj_name.c_str());
      } else {
        logTakeoffProgress(start_target_position, stamp, "moving to trajectory start");
      }
      return start_sample;
    }

    const double t = (stamp - start_time_).seconds();
    return reference_trajectory_.sample(t);
  }

  geometric_controller::TrajectorySample currentPositionHoldSample(
    const geometric_controller::TrajectorySample & target) const
  {
    auto sample = target;
    sample.position = {
      static_cast<double>(vehicle_local_position_.x),
      static_cast<double>(vehicle_local_position_.y),
      static_cast<double>(vehicle_local_position_.z)};
    sample.velocity = {0.0, 0.0, 0.0};
    sample.acceleration = {0.0, 0.0, 0.0};
    sample.jerk = {0.0, 0.0, 0.0};
    sample.snap = {0.0, 0.0, 0.0};
    if (vehicle_attitude_received_) {
      const auto & q = vehicle_attitude_.q;
      sample.yaw = std::atan2(
        2.0 * (q[0] * q[3] + q[1] * q[2]),
        1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
    }
    sample.yaw_rate = 0.0;
    sample.yaw_acceleration = 0.0;
    return sample;
  }

  bool isOffboardAndArmed() const
  {
    return isOffboard() && isArmed();
  }

  bool readyForStartTransition() const
  {
    if (!offboard_enabled_) {
      return true;
    }
    if (status_received_) {
      return isOffboard() && (!arm_on_start_ || isArmed());
    }
    return false;
  }

  bool shouldPublishSetpoint() const
  {
    if (!localPositionValid()) {
      return false;
    }
    return !geometric_controller::isRosController(active_controller_type_) ||
           controllerFeedbackValid();
  }

  std::string autoStartReadinessBlocker() const
  {
    if (!local_position_received_) {
      return "waiting for PX4 local position on '" + vehicle_local_position_topic_ + "'";
    }
    if (!localPositionValid()) {
      return "waiting for valid PX4 local position on '" + vehicle_local_position_topic_ + "'";
    }
    if (geometric_controller::isRosController(active_controller_type_) &&
      !controllerFeedbackValid())
    {
      if (active_controller_type_ ==
        geometric_controller::ControllerType::MAIN_GEOMETRIC_INDI)
      {
        return "waiting for INDI: " + indiFeedbackBlocker();
      }
      return "waiting for valid PX4 attitude on '" + vehicle_attitude_topic_ +
             "' and angular velocity on '" + vehicle_angular_velocity_topic_ + "'";
    }
    return {};
  }

  void logAutoStartWait(const rclcpp::Time & stamp, const std::string & reason)
  {
    if (stamp.seconds() - last_auto_start_wait_log_time_s_ < 2.0) {
      return;
    }
    last_auto_start_wait_log_time_s_ = stamp.seconds();
    RCLCPP_WARN(get_logger(), "Waiting to request Offboard/Arm: %s.", reason.c_str());
  }

  void startStartTransition(
    const geometric_controller::TrajectorySample & target, const rclcpp::Time & stamp)
  {
    const geometric_controller::Vector3 current_position{
      static_cast<double>(vehicle_local_position_.x),
      static_cast<double>(vehicle_local_position_.y),
      static_cast<double>(vehicle_local_position_.z)};
    const geometric_controller::Vector3 current_velocity{
      vehicle_local_position_.v_xy_valid ? static_cast<double>(vehicle_local_position_.vx) : 0.0,
      vehicle_local_position_.v_xy_valid ? static_cast<double>(vehicle_local_position_.vy) : 0.0,
      vehicle_local_position_.v_z_valid ? static_cast<double>(vehicle_local_position_.vz) : 0.0};

    for (size_t axis = 0; axis < 3; ++axis) {
      start_transition_coefficients_[axis] = computeQuinticCoefficients(
        current_position[axis], current_velocity[axis], 0.0, target.position[axis], 0.0, 0.0,
        start_transition_duration_s_);
    }
    const double current_yaw = controllerState().yaw;
    const double target_yaw = current_yaw + wrapPi(target.yaw - current_yaw);
    start_transition_yaw_coefficients_ = computeQuinticCoefficients(
      current_yaw, 0.0, 0.0, target_yaw, 0.0, 0.0,
      start_transition_duration_s_);
    start_transition_start_time_ = stamp;
    start_transition_active_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Start transition: current NED [%.2f, %.2f, %.2f] -> target NED "
      "[%.2f, %.2f, %.2f] in %.2f s.",
      current_position[0], current_position[1], current_position[2],
      target.position[0], target.position[1], target.position[2], start_transition_duration_s_);
  }

  geometric_controller::TrajectorySample evaluateStartTransition(
    const geometric_controller::TrajectorySample & target, const rclcpp::Time & stamp)
  {
    const double elapsed = std::max(0.0, (stamp - start_transition_start_time_).seconds());
    if (elapsed >= start_transition_duration_s_) {
      start_transition_active_ = false;
      start_transition_finished_ = true;
      return target;
    }

    const double t = elapsed;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;

    auto sample = target;
    for (size_t axis = 0; axis < 3; ++axis) {
      const auto & c = start_transition_coefficients_[axis];
      sample.position[axis] = c[0] + c[1] * t + c[2] * t2 + c[3] * t3 + c[4] * t4 + c[5] * t5;
      sample.velocity[axis] = c[1] + 2.0 * c[2] * t + 3.0 * c[3] * t2 + 4.0 * c[4] * t3 +
        5.0 * c[5] * t4;
      sample.acceleration[axis] = 2.0 * c[2] + 6.0 * c[3] * t + 12.0 * c[4] * t2 +
        20.0 * c[5] * t3;
      sample.jerk[axis] = 6.0 * c[3] + 24.0 * c[4] * t + 60.0 * c[5] * t2;
    }
    const auto & c = start_transition_yaw_coefficients_;
    sample.yaw = wrapPi(c[0] + c[1] * t + c[2] * t2 + c[3] * t3 + c[4] * t4 + c[5] * t5);
    sample.yaw_rate =
      c[1] + 2.0 * c[2] * t + 3.0 * c[3] * t2 + 4.0 * c[4] * t3 + 5.0 * c[5] * t4;
    sample.yaw_acceleration =
      2.0 * c[2] + 6.0 * c[3] * t + 12.0 * c[4] * t2 + 20.0 * c[5] * t3;
    sample.snap = {0.0, 0.0, 0.0};
    return sample;
  }

  void logTakeoffProgress(
    const geometric_controller::Vector3 & target, const rclcpp::Time & stamp,
    const std::string & reason)
  {
    if (stamp.seconds() - last_takeoff_progress_log_time_s_ < 2.0) {
      return;
    }
    last_takeoff_progress_log_time_s_ = stamp.seconds();

    if (!local_position_received_) {
      RCLCPP_WARN(get_logger(), "Waiting before trajectory start: %s.", reason.c_str());
      return;
    }

    const double dx = vehicle_local_position_.x - target[0];
    const double dy = vehicle_local_position_.y - target[1];
    const double dz = vehicle_local_position_.z - target[2];
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double speed = std::sqrt(
      vehicle_local_position_.vx * vehicle_local_position_.vx +
      vehicle_local_position_.vy * vehicle_local_position_.vy +
      vehicle_local_position_.vz * vehicle_local_position_.vz);

    RCLCPP_INFO(
      get_logger(),
      "Waiting before trajectory start: %s. current NED [%.2f, %.2f, %.2f], target NED "
      "[%.2f, %.2f, %.2f], distance=%.2f m, speed=%.2f m/s, valid xy=%s z=%s.",
      reason.c_str(), static_cast<double>(vehicle_local_position_.x),
      static_cast<double>(vehicle_local_position_.y),
      static_cast<double>(vehicle_local_position_.z),
      target[0], target[1], target[2], distance, speed,
      vehicle_local_position_.xy_valid ? "true" : "false",
      vehicle_local_position_.z_valid ? "true" : "false");
  }

  bool isOffboard() const
  {
    return status_received_ &&
           vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  bool isArmed() const
  {
    return status_received_ &&
           vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  }

  bool localPositionValid() const
  {
    return local_position_received_ && vehicle_local_position_.xy_valid &&
           vehicle_local_position_.z_valid;
  }

  bool takeoffTargetReached(const geometric_controller::Vector3 & target) const
  {
    if (!localPositionValid()) {
      return false;
    }

    const double dx = vehicle_local_position_.x - target[0];
    const double dy = vehicle_local_position_.y - target[1];
    const double dz = vehicle_local_position_.z - target[2];
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!(distance < takeoff_position_tolerance_)) {
      return false;
    }

    if (!vehicle_local_position_.v_xy_valid || !vehicle_local_position_.v_z_valid) {
      return true;
    }

    const double speed = std::sqrt(
      vehicle_local_position_.vx * vehicle_local_position_.vx +
      vehicle_local_position_.vy * vehicle_local_position_.vy +
      vehicle_local_position_.vz * vehicle_local_position_.vz);
    return std::isfinite(speed) && speed < takeoff_velocity_tolerance_;
  }

  void publishOffboardControlMode(const rclcpp::Time & stamp)
  {
    px4_msgs::msg::OffboardControlMode msg{};
    const bool ros_controller =
      geometric_controller::isRosController(active_controller_type_);
    msg.position = !ros_controller && setpoint_level_ == "position";
    msg.velocity = !ros_controller && setpoint_level_ == "velocity";
    msg.acceleration = !ros_controller && setpoint_level_ == "acceleration";
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque =
      geometric_controller::isRosController(active_controller_type_);
    msg.direct_actuator = false;
    msg.timestamp = timestampMicros(stamp);
    offboard_control_mode_publisher_->publish(msg);
  }

  static Eigen::Vector3d toEigen(const geometric_controller::Vector3 & value)
  {
    return Eigen::Vector3d(value[0], value[1], value[2]);
  }

  geometric_controller::FlatReference controllerReference(
    const geometric_controller::TrajectorySample & sample) const
  {
    geometric_controller::FlatReference reference;
    reference.position = toEigen(sample.position);
    reference.velocity = toEigen(sample.velocity);
    reference.acceleration = toEigen(sample.acceleration);
    reference.jerk = toEigen(sample.jerk);
    reference.snap = toEigen(sample.snap);
    reference.yaw = sample.yaw;
    reference.yaw_rate = sample.yaw_rate;
    reference.yaw_accel = sample.yaw_acceleration;
    return reference;
  }

  bool controllerFeedbackValid() const
  {
    if (!vehicle_attitude_received_ || !vehicle_angular_velocity_received_ ||
      !localPositionValid())
    {
      return false;
    }
    const auto & q = vehicle_attitude_.q;
    const bool attitude_valid =
      std::isfinite(q[0]) && std::isfinite(q[1]) &&
      std::isfinite(q[2]) && std::isfinite(q[3]) &&
      vehicle_attitude_.timestamp_sample > 0;
    if (!attitude_valid) {
      return false;
    }
    const auto & omega = vehicle_angular_velocity_.xyz;
    const bool angular_velocity_valid =
      std::isfinite(omega[0]) && std::isfinite(omega[1]) &&
      std::isfinite(omega[2]) &&
      vehicle_angular_velocity_.timestamp_sample > 0;
    if (!angular_velocity_valid) {
      return false;
    }
    if (active_controller_type_ ==
      geometric_controller::ControllerType::MAIN_GEOMETRIC_INDI)
    {
      return indiFeedbackBlocker().empty();
    }
    return true;
  }

  std::string indiFeedbackBlocker() const
  {
    if (!localPositionValid()) {
      return "VehicleLocalPosition is invalid";
    }
    if (!vehicle_attitude_received_) {
      return "no VehicleAttitude";
    }
    const auto & q = vehicle_attitude_.q;
    if (!std::isfinite(q[0]) || !std::isfinite(q[1]) ||
      !std::isfinite(q[2]) || !std::isfinite(q[3]) ||
      vehicle_attitude_.timestamp_sample == 0)
    {
      return "VehicleAttitude is invalid";
    }
    if (!vehicle_angular_velocity_received_) {
      return "no VehicleAngularVelocity";
    }
    const auto & omega = vehicle_angular_velocity_.xyz;
    if (!std::isfinite(omega[0]) || !std::isfinite(omega[1]) ||
      !std::isfinite(omega[2]) || vehicle_angular_velocity_.timestamp_sample == 0)
    {
      return "VehicleAngularVelocity is invalid";
    }
    if (!allocation_value_received_) {
      return "no AllocationValue";
    }
    if (!allocationTorqueFeedbackValid()) {
      return "AllocationValue.allocated_torque is invalid or stale";
    }
    if (!filtered_indi_body_rate_.allFinite() ||
      !filtered_indi_angular_acceleration_.allFinite() ||
      !filtered_indi_allocated_torque_.allFinite())
    {
      return "geometric INDI rate feedback filter is not ready";
    }
    if (controller_params_.indi_acceleration_enabled) {
      if (!allocationForceFeedbackValid()) {
        return "AllocationValue.allocated_force is invalid or stale";
      }
      if (!accelerationIndiFeedbackValid()) {
        return "VehicleLocalPosition acceleration is invalid";
      }
    }
    return {};
  }

  bool accelerationIndiFeedbackValid() const
  {
    return acceleration_indi_feedback_valid_ &&
           filtered_indi_acceleration_.allFinite();
  }

  geometric_controller::VehicleState controllerState() const
  {
    geometric_controller::VehicleState state;
    state.position = toEigen({
      static_cast<double>(vehicle_local_position_.x),
      static_cast<double>(vehicle_local_position_.y),
      static_cast<double>(vehicle_local_position_.z)});
    state.velocity = toEigen({
      static_cast<double>(vehicle_local_position_.vx),
      static_cast<double>(vehicle_local_position_.vy),
      static_cast<double>(vehicle_local_position_.vz)});
    // This mirrors PX4's MPC_INDI_A_SRC=1 path: EKF acceleration from
    // VehicleLocalPosition, passed through an independent second-order LPF.
    state.acceleration = filtered_indi_acceleration_;

    state.attitude = Eigen::Vector4d(
      static_cast<double>(vehicle_attitude_.q[0]),
      static_cast<double>(vehicle_attitude_.q[1]),
      static_cast<double>(vehicle_attitude_.q[2]),
      static_cast<double>(vehicle_attitude_.q[3]));
    state.attitude.normalize();
    // INDI uses PX4's latest allocator feedback in physical units. ROS
    // neither reconstructs it nor applies an actuator model.
    const auto & allocation = allocation_value_;
    const Eigen::Vector3d allocation_force_body(
      static_cast<double>(allocation.allocated_force[0]),
      static_cast<double>(allocation.allocated_force[1]),
      static_cast<double>(allocation.allocated_force[2]));
    const Eigen::Matrix3d attitude_rotation = Eigen::Quaterniond(
      state.attitude[0], state.attitude[1], state.attitude[2], state.attitude[3])
      .toRotationMatrix();
    state.applied_thrust_axis_force.setConstant(std::numeric_limits<double>::quiet_NaN());
    if (allocationForceFeedbackValid() && allocation_force_body.allFinite()) {
      state.applied_thrust_axis_force = -attitude_rotation * allocation_force_body;
    }
    const auto & angular_velocity = vehicle_angular_velocity_;
    const Eigen::Vector3d raw_body_rate(
      static_cast<double>(angular_velocity.xyz[0]),
      static_cast<double>(angular_velocity.xyz[1]),
      static_cast<double>(angular_velocity.xyz[2]));
    const Eigen::Vector3d raw_angular_acceleration(
      static_cast<double>(angular_velocity.xyz_derivative[0]),
      static_cast<double>(angular_velocity.xyz_derivative[1]),
      static_cast<double>(angular_velocity.xyz_derivative[2]));
    const bool use_indi_feedback =
      active_controller_type_ == geometric_controller::ControllerType::MAIN_GEOMETRIC_INDI;
    state.body_rate = use_indi_feedback ? filtered_indi_body_rate_ : raw_body_rate;
    state.angular_acceleration = use_indi_feedback ?
      filtered_indi_angular_acceleration_ : raw_angular_acceleration;
    const Eigen::Vector3d raw_allocated_torque(
      static_cast<double>(allocation.allocated_torque[0]),
      static_cast<double>(allocation.allocated_torque[1]),
      static_cast<double>(allocation.allocated_torque[2]));
    state.applied_torque = allocationTorqueFeedbackValid() ?
      (use_indi_feedback ? filtered_indi_allocated_torque_ : raw_allocated_torque) :
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    state.yaw = std::atan2(
      2.0 * (state.attitude[0] * state.attitude[3] +
      state.attitude[1] * state.attitude[2]),
      1.0 - 2.0 * (state.attitude[2] * state.attitude[2] +
      state.attitude[3] * state.attitude[3]));
    return state;
  }

  void publishControllerSetpoint(
    const geometric_controller::TrajectorySample & sample, const rclcpp::Time & stamp,
    uint64_t feedback_timestamp_sample, double dt)
  {
    if (!active_controller_) {
      return;
    }

    const bool torque_feedback_valid = allocationTorqueFeedbackValid();
    const bool force_feedback_required = controller_params_.indi_acceleration_enabled;
    const bool force_feedback_valid = allocationForceFeedbackValid();
    controller_allocation_feedback_valid_ = torque_feedback_valid &&
      (!force_feedback_required || force_feedback_valid);
    if (active_controller_type_ == geometric_controller::ControllerType::MAIN_GEOMETRIC_INDI &&
      !controller_allocation_feedback_valid_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "INDI waiting for valid %s feedback; output withheld.",
        !torque_feedback_valid ? "allocated_torque" : "allocated_force");
      return;
    }

    const auto state = controllerState();
    const auto reference = controllerReference(sample);
    auto command = active_controller_->update(state, reference, controller_params_, dt);
    if (!command.torque.allFinite() || !std::isfinite(command.collective_thrust)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s produced a non-finite command; publishing level hover fallback.",
        active_controller_->name().c_str());
      command.torque.setZero();
      command.indi_torque_feedback.setZero();
      command.indi_torque_feedback_valid = false;
      command.collective_thrust =
        controller_params_.mass * std::abs(controller_params_.gravity.z());
    }
    command.collective_thrust = std::max(0.0, command.collective_thrust);

    if (state.body_rate.norm() > 4.0) {
      Eigen::Vector4d desired_attitude = command.attitude;
      desired_attitude.normalize();
      const double quaternion_dot = std::clamp(
        std::abs(state.attitude.dot(desired_attitude)), 0.0, 1.0);
      const double attitude_error_angle = 2.0 * std::acos(quaternion_dot);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s large-rate diagnostic: |Omega|=%.3f rad/s attitude_error=%.3f rad "
        "Omega=[%.3f %.3f %.3f] Omega_d=[%.3f %.3f %.3f] "
        "alpha_d=[%.3f %.3f %.3f] tau=[%.3f %.3f %.3f] N*m.",
        active_controller_->name().c_str(), state.body_rate.norm(), attitude_error_angle,
        state.body_rate.x(), state.body_rate.y(), state.body_rate.z(),
        command.desired_body_rate.x(), command.desired_body_rate.y(),
        command.desired_body_rate.z(), command.desired_angular_acceleration.x(),
        command.desired_angular_acceleration.y(),
        command.desired_angular_acceleration.z(), command.torque.x(),
        command.torque.y(), command.torque.z());
    }

    const auto normalized = geometric_controller::normalizeWrench(
      command, controller_params_.mass, normalizedthrust_constant_,
      normalizedthrust_offset_, normalizedtorque_constant_);
    if (normalized.saturated) {
      const Eigen::Vector3d requested_normalized_torque =
        normalizedtorque_constant_.asDiagonal() * command.torque;
      const double requested_normalized_thrust =
        normalizedthrust_constant_ * command.collective_thrust / controller_params_.mass +
        normalizedthrust_offset_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s wrench saturated by normalized limits: T=%.3f N (norm %.3f) "
        "tau=[%.3f %.3f %.3f] N*m (norm=[%.3f %.3f %.3f]).",
        active_controller_->name().c_str(), command.collective_thrust,
        requested_normalized_thrust, command.torque.x(), command.torque.y(),
        command.torque.z(), requested_normalized_torque.x(),
        requested_normalized_torque.y(), requested_normalized_torque.z());
    }

    const uint64_t timestamp = timestampMicros(stamp);
    px4_msgs::msg::VehicleThrustSetpoint thrust_message{};
    thrust_message.xyz = {0.0F, 0.0F, static_cast<float>(-normalized.thrust)};
    thrust_message.timestamp = timestamp;
    thrust_message.timestamp_sample = feedback_timestamp_sample;
    vehicle_thrust_setpoint_publisher_->publish(thrust_message);

    px4_msgs::msg::VehicleTorqueSetpoint torque_message{};
    torque_message.xyz = {
      static_cast<float>(normalized.torque.x()),
      static_cast<float>(normalized.torque.y()),
      static_cast<float>(normalized.torque.z())};
    torque_message.xyz_indi_feedback = {
      static_cast<float>(normalized.indi_torque_feedback.x()),
      static_cast<float>(normalized.indi_torque_feedback.y()),
      static_cast<float>(normalized.indi_torque_feedback.z())};
    torque_message.xyz_indi_feedback_valid =
      normalized.indi_torque_feedback_valid;
    torque_message.timestamp = timestamp;
    torque_message.timestamp_sample = feedback_timestamp_sample;
    vehicle_torque_setpoint_publisher_->publish(torque_message);
  }

  bool allocationTorqueFeedbackValid() const
  {
    if (!allocation_value_received_ || allocation_value_.timestamp == 0) {
      return false;
    }
    const Eigen::Vector3d torque(
      static_cast<double>(allocation_value_.allocated_torque[0]),
      static_cast<double>(allocation_value_.allocated_torque[1]),
      static_cast<double>(allocation_value_.allocated_torque[2]));
    if (!torque.allFinite()) {
      return false;
    }
    const uint64_t reference_timestamp = vehicle_angular_velocity_.timestamp_sample;
    constexpr uint64_t kAllocationSampleMaxAgeUs = 100000;
    return reference_timestamp == 0 || reference_timestamp <= allocation_value_.timestamp ||
           reference_timestamp - allocation_value_.timestamp < kAllocationSampleMaxAgeUs;
  }

  bool allocationForceFeedbackValid() const
  {
    if (!allocation_value_received_ || allocation_value_.timestamp == 0) {
      return false;
    }
    const Eigen::Vector3d force(
      static_cast<double>(allocation_value_.allocated_force[0]),
      static_cast<double>(allocation_value_.allocated_force[1]),
      static_cast<double>(allocation_value_.allocated_force[2]));
    if (!force.allFinite()) {
      return false;
    }
    const uint64_t reference_timestamp = vehicle_local_position_.timestamp_sample;
    constexpr uint64_t kAllocationSampleMaxAgeUs = 100000;
    return reference_timestamp == 0 || reference_timestamp <= allocation_value_.timestamp ||
           reference_timestamp - allocation_value_.timestamp < kAllocationSampleMaxAgeUs;
  }

  void publishTrajectorySetpoint(
    const geometric_controller::TrajectorySample & sample, const rclcpp::Time & stamp)
  {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = nanVector();
    msg.velocity = nanVector();
    msg.acceleration = nanVector();
    msg.jerk = nanVector();

    if (setpoint_level_ == "position") {
      msg.position = toFloatArray(sample.position);
      if (velocity_feedforward_) {
        msg.velocity = toFloatArray(sample.velocity);
      }
      if (acceleration_feedforward_) {
        msg.acceleration = toFloatArray(sample.acceleration);
      }
    } else if (setpoint_level_ == "velocity") {
      msg.velocity = toFloatArray(sample.velocity);
      if (acceleration_feedforward_) {
        msg.acceleration = toFloatArray(sample.acceleration);
      }
    } else if (setpoint_level_ == "acceleration") {
      msg.acceleration = toFloatArray(sample.acceleration);
    }

    msg.yaw = finiteFloatOrNan(sample.yaw);
    msg.yawspeed = finiteFloatOrNan(sample.yaw_rate);
    msg.timestamp = timestampMicros(stamp);
    trajectory_setpoint_publisher_->publish(msg);
  }

  void publishVisualization(
    const geometric_controller::TrajectorySample & current_sample, const rclcpp::Time & stamp)
  {
    publishCurrentPose(current_sample, stamp);
    publishVehiclePose(stamp);

    if ((stamp.seconds() - path_publish_time_s_) < path_publish_period_s_) {
      return;
    }
    path_publish_time_s_ = stamp.seconds();

    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = visualization_frame_id_;

    const double duration = std::max(0.1, reference_trajectory_.previewDuration());
    // `omega_value` can be changed in flight without resetting the reference
    // phase.  Sampling from t=0 after that change uses the pre-transition
    // trajectory and can cover only a fraction of the new period.  Preview
    // from the current trajectory time instead, so a closed trajectory is
    // complete for every steady omega.
    const double preview_start_s =
      (!offboard_enabled_ || !takeoff_before_trajectory_ || trajectory_started_) ?
      std::max(0.0, (stamp - start_time_).seconds()) : 0.0;
    path.poses.reserve(static_cast<size_t>(preview_points_));
    for (int i = 0; i < preview_points_; ++i) {
      const double alpha = static_cast<double>(i) / static_cast<double>(preview_points_ - 1);
      const auto sample = reference_trajectory_.sample(preview_start_s + alpha * duration);
      path.poses.push_back(toPoseStamped(sample, stamp));
    }
    reference_path_publisher_->publish(path);
    publishVehiclePath(stamp);
  }

  void publishCurrentPose(
    const geometric_controller::TrajectorySample & sample, const rclcpp::Time & stamp)
  {
    reference_pose_publisher_->publish(toPoseStamped(sample, stamp));
  }

  void publishVehiclePose(const rclcpp::Time & stamp)
  {
    if (!localPositionValid() || !vehicle_pose_publisher_) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = visualization_frame_id_;

    const geometric_controller::Vector3 ned_position{
      static_cast<double>(vehicle_local_position_.x),
      static_cast<double>(vehicle_local_position_.y),
      static_cast<double>(vehicle_local_position_.z)};
    const auto position = visualizePosition(ned_position);
    pose.pose.position.x = position[0];
    pose.pose.position.y = position[1];
    pose.pose.position.z = position[2];

    Quaternion orientation;
    if (vehicle_attitude_received_) {
      orientation = px4NedFrdToRosEnuFlu(vehicle_attitude_.q);
    } else {
      orientation = yawToQuaternion(yawNedToEnu(vehicle_local_position_.heading));
    }
    pose.pose.orientation.w = orientation.w;
    pose.pose.orientation.x = orientation.x;
    pose.pose.orientation.y = orientation.y;
    pose.pose.orientation.z = orientation.z;
    vehicle_pose_publisher_->publish(pose);
    appendVehiclePathPose(pose);
  }

  void appendVehiclePathPose(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (vehicle_path_max_points_ <= 0) {
      vehicle_path_.clear();
      return;
    }

    if (!vehicle_path_.empty()) {
      const auto & last = vehicle_path_.back().pose.position;
      const auto & current = pose.pose.position;
      const double dx = current.x - last.x;
      const double dy = current.y - last.y;
      const double dz = current.z - last.z;
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance < vehicle_path_min_distance_m_) {
        return;
      }
    }

    vehicle_path_.push_back(pose);
    trimVehiclePath();
  }

  void trimVehiclePath()
  {
    if (vehicle_path_max_points_ <= 0) {
      vehicle_path_.clear();
      return;
    }
    const auto max_points = static_cast<size_t>(vehicle_path_max_points_);
    if (vehicle_path_.size() > max_points) {
      vehicle_path_.erase(vehicle_path_.begin(), vehicle_path_.end() - max_points);
    }
  }

  void publishVehiclePath(const rclcpp::Time & stamp)
  {
    if (!vehicle_path_publisher_ || vehicle_path_.empty()) {
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = visualization_frame_id_;
    path.poses = vehicle_path_;
    vehicle_path_publisher_->publish(path);
  }

  geometry_msgs::msg::PoseStamped toPoseStamped(
    const geometric_controller::TrajectorySample & sample, const rclcpp::Time & stamp) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = visualization_frame_id_;
    const auto position = visualizePosition(sample.position);
    pose.pose.position.x = position[0];
    pose.pose.position.y = position[1];
    pose.pose.position.z = position[2];

    const double yaw = visualization_ned_to_enu_ ? yawNedToEnu(sample.yaw) : sample.yaw;
    pose.pose.orientation.w = std::cos(0.5 * yaw);
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(0.5 * yaw);
    return pose;
  }

  geometric_controller::Vector3 visualizePosition(
    const geometric_controller::Vector3 & ned_position) const
  {
    if (!visualization_ned_to_enu_) {
      return ned_position;
    }
    return {ned_position[1], ned_position[0], -ned_position[2]};
  }

  void engageOffboardMode()
  {
    publishVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      kMavModeFlagCustomModeEnabled, kPx4CustomMainModeOffboard);
    RCLCPP_INFO(get_logger(), "Offboard mode command sent");
  }

  void arm()
  {
    publishVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      kVehicleCommandArm, kVehicleCommandParamUnused);
    RCLCPP_INFO(get_logger(), "Arm command sent");
  }

  void publishVehicleCommand(
    uint32_t command, float param1 = kVehicleCommandParamUnused,
    float param2 = kVehicleCommandParamUnused)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = target_system_;
    msg.target_component = target_component_;
    msg.source_system = source_system_;
    msg.source_component = source_component_;
    msg.from_external = true;
    msg.timestamp = timestampMicros(now());
    vehicle_command_publisher_->publish(msg);
  }

  uint64_t timestampMicros(const rclcpp::Time & stamp) const
  {
    return static_cast<uint64_t>(stamp.nanoseconds() / 1000);
  }

  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    const bool first = !status_received_;
    const auto previous_nav_state = vehicle_status_.nav_state;
    const auto previous_arming_state = vehicle_status_.arming_state;
    vehicle_status_ = *msg;
    status_received_ = true;

    if (first || previous_nav_state != vehicle_status_.nav_state ||
      previous_arming_state != vehicle_status_.arming_state)
    {
      RCLCPP_INFO(
        get_logger(), "PX4 status: nav_state=%u arming_state=%u",
        static_cast<unsigned int>(vehicle_status_.nav_state),
        static_cast<unsigned int>(vehicle_status_.arming_state));
    }
  }

  void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    const Eigen::Vector3d acceleration_raw(
      static_cast<double>(msg->ax), static_cast<double>(msg->ay),
      static_cast<double>(msg->az));
    double dt = 1.0 / std::max(outer_loop_rate_hz_, 1.0);
    if (last_acceleration_sample_timestamp_ > 0 &&
      msg->timestamp_sample > last_acceleration_sample_timestamp_)
    {
      dt = static_cast<double>(
        msg->timestamp_sample - last_acceleration_sample_timestamp_) * 1.0e-6;
    }
    const bool timestamp_valid = msg->timestamp_sample > 0 &&
      (last_acceleration_sample_timestamp_ == 0 ||
      msg->timestamp_sample > last_acceleration_sample_timestamp_);
    if (timestamp_valid && acceleration_raw.allFinite()) {
      filtered_indi_acceleration_ = acceleration_indi_filter_.update(
        acceleration_raw, dt, indi_acceleration_cutoff_hz_);
      acceleration_indi_feedback_valid_ = filtered_indi_acceleration_.allFinite();
      last_acceleration_sample_timestamp_ = msg->timestamp_sample;
    } else if (!acceleration_raw.allFinite()) {
      acceleration_indi_filter_.clear();
      acceleration_indi_feedback_valid_ = false;
    }
    vehicle_local_position_ = *msg;
    local_position_received_ = true;
  }

  void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    vehicle_attitude_ = *msg;
    vehicle_attitude_received_ = true;
  }

  void updateIndiAngularFeedback(
    const Eigen::Vector3d & body_rate, const Eigen::Vector3d & angular_acceleration,
    uint64_t timestamp_sample)
  {
    double dt = 1.0 / std::max(inner_loop_rate_hz_, 1.0);
    if (last_indi_angular_feedback_timestamp_ > 0 &&
      timestamp_sample > last_indi_angular_feedback_timestamp_)
    {
      dt = static_cast<double>(
        timestamp_sample - last_indi_angular_feedback_timestamp_) * 1.0e-6;
    }
    if (timestamp_sample > last_indi_angular_feedback_timestamp_) {
      filtered_indi_body_rate_ = indi_body_rate_filter_.update(
        body_rate, dt, indi_rate_feedback_cutoff_hz_);
      filtered_indi_angular_acceleration_ = indi_angular_acceleration_filter_.update(
        angular_acceleration, dt, indi_rate_feedback_cutoff_hz_);
      last_indi_angular_feedback_timestamp_ = timestamp_sample;
    }
  }

  void updateIndiAllocatedTorqueFeedback(
    const Eigen::Vector3d & torque, uint64_t timestamp_sample)
  {
    double dt = 1.0 / std::max(inner_loop_rate_hz_, 1.0);
    if (last_indi_torque_feedback_timestamp_ > 0 &&
      timestamp_sample > last_indi_torque_feedback_timestamp_)
    {
      dt = static_cast<double>(
        timestamp_sample - last_indi_torque_feedback_timestamp_) * 1.0e-6;
    }
    if (timestamp_sample > last_indi_torque_feedback_timestamp_) {
      filtered_indi_allocated_torque_ = indi_allocated_torque_filter_.update(
        torque, dt, indi_rate_feedback_cutoff_hz_);
      last_indi_torque_feedback_timestamp_ = timestamp_sample;
    }
  }

  void allocationValueCallback(const px4_msgs::msg::AllocationValue::SharedPtr msg)
  {
    const Eigen::Vector3d torque(
      static_cast<double>(msg->allocated_torque[0]),
      static_cast<double>(msg->allocated_torque[1]),
      static_cast<double>(msg->allocated_torque[2]));
    updateIndiAllocatedTorqueFeedback(torque, msg->timestamp_sample);
    allocation_value_ = *msg;
    allocation_value_received_ = true;
  }

  void vehicleAngularVelocityCallback(
    const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg)
  {
    const Eigen::Vector3d body_rate(
      static_cast<double>(msg->xyz[0]), static_cast<double>(msg->xyz[1]),
      static_cast<double>(msg->xyz[2]));
    const Eigen::Vector3d angular_acceleration(
      static_cast<double>(msg->xyz_derivative[0]),
      static_cast<double>(msg->xyz_derivative[1]),
      static_cast<double>(msg->xyz_derivative[2]));
    updateIndiAngularFeedback(body_rate, angular_acceleration, msg->timestamp_sample);
    vehicle_angular_velocity_ = *msg;
    vehicle_angular_velocity_received_ = true;

    if (!geometric_controller::isRosController(active_controller_type_) ||
      !offboard_enabled_ || !controller_reference_received_ ||
      !shouldPublishSetpoint())
    {
      return;
    }

    const uint64_t sample_timestamp =
      msg->timestamp_sample > 0 ? msg->timestamp_sample : msg->timestamp;
    const uint64_t nominal_period_us = static_cast<uint64_t>(
      std::llround(1.0e6 / inner_loop_rate_hz_));
    if (next_controller_sample_timestamp_ != 0 &&
      sample_timestamp < next_controller_sample_timestamp_)
    {
      return;
    }

    double dt = 1.0 / inner_loop_rate_hz_;
    if (last_controller_sample_timestamp_ != 0 &&
      sample_timestamp > last_controller_sample_timestamp_)
    {
      dt = static_cast<double>(
        sample_timestamp - last_controller_sample_timestamp_) * 1.0e-6;
    }
    last_controller_sample_timestamp_ = sample_timestamp;
    if (next_controller_sample_timestamp_ == 0) {
      next_controller_sample_timestamp_ = sample_timestamp + nominal_period_us;
    } else {
      do {
        next_controller_sample_timestamp_ += nominal_period_us;
      } while (next_controller_sample_timestamp_ <= sample_timestamp);
    }

    publishControllerSetpoint(
      controller_reference_sample_, now(), sample_timestamp, dt);
  }

  geometric_controller::TrajectoryParameters trajectory_parameters_;
  geometric_controller::ReferenceTrajectory reference_trajectory_;
  geometric_controller::ControllerParams controller_params_;
  geometric_controller::ControllerType active_controller_type_{
    geometric_controller::ControllerType::PX4_DIRECT};
  std::shared_ptr<geometric_controller::ControllerBase> active_controller_;

  bool offboard_enabled_{true};
  bool auto_start_{true};
  bool arm_on_start_{true};
  bool takeoff_before_trajectory_{true};
  bool use_start_transition_{true};
  bool velocity_feedforward_{true};
  bool acceleration_feedforward_{true};
  int prearm_setpoints_{10};
  double setpoint_rate_hz_{kSetpointRateHzDefault};
  double outer_loop_rate_hz_{kOuterLoopRateHzDefault};
  double inner_loop_rate_hz_{kInnerLoopRateHzDefault};
  double heartbeat_rate_hz_{kHeartbeatRateHzDefault};
  double auto_start_retry_period_s_{1.0};
  double start_transition_duration_s_{4.0};
  double takeoff_position_tolerance_{0.25};
  double takeoff_velocity_tolerance_{0.5};
  std::string setpoint_level_{"position"};
  double normalizedthrust_constant_{0.022058823529};
  double normalizedthrust_offset_{0.0};
  double indi_acceleration_cutoff_hz_{8.0};
  double indi_rate_feedback_cutoff_hz_{30.0};
  Eigen::Vector3d normalizedtorque_constant_{
    Eigen::Vector3d(0.319957823650, 0.319957823650, 1.962568474088)};

  std::string offboard_control_mode_topic_;
  std::string trajectory_setpoint_topic_;
  std::string vehicle_thrust_setpoint_topic_;
  std::string vehicle_torque_setpoint_topic_;
  std::string vehicle_command_topic_;
  std::string vehicle_status_topic_;
  std::string vehicle_local_position_topic_;
  std::string vehicle_attitude_topic_;
  std::string vehicle_angular_velocity_topic_;
  std::string allocation_value_topic_;
  std::string visualization_path_topic_;
  std::string visualization_pose_topic_;
  std::string visualization_vehicle_pose_topic_;
  std::string visualization_vehicle_path_topic_;
  std::string visualization_frame_id_{"map"};
  bool visualization_ned_to_enu_{true};
  int preview_points_{240};
  double path_publish_period_s_{0.5};
  int vehicle_path_max_points_{600};
  double vehicle_path_min_distance_m_{0.05};
  double path_publish_time_s_{-std::numeric_limits<double>::infinity()};
  uint8_t target_system_{1};
  uint8_t target_component_{1};
  uint8_t source_system_{1};
  uint8_t source_component_{1};

  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr
    vehicle_thrust_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr
    vehicle_torque_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_publisher_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    vehicle_local_position_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr
    vehicle_attitude_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr
    vehicle_angular_velocity_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::AllocationValue>::SharedPtr
    allocation_value_subscriber_;
  px4_msgs::msg::VehicleStatus vehicle_status_{};
  px4_msgs::msg::VehicleLocalPosition vehicle_local_position_{};
  px4_msgs::msg::VehicleAttitude vehicle_attitude_{};
  px4_msgs::msg::VehicleAngularVelocity vehicle_angular_velocity_{};
  px4_msgs::msg::AllocationValue allocation_value_{};
  geometric_controller::TrajectorySample controller_reference_sample_{};
  std::vector<geometry_msgs::msg::PoseStamped> vehicle_path_;
  bool status_received_{false};
  bool local_position_received_{false};
  bool acceleration_indi_feedback_valid_{false};
  bool vehicle_attitude_received_{false};
  bool vehicle_angular_velocity_received_{false};
  bool allocation_value_received_{false};
  bool controller_allocation_feedback_valid_{false};
  bool controller_reference_received_{false};
  bool parameters_pending_{false};
  bool trajectory_reset_pending_{false};
  bool reference_parameters_pending_{false};
  bool controller_reset_pending_{false};
  bool trajectory_started_{false};
  bool start_transition_active_{false};
  bool start_transition_finished_{false};
  bool trajectory_setpoint_gate_open_{false};
  bool auto_start_ready_logged_{false};
  bool auto_start_requested_{false};
  bool prefer_trajectory_type_{false};
  bool syncing_selector_parameters_{false};
  VectorButterworthLowPass2p acceleration_indi_filter_{};
  VectorButterworthLowPass2p indi_body_rate_filter_{};
  VectorButterworthLowPass2p indi_angular_acceleration_filter_{};
  VectorButterworthLowPass2p indi_allocated_torque_filter_{};
  Eigen::Vector3d filtered_indi_acceleration_{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d filtered_indi_body_rate_{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d filtered_indi_angular_acceleration_{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d filtered_indi_allocated_torque_{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  uint64_t last_acceleration_sample_timestamp_{0};
  uint64_t last_indi_angular_feedback_timestamp_{0};
  uint64_t last_indi_torque_feedback_timestamp_{0};
  uint64_t last_controller_sample_timestamp_{0};
  uint64_t next_controller_sample_timestamp_{0};
  double last_auto_start_command_time_s_{-std::numeric_limits<double>::infinity()};
  double last_auto_start_wait_log_time_s_{-std::numeric_limits<double>::infinity()};
  double last_takeoff_progress_log_time_s_{-std::numeric_limits<double>::infinity()};
  double last_status_topic_warning_time_s_{-std::numeric_limits<double>::infinity()};
  int trajectory_type_{1};
  std::array<std::array<double, 6>, 3> start_transition_coefficients_{};
  std::array<double, 6> start_transition_yaw_coefficients_{};
  rclcpp::Time start_transition_start_time_;
  uint64_t offboard_heartbeat_counter_{0};
  rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryOffboardNode>());
  rclcpp::shutdown();
  return 0;
}
