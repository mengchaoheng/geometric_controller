#include "geometric_controller/reference_trajectory.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

bool startsWith(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
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
  const std::string & description, double min, double max, double step)
{
  (void)step;
  auto descriptor = describeParameter(description);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = min;
  range.to_value = max;
  range.step = 0.0;
  descriptor.floating_point_range.push_back(range);
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describeInteger(
  const std::string & description, int64_t min, int64_t max, uint64_t step)
{
  auto descriptor = describeParameter(description);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = min;
  range.to_value = max;
  range.step = step;
  descriptor.integer_range.push_back(range);
  return descriptor;
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
    configureRosInterfaces();
    configureTimer();

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&TrajectoryOffboardNode::onSetParameters, this, std::placeholders::_1));

    start_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "Trajectory offboard ready: trajName=%s omega_value=%.3f setpoint.level=%s",
      trajectory_parameters_.traj_name.c_str(), trajectory_parameters_.omega_value,
      setpoint_level_.c_str());
    if (!auto_start_) {
      RCLCPP_WARN(
        get_logger(),
        "auto_start is false: publishing trajectory setpoints only. PX4 will not move unless it is "
        "armed and in Offboard, or you relaunch with auto_start:=true.");
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
      "offboard.enabled", true, describeParameter("Publish PX4 offboard heartbeat and setpoints."));
    declare_parameter<bool>(
      "offboard.auto_start", true,
      describeParameter("Automatically request PX4 Offboard mode and Arm after warmup setpoints."));
    declare_parameter<bool>(
      "offboard.arm_on_start", true, describeParameter("Arm the vehicle during automatic start."));
    declare_parameter<int>(
      "offboard.prearm_setpoints", 10,
      describeInteger("Number of setpoints to publish before requesting Offboard/Arm.", 0, 200, 1));
    declare_parameter<double>(
      "offboard.publish_rate_hz", 50.0,
      describeDouble("Offboard heartbeat and setpoint publish rate [Hz].", 2.1, 200.0, 1.0));
    declare_parameter<double>(
      "offboard.auto_start_retry_period_s", 1.0,
      describeDouble("Retry period for automatic Offboard/Arm commands [s].", 0.2, 10.0, 0.1));
    declare_parameter<bool>(
      "offboard.takeoff_before_trajectory", true,
      describeParameter("Hold the trajectory start setpoint until the vehicle reaches it."));
    declare_parameter<bool>(
      "offboard.use_start_transition", true,
      describeParameter("Use a quintic transition from current vehicle position to the trajectory start point."));
    declare_parameter<double>(
      "offboard.start_transition_duration_s", 4.0,
      describeDouble("Nominal quintic transition duration to the trajectory start point [s].", 0.5, 20.0, 0.1));
    declare_parameter<double>(
      "offboard.takeoff_position_tolerance", 0.25,
      describeDouble("Position tolerance for releasing the trajectory from its start point [m].", 0.02, 2.0, 0.01));
    declare_parameter<double>(
      "offboard.takeoff_velocity_tolerance", 0.5,
      describeDouble("Velocity tolerance for releasing the trajectory from its start point [m/s].", 0.02, 3.0, 0.01));

    declare_parameter<std::string>(
      "setpoint.level", "position",
      describeParameter("PX4 setpoint level: position, velocity, or acceleration."));
    declare_parameter<bool>(
      "setpoint.velocity_feedforward", true,
      describeParameter("Include velocity feedforward when setpoint.level is position."));
    declare_parameter<bool>(
      "setpoint.acceleration_feedforward", true,
      describeParameter("Include acceleration feedforward when setpoint.level is position or velocity."));
    declare_parameter<bool>(
      "setpoint.jerk_logging", false,
      describeParameter("Publish jerk in TrajectorySetpoint for PX4 logging. It is not used by mc_pos_control."));
    declare_parameter<bool>(
      "setpoint.yaw_rate_feedforward", true,
      describeParameter("Include yaw-rate feedforward in TrajectorySetpoint."));

    declare_parameter<std::string>("px4.offboard_control_mode_topic", "/fmu/in/offboard_control_mode");
    declare_parameter<std::string>("px4.trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
    declare_parameter<std::string>("px4.vehicle_command_topic", "/fmu/in/vehicle_command");
    declare_parameter<std::string>("px4.vehicle_status_topic", "/fmu/out/vehicle_status");
    declare_parameter<std::string>("px4.vehicle_status_topic_secondary", "/fmu/out/vehicle_status_v1");
    declare_parameter<std::string>("px4.vehicle_local_position_topic", "/fmu/out/vehicle_local_position");
    declare_parameter<std::string>("px4.vehicle_local_position_topic_secondary", "/fmu/out/vehicle_local_position_v1");
    declare_parameter<std::string>("px4.vehicle_odometry_topic", "/fmu/out/vehicle_odometry");
    declare_parameter<int>("px4.target_system", 1);
    declare_parameter<int>("px4.target_component", 1);
    declare_parameter<int>("px4.source_system", 1);
    declare_parameter<int>("px4.source_component", 1);

    declare_parameter<std::string>("visualization.path_topic", "reference/trajectory");
    declare_parameter<std::string>("visualization.current_pose_topic", "reference/current_pose");
    declare_parameter<std::string>("visualization.vehicle_pose_topic", "vehicle/current_pose");
    declare_parameter<std::string>("visualization.frame_id", "map");
    declare_parameter<bool>("visualization.ned_to_enu", true);
    declare_parameter<int>("visualization.preview_points", 240);
    declare_parameter<double>("visualization.path_publish_period_s", 0.5);

    declare_parameter<std::string>(
      "trajName", "figure8_horizontal",
      describeParameter(
        "Trajectory name: figure8_horizontal, figure8_vertical, helix_flip, helix_flip_y, flip_loop_sine, fast_circle."));
    declare_parameter<int>(
      "trajectory_type", 1,
      describeInteger(
        "ROS1-style trajectory selector: 1 figure8_horizontal, 2 figure8_vertical, 3 helix_flip, 4 helix_flip_y, 5 flip_loop_sine, 6 fast_circle.",
        1, 6, 1));
    declare_parameter<double>(
      "omega_value", 0.5,
      describeDouble("Fixed trajectory angular rate omega.value [rad/s].", 0.01, 4.0, 0.01));
    declare_parameter<double>(
      "path_preview_cycles", 10.0,
      describeDouble("RViz path preview length for periodic trajectories [cycles].", 1.0, 50.0, 1.0));
    declare_parameter<bool>(
      "trajectory_yaw_lock", false, describeParameter("Use trajectory_yaw_fixed instead of trajectory heading."));
    declare_parameter<double>(
      "trajectory_yaw_fixed", 0.0, describeDouble("Fixed yaw when trajectory_yaw_lock is true [rad].", -kPi, kPi, 0.01));
    declare_parameter<double>(
      "origin_x", 0.0, describeDouble("Trajectory origin x in PX4 NED frame [m].", -20.0, 20.0, 0.1));
    declare_parameter<double>(
      "origin_y", 0.0, describeDouble("Trajectory origin y in PX4 NED frame [m].", -20.0, 20.0, 0.1));

    declare_parameter<double>(
      "figure8_horizontal_Ax", 2.0, describeDouble("figure8_horizontal.Ax [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "figure8_horizontal_Ay", 2.0, describeDouble("figure8_horizontal.Ay [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "figure8_horizontal_Hc", 3.0, describeDouble("figure8_horizontal.Hc [m].", 0.5, 10.0, 0.1));
    declare_parameter<double>(
      "figure8_horizontal_theta0", 0.0, describeDouble("figure8_horizontal.theta0 [rad].", -kPi, kPi, 0.01));

    declare_parameter<double>(
      "figure8_vertical_Ay", 2.0, describeDouble("figure8_vertical.Ay [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "figure8_vertical_Az", 2.0, describeDouble("figure8_vertical.Az [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "figure8_vertical_Hc", 3.0, describeDouble("figure8_vertical.Hc [m].", 0.5, 10.0, 0.1));
    declare_parameter<double>(
      "figure8_vertical_theta0", -0.7853981633974483,
      describeDouble("figure8_vertical.theta0 [rad].", -kPi, kPi, 0.01));

    declare_parameter<double>(
      "helix_flip_Ay", 2.0, describeDouble("helix_flip.Ay [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "helix_flip_Az", 2.0, describeDouble("helix_flip.Az [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "helix_flip_Hc", 3.0, describeDouble("helix_flip.Hc [m].", 0.5, 10.0, 0.1));
    declare_parameter<double>(
      "helix_flip_Vx", 0.30, describeDouble("helix_flip.Vx [m/s].", -5.0, 5.0, 0.1));
    declare_parameter<double>(
      "helix_flip_theta0", 0.0, describeDouble("helix_flip.theta0 [rad].", -kPi, kPi, 0.01));

    declare_parameter<double>(
      "helix_flip_y_Ax", 2.0, describeDouble("helix_flip_y.Ax [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "helix_flip_y_Az", 2.0, describeDouble("helix_flip_y.Az [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "helix_flip_y_Hc", 3.0, describeDouble("helix_flip_y.Hc [m].", 0.5, 10.0, 0.1));
    declare_parameter<double>(
      "helix_flip_y_Vy", 0.30, describeDouble("helix_flip_y.Vy [m/s].", -5.0, 5.0, 0.1));
    declare_parameter<double>(
      "helix_flip_y_theta0", 0.0, describeDouble("helix_flip_y.theta0 [rad].", -kPi, kPi, 0.01));

    declare_parameter<double>(
      "flip_loop_sine_Ay", 2.0, describeDouble("flip_loop_sine.Ay [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "flip_loop_sine_Az", 2.0, describeDouble("flip_loop_sine.Az [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "flip_loop_sine_Hc", 3.0, describeDouble("flip_loop_sine.Hc [m].", 0.5, 10.0, 0.1));
    declare_parameter<double>(
      "flip_loop_sine_Vx", 0.0, describeDouble("flip_loop_sine.Vx [m/s].", -5.0, 5.0, 0.1));
    declare_parameter<double>(
      "flip_loop_sine_theta0", 0.0, describeDouble("flip_loop_sine.theta0 [rad].", -kPi, kPi, 0.01));

    declare_parameter<double>(
      "fast_circle_Ax", 3.0, describeDouble("fast_circle.Ax [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "fast_circle_Ay", 3.0, describeDouble("fast_circle.Ay [m].", 0.1, 10.0, 0.1));
    declare_parameter<double>(
      "fast_circle_Hc", 3.0, describeDouble("fast_circle.Hc [m].", 0.5, 10.0, 0.1));
    declare_parameter<double>(
      "fast_circle_theta0", 0.0, describeDouble("fast_circle.theta0 [rad].", -kPi, kPi, 0.01));
  }

  void loadParameters()
  {
    offboard_enabled_ = get_parameter("offboard.enabled").as_bool();
    auto_start_ = get_parameter("offboard.auto_start").as_bool();
    arm_on_start_ = get_parameter("offboard.arm_on_start").as_bool();
    prearm_setpoints_ =
      static_cast<int>(std::max<int64_t>(0, get_parameter("offboard.prearm_setpoints").as_int()));
    publish_rate_hz_ = std::max(2.1, get_parameter("offboard.publish_rate_hz").as_double());
    auto_start_retry_period_s_ =
      std::max(0.2, get_parameter("offboard.auto_start_retry_period_s").as_double());
    takeoff_before_trajectory_ = get_parameter("offboard.takeoff_before_trajectory").as_bool();
    use_start_transition_ = get_parameter("offboard.use_start_transition").as_bool();
    start_transition_duration_s_ =
      std::max(0.5, get_parameter("offboard.start_transition_duration_s").as_double());
    takeoff_position_tolerance_ =
      std::max(0.01, get_parameter("offboard.takeoff_position_tolerance").as_double());
    takeoff_velocity_tolerance_ =
      std::max(0.01, get_parameter("offboard.takeoff_velocity_tolerance").as_double());

    setpoint_level_ = normalizeSetpointLevel(get_parameter("setpoint.level").as_string());
    velocity_feedforward_ = get_parameter("setpoint.velocity_feedforward").as_bool();
    acceleration_feedforward_ = get_parameter("setpoint.acceleration_feedforward").as_bool();
    jerk_logging_ = get_parameter("setpoint.jerk_logging").as_bool();
    yaw_rate_feedforward_ = get_parameter("setpoint.yaw_rate_feedforward").as_bool();

    offboard_control_mode_topic_ = get_parameter("px4.offboard_control_mode_topic").as_string();
    trajectory_setpoint_topic_ = get_parameter("px4.trajectory_setpoint_topic").as_string();
    vehicle_command_topic_ = get_parameter("px4.vehicle_command_topic").as_string();
    vehicle_status_topic_ = get_parameter("px4.vehicle_status_topic").as_string();
    vehicle_status_topic_secondary_ = get_parameter("px4.vehicle_status_topic_secondary").as_string();
    vehicle_local_position_topic_ = get_parameter("px4.vehicle_local_position_topic").as_string();
    vehicle_local_position_topic_secondary_ =
      get_parameter("px4.vehicle_local_position_topic_secondary").as_string();
    vehicle_odometry_topic_ = get_parameter("px4.vehicle_odometry_topic").as_string();
    target_system_ =
      static_cast<uint8_t>(std::max<int64_t>(1, get_parameter("px4.target_system").as_int()));
    target_component_ =
      static_cast<uint8_t>(std::max<int64_t>(1, get_parameter("px4.target_component").as_int()));
    source_system_ =
      static_cast<uint8_t>(std::max<int64_t>(1, get_parameter("px4.source_system").as_int()));
    source_component_ =
      static_cast<uint8_t>(std::max<int64_t>(1, get_parameter("px4.source_component").as_int()));

    visualization_path_topic_ = get_parameter("visualization.path_topic").as_string();
    visualization_pose_topic_ = get_parameter("visualization.current_pose_topic").as_string();
    visualization_vehicle_pose_topic_ = get_parameter("visualization.vehicle_pose_topic").as_string();
    visualization_frame_id_ = get_parameter("visualization.frame_id").as_string();
    visualization_ned_to_enu_ = get_parameter("visualization.ned_to_enu").as_bool();
    preview_points_ =
      static_cast<int>(std::max<int64_t>(2, get_parameter("visualization.preview_points").as_int()));
    path_publish_period_s_ =
      std::max(0.05, get_parameter("visualization.path_publish_period_s").as_double());

    const auto traj_name_from_param =
      geometric_controller::normalizeTrajectoryType(get_parameter("trajName").as_string());
    const int trajectory_type_from_param = static_cast<int>(
      std::clamp<int64_t>(get_parameter("trajectory_type").as_int(), 1, 6));
    if (prefer_trajectory_type_) {
      trajectory_type_ = trajectory_type_from_param;
      trajectory_parameters_.traj_name = geometric_controller::trajectoryTypeNameFromId(trajectory_type_);
    } else {
      trajectory_parameters_.traj_name = traj_name_from_param;
      trajectory_type_ = geometric_controller::trajectoryTypeIdFromName(trajectory_parameters_.traj_name);
    }
    trajectory_parameters_.omega_value = get_parameter("omega_value").as_double();
    trajectory_parameters_.path_preview_cycles = get_parameter("path_preview_cycles").as_double();
    trajectory_parameters_.trajectory_yaw_lock = get_parameter("trajectory_yaw_lock").as_bool();
    trajectory_parameters_.trajectory_yaw_fixed = get_parameter("trajectory_yaw_fixed").as_double();
    trajectory_parameters_.origin_x = get_parameter("origin_x").as_double();
    trajectory_parameters_.origin_y = get_parameter("origin_y").as_double();

    trajectory_parameters_.figure8_horizontal_Ax = get_parameter("figure8_horizontal_Ax").as_double();
    trajectory_parameters_.figure8_horizontal_Ay = get_parameter("figure8_horizontal_Ay").as_double();
    trajectory_parameters_.figure8_horizontal_Hc = get_parameter("figure8_horizontal_Hc").as_double();
    trajectory_parameters_.figure8_horizontal_theta0 = get_parameter("figure8_horizontal_theta0").as_double();
    trajectory_parameters_.figure8_vertical_Ay = get_parameter("figure8_vertical_Ay").as_double();
    trajectory_parameters_.figure8_vertical_Az = get_parameter("figure8_vertical_Az").as_double();
    trajectory_parameters_.figure8_vertical_Hc = get_parameter("figure8_vertical_Hc").as_double();
    trajectory_parameters_.figure8_vertical_theta0 = get_parameter("figure8_vertical_theta0").as_double();
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
    trajectory_parameters_.flip_loop_sine_theta0 = get_parameter("flip_loop_sine_theta0").as_double();
    trajectory_parameters_.fast_circle_Ax = get_parameter("fast_circle_Ax").as_double();
    trajectory_parameters_.fast_circle_Ay = get_parameter("fast_circle_Ay").as_double();
    trajectory_parameters_.fast_circle_Hc = get_parameter("fast_circle_Hc").as_double();
    trajectory_parameters_.fast_circle_theta0 = get_parameter("fast_circle_theta0").as_double();
  }

  void configureRosInterfaces()
  {
    auto px4_pub_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    px4_pub_qos.best_effort();
    px4_pub_qos.transient_local();

    auto px4_sub_qos = rclcpp::QoS(rclcpp::KeepLast(5));
    px4_sub_qos.best_effort();
    px4_sub_qos.durability_volatile();

    offboard_control_mode_publisher_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>(offboard_control_mode_topic_, px4_pub_qos);
    trajectory_setpoint_publisher_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(trajectory_setpoint_topic_, px4_pub_qos);
    vehicle_command_publisher_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(vehicle_command_topic_, px4_pub_qos);

    if (!vehicle_status_topic_.empty()) {
      vehicle_status_subscriber_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        vehicle_status_topic_, px4_sub_qos,
        std::bind(&TrajectoryOffboardNode::vehicleStatusCallback, this, std::placeholders::_1));
    }
    if (!vehicle_status_topic_secondary_.empty() &&
      vehicle_status_topic_secondary_ != vehicle_status_topic_)
    {
      vehicle_status_secondary_subscriber_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        vehicle_status_topic_secondary_, px4_sub_qos,
        std::bind(&TrajectoryOffboardNode::vehicleStatusCallback, this, std::placeholders::_1));
    }
    if (!vehicle_local_position_topic_.empty()) {
      vehicle_local_position_subscriber_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        vehicle_local_position_topic_, px4_sub_qos,
        std::bind(&TrajectoryOffboardNode::vehicleLocalPositionCallback, this, std::placeholders::_1));
    }
    if (!vehicle_local_position_topic_secondary_.empty() &&
      vehicle_local_position_topic_secondary_ != vehicle_local_position_topic_)
    {
      vehicle_local_position_secondary_subscriber_ =
        create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        vehicle_local_position_topic_secondary_, px4_sub_qos,
        std::bind(&TrajectoryOffboardNode::vehicleLocalPositionCallback, this, std::placeholders::_1));
    }
    if (!vehicle_odometry_topic_.empty()) {
      vehicle_odometry_subscriber_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        vehicle_odometry_topic_, px4_sub_qos,
        std::bind(&TrajectoryOffboardNode::vehicleOdometryCallback, this, std::placeholders::_1));
    }

    reference_path_publisher_ = create_publisher<nav_msgs::msg::Path>(visualization_path_topic_, 10);
    reference_pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(visualization_pose_topic_, 10);
    vehicle_pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(visualization_vehicle_pose_topic_, 10);
  }

  void configureTimer()
  {
    if (timer_) {
      timer_->cancel();
    }
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&TrajectoryOffboardNode::timerCallback, this));
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

      if (name == "trajName" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
        !geometric_controller::isSupportedTrajectoryType(parameter.as_string()))
      {
        result.successful = false;
        result.reason = "Unsupported trajName";
        return result;
      }
      if (name == "trajectory_type" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER &&
        (parameter.as_int() < 1 || parameter.as_int() > 6))
      {
        result.successful = false;
        result.reason = "trajectory_type must be in [1, 6]";
        return result;
      }
      if (name == "setpoint.level" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
        normalizeSetpointLevel(parameter.as_string()) == "invalid")
      {
        result.successful = false;
        result.reason = "setpoint.level must be position, velocity, or acceleration";
        return result;
      }
      if ((name == "offboard.publish_rate_hz" || name == "offboard.auto_start_retry_period_s" ||
        name == "offboard.start_transition_duration_s" || name == "omega_value") &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
        parameter.as_double() <= 0.0)
      {
        result.successful = false;
        result.reason = name + " must be positive";
        return result;
      }

      if (parameterAffectsTrajectoryRestart(name)) {
        trajectory_reset_pending_ = true;
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
    if (name == "trajName" || name == "trajectory_type" || name == "origin_x" || name == "origin_y" ||
      name == "offboard.takeoff_before_trajectory" || name == "offboard.use_start_transition" ||
      name == "offboard.start_transition_duration_s")
    {
      return true;
    }
    return startsWith(name, "figure8_") || startsWith(name, "helix_") ||
      startsWith(name, "flip_loop_") || startsWith(name, "fast_circle_");
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

    const double previous_rate = publish_rate_hz_;
    const std::string previous_offboard_topic = offboard_control_mode_topic_;
    const std::string previous_trajectory_topic = trajectory_setpoint_topic_;
    const std::string previous_command_topic = vehicle_command_topic_;
    const std::string previous_status_topic = vehicle_status_topic_;
    const std::string previous_status_secondary_topic = vehicle_status_topic_secondary_;
    const std::string previous_local_position_topic = vehicle_local_position_topic_;
    const std::string previous_local_position_secondary_topic = vehicle_local_position_topic_secondary_;
    const std::string previous_odometry_topic = vehicle_odometry_topic_;
    const std::string previous_path_topic = visualization_path_topic_;
    const std::string previous_pose_topic = visualization_pose_topic_;
    const std::string previous_vehicle_pose_topic = visualization_vehicle_pose_topic_;
    const bool preserve_phase =
      !trajectory_reset_pending_ && (trajectory_started_ || !takeoff_before_trajectory_);
    const double elapsed_time_s = std::max(0.0, (now() - start_time_).seconds());
    const double previous_theta = preserve_phase ? reference_trajectory_.theta(elapsed_time_s) : 0.0;

    loadParameters();
    syncSelectorParameters();

    if (preserve_phase) {
      geometric_controller::ReferenceTrajectory candidate(trajectory_parameters_);
      const double candidate_theta = candidate.theta(elapsed_time_s);
      const double theta_delta = previous_theta - candidate_theta;
      if (std::abs(theta_delta) > 1e-9) {
        trajectory_parameters_.phase_shift += theta_delta;
        RCLCPP_INFO(
          get_logger(),
          "Trajectory omega updated with continuous phase: trajName=%s omega_value=%.3f phase_shift=%.3f",
          trajectory_parameters_.traj_name.c_str(), trajectory_parameters_.omega_value,
          trajectory_parameters_.phase_shift);
      }
    }

    reference_trajectory_.setParameters(trajectory_parameters_);

    const bool topics_changed =
      previous_offboard_topic != offboard_control_mode_topic_ ||
      previous_trajectory_topic != trajectory_setpoint_topic_ ||
      previous_command_topic != vehicle_command_topic_ ||
      previous_status_topic != vehicle_status_topic_ ||
      previous_status_secondary_topic != vehicle_status_topic_secondary_ ||
      previous_local_position_topic != vehicle_local_position_topic_ ||
      previous_local_position_secondary_topic != vehicle_local_position_topic_secondary_ ||
      previous_odometry_topic != vehicle_odometry_topic_ ||
      previous_path_topic != visualization_path_topic_ ||
      previous_pose_topic != visualization_pose_topic_ ||
      previous_vehicle_pose_topic != visualization_vehicle_pose_topic_;

    if (topics_changed) {
      configureRosInterfaces();
    }
    if (std::abs(previous_rate - publish_rate_hz_) > 1e-6) {
      configureTimer();
    }
    if (trajectory_reset_pending_) {
      resetTrajectoryStart("trajectory parameters changed");
    }

    parameters_pending_ = false;
    trajectory_reset_pending_ = false;
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
    offboard_setpoint_counter_ = 0;
    offboard_command_sent_ = false;
    last_auto_start_command_time_s_ = -std::numeric_limits<double>::infinity();
    RCLCPP_INFO(get_logger(), "Trajectory reset: %s", reason.c_str());
  }

  void timerCallback()
  {
    applyPendingParameters();

    const auto stamp = now();
    const auto setpoint = currentReference(stamp);
    publishVisualization(setpoint, stamp);

    if (!offboard_enabled_) {
      return;
    }

    publishOffboardControlMode(stamp);
    publishTrajectorySetpoint(setpoint, stamp);

    maybeSendAutoStartCommands(stamp);
    if (offboard_setpoint_counter_ < static_cast<uint64_t>(prearm_setpoints_ + 1)) {
      ++offboard_setpoint_counter_;
    }
  }

  void maybeSendAutoStartCommands(const rclcpp::Time & stamp)
  {
    if (!auto_start_ || offboard_setpoint_counter_ < static_cast<uint64_t>(prearm_setpoints_)) {
      return;
    }
    if (isOffboardAndArmed()) {
      return;
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
    offboard_command_sent_ = true;
    last_auto_start_command_time_s_ = stamp.seconds();
  }

  geometric_controller::TrajectorySample currentReference(const rclcpp::Time & stamp)
  {
    if (!takeoff_before_trajectory_) {
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

      if (use_start_transition_ && localPositionValid() && !start_transition_finished_)
      {
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

  bool isOffboardAndArmed() const
  {
    return isOffboard() && isArmed();
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
    start_transition_start_time_ = stamp;
    start_transition_active_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Start transition: current NED [%.2f, %.2f, %.2f] -> target NED [%.2f, %.2f, %.2f] in %.2f s.",
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
      static_cast<double>(vehicle_local_position_.y), static_cast<double>(vehicle_local_position_.z),
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
    return local_position_received_ && vehicle_local_position_.xy_valid && vehicle_local_position_.z_valid;
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
    msg.position = setpoint_level_ == "position";
    msg.velocity = setpoint_level_ == "velocity";
    msg.acceleration = setpoint_level_ == "acceleration";
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    msg.timestamp = timestampMicros(stamp);
    offboard_control_mode_publisher_->publish(msg);
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

    if (jerk_logging_) {
      msg.jerk = toFloatArray(sample.jerk);
    }
    msg.yaw = finiteFloatOrNan(sample.yaw);
    msg.yawspeed = yaw_rate_feedforward_ ? finiteFloatOrNan(sample.yaw_rate) :
      std::numeric_limits<float>::quiet_NaN();
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
    path.poses.reserve(static_cast<size_t>(preview_points_));
    for (int i = 0; i < preview_points_; ++i) {
      const double alpha = static_cast<double>(i) / static_cast<double>(preview_points_ - 1);
      const auto sample = reference_trajectory_.sample(alpha * duration);
      path.poses.push_back(toPoseStamped(sample, stamp));
    }
    reference_path_publisher_->publish(path);
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
    if (vehicle_odometry_received_) {
      orientation = px4NedFrdToRosEnuFlu(vehicle_odometry_.q);
    } else {
      orientation = yawToQuaternion(yawNedToEnu(vehicle_local_position_.heading));
    }
    pose.pose.orientation.w = orientation.w;
    pose.pose.orientation.x = orientation.x;
    pose.pose.orientation.y = orientation.y;
    pose.pose.orientation.z = orientation.z;
    vehicle_pose_publisher_->publish(pose);
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
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    RCLCPP_INFO(get_logger(), "Offboard mode command sent");
  }

  void arm()
  {
    publishVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
    RCLCPP_INFO(get_logger(), "Arm command sent");
  }

  void publishVehicleCommand(uint32_t command, float param1 = 0.0f, float param2 = 0.0f)
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
        get_logger(), "PX4 status: nav_state=%u arming_state=%u accepts_offboard=%s",
        static_cast<unsigned int>(vehicle_status_.nav_state),
        static_cast<unsigned int>(vehicle_status_.arming_state),
        vehicle_status_.accepts_offboard_setpoints ? "true" : "false");
    }
  }

  void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    vehicle_local_position_ = *msg;
    local_position_received_ = true;
  }

  void vehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    vehicle_odometry_ = *msg;
    vehicle_odometry_received_ = true;
  }

  geometric_controller::TrajectoryParameters trajectory_parameters_;
  geometric_controller::ReferenceTrajectory reference_trajectory_;

  bool offboard_enabled_{true};
  bool auto_start_{true};
  bool arm_on_start_{true};
  bool takeoff_before_trajectory_{true};
  bool use_start_transition_{true};
  bool velocity_feedforward_{true};
  bool acceleration_feedforward_{true};
  bool jerk_logging_{false};
  bool yaw_rate_feedforward_{true};
  int prearm_setpoints_{10};
  double publish_rate_hz_{50.0};
  double auto_start_retry_period_s_{1.0};
  double start_transition_duration_s_{4.0};
  double takeoff_position_tolerance_{0.25};
  double takeoff_velocity_tolerance_{0.5};
  std::string setpoint_level_{"position"};

  std::string offboard_control_mode_topic_;
  std::string trajectory_setpoint_topic_;
  std::string vehicle_command_topic_;
  std::string vehicle_status_topic_;
  std::string vehicle_status_topic_secondary_;
  std::string vehicle_local_position_topic_;
  std::string vehicle_local_position_topic_secondary_;
  std::string vehicle_odometry_topic_;
  std::string visualization_path_topic_;
  std::string visualization_pose_topic_;
  std::string visualization_vehicle_pose_topic_;
  std::string visualization_frame_id_{"map"};
  bool visualization_ned_to_enu_{true};
  int preview_points_{240};
  double path_publish_period_s_{0.5};
  double path_publish_time_s_{-std::numeric_limits<double>::infinity()};

  uint8_t target_system_{1};
  uint8_t target_component_{1};
  uint8_t source_system_{1};
  uint8_t source_component_{1};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_publisher_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_secondary_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    vehicle_local_position_secondary_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;

  px4_msgs::msg::VehicleStatus vehicle_status_{};
  px4_msgs::msg::VehicleLocalPosition vehicle_local_position_{};
  px4_msgs::msg::VehicleOdometry vehicle_odometry_{};
  bool status_received_{false};
  bool local_position_received_{false};
  bool vehicle_odometry_received_{false};
  bool parameters_pending_{false};
  bool trajectory_reset_pending_{false};
  bool trajectory_started_{false};
  bool start_transition_active_{false};
  bool start_transition_finished_{false};
  bool prefer_trajectory_type_{false};
  bool syncing_selector_parameters_{false};
  bool offboard_command_sent_{false};
  double last_auto_start_command_time_s_{-std::numeric_limits<double>::infinity()};
  double last_takeoff_progress_log_time_s_{-std::numeric_limits<double>::infinity()};
  int trajectory_type_{1};
  std::array<std::array<double, 6>, 3> start_transition_coefficients_{};
  rclcpp::Time start_transition_start_time_;
  uint64_t offboard_setpoint_counter_{0};
  rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryOffboardNode>());
  rclcpp::shutdown();
  return 0;
}
