# geometric_controller

**English** | [简体中文](README.zh-CN.md)

This package provides ROS 2 trajectory generation and low-level controllers for
PX4 Offboard operation. It preserves the parallel trajectory/controller
selection model of `mavros_controllers/UAV_Algorithm_Benchmark` and provides two
mutually exclusive control paths:

- `controller_type=8` publishes `TrajectorySetpoint`; the built-in PX4
  position, velocity, attitude, and rate controllers perform the tracking.
- `controller_type=0..7` computes physical thrust `T [N]` and body-frame moment
  `τ [N·m]` from the reference and PX4 state. It converts them to PX4-normalized
  values and publishes `VehicleThrustSetpoint` and `VehicleTorqueSetpoint`.
  This bypasses the PX4 position, velocity, attitude, and rate controllers,
  retaining only control allocation and actuator output.

The safer `controller_type=8` is the default. Validate vehicle parameters,
signs, gains, and saturation in SITL before using a low-level mode.

## Dependencies and version requirements

The currently validated environment is listed below. PX4 uORB messages change
between versions, so the PX4 firmware and `px4_msgs` must be used as a matched
pair. Do not select unrelated versions based only on repository names.

| Component | Validated version | Requirement |
|---|---|---|
| Operating system | Ubuntu 24.04.4 LTS | Ubuntu 24.04 recommended |
| ROS 2 | Jazzy | Currently supported and tested ROS 2 distribution |
| C++ | C++17 | The compiler must support C++17 |
| PX4 firmware | `mengchaoheng/DuctedFanUAV-Autopilot:df-main`, commit `1010e3fcb037bb552cd809be34c36e97cf94c18d` | Must contain the Iris, Offboard wrench, and DDS configuration described below |
| `px4_msgs` | `v1.17.0` baseline, commit `86d8239e962f6939e05c3737784f60c02fa884db`, synchronized with the `df-main` message definitions | Every field must match the PX4 `msg/` tree used to build the firmware |
| `px4_ros_com` | `main`, commit `86e9aeb20e55a4673fa8a9f1c29ea06a6c5ad1af` | Reference and ROS 2 Offboard example only; not a direct build dependency |
| Micro XRCE-DDS Agent | A version compatible with the PX4 `uxrce_dds_client` above | Must run for SITL or hardware DDS communication |
| Eigen | 3.4.0 validated | ROS dependencies: `eigen` and `eigen3_cmake_module` |
| Qt | Qt 5.15.13 validated | Only the control panel requires `Qt5::Widgets` |

Use the following PX4 repository:

```text
https://github.com/mengchaoheng/DuctedFanUAV-Autopilot.git
branch: df-main
```

This controller targets the `gz_iris` in that branch, not the stock `gz_x500`.
The corresponding airframe must provide:

```text
MPC_MASS = 0.75 kg
MC_J_X   = 0.0025 kg·m²
MC_J_Y   = 0.0021 kg·m²
MC_J_Z   = 0.0043 kg·m²
CA_ROTORx_CT = 8.5 N
CA_ROTORx_KM = ±0.0157 m
```

For the ROS-side moment loop to run at the actual PX4 state update rate,
`src/modules/uxrce_dds_client/dds_topics.yaml` must contain at least:

```yaml
- topic: /fmu/out/vehicle_angular_velocity
  type: px4_msgs::msg::VehicleAngularVelocity
  rate_limit: 250.

- topic: /fmu/out/vehicle_attitude
  type: px4_msgs::msg::VehicleAttitude
  rate_limit: 250.

- topic: /fmu/out/vehicle_local_position
  type: px4_msgs::msg::VehicleLocalPosition
  rate_limit: 125.

- topic: /fmu/out/vehicle_odometry
  type: px4_msgs::msg::VehicleOdometry
  rate_limit: 125.
```

Regenerate and rebuild PX4 after changing DDS topics or uORB messages:

```bash
cd /path/to/DuctedFanUAV-Autopilot
DONT_RUN=1 make px4_sitl gz_iris
```

The current `px4_msgs` tree also synchronizes `VehicleControlMode`,
`VehicleStatus`, and `VehicleTorqueSetpoint` with `df-main`. The
`VehicleTorqueSetpoint.xyz_indi_feedback` field exists only to preserve the DDS
wire type; this controller does not consume it. If the PX4 and ROS message
definitions differ, topics may fail to match, fields may be misaligned, or
Offboard mode detection may fail. Rebuilding this package alone cannot correct
that condition: synchronize `px4_msgs` first.

Direct ROS dependencies are recorded in `package.xml`: `rclcpp`, `px4_msgs`,
`geometry_msgs`, `nav_msgs`, `rcl_interfaces`, Eigen, Qt5, RViz2, and ROS 2
launch. Build `geometric_controller`, the matching `px4_msgs`, and optionally
`px4_ros_com` in the same colcon workspace.

## Controller selection

The IDs match the ROS 1 `UAV_Algorithm_Benchmark` branch:

| ID | Name | ROS-side output |
|---:|---|---|
| 0 | `legacy_geometric` | Full thrust and rigid-body moment |
| 1 | `main_geometric` | `main.m` geometric controller |
| 2 | `main_lee` | Lee SO(3) controller |
| 3 | `main_johnson` | Johnson logarithmic SO(3) controller |
| 4 | `main_sun_dfbc` | Sun differential-flatness controller |
| 5 | `main_sun_dfbc_indi` | Sun DFBC + INDI |
| 6 | `main_tal` | Tal differential flatness + INDI |
| 7 | `main_geometric_indi` | Geometric + INDI |
| 8 | `px4_direct` | `TrajectorySetpoint` and built-in PX4 controllers |

The implementation uses the PX4 NED world frame and FRD body frame. It does not
perform implicit ENU/FLU conversion inside the control loop. RViz visualization
alone can be converted to ENU with `visualization.ned_to_enu`.

Controllers 0–7 publish:

```text
OffboardControlMode.thrust_and_torque = true
VehicleThrustSetpoint.xyz = [0, 0, -T_normalized]
VehicleTorqueSetpoint.xyz = [τx, τy, τz]_normalized
```

This follows PX4 ROS 2 Offboard control-level selection. PX4 requires a
continuous `OffboardControlMode` proof-of-life stream above approximately 2 Hz;
this package uses a 5 Hz heartbeat by default. Mode 8 publishes trajectory
setpoints at 250 Hz. Modes 0–7 use a 125 Hz position/reference outer loop and a
250 Hz moment loop driven by `VehicleAngularVelocity`, using the latest
`VehicleAttitude` (approximately 200–250 Hz in Iris SITL). See
[PX4 ROS 2 Offboard Control](https://docs.px4.io/main/en/ros2/offboard_control).

When switching from mode 8 to modes 0–7, the node first publishes a
`thrust_and_torque` heartbeat. It waits for `VehicleControlMode` to confirm that
the PX4 attitude/rate loops are disabled and control allocation is enabled,
then resets the controller and starts wrench output. Switching among modes 0–7
does not repeat this PX4 mode handshake, avoiding a stale-moment hold during
controller changes.

## Physical wrench to PX4 normalization

Controllers use SI units internally. Instead of reconstructing the PX4
effectiveness matrix on the ROS side, normalization uses explicit parameters,
as in `mavros_controllers`:

```text
normalizedthrust_constant = mass / total_max_thrust
T_normalized = normalizedthrust_constant * (T / mass)
             + normalizedthrust_offset
τ_normalized = diag(normalizedtorque_constant_r,
                    normalizedtorque_constant_p,
                    normalizedtorque_constant_y) * τ
```

With zero offset, `T_normalized = T / total_max_thrust`. The three torque
constants convert physical roll, pitch, and yaw moments into PX4-normalized
torque and must match the output capability or calibration of the active PX4
airframe.

Normalized torque is clamped to `[-1, 1]` and collective thrust to `[0, 1]`
before publication; thrust is published along FRD `-z`. The node emits
throttled warnings when saturation occurs.

Controllers 0–7 run completely on the ROS side, and PX4 receives only
normalized thrust/moment setpoints. Only mode 8 delegates trajectory tracking
to the built-in PX4 controller. ROS INDI therefore does not subscribe to PX4
control-allocation feedback. Consistent with `main.m`, each controller stores
its previous requested physical thrust and moment and feeds their
second-order-Butterworth-filtered values into the next incremental update:

```text
F_c = F_0 + mass * (a_c - a_0)
τ_c = τ_0 + J * (α_c - α_0)
```

Here `F_0` and `τ_0` are filtered versions of the controller's own previous
request. Translational acceleration `a_0` does not use
`VehicleLocalPosition.ax/ay/az`. Like `mc_pos_control`, it processes
`VehicleLocalPosition.vx/vy/vz` through a velocity notch, first-order velocity
low-pass, numerical differentiation, and first-order derivative low-pass. The
current `df-main` parameters correspond to `MPC_VEL_LP=0`,
`MPC_VEL_NF_FRQ=0`, `MPC_VEL_NF_BW=5`, and `MPC_VELD_LP=5 Hz`.
Tal/Geometric-INDI then still applies the outer-loop second-order Butterworth
feedback filtering required by the `main.m` control law; state acquisition
filtering does not replace controller algorithm state.

Rotational acceleration `α_0` directly uses
`VehicleAngularVelocity.xyz_derivative`. PX4 has already differentiated this
field and applied the `IMU_DGYRO_CUTOFF` second-order low-pass, so the ROS side
does not filter it again and add phase delay. Extra fields in the `df-main`
`VehicleTorqueSetpoint` are present only for DDS layout compatibility; this
node sends zeros with `xyz_indi_feedback_valid=false`.

## Vehicle parameter file

Only `config/vehicles/iris.yaml` is provided. It comes from the
`DuctedFanUAV-Autopilot/df-main` Iris airframe and matches the plant parameters
in `UAV_Algorithm_Benchmark/main.m`: mass `0.75 kg`, inertia
`[0.0025, 0.0021, 0.0043] kg·m²`, and a maximum thrust of `8.5 N` for each of
four motors.

The main parameter file is loaded first and the vehicle file second, so vehicle
parameters override identically named values:

```bash
ros2 launch geometric_controller geometric_controller.launch.py \
  vehicle_param_file:=$HOME/ws_sensor_combined/src/geometric_controller/config/vehicles/iris.yaml
```

The vehicle parameter file must match the PX4/Gazebo model being launched. Both
the default launch target and default vehicle configuration use the `df-main`
`gz_iris`.

## Build and test

The recorded environment is Ubuntu 24.04 and ROS 2 Jazzy. The workspace
`px4_msgs` uses `v1.17.0` as its baseline, with `VehicleStatus` and
`VehicleTorqueSetpoint` layouts synchronized to the local
`DuctedFanUAV-Autopilot/df-main`. `px4_msgs` must match the PX4 firmware message
version.

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to geometric_controller
source install/setup.bash
colcon test --packages-select geometric_controller
colcon test-result --verbose
```

Controller tests cover ID mapping, the physical hover wrench from every
controller, finite output over repeated updates, yaw-reference behavior,
normalization, and the acceleration feedback filter.

## Launch

Launch `gz_iris`, Micro XRCE-DDS Agent, the controller node, RViz, and the
control panel with one command:

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch geometric_controller sitl_geometric_controller.launch.py
```

Common launch arguments:

```bash
ros2 launch geometric_controller sitl_geometric_controller.launch.py \
  px4_dir:=~/PX4-Autopilot \
  px4_model:=gz_iris \
  headless:=true \
  launch_rviz:=false
```

`~/PX4-Autopilot` must point to the
[`df-main` branch of `mengchaoheng/DuctedFanUAV-Autopilot`](https://github.com/mengchaoheng/DuctedFanUAV-Autopilot/tree/df-main).

If PX4 SITL and the Agent are already running:

```bash
ros2 launch geometric_controller geometric_controller.launch.py
```

PX4 arming checks may require QGroundControl or an RC/datalink connection. The
node waits for valid `VehicleLocalPosition`; controllers 0–7 additionally wait
for valid `VehicleAttitude` and `VehicleAngularVelocity` before publishing
setpoints and requesting Offboard/Arm. PX4 `dds_topics.yaml` must expose
`/fmu/out/vehicle_attitude` and `/fmu/out/vehicle_angular_velocity` at 250 Hz.

## Recommended validation sequence

1. Keep `controller_type=8` and verify that the `df-main` `gz_iris` completes
   the takeoff transition and tracks the reference.
2. Check the PX4 thrust/moment normalization scales printed at node startup.
3. Temporarily set `offboard.auto_start` and `offboard.arm_on_start` to `false`.
4. Inspect hover output, axis signs, and saturation warnings on the ground.
5. Start with a slow, constant-altitude trajectory, then test controllers 1,
   2, 3, and 4 incrementally.
6. Test derivative/filter-dependent controllers 5, 6, and 7 last.

Select controllers from the panel or with ROS parameters:

```bash
ros2 param set /trajectory_offboard_node controller_type 1
ros2 param set /trajectory_offboard_node trajectory_type 6
ros2 param set /trajectory_offboard_node omega_value 0.5
```

Switching online from mode 8 to modes 0–7 immediately changes the PX4 Offboard
control level. Treat in-flight switching as a high-risk operation: validate it
in SITL first and ensure continuity between the active state and reference.

## Control panel

The C++/Qt panel has two tabs:

- `Reference`: trajectory type, angular rate, shape, heading, and takeoff
  transition parameters.
- `Controller`: controller ID, 125 Hz outer-loop and 250 Hz inner-loop rates,
  `Kp/Kv/KR/KOmega`, mass, inertia, INDI cutoff frequencies, and PX4 physical
  normalization parameters.

Trajectory IDs retain the ROS 1 numbering:

| ID | Trajectory |
|---:|---|
| 1 | `figure8_horizontal` |
| 2 | `figure8_vertical` |
| 3 | `helix_flip` |
| 4 | `helix_flip_y` |
| 5 | `flip_loop_sine` |
| 6 | `fast_circle` |

Changing trajectory type or shape restarts the transition to the trajectory
start. Changing `omega_value` online compensates phase to prevent a reference
position discontinuity. With `trajectory_yaw_lock=true`, the controller uses
constant `trajectory_yaw_fixed`, with zero yaw rate and yaw acceleration. With
it set to `false`, the controller uses the yaw, yaw rate, and yaw acceleration
provided by the trajectory generator.

Default trajectory amplitudes `Ax/Ay/Az` are `3 m` wherever applicable, and
the default center height `Hc` is `6 m`.

## Main parameters

- `controller_type`: controller 0–8.
- `ctrl_mode`: quaternion/geometric error selection for the legacy controller.
- `Kp_*`, `Kv_*`: position and velocity feedback gains.
- `KR_*`, `KOmega_*`: attitude and angular-rate gains.
- `max_acc`: translational feedback-acceleration limit.
- `indi_filter_cutoff_hz`: INDI second-order low-pass cutoff.
- `indi_velocity_lpf_hz`: corresponds to PX4 `MPC_VEL_LP`.
- `indi_velocity_notch_hz`, `indi_velocity_notch_bandwidth_hz`: correspond to
  `MPC_VEL_NF_FRQ` and `MPC_VEL_NF_BW`.
- `indi_velocity_derivative_lpf_hz`: corresponds to PX4 `MPC_VELD_LP`.
- `outer_loop_rate_hz`: position/reference outer-loop rate for modes 0–7;
  default 125 Hz.
- `inner_loop_rate_hz`: angular-rate feedback and wrench publication rate for
  modes 0–7; default 250 Hz.
- `normalizedthrust_constant`: `mass / total_max_thrust`.
- `normalizedthrust_offset`: normalized collective-thrust offset.
- `normalizedtorque_constant_r/p/y`: physical moment to PX4-normalized moment
  scales.
- `offboard.heartbeat_rate_hz`: Offboard heartbeat, range 3–10 Hz.
- `offboard.setpoint_rate_hz`: mode 8 PX4 trajectory-setpoint rate, range
  50–250 Hz.
- `setpoint.level`: mode 8 only; selects position, velocity, or acceleration.

The control laws remain aligned with `main.m`, while default gains account for
the additional PX4/Gazebo control allocation and motor dynamics. The Iris
direct-wrench defaults are `KR=[150,150,80]` and `KOmega=[50,50,3]`. The yaw
gains are tuned independently for the `df-main` Iris ROS 2/DDS direct-wrench
path: they strengthen attitude-error recovery while reducing the weight of
delayed angular-rate feedback. Applying the ideal-rigid-body gains
`[150,150,3]` and `[20,20,8]` directly excites an approximately 43 Hz yaw limit
cycle in the current SITL.

Controller 0 computes and uses analytic desired angular velocity and angular
acceleration when producing physical moments on the ROS side, rather than
retaining only the attitude-error component that the original MAVROS
implementation passed to the PX4 rate loop.

Mode 8 publishes position, velocity/acceleration feedforward, and the trajectory
generator yaw and yaw rate. Yaw-rate forwarding no longer has a separate
disable switch. Modes 0–7 always use the complete reference:
`position/velocity/acceleration/jerk/snap/yaw/yaw_rate/yaw_accel`.

## Runtime checks

```bash
ros2 topic list | grep /fmu
ros2 topic echo /fmu/in/vehicle_thrust_setpoint --once
ros2 topic echo /fmu/in/vehicle_torque_setpoint --once
ros2 topic echo /fmu/in/trajectory_setpoint --once
ros2 topic hz /fmu/out/vehicle_angular_velocity
ros2 topic hz /fmu/in/vehicle_torque_setpoint
ros2 topic list -t | grep vehicle_status
```

After entering Offboard and arming normally, `VehicleStatus` should generally
report `nav_state: 14` and `arming_state: 2`. The default horizontal
figure-eight starts at NED `[0, 0, -6] m`.

## References

- [mengchaoheng/UAV_Algorithm_Benchmark](https://github.com/mengchaoheng/UAV_Algorithm_Benchmark)
- [mengchaoheng/mavros_controllers (`UAV_Algorithm_Benchmark` branch)](https://github.com/mengchaoheng/mavros_controllers/tree/UAV_Algorithm_Benchmark)
- [mengchaoheng/DuctedFanUAV-Autopilot (`df-main` branch)](https://github.com/mengchaoheng/DuctedFanUAV-Autopilot/tree/df-main)
- [PX4 ROS 2 Offboard Control](https://docs.px4.io/main/en/ros2/offboard_control)
- [SaxionMechatronics/px4_offboard_lowlevel](https://github.com/SaxionMechatronics/px4_offboard_lowlevel)
- [Jaeyoung-Lim/px4-offboard](https://github.com/Jaeyoung-Lim/px4-offboard)
- [kousheekc/nmpc_px4_ros2](https://github.com/kousheekc/nmpc_px4_ros2)

Controller mathematics comes from `main.m` and the ROS 1 branch. ROS 2/PX4
transport, the Offboard heartbeat, and thrust/moment interfaces follow the PX4
official example and `px4_offboard_lowlevel`. `nmpc_px4_ros2` was used to check
ROS 2 node and PX4 interface organization; no additional NMPC controller is
included in the controller IDs.
