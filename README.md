# geometric_controller

**English** | [简体中文](README.zh-CN.md)

`geometric_controller` is a ROS 2 package for PX4 Offboard trajectory tracking
and full-wrench control. It generates analytic trajectories, computes desired
physical thrust and torque in ROS, and sends them to the PX4 control allocator.
RViz visualization and a runtime tuning panel are included.

The controller uses NED world coordinates, FRD body coordinates, and SI units.
RViz can display ENU coordinates without changing the internal control frame.

## Architecture

```text
ReferenceTrajectory
  -> FlatReference {p, v, a, jerk, snap, yaw, yaw_rate, yaw_accel}
  -> ROS controller
  -> physical thrust [N] and torque [N*m]
  -> fixed vehicle normalization
  -> VehicleThrustSetpoint + VehicleTorqueSetpoint
  -> PX4 control allocator
  -> actuators

PX4 state/allocator feedback
  -> VehicleLocalPosition
  -> VehicleAttitude
  -> VehicleAngularVelocity
  -> AllocationValue
  -> ROS controller
```

Modes 1–5 set `OffboardControlMode.thrust_and_torque=true`. They bypass the PX4
position, attitude, and rate controllers while retaining PX4 control allocation
and actuator output. Mode 6 publishes `TrajectorySetpoint` and uses the built-in
PX4 controller cascade.

## Controllers

| ID | Name | Output |
|---:|---|---|
| 1 | `main_geometric` | Physical thrust and torque from ROS |
| 2 | `main_lee` | Physical thrust and torque from ROS |
| 3 | `main_johnson` | Physical thrust and torque from ROS |
| 4 | `main_sun_dfbc` | Physical thrust and torque from ROS |
| 5 | `main_geometric_indi` | Geometric INDI thrust and torque from ROS |
| 6 | `px4_direct` | Built-in PX4 position, attitude, and rate control |

The default is `main_geometric_indi`, with rate and acceleration INDI enabled.

Controller mapping to the MATLAB reference implementation:

| ROS controller | `main.m` implementation |
|---|---|
| `main_geometric` | `controllerPDGeometric` |
| `main_lee` | Lee controller |
| `main_johnson` | Johnson controller |
| `main_sun_dfbc` | Sun DFBC controller |
| `main_geometric_indi` | `controllerGeometricINDI` |

`main_geometric` and `main_geometric_indi` use different constructions for the
desired attitude derivatives. Disabling either INDI switch in mode 5 replaces
only that incremental law with direct force or torque computation; the mode 5
Sun attitude, angular-rate, and angular-acceleration reference path remains in use.

## Geometric INDI

This section documents the current source-code data path and control computation,
using the public `main.m` as the implementation reference. Mode 5 implements:

```text
a_c = a_r + Kp (p_r - p) + Kv (v_r - v)
F_c = F_0 - m (a_c - a_0)

alpha_c = KR Log(R^T R_c) + KOmega (omega_r - omega_0) + alpha_r
tau_c = tau_0 + J (alpha_c - alpha_0)
```

`R_c` is constructed from the desired thrust direction and reference yaw.
The Sun method produces `omega_r` and `alpha_r` from reference jerk and snap,
the current attitude and angular velocity, and thrust.

### Acceleration INDI signal path

```text
VehicleLocalPosition.ax/ay/az
  -> ROS 2-pole LPF(indi_acceleration_cutoff_hz)
  -> a_0

PX4 final actuator setpoint x effectiveness matrix
  -> PX4 2-pole LPF(CA_FORCE_CUTOFF)
  -> AllocationValue.allocated_force
  -> FRD-to-NED conversion using attitude at each AllocationValue event
  -> PX4-timestamped history/interpolation(indi_force_delay_s)
  -> F_0

F_c = F_0 - mass * (a_c - a_0)
```

The ROS implementation uses sources corresponding to PX4
`MPC_INDI_A_SRC=1` and `MPC_INDI_F_SRC=0`. `allocated_force` has already passed
through `CA_FORCE_CUTOFF` in PX4, so ROS does not filter it again.

`VehicleLocalPosition.timestamp_sample` and `AllocationValue.timestamp` use the
same PX4 HRT microsecond clock. ROS selects or interpolates `F_0` from force
history at the acceleration sample time minus `indi_force_delay_s`. ROS receipt
time is not used for this alignment. A zero delay adds no configured offset.

### Rate INDI signal path

```text
VehicleAngularVelocity.xyz
  -> omega_0

VehicleAngularVelocity.xyz_derivative
  -> alpha_0

PX4 final actuator setpoint x effectiveness matrix
  -> PX4 2-pole LPF(CA_TORQ_CUTOFF)
  -> AllocationValue.allocated_torque
  -> tau_0

tau_c = tau_0 + J * (alpha_c - alpha_0)
```

ROS uses the PX4 angular velocity, angular acceleration, and allocated torque
directly. It adds no rate-feedback filter, delay, or timestamp pairing. PX4
determines the bandwidth of these feedback signals:

| Signal | PX4 processing | ROS processing |
|---|---|---|
| `omega_0` | `IMU_GYRO_CUTOFF` and notch filters | Direct use |
| `alpha_0` | `IMU_DGYRO_CUTOFF` | Direct use |
| `tau_0` | `CA_TORQ_CUTOFF` | Direct use |
| `F_0` | `CA_FORCE_CUTOFF` | Frame conversion and history interpolation |
| `a_0` | No ROS-side INDI preprocessing | 2-pole `indi_acceleration_cutoff_hz` LPF |

The current PX4 configuration uses:

```text
IMU_GYRO_CUTOFF=125
IMU_DGYRO_CUTOFF=10
CA_TORQ_CUTOFF=8
CA_FORCE_CUTOFF=8
```

The corresponding acceleration INDI configuration in ROS is:

```yaml
indi_acceleration_cutoff_hz: 8.0
indi_force_delay_s: 0.0
```

### Startup and switches

`indi_rate_enabled` and `indi_acceleration_enabled` are independent:

| Rate INDI | Acceleration INDI | Mode 5 behavior |
|---|---|---|
| Off | Off | Sun reference path, direct force, direct torque |
| On | Off | Direct force and rate INDI |
| Off | On | Acceleration INDI and direct torque |
| On | On | Complete Geometric INDI |

Before the first finite `AllocationValue` arrives, the controller uses direct
computation to establish an initial wrench. The enabled incremental laws engage
after feedback becomes available. No fixed feedback-age threshold is applied.

### PCA torque decomposition

With rate INDI enabled, ROS publishes the total torque and the PX4-compatible
INDI feedback component:

```text
tau_feedback = tau_0 - J * alpha_0
tau_c = J * alpha_c + tau_feedback

VehicleTorqueSetpoint.xyz = normalized(tau_c)
VehicleTorqueSetpoint.xyz_indi_feedback = normalized(tau_feedback)
VehicleTorqueSetpoint.xyz_indi_feedback_valid = true
```

Both components use the same torque normalization and final clipping ratio.
ROS preserves the decomposition but does not enable PCA. PX4 decides whether to
consume it from the airframe, allocation matrices, and `CA_METHOD`: the df4 PCA
path can use the priority component, while the iris WLS path ignores it and uses
the total torque.

## Wrench normalization

ROS controllers first compute a physical wrench, then convert it to the
dimensionless setpoints accepted by the PX4 allocator:

```text
T_normalized = normalizedthrust_constant * T / mass
tau_normalized = diag(normalizedtorque_constant_r,
                      normalizedtorque_constant_p,
                      normalizedtorque_constant_y) * tau
```

The parameters correspond to PX4 allocation scales as follows:

```text
normalizedthrust_constant = mass / T_max = MPC_THR_HOVER / g
normalizedtorque_constant_* = AllocationValue.torque_setpoint_scale[*]
```

Current iris values are stored in
[config/vehicles/iris.yaml](config/vehicles/iris.yaml). Vehicle mass, inertia,
and thrust/torque normalization constants must be updated together when changing
airframes.

## Control rates and DDS

ROS wrench control is feedback-driven:

- each new `VehicleAngularVelocity` runs rate control and publishes a wrench;
- each new `VehicleLocalPosition` updates acceleration INDI;
- the latest desired thrust vector is held between position-state updates;
- attitude and `AllocationValue` update their caches at their source rates.

`offboard.setpoint_rate_hz` refreshes trajectory references, previews, and PX4
setpoints for mode 6. It is not the ROS wrench-control rate. Measured rates are
available in the tuning panel and status topic.

PX4 DDS should publish the controller feedback topics at their source rates:

```yaml
- topic: /fmu/out/vehicle_angular_velocity
  type: px4_msgs::msg::VehicleAngularVelocity
  rate_limit: unlimited
- topic: /fmu/out/vehicle_attitude
  type: px4_msgs::msg::VehicleAttitude
  rate_limit: unlimited
- topic: /fmu/out/vehicle_local_position
  type: px4_msgs::msg::VehicleLocalPosition
  rate_limit: unlimited
- topic: /fmu/out/allocation_value
  type: px4_msgs::msg::AllocationValue
```

## Trajectories

`ReferenceTrajectory` supplies position, velocity, acceleration, jerk, snap,
and yaw derivatives. Available configured trajectories include:

- `figure8_horizontal`
- `figure8_vertical`
- `helix_flip`
- `helix_flip_y`
- `flip_loop_sine`
- `fast_circle`

Set the trajectory with `trajName` and its phase rate with `omega_value`. The
takeoff sequence first reaches the periodic trajectory start, then enters the
periodic motion. With `trajectory_yaw_lock=true`, `trajectory_yaw_fixed` is used
and yaw rate and acceleration are zero; the Sun mapping still generates roll
and pitch reference derivatives.

## Installation

Validated environment: Ubuntu 24.04, ROS 2 Jazzy, PX4 v1.18, and
`px4_msgs release/1.18`.

### PX4

```bash
cd ~
git clone --branch df-main --single-branch \
  https://github.com/mengchaoheng/DuctedFanUAV-Autopilot.git PX4-Autopilot
cd ~/PX4-Autopilot
make px4_sitl
```

### px4_msgs

The project requires `AllocationValue` and the extended
`VehicleTorqueSetpoint`:

```bash
cd ~/ws_sensor_combined/src
git clone --branch release/1.18 --single-branch \
  https://github.com/PX4/px4_msgs.git
git -C px4_msgs checkout 598c7aad7b2386f9406ebd2a2f841619fddc3c78
git -C px4_msgs apply \
  ../geometric_controller/patches/px4_msgs-release-1.18.patch
```

The message field order and types must match PX4 `df-main`.

### Build

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to geometric_controller
source install/setup.bash
```

## Launch

Two launch methods are supported.

### Method 1: one-command launch

Start PX4 SITL, Micro XRCE-DDS Agent, the controller node, RViz, and the tuning
panel with one launch command:

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch geometric_controller sitl_geometric_controller.launch.py
```

### Method 2: three terminals

Terminal 1, start PX4 SITL and `gz_iris`:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_iris
```

Terminal 2, start Micro XRCE-DDS Agent:

```bash
MicroXRCEAgent udp4 -p 8888
```

Terminal 3, start the controller node, RViz, and the tuning panel:

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch geometric_controller geometric_controller.launch.py
```

## Configuration and runtime control

Primary configuration files:

- [config/controller.yaml](config/controller.yaml): controller, INDI,
  trajectory, Offboard, PX4 topic, and visualization parameters;
- [config/vehicles/iris.yaml](config/vehicles/iris.yaml): vehicle mass,
  inertia, and normalization constants.

Common runtime changes:

```bash
ros2 param set /trajectory_offboard_node controller_type 5
ros2 param set /trajectory_offboard_node indi_rate_enabled true
ros2 param set /trajectory_offboard_node indi_acceleration_enabled true
ros2 param set /trajectory_offboard_node omega_value 0.5
```

The tuning panel controls trajectories, controller selection, INDI switches,
`Kp/Kv/KR/KOmega`, mass, inertia, and wrench normalization. It also displays
measured state-feedback and control-callback rates.

Read the control-rate status with:

```bash
ros2 topic echo /controller/control_rate_status --once
```

## References

- [UAV_Algorithm_Benchmark / main.m](https://github.com/mengchaoheng/UAV_Algorithm_Benchmark)
- [DuctedFanUAV-Autopilot df-main](https://github.com/mengchaoheng/DuctedFanUAV-Autopilot/tree/df-main)
- [PX4 ROS 2 Offboard Control](https://docs.px4.io/main/en/ros2/offboard_control)
