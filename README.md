# geometric_controller

**English** | [简体中文](README.zh-CN.md)

ROS 2 trajectory generation and low-level controllers for PX4 Offboard. The
control code uses NED, body FRD, and SI units. RViz conversion to ENU is
visualization-only.

## Controllers

| ID | Name | Output |
|---:|---|---|
| 0 | `legacy_geometric` | ROS physical thrust and moment |
| 1 | `main_geometric` | ROS physical thrust and moment |
| 2 | `main_lee` | ROS physical thrust and moment |
| 3 | `main_johnson` | ROS physical thrust and moment |
| 4 | `main_sun_dfbc` | ROS physical thrust and moment |
| 5 | `main_geometric_indi` | ROS Geometric INDI physical thrust and moment |
| 6 | `px4_direct` | `TrajectorySetpoint` and built-in PX4 controllers |

Modes 0–5 set `OffboardControlMode.thrust_and_torque=true` and publish
`VehicleThrustSetpoint` plus `VehicleTorqueSetpoint`. They bypass the PX4
position, attitude, and rate controllers while retaining PX4 control allocation
and actuator output. Mode 6 is the default baseline.

## Installation

The validated environment is Ubuntu 24.04, ROS 2 Jazzy, PX4 v1.18-based
`df-main`, and `px4_msgs release/1.18`.

```bash
cd ~
git clone --branch df-main --single-branch \
  https://github.com/mengchaoheng/DuctedFanUAV-Autopilot.git PX4-Autopilot
cd ~/PX4-Autopilot
make px4_sitl
```

`make px4_sitl` builds SITL only. The launch command below starts `gz_iris`.

Start from the official `px4_msgs` branch and apply this project's patch:

```bash
cd ~/ws_sensor_combined/src
git clone --branch release/1.18 --single-branch \
  https://github.com/PX4/px4_msgs.git
git -C px4_msgs checkout 598c7aad7b2386f9406ebd2a2f841619fddc3c78
git -C px4_msgs apply \
  ../geometric_controller/patches/px4_msgs-release-1.18.patch
```

The patch contains three interface changes:

1. Add `VehicleAccelerationIndiFeedback.msg`.
2. Add `AllocationValue.msg`.
3. Extend `VehicleTorqueSetpoint.msg` with `xyz_indi_feedback[3]` and
   `xyz_indi_feedback_valid`.

Field order and types must match the messages in `df-main`.

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs --cmake-clean-cache
source install/setup.bash
colcon build --packages-select geometric_controller --cmake-clean-cache
```

## PX4 DDS rates

The PX4 branch publishes the velocity-derivative result already computed by
`mc_pos_control::set_vehicle_states()` as `VehicleAccelerationIndiFeedback`; this does
not change the position-control law. The DDS configuration is:

```yaml
- topic: /fmu/out/vehicle_acceleration_indi_feedback
  type: px4_msgs::msg::VehicleAccelerationIndiFeedback
  rate_limit: unlimited

- topic: /fmu/out/allocation_value
  type: px4_msgs::msg::AllocationValue
  rate_limit: unlimited

- topic: /fmu/out/vehicle_angular_velocity
  type: px4_msgs::msg::VehicleAngularVelocity
  rate_limit: unlimited

- topic: /fmu/out/vehicle_attitude
  type: px4_msgs::msg::VehicleAttitude
  rate_limit: unlimited

- topic: /fmu/out/vehicle_local_position
  type: px4_msgs::msg::VehicleLocalPosition
  rate_limit: unlimited
```

Run `make px4_sitl` after changing the file. `unlimited` removes DDS-side
downsampling; it does not upsample a source. PX4 callbacks overwrite cached
samples. A 100 Hz timer updates the reference, while translational INDI updates
every 10 ms and holds its thrust vector between updates. Each fresh angular-rate
sample can trigger rotational INDI, capped by `inner_loop_rate_hz=250`; every
update reads the latest cached state and allocation feedback.

Native simulation and hardware rates may therefore differ. A hardware attitude
source near 200 Hz is reused by some 250 Hz rotational updates as part of this
latest-sample schedule. Set `inner_loop_rate_hz=200` if control computation
should not exceed the attitude source rate.

This removes waiting introduced by a DDS rate gate, but it cannot remove PX4
filtering, scheduling, or actuator delay. The controller-specific profile also
reduces unused traffic: `sensor_combined` to 20 Hz, status topics to 5 Hz, and
GPS/global position to 10 Hz. If the DDS transport is bandwidth-limited, cap
`vehicle_angular_velocity` and `allocation_value` at 250 Hz while retaining the
other control feedback topics as `unlimited`.

## Geometric INDI

[main.tex](https://github.com/mengchaoheng/geometric_controller/blob/main/main.tex)
and its PDF are the algorithm specification;
[main.m](https://github.com/mengchaoheng/UAV_Algorithm_Benchmark/blob/main/main.m)
is the discrete implementation reference. Mode 5 implements:

```text
a_c = Kp (p_r - p) + Kv (v_r - v) + a_r
(T b_z)_c = (T b_z)_0 - m (a_c - a_0)

α_c = Kθ Log(Rᵀ R_c) + Kω (ω_r - ω) + α_r
τ_c = τ_0 + J (α_c - α_0)
```

ROS subscriptions cache the newest PX4 data; the angular-rate callback also
triggers the rotational loop at the configured maximum rate:

- `a_0` is the latest `VehicleAccelerationIndiFeedback`: PX4's NED velocity notch,
  velocity LPF, finite difference, and derivative LPF output. ROS does not
  filter it again.
- `α_0` is `VehicleAngularVelocity.xyz_derivative`, already differentiated and
  filtered by PX4's native IMU pipeline.
- `(T b_z)_0` and `τ_0` use the filtered physical force and moment in the
  latest `AllocationValue`. The body-FRD force is rotated to NED and converted
  to the paper's positive `T b_z` convention.
- The translational equation runs at 100 Hz. Fresh angular-rate samples trigger
  the rotational equation up to 250 Hz. Both layers read the newest cache from
  the other topics; equal timestamps across sources are not required.

The IMU `VehicleAcceleration` is not used. The `mc_pos_control` module updates
and publishes this feedback on local-position samples even while it does not
own the control mode. PX4 `MC_INDI_RATE_EN` is not used by mode 5 because ROS
publishes the final wrench and bypasses the PX4 rate controller.

`indi_acceleration_enabled=true` is the complete algorithm and the default.
Set it to `false` to retain rotational INDI with direct geometric thrust for
layer-by-layer diagnosis.

`tau_0` and `alpha_0` do not require identical low-pass cutoff frequencies.
As described in the state-estimation section of `main.tex`, the control input
may use a lower cutoff than the state derivative to reduce jitter. Filter
frequencies tune bandwidth and noise; they are not a mode-5 enable condition.

### PCA priority fields

Mode 5 publishes:

```text
VehicleTorqueSetpoint.xyz = τ_c
τ_H = τ_0 - J α_0
τ_L = J α_c
τ_c = τ_H + τ_L
VehicleTorqueSetpoint.xyz_indi_feedback = τ_H
```

Both moment fields use the same fixed
`normalizedtorque_constant_r/p/y`. The PX4 allocator decides whether to use
the optional priority component. Without PCA, the total moment remains
available in `xyz`.

## Normalization

```text
T_normalized = normalizedthrust_constant * T / mass
             + normalizedthrust_offset
τ_normalized = diag(normalizedtorque_constant_r,
                    normalizedtorque_constant_p,
                    normalizedtorque_constant_y) τ
```

These fixed scales must match the airframe. The controller does not replace
them at runtime with `AllocationValue.torque_setpoint_scale`. The values are
effectively equal in current SITL; real-hardware DDS latency still needs
measurement.

## Build, test, and launch

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to geometric_controller
source install/setup.bash
colcon test --packages-select geometric_controller
colcon test-result --verbose
```

Launch `gz_iris`, the DDS Agent, controller, RViz, and panel:

```bash
ros2 launch geometric_controller sitl_geometric_controller.launch.py
```

If PX4 SITL and the Agent already run in separate terminals:

```bash
ros2 launch geometric_controller geometric_controller.launch.py
```

## Tuning panel

The panel exposes trajectory values, controller selection, 100/250 Hz loop
rates, `Kp/Kv/KR/KOmega`, mass, inertia, and wrench
normalization. Numeric fields have no artificial limits; parameter validity is
the user's responsibility. Background refresh does not overwrite text being
edited, so multi-digit values such as `50` can be entered normally.

Important INDI parameters:

- `indi_acceleration_enabled`
- `outer_loop_rate_hz` (default 100 Hz)
- `inner_loop_rate_hz` (default maximum 250 Hz)
- `indi_Kp_*` and `indi_Kv_*`, independent from other controllers' `Kp/Kv`
- `indi_Ktheta_*` and `indi_Komega_*`
- `yaw_torque_cutoff_hz`, the common yaw-moment output LPF for the direct
  wrench path (1 Hz by default; 0 disables it)

PX4 parameters `MPC_VEL_LP`, `MPC_VEL_NF_FRQ`, `MPC_VEL_NF_BW`, and
`MPC_VELD_LP` configure the translational feedback filters. ROS does not expose
a duplicate set.

```bash
ros2 param set /trajectory_offboard_node controller_type 5
ros2 param set /trajectory_offboard_node indi_acceleration_enabled false
ros2 param set /trajectory_offboard_node indi_acceleration_enabled true
```

## Rate checks

```bash
ros2 topic hz /fmu/out/vehicle_local_position
ros2 topic hz /fmu/out/vehicle_acceleration_indi_feedback
ros2 topic hz /fmu/out/vehicle_attitude
ros2 topic hz /fmu/out/vehicle_angular_velocity
ros2 topic hz /fmu/out/allocation_value
ros2 topic hz /fmu/in/vehicle_torque_setpoint
```

## References

- [UAV_Algorithm_Benchmark](https://github.com/mengchaoheng/UAV_Algorithm_Benchmark)
- [DuctedFanUAV-Autopilot `df-main`](https://github.com/mengchaoheng/DuctedFanUAV-Autopilot/tree/df-main)
- [PX4 ROS 2 Offboard Control](https://docs.px4.io/main/en/ros2/offboard_control)
