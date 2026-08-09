# geometric_controller

**English** | [简体中文](README.zh-CN.md)

ROS 2 trajectory generation and low-level controllers for PX4 Offboard. The
control code uses NED, body FRD, and SI units. RViz conversion to ENU is
visualization-only.

## Controllers

| ID | Name | Output |
|---:|---|---|
| 1 | `main_geometric` | ROS physical thrust and moment |
| 2 | `main_lee` | ROS physical thrust and moment |
| 3 | `main_johnson` | ROS physical thrust and moment |
| 4 | `main_sun_dfbc` | ROS physical thrust and moment |
| 5 | `main_geometric_indi` | ROS Geometric INDI physical thrust and moment |
| 6 | `px4_direct` | `TrajectorySetpoint` and built-in PX4 controllers |

Modes 1–5 set `OffboardControlMode.thrust_and_torque=true` and publish
`VehicleThrustSetpoint` plus `VehicleTorqueSetpoint`. They bypass the PX4
position, attitude, and rate controllers while retaining PX4 control allocation
and actuator output. Mode 5 is the startup default with both INDI loops enabled;
mode 6 is available for an independent PX4-chain check.

The `main.m` mapping is: mode 1 is `controllerName="geometric"` /
`controllerPDGeometric`, modes 2–4 are `lee`, `johnson`, and `sun_dfbc`, mode 5
is `controllerName="geometric_indi"` / `controllerGeometricINDI`, and mode 6 is
the ROS/PX4 counterpart of the `px4_iris` cascade. The former mode 0
`legacy_geometric` and its private `ctrl_mode` had no `main.m` counterpart and
have been removed without renumbering IDs 1–6.

`controllerPDGeometric` is not simply `controllerGeometricINDI` with INDI
removed. The former differentiates its closed-loop thrust-axis command to form
desired attitude derivatives; the latter uses the Sun Eq. (14)–(24) attitude
and reference-rate construction. ROS preserves that distinction. Disabling an
INDI switch in mode 5 bypasses only that incremental law and directly inverts
the desired force or moment; it stays on mode 5's Sun reference path and never
calls or falls back to mode 1.

The core distinction is:

```text
main_geometric (main.m: controllerPDGeometric)
  ac = ad + Kp(pd-p) + Kv(vd-v)
  Fc = m(g e3-ac)
  differentiate the closed-loop Fc chain for Rd, Omega_d, alpha_d
  alpha_c = KR Log(R'Rd) + KOmega(Omega_d^B-Omega) + alpha_d^B
  tau_c = Omega×J Omega + J alpha_c

main_geometric_indi (paper increments)
  ac = ad + Kp(pd-p) + Kv(vd-v)
  acceleration INDI on:  Fc = F0 - m(ac-a0)
                    off: Fc = m(g e3-ac)
  form Rd from Fc and yaw; use Sun Eq. (18)-(24) for Omega_r, alpha_r
  alpha_c = KR Log(R'Rd) + KOmega(Omega_r-Omega0) + alpha_r
  rate INDI on:  tau_c = tau0 + J(alpha_c-alpha0)
            off: tau_c = Omega×J Omega + J alpha_c
```

ROS retains the nominal `main.tex/main.m` unbounded incremental force and full
SO(3) Log. The two off branches bypass only the corresponding increment and
never delegate to mode 1.

### Control allocation for aggressive geometric INDI

The earlier vertical-figure-eight climb and circle oscillation were caused by
control allocation, not by Eq. (55), the INDI feedback filters, or the SO(3)
Log. When the requested wrench was infeasible, the old allocation and clipping
path did not preserve the attitude-control axes with the priority required by
`main.tex`. Losing roll/pitch moment authority made geometric INDI appear to
diverge even while its position error could remain temporarily small.

PX4 now uses the same bounded QCAT weighted-least-squares allocation as the
reference implementation. Its wrench weights strongly prioritize roll and
pitch, then yaw, while collective force yields first under saturation. Thus the
allocator returns the achieved bounded wrench used by `AllocationValue`, and
geometric INDI receives feedback from the same allocation law that applied the
command. No low-thrust `mg` floor, force-residual clamp, observer, alternate
attitude error, or filter workaround is added to the controller.

### Why the first switch to built-in PX4 control can pause

This is a control-ownership handover, not an INDI equation. Modes 1–5 publish
`thrust_and_torque=true`, so PX4 position, attitude, and rate control are
disabled. Mode 6 changes this to `position=true` through DDS, Commander, and
`VehicleControlMode`. While disabled, `mc_pos_control` clears its old trajectory
setpoint. On the rising enable edge it records `_time_position_control_enabled`
and rejects a setpoint older than that instant, temporarily generating a
failsafe setpoint if necessary. It then takes a new local-position cycle to
produce an attitude setpoint, a new attitude cycle to produce a rate setpoint,
and a gyro cycle to produce torque before allocation has a fully new wrench.

ROS publishes the new `OffboardControlMode` and `TrajectorySetpoint` from the
same reference-timer callback, but separate DDS topics and PX4 modules cannot
switch atomically. Allocation therefore briefly holds the last ROS wrench or
waits for the first PX4 wrench. The first entry from the bypassed state is most
visible; later cycles already have valid PX4 state and setpoints. Switching
only `controller_type` does not restart the trajectory phase, so this is not
the quintic transition or a trajectory reset.

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

The patch adds `AllocationValue.msg` and extends `VehicleTorqueSetpoint.msg`
with the PCA priority fields. Rate INDI always publishes the decomposition;
PX4 consumes it only for a PCA allocation method. WLS and the other allocation
methods use the total wrench and ignore the split. `AllocationValue` carries
the final allocated-wrench feedback.

Field order and types must match the messages in `df-main`.

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs --cmake-clean-cache
source install/setup.bash
colcon build --packages-select geometric_controller --cmake-clean-cache
```

## PX4 DDS rates

Mode 5 uses the standard state topics plus PX4's final allocated-wrench
feedback. Remove the DDS rate limit from `vehicle_attitude` and
`vehicle_local_position`, and publish `allocation_value` without a rate limit:

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

Run `make px4_sitl` after changing the file. `unlimited` removes DDS-side
downsampling; it does not upsample a source. Control rates are feedback-driven:
every new `VehicleAngularVelocity` sample runs rate INDI and publishes a wrench;
every new `VehicleLocalPosition` sample updates acceleration INDI and its force
is held between samples. SITL therefore runs at about 250/125 Hz and the measured
hardware at about 800/100 Hz. Attitude and allocation feedback are cached at
their own rates. The panel displays measured rates; there are no inner/outer
control-rate tuning parameters.

The 200 Hz attitude versus 800 Hz angular rate is produced by PX4 itself:
`vehicle_angular_velocity` follows high-rate gyro updates, while EKF2
`vehicle_attitude` follows the integrated `vehicle_imu` stream. The measured
hardware rates were about 801 Hz and 201 Hz. The rate loop need not be reduced
to the attitude rate; it uses the newest cached attitude at each rate sample.
`offboard.setpoint_rate_hz=250` refreshes trajectory references, previews, and
PX4-direct setpoints only. It no longer schedules ROS wrench control, so it is
not an inner- or outer-loop control rate.

This removes waiting introduced by a DDS rate gate, but it cannot remove PX4
filtering, scheduling, or actuator delay. The controller-specific profile also
reduces unused traffic: `sensor_combined` to 20 Hz, status topics to 5 Hz, and
GPS/global position to 10 Hz. If the DDS transport is bandwidth-limited, cap
`vehicle_angular_velocity` at 250 Hz while retaining the other control feedback
topics as `unlimited`.

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

### Trajectory and Sun reference generation

There are two distinct sources. The trajectory curves and analytic
`p/v/a/jerk/snap` derivatives are structurally the same as `main.m`'s
`evalFigure8*`, `evalHelixFlip*`, and `evalFastCircle` functions. The mapping
from incremental thrust vector and heading to `R_c`, followed by the jerk/snap,
current-state, and achieved-thrust calculation of `ω_r/α_r`, is Sun Eq. (14)-(24).
It corresponds to `geometricINDIReferenceCommand()` and
`sunFlatnessReferenceRates()` in `main.m`; MATLAB is the implementation
reference here, not the original source of that method.

The current runtime configuration is not numerically identical to the values
at the top of `main.m`: ROS uses a 3 m horizontal figure eight at 6 m altitude,
while the current MATLAB file uses 2 m and 3 m. The ROS YAML also locks yaw to
zero, whereas MATLAB makes horizontal-figure-eight and circle yaw follow the
horizontal velocity. ROS additionally ramps phase speed for one second after
the stationary takeoff segment. Thus this is the same curve formula plus the
Sun reference map, with ROS flight parameters and takeoff connection—not a
sample-for-sample replay of one MATLAB configuration.

With `trajectory_yaw_lock=true`, the trajectory layer explicitly supplies
`psi=trajectory_yaw_fixed` and `psi_dot=psi_ddot=0`. This locks only heading:
Sun's roll/pitch reference rates and accelerations are still generated from
`j_r/s_r`, current attitude/rate, and `T0`. The takeoff quintic also supplies
its analytic jerk and snap before entering the same Sun mapping.

In short: acceleration INDI reimplements the `mc_pos_control` two-pole `a0`
filter and `F0` delay/interpolation in ROS, while reusing the allocator-filtered
`allocated_force`. Rate INDI reimplements no filter or delay in ROS and directly
reuses PX4 `xyz`, `xyz_derivative`, and allocator-filtered `allocated_torque`.

### Acceleration INDI: PX4-to-ROS correspondence

Native PX4 `mc_pos_control` with `MPC_INDI_A_SRC=1` and `MPC_INDI_F_SRC=0`:

```text
VehicleLocalPosition.ax/ay/az
  → 2-pole LPF(MPC_INDI_A_LP) → a0

final actuator setpoint × effectiveness matrix
  → 2-pole LPF(CA_FORCE_CUTOFF) → allocated_force
  → history interpolation[VLP.timestamp_sample-MPC_INDI_F_DLY] → F0

F0,a0 → acceleration INDI
```

ROS mode 5:

```text
VehicleLocalPosition.ax/ay/az (unchanged over DDS)
  → ROS 2-pole LPF(indi_acceleration_cutoff_hz) → a0

AllocationValue.allocated_force (already processed by PX4 CA_FORCE_CUTOFF)
  → FRD-to-NED using the attitude at each AllocationValue event
  → paper-positive T*b3 stored in PX4-timestamped history
  → ROS history interpolation[VLP.timestamp_sample-indi_force_delay_s] → F0

Fc = F0 - mass*(ac-a0)
```

| Function | Native PX4 acceleration INDI | ROS mode 5 | PX4 Iris / ROS default |
|---|---|---|---:|
| Enable | `MPC_INDI_ACC_EN` | `indi_acceleration_enabled` | PX4 off / ROS on by default |
| Acceleration source | `MPC_INDI_A_SRC=1` | fixed `VehicleLocalPosition.ax/ay/az` | `1` |
| Two-pole `a0` LPF | `MPC_INDI_A_LP`, executed in `mc_pos_control` | `indi_acceleration_cutoff_hz`; zero bypasses it | `8 Hz / 8 Hz` |
| Force source | `MPC_INDI_F_SRC=0` | fixed physical `allocated_force` | `0` |
| Two-pole `F0` LPF | `CA_FORCE_CUTOFF`, executed in PX4 allocator | not repeated; receives the filtered field | `8 Hz` |
| `F0` delay/interpolation | `MPC_INDI_F_DLY`, executed in `mc_pos_control` | `indi_force_delay_s`; zero adds no delay | `0.01 s / 0 s` |
| Mass | `MPC_MASS` | `mass` | `0.75 kg` |

`MPC_INDI_A_LP` does not alter the DDS `VehicleLocalPosition` message, so ROS
reproduces it with `indi_acceleration_cutoff_hz=8`. `CA_FORCE_CUTOFF=8` already
alters `AllocationValue.allocated_force`, so ROS does not filter that field
again. The current ROS default `indi_force_delay_s=0` selects `F0` at the
acceleration sample time without an additional configured delay.
Both that field and `AllocationValue.timestamp` use the same PX4 HRT microsecond
clock, although they represent different physical events. This mirrors PX4's
call to `getDelayedAllocatedThrustAcceleration()`; ROS receipt time is not used
for alignment.

With PX4 fixed at `IMU_GYRO_CUTOFF=125`, `IMU_DGYRO_CUTOFF=10`,
`CA_TORQ_CUTOFF=8`, and `CA_FORCE_CUTOFF=8`, the corresponding ROS values are:

```yaml
indi_acceleration_cutoff_hz: 8.0
indi_force_delay_s: 0.0
```

These parameters reproduce PX4's acceleration-feedback filtering and force
delay; control-allocation feasibility and axis priority are handled separately
by the WLS allocator.

### Rate INDI: PX4-to-ROS correspondence

Native PX4 `mc_rate_control`:

```text
gyro → PX4 VehicleAngularVelocity module
  ├→ xyz            (IMU_GYRO_CUTOFF and notches) → ω0
  └→ xyz_derivative (differentiate + IMU_DGYRO_CUTOFF) → α0

final actuator setpoint × effectiveness matrix
  → 2-pole LPF(CA_TORQ_CUTOFF) → allocated_torque = τ0

τc = τ0 + J[MC_INDI_*_P*(ωsp-ω0)-α0]
```

ROS mode 5 subscribes to those same PX4 outputs:

```text
VehicleAngularVelocity.xyz            → ω0 directly
VehicleAngularVelocity.xyz_derivative → α0 directly
newest AllocationValue.allocated_torque → τ0 directly

αc = KR*Log(R'Rd) + KOmega*(ωr-ω0) + αr
τc = τ0 + J*(αc-α0)
```

ROS adds no filtering, delay, history, or timestamp pairing to these three rate
feedback values. “Direct” means no processing after reception; PX4 has already
performed the processing shown below before publishing them.

| Function | Native PX4 rate INDI | ROS mode 5 | Iris value |
|---|---|---|---:|
| Enable | `MC_INDI_RATE_EN` | `indi_rate_enabled` | PX4 off / ROS on by default |
| `ω0` preprocessing | `IMU_GYRO_CUTOFF` and notches in PX4 sensor module | use `xyz` directly | `125 Hz` |
| `α0` LPF | `IMU_DGYRO_CUTOFF` in PX4 sensor module | use `xyz_derivative` directly | `10 Hz` |
| Two-pole `τ0` LPF | `CA_TORQ_CUTOFF` in PX4 allocator | use `allocated_torque` directly | `8 Hz` |
| Inertia | `MC_J_X/Y/Z` | `inertia_x/y/z` | `0.0025/0.0021/0.0043 kg m²` |
| Rate-error gain | `MC_INDI_R/P/Y_P` | `KOmega_r/p/y`; ROS also includes `KR` and `αr` | PX4 `10/10/10`; ROS `20/20/8` |

`IMU_GYRO_CUTOFF`, `IMU_DGYRO_CUTOFF`, `CA_FORCE_CUTOFF`, and
`CA_TORQ_CUTOFF` are PX4 parameters. ROS neither reads nor overwrites them, but
they directly determine the bandwidth of feedback received by ROS. In contrast,
`MPC_INDI_A_LP` and `MPC_INDI_F_DLY` affect only PX4 `mc_pos_control`; ROS must
reproduce them with `indi_acceleration_cutoff_hz` and `indi_force_delay_s`.

`allocated_force/allocated_torque` are physical wrench values computed by the
allocator from final actuator commands and the effectiveness matrix, not sensor
measurements. This controller uses filtered `allocated_*`, not `raw_allocated_*`.

The IMU `VehicleAcceleration` and PX4 `AccelerationIndiStatus` are not used.
PX4 `MC_INDI_RATE_EN` is also not used by mode 5 because ROS publishes the final
wrench and bypasses the PX4 rate controller.

Selecting mode 5 enables both INDI loops by default. The panel also checks both
boxes whenever mode 5 is selected; either remains independently switchable for
diagnosis:

1. Both false: mode 5 keeps its Sun reference path and directly inverts force and moment.
2. Rate only: direct desired force plus rate INDI.
3. Both true: complete rate plus acceleration INDI.

Enabling both switches before arming is also supported and has been verified in
SITL from takeoff through the periodic trajectory.

The switches are independent, and a disabled loop does not gate output on its
corresponding `AllocationValue` feedback.

When takeoff reaches the periodic trajectory start, its phase rate ramps from
zero to `omega_value` over one second. This keeps the zero-velocity takeoff
quintic continuous with the nonzero initial velocity of trajectories such as
the horizontal figure eight.

Because `AllocationValue` is produced only after PX4 receives a wrench, mode 5
temporarily bypasses both incremental laws and publishes its own direct-inverse
wrench until the first finite allocated force/moment arrives. It never switches
to `main_geometric`; once engaged it stays in INDI. There is no feedback-age
threshold, rate-feedback timestamp pairing, stream timeout, or automatic
runtime fallback. `timestamp_sample` is used only for the F_SRC=0 force/
acceleration history selection and interpolation; it is not a "newer than
100 ms" gate.

### PCA priority fields

When rate INDI is enabled, mode 5 preserves the same decomposition as PX4:

```text
τ_feedback = τ0 - J*α0
τ_c = J*α_c + τ_feedback
VehicleTorqueSetpoint.xyz = normalized τ_c
VehicleTorqueSetpoint.xyz_indi_feedback = normalized τ_feedback
VehicleTorqueSetpoint.xyz_indi_feedback_valid = true
```

Both components receive the same torque normalization and final clipping ratio.
ROS does not decide whether PCA is active: PX4 consumes the split only when the
selected airframe/allocation method uses PCA (for example the configured df4
path). The normal iris WLS path ignores it and uses the total torque.

## Normalization

```text
T_normalized = normalizedthrust_constant * T / mass
τ_normalized = diag(normalizedtorque_constant_r,
                    normalizedtorque_constant_p,
                    normalizedtorque_constant_y) τ
```

Here `normalizedthrust_constant=h/g=m/T_max`. These fixed scales must match the
airframe. The current SITL values come from
the Iris effectiveness matrix; verify them again for a different airframe.

Their direct relationship to PX4 allocator quantities is:

```text
s_Fz = AllocationValue.force_setpoint_scale[2] = 1/T_max
normalizedthrust_constant = mass*s_Fz = mass/T_max = h/g

D_tau = diag(AllocationValue.torque_setpoint_scale)
normalizedtorque_constant_r/p/y = the three diagonal entries of D_tau
```

The messages actually published are therefore:

```text
VehicleThrustSetpoint.xyz = [0, 0, -s_Fz*T]
VehicleTorqueSetpoint.xyz = D_tau*τ
```

`force_setpoint_scale` and `torque_setpoint_scale` are generated by PX4 control
allocator from its effectiveness matrix and normalization rules and published
in `AllocationValue`; they are not independent PX4 parameters. Per the project
requirement, this controller uses fixed user parameters rather than consuming
the message scales each cycle. The fixed Iris values should equal those fields.
`MPC_THR_HOVER=h` provides the additional thrust check
`normalizedthrust_constant=MPC_THR_HOVER/g`. The F_SRC=0 INDI feedback equation
does not use these scales; they only convert the final physical wrench into the
dimensionless setpoints accepted by the PX4 allocator.

## Build and launch

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to geometric_controller
source install/setup.bash
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

The panel exposes trajectory values, controller selection,
`Kp/Kv/KR/KOmega`, mass, inertia, and wrench
normalization. Numeric fields have no artificial limits; parameter validity is
the user's responsibility. Background refresh does not overwrite text being
edited, so multi-digit values such as `50` can be entered normally. A read-only
field shows measured acceleration/rate-control and feedback rates.

Important INDI parameters:

- `indi_rate_enabled`
- `indi_acceleration_enabled`
- `indi_acceleration_cutoff_hz` (EKF-acceleration LPF, default `8 Hz`)
- `indi_force_delay_s` (F_SRC=0 force delay, default `0 s`)
- `Kp_*`, `Kv_*`, `KR_*`, and `KOmega_*`; ROS modes 1 and 5 share this parameter
  interface. Current configuration: `Kp=[10,10,10]`, `Kv=[6,6,6]`,
  `KR=[150,150,30]`, and `KOmega=[15,15,10]`

PX4 Iris retains `MPC_INDI_A_LP=8` and `MPC_INDI_F_DLY=0.01` for native PX4
acceleration INDI. ROS implements the corresponding processing independently
and does not read those PX4 parameters; its configured force delay is currently zero.

```bash
ros2 param set /trajectory_offboard_node controller_type 5
ros2 param set /trajectory_offboard_node indi_rate_enabled true
ros2 param set /trajectory_offboard_node indi_acceleration_enabled true
```

## Rate checks

```bash
ros2 topic echo /controller/control_rate_status --once
```

This reports the callback rates actually received and consumed by the node,
including VLP/acceleration control, gyro/rate output, attitude, and allocation,
without requiring the caller to account for PX4 message-version suffixes.

## References

- [UAV_Algorithm_Benchmark](https://github.com/mengchaoheng/UAV_Algorithm_Benchmark)
- [DuctedFanUAV-Autopilot `df-main`](https://github.com/mengchaoheng/DuctedFanUAV-Autopilot/tree/df-main)
- [PX4 ROS 2 Offboard Control](https://docs.px4.io/main/en/ros2/offboard_control)
