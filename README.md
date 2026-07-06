# geometric_controller

ROS 2 / PX4 offboard trajectory tracking package. The first version sends
`px4_msgs::msg::TrajectorySetpoint` to PX4 so the PX4 built-in position
controller tracks analytic reference trajectories. The trajectory generator is
kept separate from the PX4 adapter so a later ROS 2-side controller can reuse
the same reference interface and publish lower-level commands.

## Launch File Format

The package has two user-facing launch files:

- `geometric_controller.launch.py`: starts `trajectory_offboard_node`, RViz, and
  the C++/Qt tuning panel. Use this when PX4 SITL and Micro XRCE-DDS Agent are
  already running in separate terminals.
- `sitl_geometric_controller.launch.py`: starts PX4 SITL, Micro XRCE-DDS Agent,
  and includes `geometric_controller.launch.py`. Use this for one-command SITL.

ROS 2 supports Python, XML, and YAML launch frontends. This package uses Python
launch only, because the SITL entry has conditional `headless` behavior,
external processes, and launch-file inclusion. Controller defaults live in
`config/controller.yaml`; launch files load YAML and do not duplicate controller
parameter defaults.

PX4's `px4_ros_com/launch/offboard_control_launch.yaml` is the minimal official
example: it only launches the offboard node. This package keeps the same idea,
but adds a parameter file, RViz, and a small C++/Qt trajectory tuning panel.

## Prerequisites

Follow the PX4 ROS 2 installation and setup guide first:

https://docs.px4.io/main/en/ros2/user_guide

The PX4 SITL simulator and the Micro XRCE-DDS Agent must both be running before
this node can exchange `/fmu/...` topics with PX4.

## Recorded Environment

Project environment to keep matched:

- OS: Ubuntu 24.04 LTS
- ROS 2: Jazzy
- PX4 Autopilot: `v1.17.0`
- `px4_ros_com`: `~/ws_sensor_combined/src/px4_ros_com`
  - remote: `https://github.com/PX4/px4_ros_com.git`
  - branch: `main`
  - revision: `beta-384-g86e9aeb` (`86e9aeb`)
- `px4_msgs`: `~/ws_sensor_combined/src/px4_msgs`
  - remote: `https://github.com/PX4/px4_msgs.git`
  - branch: `main`
  - revision: `ff7ae28`

Keep `px4_msgs` and `px4_ros_com` aligned with the PX4 firmware version used in
SITL. If PX4 is updated, record the new PX4 tag and the matching message
repository revisions here.

This controller derives PX4 versioned output topic names from
`px4_msgs::msg::<Message>::MESSAGE_VERSION`, for example
`/fmu/out/vehicle_local_position_v1`. Keep the ROS 2 `px4_msgs` package matched
to the PX4-Autopilot checkout used for SITL. If you switch PX4 branches or tags,
sync the message definitions from that PX4 checkout and rebuild:

```bash
cd ~/ws_sensor_combined
cp -a ~/PX4-Autopilot/msg/*.msg src/px4_msgs/msg/
cp -a ~/PX4-Autopilot/msg/versioned/*.msg src/px4_msgs/msg/
cp -a ~/PX4-Autopilot/srv/*.srv src/px4_msgs/srv/
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to geometric_controller
source install/setup.bash
```

If PX4 and ROS 2 must intentionally use different message versions, use the PX4
ROS 2 message translation node instead.

## Build

Build from the workspace root, not from `src`:

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to geometric_controller
source install/setup.bash
```

If your ROS 2 distro is not Jazzy, replace `jazzy` with the output of
`echo $ROS_DISTRO`.

There may also be an old `~/ws_sensor_combined/src/install`
workspace on this machine. Avoid sourcing that one unless you intentionally
built from `~/ws_sensor_combined/src`; otherwise you can easily
run an older installed node while editing the newer source.

## One-Command SITL Launch

After building, this launch starts PX4 SITL, Micro XRCE-DDS Agent, the
controller, RViz, and the tuning panel from one ROS 2 launch process.

Open QGroundControl and wait until it connects to PX4 before starting this
launch. This follows the PX4 ROS 2 Offboard example; with the default PX4 SITL
arming checks, PX4 will not arm without a QGroundControl datalink or RC
connection.

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch geometric_controller sitl_geometric_controller.launch.py
```

The controller waits until PX4 `VehicleLocalPosition` is valid, then logs
`Ready to request takeoff` before it requests Offboard mode and Arm.

Headless Gazebo example:

```bash
ros2 launch geometric_controller sitl_geometric_controller.launch.py headless:=true
```

If your PX4 checkout is somewhere else, pass it explicitly:

```bash
ros2 launch geometric_controller sitl_geometric_controller.launch.py \
  px4_dir:=~/dev/PX4-Autopilot
```

Defaults:

- PX4 path: `~/PX4-Autopilot`
- PX4 model: `gz_x500`
- workspace path: `~/ws_sensor_combined`
- ROS 2 distro: `jazzy`
- PX4 GCS datalink arming requirement: enabled
- trajectory setpoint rate: `100.0 Hz`
- OffboardControlMode heartbeat rate: `5.0 Hz`

## Start PX4 SITL and DDS Bridge

If you do not use the one-command launch, open separate terminals and keep them
running.

Terminal 1, PX4 SITL:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

For headless Gazebo:

```bash
HEADLESS=1 make px4_sitl gz_x500
```

Terminal 2, Micro XRCE-DDS Agent:

```bash
MicroXRCEAgent udp4 -p 8888
```

The relative order of these two is not critical, but both should be alive before
launching `trajectory_offboard_node`. Starting the agent first is usually the
least surprising.

## Launch This Package

Terminal 3:

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch geometric_controller geometric_controller.launch.py
```

This starts:

- `trajectory_offboard_node`
- RViz2, showing `/reference/trajectory`, `/reference/current_pose`, and
  `/vehicle/current_pose`
- `trajectory_control_panel`, a small C++/Qt panel for the common trajectory
  parameters

By default `offboard.auto_start` is `true`, matching the ROS 1 workflow: after
the PX4-required heartbeat warmup, the node requests Offboard mode and arms the
vehicle. It keeps publishing the trajectory start point first; after the vehicle
reaches that point, trajectory time starts and PX4 tracks the selected path.
Before releasing trajectory time, `offboard.use_start_transition` guides the
reference from the current vehicle position to the trajectory start point with a
quintic polynomial, similar to the ROS 1 transition logic.

The current version still uses PX4's built-in multicopter position controller;
there is no ROS 2-side force/torque controller in this step. With the default
`setpoint.level=position`, it publishes `OffboardControlMode.position=true` and
`TrajectorySetpoint` with position, velocity feedforward, acceleration
feedforward, yaw, and yawspeed. PX4 uses the non-`NaN` velocity and acceleration
fields as feedforward terms, which makes moving references less laggy than pure
position setpoints.

The PX4 input streams are split into two timers:

- `offboard.heartbeat_rate_hz`: `OffboardControlMode`, default `5 Hz`,
  constrained to `3-10 Hz`.
- `offboard.setpoint_rate_hz`: `TrajectorySetpoint`, default `100 Hz`,
  constrained to `50-250 Hz` and published after PX4 local position is valid so
  PX4 has setpoints before the Offboard request.

This follows PX4's ROS 2 split between Offboard proof-of-life and actual
setpoints. PX4's official example uses a `100 ms` timer for a minimal `10 Hz`
demo, while the Saxion low-level reference separates a roughly `3 Hz` offboard
timer from a `100 Hz` controller timer.

For a pure RViz preview that does not write to `/fmu/in/...`, set
`offboard.enabled: false` in `config/controller.yaml` or pass a custom full
parameter file with `param_file:=...`.

For manual Offboard/Arm testing, keep `offboard.enabled: true` and set
`offboard.auto_start: false` in the YAML.

For a circular trajectory, set `trajName: fast_circle` and adjust
`omega_value` in the YAML.

Run the controller without RViz or the tuning panel:

```bash
ros2 launch geometric_controller geometric_controller.launch.py \
  launch_rviz:=false launch_tuning_panel:=false
```

If the vehicle still does not move, check that PX4 accepted Offboard and Arm:

```bash
ros2 topic list -t | grep vehicle_status
ros2 topic echo /fmu/out/<vehicle_status_topic> --once --qos-reliability best_effort
```

Expected values are `nav_state: 14` and `arming_state: 2`.

For the default `figure8_horizontal` parameters, the start setpoint is in PX4
NED coordinates:

```text
x = 0.0
y = 0.0
z = -3.0
```

So from the ground it should first climb to 3 m, then start the trajectory after
it reaches the start tolerance.

## RViz and Parameter Panel

RViz is for visualization only. It shows the generated reference path, the green
current reference axes, and the red vehicle axes.

The tuning panel is `trajectory_control_panel`. It is a C++/Qt executable in
this package, not a Python node and not `rqt_reconfigure`. ROS 2 parameters do
not provide the same named enum dropdown metadata that ROS 1
`dynamic_reconfigure` used, so the package uses this dedicated panel for the
ROS 1-style trajectory-name dropdown.

The panel talks directly to `/trajectory_offboard_node` and only shows the
common trajectory parameters:

- `omega_value`
- `trajName` as a dropdown list
- `transition_duration_s`
- `position_tolerance`
- `velocity_tolerance`
- `trajectory_yaw_lock`
- `trajectory_yaw_fixed`
- the shape parameters for the selected trajectory type

Changing `trajectory_type` or shape parameters resets the trajectory start.
Changing `omega_value` updates the speed online without moving the reference to
a different phase. This matches the ROS 1 `shape_phase_shift_` behavior: when
omega changes while the trajectory is running, the node compensates the phase so
the current reference position stays continuous. This package currently
implements the same fixed angular-rate form used by `main.m`:

```text
theta(t) = theta0 + phase_shift + omega_value * t
```

## Runtime Parameters

Supported trajectory names:

- `figure8_horizontal`
- `figure8_vertical`
- `helix_flip`
- `helix_flip_y`
- `flip_loop_sine`
- `fast_circle`

`trajectory_type` uses the ROS 1 dynamic-reconfigure numbering:

- `1`: `figure8_horizontal`
- `2`: `figure8_vertical`
- `3`: `helix_flip`
- `4`: `helix_flip_y`
- `5`: `flip_loop_sine`
- `6`: `fast_circle`

Useful online parameter changes:

```bash
ros2 param set /trajectory_offboard_node trajectory_type 6
ros2 param set /trajectory_offboard_node omega_value 1.0
ros2 param set /trajectory_offboard_node trajectory_yaw_lock true
ros2 param set /trajectory_offboard_node trajectory_yaw_fixed 0.0
```

Advanced PX4/offboard plumbing parameters still live on
`/trajectory_offboard_node`, but they are not shown in the tuning panel.

## Sanity Checks

Check PX4 topics:

```bash
ros2 topic list | grep /fmu
```

Check vehicle status:

```bash
ros2 topic list -t | grep vehicle_status
ros2 topic echo /fmu/out/<vehicle_status_topic> --once --qos-reliability best_effort
```

If `/fmu/...` topics do not appear, check that PX4 SITL is running and that
`MicroXRCEAgent udp4 -p 8888` is connected.

Check the actual setpoint being sent to PX4:

```bash
ros2 topic echo /fmu/in/trajectory_setpoint --once \
  --qos-reliability best_effort --qos-durability transient_local
```

If the vehicle reaches the start point but does not begin the trajectory, watch
the controller log line `Waiting before trajectory start...`. The release
condition is based on valid local position and distance to the start target, so
it should no longer get stuck merely because `VehicleStatus` discovery is slow.
