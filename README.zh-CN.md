# geometric_controller

[English](README.md) | **简体中文**

这是一个面向 PX4 Offboard 的 ROS 2 轨迹与低层控制器包。它保留
`mavros_controllers/UAV_Algorithm_Benchmark` 的“参考轨迹和控制器并列选择”
结构，并提供两类互斥的控制链路：

- `controller_type=8`：发送 `TrajectorySetpoint`，由 PX4 内置位置、速度、
  姿态和角速度控制器完成跟踪。
- `controller_type=0..7`：ROS 2 根据参考轨迹和 PX4 状态计算物理推力
  `T [N]` 与机体系力矩 `τ [N·m]`，转换为 PX4 归一化量后发送
  `VehicleThrustSetpoint` 和 `VehicleTorqueSetpoint`。此时绕过 PX4 的位置、
  速度、姿态和角速度控制器，仅保留 PX4 control allocator 和执行器输出。

默认是较安全的 `controller_type=8`。低层模式必须先在 SITL 中验证车辆参数、
符号、增益和饱和情况。

## 依赖与版本要求

本包当前验证环境如下。PX4 uORB 消息会随版本变化，因此 PX4 固件与
`px4_msgs` 必须成对使用，不能只根据仓库名称随意选择不同版本。

| 组件 | 当前验证版本 | 要求 |
|---|---|---|
| 操作系统 | Ubuntu 24.04.4 LTS | 推荐 Ubuntu 24.04 |
| ROS 2 | Jazzy | 当前正式支持和测试的 ROS 2 发行版 |
| C++ | C++17 | 编译器必须支持 C++17 |
| PX4 固件 | 基于 PX4 v1.18.0 的 `mengchaoheng/DuctedFanUAV-Autopilot:df-main`，提交 `912d45ca037221676992c208d0dce48f99eb04f0` | 必须包含下述 Iris、Offboard wrench 和 DDS 配置 |
| `px4_msgs` | 官方 `release/1.18`，提交 `598c7aad7b2386f9406ebd2a2f841619fddc3c78` | 不增加控制器专用字段；所有序列化字段必须匹配 PX4 v1.18.0 的 `msg/` |
| `px4_ros_com` | `main`，提交 `86e9aeb20e55a4673fa8a9f1c29ea06a6c5ad1af` | 仅作为 ROS 2 Offboard 示例和参考，不是本包的直接编译依赖 |
| Micro XRCE-DDS Agent | 与上述 PX4 `uxrce_dds_client` 兼容的版本 | 运行 SITL/真机 DDS 通信时必须启动 |
| Eigen | 3.4.0（已验证） | ROS 依赖：`eigen`、`eigen3_cmake_module` |
| Qt | Qt 5.15.13（已验证） | 仅控制面板需要 `Qt5::Widgets` |

PX4 仓库使用：

```text
https://github.com/mengchaoheng/DuctedFanUAV-Autopilot.git
branch: df-main
```

当前控制器不是面向 stock `gz_x500`，而是面向该分支的 `gz_iris`。对应
airframe 必须提供：

```text
MPC_MASS = 0.75 kg
MC_J_X   = 0.0025 kg·m²
MC_J_Y   = 0.0021 kg·m²
MC_J_Z   = 0.0043 kg·m²
CA_ROTORx_CT = 8.5 N
CA_ROTORx_KM = ±0.0157 m
```

为了让 ROS 侧力矩环以 PX4 实际状态更新率运行，
`src/modules/uxrce_dds_client/dds_topics.yaml` 至少需要包含：

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

修改 DDS topic 或 uORB 消息后必须重新生成并编译 PX4：

```bash
cd /path/to/DuctedFanUAV-Autopilot
DONT_RUN=1 make px4_sitl gz_iris
```

直接使用未经控制器专用修改的官方 `px4_msgs release/1.18`。PX4 提交
`912d45ca` 已从 `VehicleTorqueSetpoint` 删除原有 PCA/INDI 拆分字段，因此
PX4 与官方 ROS 消息都只包含 `timestamp`、`timestamp_sample` 和 `xyz`。
若 PX4 与 ROS 侧消息
定义不一致，可能出现 topic 无法匹配、字段错位或 Offboard 模式判断错误；
这种情况下不能通过仅重编译本包解决，必须先重新同步 `px4_msgs`。

本包的直接 ROS 依赖记录在 `package.xml`，包括 `rclcpp`、`px4_msgs`、
`geometry_msgs`、`nav_msgs`、`rcl_interfaces`、Eigen、Qt5、RViz2 和 ROS 2
launch。推荐把 `geometric_controller`、匹配的 `px4_msgs` 和可选的
`px4_ros_com` 放在同一个 colcon 工作空间中编译。

## 控制器选择

编号与 ROS 1 `UAV_Algorithm_Benchmark` 分支保持一致：

| ID | 名称 | ROS 侧输出 |
|---:|---|---|
| 0 | `legacy_geometric` | 完整推力和刚体力矩 |
| 1 | `main_geometric` | `main.m` geometric controller |
| 2 | `main_lee` | Lee SO(3) controller |
| 3 | `main_johnson` | Johnson logarithmic SO(3) controller |
| 4 | `main_sun_dfbc` | Sun differential-flatness controller |
| 5 | `main_sun_dfbc_indi` | Sun DFBC + INDI |
| 6 | `main_tal` | Tal differential flatness + INDI |
| 7 | `main_geometric_indi` | geometric + INDI |
| 8 | `px4_direct` | `TrajectorySetpoint`，PX4 内置控制器 |

实现采用 PX4 的 NED 世界坐标系和 FRD 机体系，不在控制环中隐式转换
ENU/FLU。仅 RViz 显示可以通过 `visualization.ned_to_enu` 转成 ENU。

控制器 0–7 发布：

```text
OffboardControlMode.thrust_and_torque = true
VehicleThrustSetpoint.xyz = [0, 0, -T_normalized]
VehicleTorqueSetpoint.xyz = [τx, τy, τz]_normalized
```

这符合 PX4 的 ROS 2 Offboard 模式选择方式。PX4 要求先连续发送
`OffboardControlMode`，且证明在线的频率必须高于约 2 Hz；本包默认 heartbeat
为 5 Hz。模式 8 的轨迹 setpoint 为 250 Hz；模式 0–7 使用 125 Hz
位置/参考外环；250 Hz `VehicleAngularVelocity` 驱动力矩内环，并使用最新的
`VehicleAttitude`（Iris SITL 实测约 200–250 Hz）。参见
[PX4 ROS 2 Offboard Control](https://docs.px4.io/main/en/ros2/offboard_control)。

从模式 8 切到模式 0–7 时，节点先发送 `thrust_and_torque` heartbeat，等
`VehicleControlMode` 确认 PX4 姿态/角速度环已关闭且 control allocation 已启用，
再复位控制器并启动 wrench。模式 0–7 之间切换不重复等待这个 PX4 模式握手，
避免在控制器切换期间保持旧力矩。

## 物理 wrench 到 PX4 归一化量

控制器内部始终使用 SI 单位。归一化不在 ROS 侧重建 PX4 effectiveness
matrix，而是像 `mavros_controllers` 一样使用显式参数：

```text
normalizedthrust_constant = mass / total_max_thrust
T_normalized = normalizedthrust_constant * (T / mass)
             + normalizedthrust_offset
τ_normalized = diag(normalizedtorque_constant_r,
                    normalizedtorque_constant_p,
                    normalizedtorque_constant_y) * τ
```

因此零 offset 时 `T_normalized = T / total_max_thrust`。三个 torque constant
分别把 ROS 侧物理 roll/pitch/yaw moment 转成 PX4 归一化 torque；它们应和
PX4 当前 airframe 的物理输出能力或标定结果一致。

归一化量在发布前限制到 PX4 消息的 `[-1, 1]` 范围；集体推力限制到
`[0, 1]`，FRD 中沿 `-z` 发布。发生限幅时节点会输出节流警告。

模式 0–7 的控制器全部在 ROS 侧计算，PX4 只接收归一化 thrust/torque
setpoint；模式 8 才把参考轨迹交给 PX4 内置控制器。因此 ROS INDI 不订阅
PX4 control allocator 的反馈。与 `main.m` 一致，控制器保存自己上一拍输出的
物理推力/力矩，并在下一拍经过二阶 Butterworth 低通后作为增量输入反馈：

```text
F_c = F_0 + mass * (a_c - a_0)
τ_c = τ_0 + J * (α_c - α_0)
```

这里 `F_0`、`τ_0` 是控制器自身上一拍请求值的滤波结果。平动加速度 `a_0`
不使用 `VehicleLocalPosition.ax/ay/az`，而是像 `mc_pos_control` 一样对
`VehicleLocalPosition.vx/vy/vz` 的 NED 速度依次执行 notch、速度一阶低通、
数值微分和微分一阶低通。当前 df-main 参数对应
`MPC_VEL_LP=0`、`MPC_VEL_NF_FRQ=0`、`MPC_VEL_NF_BW=5`、
`MPC_VELD_LP=5 Hz`。Tal/Geometric-INDI 随后仍执行 `main.m` 控制律中定义的
外环二阶 Butterworth 反馈滤波；状态获取滤波不能替代控制器自身的算法状态。
转动加速度 `α_0` 直接使用
`VehicleAngularVelocity.xyz_derivative`：该字段已在 PX4 中完成微分并由
`IMU_DGYRO_CUTOFF` 二阶低通，ROS 侧不再重复滤波，以免增加相位延迟。
INDI 状态完全保留在 ROS 控制器内部；`VehicleTorqueSetpoint` 只通过 `xyz`
发送最终归一化力矩。

## 车辆参数文件

只提供 `config/vehicles/iris.yaml`，来自
  `DuctedFanUAV-Autopilot/df-main` 的 Iris airframe，并与
  `UAV_Algorithm_Benchmark/main.m` 的 plant 参数一致：质量 `0.75 kg`、
  惯量 `[0.0025, 0.0021, 0.0043] kg·m²`、四个电机各 `8.5 N` 最大推力。

主参数文件先加载，车辆文件后加载，因此车辆文件会覆盖同名参数：

```bash
ros2 launch geometric_controller geometric_controller.launch.py \
  vehicle_param_file:=$HOME/ws_sensor_combined/src/geometric_controller/config/vehicles/iris.yaml
```

车辆参数文件必须与实际启动的 PX4/Gazebo 模型一致。默认启动目标和默认车辆
配置都是 `df-main` 的 `gz_iris`。

## 编译与测试

当前记录环境为 Ubuntu 24.04、ROS 2 Jazzy、PX4 v1.18.0 和官方
`px4_msgs release/1.18`。`px4_msgs` 必须和 PX4 固件消息版本匹配；本项目
不要求任何自定义 PX4 消息字段。

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to geometric_controller
source install/setup.bash
colcon test --packages-select geometric_controller
colcon test-result --verbose
```

控制器算法单测检查编号映射、各控制器的物理悬停 wrench，以及连续更新时输出
是否保持有限。

## 启动

一条命令启动 `gz_iris`、Micro XRCE-DDS Agent、控制节点、RViz 和控制面板：

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch geometric_controller sitl_geometric_controller.launch.py
```

常用启动参数：

```bash
ros2 launch geometric_controller sitl_geometric_controller.launch.py \
  px4_dir:=~/PX4-Autopilot \
  px4_model:=gz_iris \
  headless:=true \
  launch_rviz:=false
```

这里的 `~/PX4-Autopilot` 应指向
[`mengchaoheng/DuctedFanUAV-Autopilot` 的 `df-main` 分支](https://github.com/mengchaoheng/DuctedFanUAV-Autopilot/tree/df-main)。

如果 PX4 SITL 和 Agent 已经单独运行：

```bash
ros2 launch geometric_controller geometric_controller.launch.py
```

PX4 默认 arming check 可能要求 QGroundControl 或 RC/datalink 在线。节点等待有效
`VehicleLocalPosition`；控制器 0–7 还会等待有效的 `VehicleAttitude`
姿态和 `VehicleAngularVelocity` 角速度，之后才开始发布 setpoint 并请求
Offboard/Arm。PX4 的 `dds_topics.yaml` 必须以 250 Hz 启用
`/fmu/out/vehicle_attitude` 和 `/fmu/out/vehicle_angular_velocity`。

## 推荐验证顺序

1. 保持 `controller_type=8`，确认 `df-main` 的 `gz_iris` 能完成起飞过渡和参考轨迹跟踪。
2. 检查节点启动时打印的 PX4 thrust/torque normalization scale。
3. 将 `offboard.auto_start` 和 `offboard.arm_on_start` 暂时设为 `false`。
4. 在地面观察控制器悬停输出、轴向符号和饱和警告。
5. 先选择低速、定高轨迹，再逐步测试控制器 1、2、3、4。
6. 最后测试依赖微分与滤波质量的控制器 5、6、7。

运行中可从面板选择控制器，也可使用参数命令：

```bash
ros2 param set /trajectory_offboard_node controller_type 1
ros2 param set /trajectory_offboard_node trajectory_type 6
ros2 param set /trajectory_offboard_node omega_value 0.5
```

从模式 8 在线切换到 0–7 会立刻改变 PX4 Offboard 所需的控制层级。飞行中切换
必须视为高风险操作；应先在 SITL 中验证且保证当前参考和状态连续。

## 控制面板

C++/Qt 面板分为两个页签：

- `Reference`：轨迹类型、角速度、形状、航向和起飞过渡参数。
- `Controller`：控制器编号、125 Hz 外环/250 Hz 内环频率、
  `Kp/Kv/KR/KOmega`、质量、惯量、INDI 截止频率，以及 PX4 物理归一化参数。

轨迹类型沿用 ROS 1 编号：

| ID | 轨迹 |
|---:|---|
| 1 | `figure8_horizontal` |
| 2 | `figure8_vertical` |
| 3 | `helix_flip` |
| 4 | `helix_flip_y` |
| 5 | `flip_loop_sine` |
| 6 | `fast_circle` |

修改轨迹类型或形状会重新执行起点过渡；在线修改 `omega_value` 会补偿相位，
避免参考位置跳变。`trajectory_yaw_lock=true` 时使用恒定的
`trajectory_yaw_fixed`，且 yaw rate/yaw acceleration 均为零；设为 `false`
时，控制器使用轨迹生成器提供的 yaw、yaw rate 和 yaw acceleration。

各轨迹适用的 `Ax/Ay/Az` 默认均为 `3 m`，中心高度 `Hc` 默认均为 `6 m`。

## 主要参数

- `controller_type`：控制器 0–8。
- `ctrl_mode`：legacy controller 的 quaternion/geometric error 选择。
- `Kp_*`, `Kv_*`：位置和速度反馈增益。
- `KR_*`, `KOmega_*`：姿态和角速度增益。
- `max_acc`：平移反馈加速度限幅。
- `indi_filter_cutoff_hz`：INDI 二阶低通截止频率。
- `indi_velocity_lpf_hz`：对应 PX4 `MPC_VEL_LP`。
- `indi_velocity_notch_hz`、`indi_velocity_notch_bandwidth_hz`：对应
  `MPC_VEL_NF_FRQ`、`MPC_VEL_NF_BW`。
- `indi_velocity_derivative_lpf_hz`：对应 PX4 `MPC_VELD_LP`。
- `outer_loop_rate_hz`：模式 0–7 的位置/参考外环频率，默认 125 Hz。
- `inner_loop_rate_hz`：模式 0–7 的角速度反馈与 wrench 发布频率，默认 250 Hz。
- `normalizedthrust_constant`：`mass / total_max_thrust`。
- `normalizedthrust_offset`：归一化集体推力 offset。
- `normalizedtorque_constant_r/p/y`：物理力矩到 PX4 归一化力矩的三个比例。
- `offboard.heartbeat_rate_hz`：Offboard heartbeat，范围 3–10 Hz。
- `offboard.setpoint_rate_hz`：模式 8 的 PX4 轨迹 setpoint 频率，范围 50–250 Hz。
- `setpoint.level`：仅模式 8 使用，选择 position/velocity/acceleration。

控制律保持与 `main.m` 一致；默认增益则适配 PX4/Gazebo 中额外的 control
allocation 与电机动态。Iris 直 wrench 默认使用
`KR=[150,150,80]`、`KOmega=[50,50,3]`。yaw 增益针对 df-main Iris 的
ROS 2/DDS 直 wrench 链路单独整定：增强姿态误差恢复，同时降低延迟角速度
反馈的权重。直接使用理想刚体的 `[150,150,3]` 与 `[20,20,8]` 会在当前
SITL 激发约 43 Hz 的 yaw 极限环。

控制器 0 在 ROS 侧直接输出物理力矩时会计算并使用解析的期望角速度、角加速度，
而不是只保留原 MAVROS 版本交给 PX4 rate loop 的姿态误差部分。

模式 8 发送 position、velocity/acceleration feedforward，以及轨迹生成器给出的
yaw 和 yaw rate；yaw rate 不再有可关闭的第二开关。
模式 0–7 始终使用完整参考
`position/velocity/acceleration/jerk/snap/yaw/yaw_rate/yaw_accel`。

## 状态检查

```bash
ros2 topic list | grep /fmu
ros2 topic echo /fmu/in/vehicle_thrust_setpoint --once
ros2 topic echo /fmu/in/vehicle_torque_setpoint --once
ros2 topic echo /fmu/in/trajectory_setpoint --once
ros2 topic hz /fmu/out/vehicle_angular_velocity
ros2 topic hz /fmu/in/vehicle_torque_setpoint
ros2 topic list -t | grep vehicle_status
```

正常进入 Offboard 并解锁时，`VehicleStatus` 通常应显示 `nav_state: 14`、
`arming_state: 2`。默认水平 8 字起点是 NED `[0, 0, -6] m`。

## 参考实现

- [mengchaoheng/UAV_Algorithm_Benchmark](https://github.com/mengchaoheng/UAV_Algorithm_Benchmark)
- [mengchaoheng/mavros_controllers (`UAV_Algorithm_Benchmark` branch)](https://github.com/mengchaoheng/mavros_controllers/tree/UAV_Algorithm_Benchmark)
- [mengchaoheng/DuctedFanUAV-Autopilot (`df-main` branch)](https://github.com/mengchaoheng/DuctedFanUAV-Autopilot/tree/df-main)
- [PX4 ROS 2 Offboard Control](https://docs.px4.io/main/en/ros2/offboard_control)
- [SaxionMechatronics/px4_offboard_lowlevel](https://github.com/SaxionMechatronics/px4_offboard_lowlevel)
- [Jaeyoung-Lim/px4-offboard](https://github.com/Jaeyoung-Lim/px4-offboard)
- [kousheekc/nmpc_px4_ros2](https://github.com/kousheekc/nmpc_px4_ros2)

控制器数学来自 `main.m` 与 ROS 1 分支；ROS 2/PX4 transport、Offboard
heartbeat 和 thrust/torque 接口参考 PX4 官方示例与
`px4_offboard_lowlevel`。`nmpc_px4_ros2` 用于核对 ROS 2 节点和 PX4
接口组织，本次控制器编号中没有额外加入 NMPC。
