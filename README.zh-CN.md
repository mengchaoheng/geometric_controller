# geometric_controller

[English](README.md) | **简体中文**

`geometric_controller` 是一个面向 PX4 Offboard 的 ROS 2 轨迹跟踪与全 wrench
控制器包。它生成解析轨迹，在 ROS 中计算期望推力和力矩，并将结果发送给 PX4
control allocator。项目提供 RViz 可视化和运行时调参面板。

控制器内部统一使用：

- 世界坐标系：NED；
- 机体坐标系：FRD；
- 物理量：SI 单位；
- RViz 显示：可配置为 ENU，不改变控制器内部坐标系。

## 系统结构

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

模式 1–5 令 `OffboardControlMode.thrust_and_torque=true`，绕过 PX4 的位置、
姿态和角速度控制器，但保留 PX4 control allocator 与执行器输出。模式 6 发布
`TrajectorySetpoint`，使用 PX4 内置串级控制器。

## 控制器

| ID | 名称 | 控制输出 |
|---:|---|---|
| 1 | `main_geometric` | ROS 计算物理推力和力矩 |
| 2 | `main_lee` | ROS 计算物理推力和力矩 |
| 3 | `main_johnson` | ROS 计算物理推力和力矩 |
| 4 | `main_sun_dfbc` | ROS 计算物理推力和力矩 |
| 5 | `main_geometric_indi` | ROS 计算 Geometric INDI 推力和力矩 |
| 6 | `px4_direct` | PX4 内置位置、姿态和角速度控制链 |

默认控制器为 `main_geometric_indi`，rate INDI 和 acceleration INDI 默认开启。

控制器名称与参考 MATLAB 实现的对应关系：

| ROS 控制器 | `main.m` 实现 |
|---|---|
| `main_geometric` | `controllerPDGeometric` |
| `main_lee` | Lee controller |
| `main_johnson` | Johnson controller |
| `main_sun_dfbc` | Sun DFBC controller |
| `main_geometric_indi` | `controllerGeometricINDI` |

`main_geometric` 和 `main_geometric_indi` 使用不同的姿态参考导数构造。关闭模式 5
中的某个 INDI 开关只会把对应增量律替换为直接力或力矩计算，仍保留模式 5 的
Sun 参考姿态、角速度和角加速度生成链。

## Geometric INDI

本节只描述当前源代码的数据接口和控制计算，并以公开的 `main.m` 为实现参考。
模式 5 的主要控制律为：

```text
a_c = a_r + Kp (p_r - p) + Kv (v_r - v)
F_c = F_0 - m (a_c - a_0)

alpha_c = KR Log(R^T R_c) + KOmega (omega_r - omega_0) + alpha_r
tau_c = tau_0 + J (alpha_c - alpha_0)
```

`R_c` 由期望推力方向和参考 yaw 构造。`omega_r` 与 `alpha_r` 使用 Sun 方法，
根据参考 jerk、snap、当前姿态/角速度和推力生成。

### Acceleration INDI 信号流

```text
VehicleLocalPosition.ax/ay/az
  -> ROS 二阶低通(indi_acceleration_cutoff_hz)
  -> a_0

PX4 final actuator setpoint x effectiveness matrix
  -> PX4 二阶低通(CA_FORCE_CUTOFF)
  -> AllocationValue.allocated_force
  -> 在 AllocationValue 到达时用对应姿态从 FRD 转换到 NED
  -> PX4 时间戳历史与插值(indi_force_delay_s)
  -> F_0

F_c = F_0 - mass * (a_c - a_0)
```

ROS 固定采用与 PX4 `MPC_INDI_A_SRC=1`、`MPC_INDI_F_SRC=0` 对应的数据源。
`allocated_force` 已在 PX4 中经过 `CA_FORCE_CUTOFF`，ROS 不重复滤波该字段。

`VehicleLocalPosition.timestamp_sample` 和 `AllocationValue.timestamp` 都使用 PX4 HRT
微秒时基。ROS 以加速度样本时刻减去 `indi_force_delay_s`，从分配力历史中选择或
插值得到 `F_0`；不使用 ROS 消息接收时间进行对齐。参数为零时不附加延迟。

### Rate INDI 信号流

```text
VehicleAngularVelocity.xyz
  -> omega_0

VehicleAngularVelocity.xyz_derivative
  -> alpha_0

PX4 final actuator setpoint x effectiveness matrix
  -> PX4 二阶低通(CA_TORQ_CUTOFF)
  -> AllocationValue.allocated_torque
  -> tau_0

tau_c = tau_0 + J * (alpha_c - alpha_0)
```

ROS 直接使用 PX4 发布的角速度、角加速度和已分配力矩，不在 ROS 中增加 rate
反馈滤波、延迟或时间戳配对。相关反馈带宽由 PX4 参数决定：

| 信号 | PX4 处理参数 | ROS 处理 |
|---|---|---|
| `omega_0` | `IMU_GYRO_CUTOFF` 和 notch filters | 直接使用 |
| `alpha_0` | `IMU_DGYRO_CUTOFF` | 直接使用 |
| `tau_0` | `CA_TORQ_CUTOFF` | 直接使用 |
| `F_0` | `CA_FORCE_CUTOFF` | 坐标转换、历史选择和插值 |
| `a_0` | 无 ROS 前置 INDI 低通 | `indi_acceleration_cutoff_hz` 二阶低通 |

项目当前 PX4 配置采用：

```text
IMU_GYRO_CUTOFF=125
IMU_DGYRO_CUTOFF=10
CA_TORQ_CUTOFF=8
CA_FORCE_CUTOFF=8
```

ROS 中对应的 acceleration INDI 参数为：

```yaml
indi_acceleration_cutoff_hz: 8.0
indi_force_delay_s: 0.0
```

### 启动与开关行为

`indi_rate_enabled` 和 `indi_acceleration_enabled` 可独立设置：

| Rate INDI | Acceleration INDI | 模式 5 行为 |
|---|---|---|
| 关 | 关 | Sun 参考链 + 直接期望力 + 直接期望力矩 |
| 开 | 关 | 直接期望力 + rate INDI |
| 关 | 开 | acceleration INDI + 直接期望力矩 |
| 开 | 开 | 完整 Geometric INDI |

控制器在第一份有限 `AllocationValue` 到达前使用直接计算产生初始 wrench；反馈
建立后接入已启用的增量律。运行过程不设置固定反馈年龄门限。

### PCA 力矩分解

启用 rate INDI 时，ROS 按 PX4 的定义发送总力矩及 INDI 反馈分量：

```text
tau_feedback = tau_0 - J * alpha_0
tau_c = J * alpha_c + tau_feedback

VehicleTorqueSetpoint.xyz = normalized(tau_c)
VehicleTorqueSetpoint.xyz_indi_feedback = normalized(tau_feedback)
VehicleTorqueSetpoint.xyz_indi_feedback_valid = true
```

两个分量使用相同的力矩归一化和最终裁剪比例。ROS 始终保留该分解，但不决定
是否启用 PCA。PX4 根据机型、分配矩阵和 `CA_METHOD` 选择是否使用它：df4 的
PCA 路径可消费优先级分量，iris 的 WLS 路径忽略该分量并使用总力矩。

## Wrench 归一化

ROS 控制器先计算物理 wrench，再转换为 PX4 allocator 接收的无量纲 setpoint：

```text
T_normalized = normalizedthrust_constant * T / mass
tau_normalized = diag(normalizedtorque_constant_r,
                      normalizedtorque_constant_p,
                      normalizedtorque_constant_y) * tau
```

参数与 PX4 分配器尺度的关系为：

```text
normalizedthrust_constant = mass / T_max = MPC_THR_HOVER / g
normalizedtorque_constant_* = AllocationValue.torque_setpoint_scale[*]
```

当前 iris 参数位于 [config/vehicles/iris.yaml](config/vehicles/iris.yaml)。切换机型时
必须同步质量、惯量和推力/力矩归一化常数。

## 控制频率与 DDS

ROS wrench 控制由反馈消息驱动：

- 新的 `VehicleAngularVelocity` 触发 rate 控制和 wrench 发布；
- 新的 `VehicleLocalPosition` 更新 acceleration INDI；
- 两次位置状态之间保持最近的期望推力向量；
- 姿态和 `AllocationValue` 以各自频率更新缓存。

`offboard.setpoint_rate_hz` 用于轨迹参考、预览和模式 6 的 PX4 setpoint 刷新，
不是 ROS wrench 控制频率。实际频率可从调参面板或状态话题读取。

PX4 DDS 配置应允许以下输出按源频率发送：

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

## 轨迹

`ReferenceTrajectory` 提供位置、速度、加速度、jerk、snap 及 yaw 导数。当前配置
包括：

- `figure8_horizontal`
- `figure8_vertical`
- `helix_flip`
- `helix_flip_y`
- `flip_loop_sine`
- `fast_circle`

轨迹类型由 `trajName` 设置，`omega_value` 设置相位速度。起飞流程先到达周期轨迹
起点，再进入周期运动。`trajectory_yaw_lock=true` 时使用
`trajectory_yaw_fixed`，同时令 yaw rate 和 yaw acceleration 为零；滚转和俯仰
参考导数仍由 Sun 映射生成。

## 安装

验证环境：Ubuntu 24.04、ROS 2 Jazzy、PX4 v1.18 系列和
`px4_msgs release/1.18`。

### PX4

```bash
cd ~
git clone --branch df-main --single-branch \
  https://github.com/mengchaoheng/DuctedFanUAV-Autopilot.git PX4-Autopilot
cd ~/PX4-Autopilot
make px4_sitl
```

### px4_msgs

本项目需要 `AllocationValue` 和扩展后的 `VehicleTorqueSetpoint`：

```bash
cd ~/ws_sensor_combined/src
git clone --branch release/1.18 --single-branch \
  https://github.com/PX4/px4_msgs.git
git -C px4_msgs checkout 598c7aad7b2386f9406ebd2a2f841619fddc3c78
git -C px4_msgs apply \
  ../geometric_controller/patches/px4_msgs-release-1.18.patch
```

消息字段顺序和类型必须与 PX4 `df-main` 一致。

### 编译

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to geometric_controller
source install/setup.bash
```

## 启动

本项目有两种启动方式。

### 方式一：一键启动

一条 launch 命令启动 PX4 SITL、Micro XRCE-DDS Agent、本项目控制节点、RViz
和调参面板：

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch geometric_controller sitl_geometric_controller.launch.py
```

### 方式二：三个终端分别启动

终端 1，启动 PX4 SITL 和 `gz_iris`：

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_iris
```

终端 2，启动 Micro XRCE-DDS Agent：

```bash
MicroXRCEAgent udp4 -p 8888
```

终端 3，启动本项目的控制节点、RViz 和调参面板：

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch geometric_controller geometric_controller.launch.py
```

## 配置与运行时操作

主要配置文件：

- [config/controller.yaml](config/controller.yaml)：控制器、INDI、轨迹、Offboard、
  PX4 话题和可视化参数；
- [config/vehicles/iris.yaml](config/vehicles/iris.yaml)：机型质量、惯量和归一化常数。

常用运行时参数：

```bash
ros2 param set /trajectory_offboard_node controller_type 5
ros2 param set /trajectory_offboard_node indi_rate_enabled true
ros2 param set /trajectory_offboard_node indi_acceleration_enabled true
ros2 param set /trajectory_offboard_node omega_value 0.5
```

调参面板可设置轨迹、控制器、INDI 开关、`Kp/Kv/KR/KOmega`、质量、惯量和
wrench 归一化参数，并显示状态反馈与控制回调的实测频率。

查看控制频率状态：

```bash
ros2 topic echo /controller/control_rate_status --once
```

## 参考

- [UAV_Algorithm_Benchmark / main.m](https://github.com/mengchaoheng/UAV_Algorithm_Benchmark)
- [DuctedFanUAV-Autopilot df-main](https://github.com/mengchaoheng/DuctedFanUAV-Autopilot/tree/df-main)
- [PX4 ROS 2 Offboard Control](https://docs.px4.io/main/en/ros2/offboard_control)
