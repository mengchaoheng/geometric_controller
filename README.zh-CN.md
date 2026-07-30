# geometric_controller

[English](README.md) | **简体中文**

这是一个面向 PX4 Offboard 的 ROS 2 轨迹与低层控制器包。控制器内部使用
NED 世界坐标系、FRD 机体系和 SI 单位；RViz 可单独转换为 ENU。

## 控制器

| ID | 名称 | 输出 |
|---:|---|---|
| 0 | `legacy_geometric` | ROS 物理推力和力矩 |
| 1 | `main_geometric` | ROS 物理推力和力矩 |
| 2 | `main_lee` | ROS 物理推力和力矩 |
| 3 | `main_johnson` | ROS 物理推力和力矩 |
| 4 | `main_sun_dfbc` | ROS 物理推力和力矩 |
| 5 | `main_geometric_indi` | ROS Geometric INDI 物理推力和力矩 |
| 6 | `px4_direct` | `TrajectorySetpoint`，使用 PX4 内置串级控制器 |

模式 0–5 设置 `OffboardControlMode.thrust_and_torque=true`，发布
`VehicleThrustSetpoint` 和 `VehicleTorqueSetpoint`，绕过 PX4 的位置、姿态
和角速度控制器，仅使用 PX4 control allocator 与执行器输出。默认模式 6
用于先验证 PX4、模型和轨迹链路。

## 安装

验证环境为 Ubuntu 24.04、ROS 2 Jazzy、PX4 v1.18 系列和
`px4_msgs release/1.18`。

PX4 使用本项目维护的固件分支：

```bash
cd ~
git clone --branch df-main --single-branch \
  https://github.com/mengchaoheng/DuctedFanUAV-Autopilot.git PX4-Autopilot
cd ~/PX4-Autopilot
make px4_sitl
```

`make px4_sitl` 只编译 PX4 SITL。运行 `gz_iris` 由后面的 launch 或单独的
PX4 运行命令完成。

ROS 消息从官方分支开始，并应用本项目 patch：

```bash
cd ~/ws_sensor_combined/src
git clone --branch release/1.18 --single-branch \
  https://github.com/PX4/px4_msgs.git
git -C px4_msgs checkout 598c7aad7b2386f9406ebd2a2f841619fddc3c78
git -C px4_msgs apply \
  ../geometric_controller/patches/px4_msgs-release-1.18.patch
```

该 patch 同步三项 PX4 消息接口：

1. 新增 `VehicleAccelerationIndiFeedback.msg`。
2. 新增 `AllocationValue.msg`。
3. 给 `VehicleTorqueSetpoint.msg` 增加
   `xyz_indi_feedback[3]` 和 `xyz_indi_feedback_valid`。

字段顺序和类型必须与 `df-main` 中的 PX4 消息完全一致。修改消息后重新编译：

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs --cmake-clean-cache
source install/setup.bash
colcon build --packages-select geometric_controller --cmake-clean-cache
```

## PX4 DDS 频率

PX4 固件分支在不改变位置控制律的前提下，将
`mc_pos_control::set_vehicle_states()` 已经计算的速度差分滤波结果发布为
`VehicleAccelerationIndiFeedback`。DDS 配置为：

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

修改后执行 `make px4_sitl`。`unlimited` 取消 DDS 侧的降采样上限，但不会
把源消息升频。控制器调度为：

- 各订阅回调保存最新样本；其中角速度回调还负责按最高频率触发转动环；
- 100 Hz 定时器更新参考信号；平动 INDI 每 10 ms 更新一次并在其间保持
  最新推力向量；
- 新的角速度样本触发转动 INDI，`inner_loop_rate_hz=250` 限制其最高为
  250 Hz；每次计算读取当时最新的姿态、角加速度、位置、加速度和分配反馈。

因此仿真和真机的 PX4 原生频率可以不同，不需要让姿态与角速度消息逐帧
配对，也不会让约 800 Hz 的真机角速度直接驱动 800 Hz ROS 控制计算。真机
约 200 Hz 的姿态会在部分 250 Hz 转动环更新中复用一次，这是 latest-sample
调度的预期行为；若希望控制计算不高于姿态源频率，可把
`inner_loop_rate_hz` 设为 200。
`unlimited` 可减少 DDS 限速造成的等待；不能消除 PX4 内部滤波、调度和
物理执行器延迟。该配置同时把不参与本控制器的高频 DDS 话题降频，例如
`sensor_combined` 为 20 Hz、状态类为 5 Hz、GPS/全局位置为 10 Hz。
若实际 DDS 传输带宽不足，可把 `vehicle_angular_velocity` 和
`allocation_value` 限为 250 Hz；其他控制反馈保持 `unlimited`。

## Geometric INDI

[main.tex](https://github.com/mengchaoheng/geometric_controller/blob/main/main.tex)
和对应 PDF 是算法规范，
[main.m](https://github.com/mengchaoheng/UAV_Algorithm_Benchmark/blob/main/main.m)
用于核对离散实现。ROS 直接实现：

```text
a_c = Kp (p_r - p) + Kv (v_r - v) + a_r
(T b_z)_c = (T b_z)_0 - m (a_c - a_0)

α_c = Kθ Log(Rᵀ R_c) + Kω (ω_r - ω) + α_r
τ_c = τ_0 + J (α_c - α_0)
```

数据来源和调度如下：

- ROS 订阅保存最新 PX4 数据；角速度回调按上述频率触发转动环。
- `a_0` 使用最新 `VehicleAccelerationIndiFeedback`。它是 PX4
  `mc_pos_control` 对 NED 速度执行 notch、velocity LPF、差分和 derivative
  LPF 后的原生结果；ROS 不再重复滤波。
- `α_0` 直接使用 `VehicleAngularVelocity.xyz_derivative`；PX4 已在原生
  IMU 链路中完成差分和 `IMU_DGYRO_CUTOFF` 滤波。
- `(T b_z)_0` 和 `τ_0` 使用最新 `AllocationValue` 的已滤波物理力/力矩；
  力从 body FRD 旋转到 NED 并转换成论文的正 `T b_z` 约定。
- 平动式以 100 Hz 执行；转动式由新角速度样本触发，最高 250 Hz。两层均
  读取其他话题的最新缓存，不要求不同源消息具有相同时间戳。

这里不使用 IMU `VehicleAcceleration`。`mc_pos_control` 模块即使没有取得
位置控制权，也会随 `VehicleLocalPosition` 更新滤波状态并发布上述反馈。
PX4 的 `MC_INDI_RATE_EN` 不参与模式 5：ROS 发布最终 wrench，PX4 rate
controller 被绕过。

`indi_acceleration_enabled=true` 是默认完整算法。设为 `false` 时只启用
转动 INDI，推力改用直接几何外环，可用于分层诊断。

`τ_0` 与 `α_0` 不要求使用完全相同的低通截止频率。按照
`main.tex` 的状态估计说明，控制输入可以采用比状态导数更低的截止频率来
减小抖动。滤波频率用于带宽和噪声调节，不是模式 5 的启用条件。

### PCA 优先级字段

模式 5 同时填写总力矩和论文分解：

```text
VehicleTorqueSetpoint.xyz = τ_c
τ_H = τ_0 - J α_0
τ_L = J α_c
τ_c = τ_H + τ_L
VehicleTorqueSetpoint.xyz_indi_feedback = τ_H
```

两者使用同一个固定 `normalizedtorque_constant_r/p/y` 从 N·m 转成 PX4
归一化量。PX4 是否使用额外优先级字段由分配器参数和内部逻辑决定；未启用
PCA 时仍使用 `xyz` 中的总力矩。

## 归一化参数

```text
T_normalized = normalizedthrust_constant * T / mass
             + normalizedthrust_offset
τ_normalized = diag(normalizedtorque_constant_r,
                    normalizedtorque_constant_p,
                    normalizedtorque_constant_y) τ
```

这些比例必须匹配实际 airframe。控制器不从
`AllocationValue.torque_setpoint_scale` 实时改变比例；该值和固定比例在当前
SITL 中基本一致，实际硬件的 DDS 延迟仍需单独测量。

## 编译、测试和启动

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to geometric_controller
source install/setup.bash
colcon test --packages-select geometric_controller
colcon test-result --verbose
```

一条命令启动 `gz_iris`、Agent、控制节点、RViz 和调参面板：

```bash
ros2 launch geometric_controller sitl_geometric_controller.launch.py
```

如果 PX4 SITL 与 Micro XRCE-DDS Agent 已在其他终端运行：

```bash
ros2 launch geometric_controller geometric_controller.launch.py
```

## 调参面板

面板可修改轨迹、控制器、100/250 Hz 环频率、`Kp/Kv/KR/KOmega`、质量、
惯量和 wrench 归一化参数。数值框不设置人工上下限，参数
正确性由用户负责；后台刷新不会覆盖正在输入的内容，因此可以正常输入
`50` 等多位数值。

主要 INDI 参数：

- `indi_acceleration_enabled`：是否启用平动 INDI。
- `outer_loop_rate_hz`：默认 100 Hz。
- `inner_loop_rate_hz`：默认最高 250 Hz。
- `indi_Kp_*`、`indi_Kv_*`：平动 INDI 增益，独立于其他控制器的
  `Kp/Kv`。
- `indi_Ktheta_*`、`indi_Komega_*`：转动 INDI 增益。
- `yaw_torque_cutoff_hz`：直接 wrench 路径共用的 yaw 力矩输出低通，默认
  1 Hz，设为 0 可关闭。

平动反馈滤波直接使用 PX4 参数 `MPC_VEL_LP`、`MPC_VEL_NF_FRQ`、
`MPC_VEL_NF_BW` 和 `MPC_VELD_LP`；ROS 面板不再保留一套重复参数。

运行中可切换：

```bash
ros2 param set /trajectory_offboard_node controller_type 5
ros2 param set /trajectory_offboard_node indi_acceleration_enabled false
ros2 param set /trajectory_offboard_node indi_acceleration_enabled true
```

## 状态检查

```bash
ros2 topic hz /fmu/out/vehicle_local_position
ros2 topic hz /fmu/out/vehicle_acceleration_indi_feedback
ros2 topic hz /fmu/out/vehicle_attitude
ros2 topic hz /fmu/out/vehicle_angular_velocity
ros2 topic hz /fmu/out/allocation_value
ros2 topic hz /fmu/in/vehicle_torque_setpoint
```

## 参考

- [UAV_Algorithm_Benchmark](https://github.com/mengchaoheng/UAV_Algorithm_Benchmark)
- [DuctedFanUAV-Autopilot `df-main`](https://github.com/mengchaoheng/DuctedFanUAV-Autopilot/tree/df-main)
- [PX4 ROS 2 Offboard Control](https://docs.px4.io/main/en/ros2/offboard_control)
