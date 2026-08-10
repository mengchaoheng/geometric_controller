# geometric_controller

[English](README.md) | **简体中文**

`geometric_controller` 是一个面向 PX4 与 ROS 2 的飞行器轨迹跟踪控制软件包，
用于解析参考轨迹生成、几何控制、增量非线性动态逆（INDI）以及 PX4 Offboard
飞行。ROS 侧计算总推力与机体系控制力矩，PX4 负责控制分配和执行器驱动。

算法结构与参考轨迹实现主要参照开源项目
[`UAV_Algorithm_Benchmark`](https://github.com/mengchaoheng/UAV_Algorithm_Benchmark)。

## 控制器

项目提供以下控制器：

1. `main_geometric` [[1, 3]](#参考实现与文献)
2. `main_lee` [[2]](#参考实现与文献)
3. `main_johnson` [[3]](#参考实现与文献)
4. `main_sun_dfbc` [[4]](#参考实现与文献)
5. `main_geometric_indi` [[1, 3, 4, 5]](#参考实现与文献)
6. `px4_direct` [[6]](#参考实现与文献)

其中 `main_geometric_indi` 的具体实现以 [1] 为准；[4] 对应其参考角运动生成方法，
[5] 对应 INDI 与微分平坦性控制基础。
采用 SO(3) Log 姿态误差的控制器使用 [3] 的全角度误差定义；其数值计算采用
PX4 `AttitudeControl` [6] 的规范四元数主值实现，并参考 [7, 8] 处理小角度和
接近 180°的情形。`main_sun_dfbc` 保持 [4] 的倾斜优先误差结构，在推力轴反平行
奇异点采用相同的规范四元数回退。

模式 1–5 在 ROS 中完成轨迹跟踪、姿态与角速度控制，并向 PX4 发布总推力和
机体系控制力矩指令；PX4 保留控制分配与执行器输出。`px4_direct` 发布位置轨迹
指令并使用 PX4 内置串级控制器。

默认控制器为 `main_geometric_indi`，其外环加速度 INDI 与内环角加速度 INDI 默认开启。
`main_geometric` 对应开源项目中的 `geometric_pd / controllerGeometricPD`，
`main_geometric_indi` 对应 `controllerGeometricINDI`。两者采用不同的期望姿态
导数构造方法；关闭模式 5 的 INDI 开关仅旁路相应增量控制律，不改变其参考姿态
与参考角运动生成方法。

## Geometric INDI

Geometric INDI 由外环加速度 INDI 与内环角加速度 INDI 两个增量控制环组成：

```text
a_c = a_r + Kp(p_r - p) + Kv(v_r - v)
F_c = F_0 - m(a_c - a_0)

alpha_c = KR Log(R^T R_c) + KOmega(omega_r - omega) + alpha_r
tau_c = tau_0 + J(alpha_c - alpha_0)
```

期望姿态由期望推力方向与参考航向构造；参考角速度和角加速度由参考轨迹的
高阶导数通过 Sun 方法计算。

### 外环加速度 INDI

- `a_0`：来自 `VehicleLocalPosition` 的加速度，经 ROS 二阶低通滤波器处理；
- `F_0`：来自 PX4 `AllocationValue.allocated_force`；该信号已在 PX4 中经过
  `CA_FORCE_CUTOFF`，ROS 不重复滤波；
- `indi_acceleration_cutoff_hz`：ROS 加速度反馈滤波器截止频率；
- `indi_force_delay_s`：按 PX4 HRT 时间基从分配力历史中选择或插值 `F_0`。

`VehicleLocalPosition.timestamp_sample` 与 `AllocationValue.timestamp` 使用相同的
PX4 HRT 时基，因而延迟补偿不依赖 ROS 消息接收时间。当前默认参数为：

```yaml
indi_acceleration_cutoff_hz: 8.0
indi_force_delay_s: 0.0
```

### 内环角加速度 INDI

- `omega`：直接使用 `VehicleAngularVelocity.xyz`；
- `alpha_0`：直接使用 `VehicleAngularVelocity.xyz_derivative`；
- `tau_0`：直接使用 `AllocationValue.allocated_torque`。

内环角加速度 INDI 不在 ROS 中附加滤波或延迟。角速度、角加速度和已分配力矩的带宽
分别由 PX4 的 `IMU_GYRO_CUTOFF`、`IMU_DGYRO_CUTOFF` 和
`CA_TORQ_CUTOFF` 决定。

### INDI 开关与初始化

`indi_acceleration_enabled` 和 `indi_rate_enabled` 可独立设置。关闭某一开关时，
控制器以直接模型反演替代相应增量控制律。第一份有效 `AllocationValue` 建立前，
控制器使用直接计算生成初始推力与力矩；分配反馈建立后接入已启用的 INDI 环。

### PCA 力矩优先级

启用内环角加速度 INDI 时，ROS 同时发送总力矩和 INDI 反馈分量：

```text
tau_feedback = tau_0 - J alpha_0
tau_c = J alpha_c + tau_feedback
```

ROS 只保留并发送该分解，不决定 PCA 是否启用。PX4 根据机型、控制分配矩阵和
`CA_METHOD` 选择是否使用优先级分量：df4 的 PCA 配置可使用该分量，iris 的
WLS 配置只使用总力矩。

## 控制指令归一化

控制器输出的总推力和机体系力矩在发送至 PX4 前按机型参数归一化：

```text
T_n = normalizedthrust_constant * T / mass
tau_n = diag(normalizedtorque_constant_r,
             normalizedtorque_constant_p,
             normalizedtorque_constant_y) tau
```

其中：

```text
normalizedthrust_constant = mass / T_max = MPC_THR_HOVER / g
```

iris 的质量、惯量与归一化参数位于
[config/vehicles/iris.yaml](config/vehicles/iris.yaml)。更换机型时应同时更新这些
参数，并与 PX4 控制分配模型保持一致。

## 参考轨迹与控制频率

项目提供水平八字、垂直八字、螺旋翻转、正弦翻转和圆轨迹。参考轨迹包含位置、
速度、加速度、jerk、snap 以及航向导数，可通过调参面板或 ROS 参数修改。

控制计算由 PX4 状态反馈驱动：`VehicleAngularVelocity` 触发内环控制与指令发布，
`VehicleLocalPosition` 触发外环加速度 INDI 更新。因此控制频率由实际状态反馈频率决定，
不是独立调参量。可通过以下话题查看实测频率：

```bash
ros2 topic echo /controller/control_rate_status --once
```

PX4 DDS 配置应将 `vehicle_angular_velocity`、`vehicle_attitude`、
`vehicle_local_position` 和 `allocation_value` 按源频率发送。

## 安装

验证环境为 Ubuntu 24.04、ROS 2 Jazzy、PX4 v1.18 系列和
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

本项目使用扩展的 `AllocationValue` 与 `VehicleTorqueSetpoint` 消息：

```bash
cd ~/ws_sensor_combined/src
git clone --branch release/1.18 --single-branch \
  https://github.com/PX4/px4_msgs.git
git -C px4_msgs checkout 598c7aad7b2386f9406ebd2a2f841619fddc3c78
git -C px4_msgs apply \
  ../geometric_controller/patches/px4_msgs-release-1.18.patch
```

消息字段顺序和类型必须与 PX4 `df-main` 保持一致。

### 编译

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to geometric_controller
source install/setup.bash
```

## 启动

项目提供两种启动方式。

### 方式一：一键启动

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch geometric_controller sitl_geometric_controller.launch.py
```

该命令启动 PX4 SITL、Micro XRCE-DDS Agent、控制节点、RViz 和调参面板。

### 方式二：三个终端分别启动

终端 1，启动 PX4 SITL：

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_iris
```

终端 2，启动 Micro XRCE-DDS Agent：

```bash
MicroXRCEAgent udp4 -p 8888
```

终端 3，启动本项目：

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch geometric_controller geometric_controller.launch.py
```

## 配置

- [config/controller.yaml](config/controller.yaml)：控制器、INDI、轨迹、Offboard
  与可视化参数；
- [config/vehicles/iris.yaml](config/vehicles/iris.yaml)：机型动力学与控制指令归一化
  参数。

运行时可通过调参面板或 ROS 参数切换控制器、轨迹及 INDI 开关：

```bash
ros2 param set /trajectory_offboard_node controller_type 5
ros2 param set /trajectory_offboard_node indi_rate_enabled true
ros2 param set /trajectory_offboard_node indi_acceleration_enabled true
```

## 参考实现与文献

1. C. Meng, [`UAV_Algorithm_Benchmark`](https://github.com/mengchaoheng/UAV_Algorithm_Benchmark),
   开源飞行控制算法项目。
2. T. Lee, M. Leok, and N. H. McClamroch, “Geometric Tracking Control of a
   Quadrotor UAV on SE(3),” *49th IEEE Conference on Decision and Control*,
   pp. 5420–5425, 2010. [doi:10.1109/CDC.2010.5717652](https://doi.org/10.1109/CDC.2010.5717652)
3. J. C. Johnson and R. W. Beard, “Globally-Attractive Logarithmic Geometric
   Control of a Quadrotor for Aggressive Trajectory Tracking,” *IEEE Control
   Systems Letters*, 2022.
   [doi:10.1109/LCSYS.2022.3141066](https://doi.org/10.1109/LCSYS.2022.3141066)
4. S. Sun, A. Romero, P. Foehn, E. Kaufmann, and D. Scaramuzza, “A Comparative
   Study of Nonlinear MPC and Differential-Flatness-Based Control for Quadrotor
   Agile Flight,” *IEEE Transactions on Robotics*, vol. 38, pp. 3357–3373,
   2022. [doi:10.1109/TRO.2022.3177279](https://doi.org/10.1109/TRO.2022.3177279)
5. E. Tal and S. Karaman, “Accurate Tracking of Aggressive Quadrotor
   Trajectories Using Incremental Nonlinear Dynamic Inversion and Differential
   Flatness,” *IEEE Transactions on Control Systems Technology*, vol. 29,
   no. 3, pp. 1203–1218, 2021.
   [doi:10.1109/TCST.2020.3001117](https://doi.org/10.1109/TCST.2020.3001117)
6. L. Meier and The PX4 Contributors, [`PX4 Autopilot`](https://github.com/PX4/PX4-Autopilot),
   Zenodo. [doi:10.5281/zenodo.595432](https://doi.org/10.5281/zenodo.595432)
7. J. Solà, “Quaternion Kinematics for the Error-State Kalman Filter,” 2017.
   [doi:10.48550/arXiv.1711.02508](https://doi.org/10.48550/arXiv.1711.02508)
8. Z. Nurlanov, “Exploring SO(3) Logarithmic Map: Degeneracies and
   Derivatives,” 2021. [PDF](https://nurlanov.me/static/uploads/nurlanov2021so3log.pdf)

本项目使用的 PX4 固件实现为
[`DuctedFanUAV-Autopilot: df-main`](https://github.com/mengchaoheng/DuctedFanUAV-Autopilot/tree/df-main)。
