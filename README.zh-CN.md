# geometric_controller

[English](README.md) | **简体中文**

`geometric_controller` 是一个面向 PX4 与 ROS 2 的飞行器轨迹跟踪控制软件包，
用于解析参考轨迹生成、几何控制、增量非线性动态逆（INDI）以及 PX4 Offboard
飞行。ROS 侧计算总推力与机体系控制力矩，PX4 负责控制分配和执行器驱动。

算法结构与参考轨迹实现主要参照开源项目
[`UAV_Algorithm_Benchmark`](https://github.com/mengchaoheng/UAV_Algorithm_Benchmark)。

## 飞行演示

<table>
  <tr>
    <th>水平八字轨迹</th>
    <th>垂直八字轨迹</th>
  </tr>
  <tr>
    <td><img src="gif/case_quad_ros2_video_h8.gif" alt="水平八字轨迹" width="100%"></td>
    <td><img src="gif/case_quad_ros2_video_v8.gif" alt="垂直八字轨迹" width="100%"></td>
  </tr>
  <tr>
    <th>正弦翻转轨迹</th>
    <th>快速圆轨迹</th>
  </tr>
  <tr>
    <td><img src="gif/case_quad_ros2_video_flip_sine.gif" alt="正弦翻转轨迹" width="100%"></td>
    <td><img src="gif/case_quad_ros2_video_fast_circle.gif" alt="快速圆轨迹" width="100%"></td>
  </tr>
</table>

## 控制器

项目提供以下控制器：

1. `main_geometric` [[1, 3]](#参考实现与文献)
2. `main_lee` [[2]](#参考实现与文献)
3. `main_johnson` [[3]](#参考实现与文献)
4. `main_sun_dfbc` [[4]](#参考实现与文献)
5. `main_geometric_indi` [[1, 3, 4, 5]](#参考实现与文献)
6. `lu_ommpc` [[9, 10]](#参考实现与文献)
7. `px4_direct` [[6]](#参考实现与文献)

其中 `main_geometric_indi` 的具体实现以 [1] 为准；[4] 对应其参考角运动生成方法，
[5] 对应 INDI 与微分平坦性控制基础。
采用 SO(3) Log 姿态误差的控制器使用 [3] 的全角度误差定义；其数值计算采用
PX4 `AttitudeControl` [6] 的规范四元数主值实现，并参考 [7, 8] 处理小角度和
接近 180°的情形。`main_sun_dfbc` 保持 [4] 的倾斜优先误差结构，在推力轴反平行
奇异点采用相同的规范四元数回退。

模式 1–5 在 ROS 中完成轨迹跟踪、姿态与角速度控制，并向 PX4 发布总推力和
机体系控制力矩指令。模式 6 按 Lu 等人的流形 MPC 直接优化总推力加速度与
机体系角速度，并发布 `VehicleRatesSetpoint`。PX4 在两种情况下都保留执行器
分配；`px4_direct`（模式 7）发布位置轨迹指令并使用 PX4 内置串级控制器。

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

控制计算由 PX4 状态反馈驱动：传统几何/INDI 控制器由
`VehicleAngularVelocity` 触发内环控制与指令发布；外环加速度 INDI 由
`VehicleLocalPosition` 更新。Lu OMMPC 从位置反馈流中使用最新样本，并按原算法
固定 100 Hz 重规划和发布；OCP 网格 `dt` 与这个控制周期相互独立。可通过以下话题
查看实测频率：

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
colcon build --packages-up-to geometric_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
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

### Lu OMMPC 与求解器测试

`lu_ommpc` 已直接嵌入本包，继续使用相同的 Iris、参考轨迹、起飞过渡、RViz 和
调参面板。面板选择 `lu_ommpc`，或运行下列命令即可切换到模式 6：

```bash
ros2 param set /trajectory_offboard_node controller_type 6
```

切回 PX4 直连使用模式 7。切换控制器时 OMMPC 会清除热启动解，下一次位置/速度
反馈到达时重新求解。在线求解耗时可直接查看：

```bash
ros2 topic echo /controller/ommpc_status
```

OMMPC 使用最新的 PX4 `VehicleLocalPosition` 和 `VehicleAttitude`，不要求
`VehicleAngularVelocity`。按照 `main.m`，重规划/发布周期固定为 `Tc=0.01 s`
（100 Hz）；若位置反馈为 125 Hz，节点用绝对相位累计器选取最新样本并保持平均
100 Hz，不重复发布旧解。面板中的 `OCP grid dt` 只是预测模型网格间隔，预测总
时域为 `N*dt`，不改变控制频率。例如 `N=8, dt=0.05` 预测 0.4 s，但仍以
100 Hz 重规划。measured control rates 同时显示 VLP、OMMPC 和姿态实测频率。

飞行面板只列出能够在参数改变后重建当前精确 LTV QP/OCP 的路径：qpOASES、DAQP、
HPIPM dense、PIQP、qpSWIFT、OSQP、OOQP、HPIPM OCP 和 qpDUNES。在线修改
`N、OCP grid dt、Q/R scale、输入界` 都会重建控制器；每个控制周期再用当前问题
更新所选求解器。在线飞行不使用任何备用求解器：失败、非有限结果或整个回调超过
10 ms 时不发布该帧的新指令，并在 `/controller/ommpc_status` 报告。状态中的
`fallback` 恒为 `false`，因此飞行效果确实来自面板中选择的求解器。

TinyMPC 是不同的近似 LTI 问题，CVXPYgen 是固定生成结构；ProxQP 当前延迟较大。
这三类仍可离线 benchmark，但不会伪装成可在线任意改参的飞行求解器。求解器的
迭代上限、容差、rho 和 warm-start 属于离线/高级求解器实验，本轮不在飞行面板暴露。

OMMPC 保留两种等价表示：

- condensed 标准 QP：`min 0.5*x'*H*x + g'*x`，`lower <= A*x <= upper`；
- 未压缩的阶段 OCP：`x[k+1]=A[k]x[k]+B[k]u[k]`，带每阶段输入上下界。

纯性能 benchmark 和在线飞行都按后端构建所需表示。在线状态分别报告
`solver_us`、QP/OCP 构造时间和整个回调时间；本轮求解器排名只使用 `solver_us`。
当前实现路径如下：

| 路径 | 建模 | 本轮使用的专用功能 |
| --- | --- | --- |
| qpOASES | 精确 condensed QP | 原生变量上下界、热启动 |
| OSQP | 精确 condensed QP | 固定稀疏结构、只更新矩阵数值和向量 |
| ProxQP | 精确 condensed QP | benchmark-only；高精度配置延迟过大 |
| DAQP | 精确 condensed QP | 原生 box QP、热启动 |
| PIQP | 精确 condensed QP | 原生变量上下界、持久 solver/update |
| HPIPM dense | 精确 condensed QP | BLASFEO dense QP、原生变量上下界 |
| qpSWIFT / OOQP | 精确 condensed QP | 库接口的一次性 dense 求解路径，作为对照 |
| `hpipm_ocp` | 精确阶段 OCP | HPIPM OCP IPM、逐阶段 A/B/Q/R 与输入界 |
| `qpdunes` | 精确阶段 OCP | qpDUNES stage intervals、输入简单界 |
| `cvxpygen_osqp` | 固定生成的阶段 OCP | 仅固定 `N=8`、论文权重，benchmark-only |
| `tinympc_lti` | **近似** LTI OCP | 每帧用第一个阶段 A/B 重新建立 Riccati cache |
| `tinympc_lti_cached` | **近似**固定 LTI OCP | 首帧建立 cache，之后只更新 x0 和输入界 |

TinyMPC 当前 C++ 主线接口使用一组固定 A/B，而 Lu OMMPC 是随参考轨迹变化的 LTV
问题。因此两个 TinyMPC 数字只衡量可部署的 LTI 近似路径，不能和其余“同一个精确
QP”的解质量直接排名。这里选择可复现、可离线生成的 CVXPYgen；没有依赖 CVXGEN
在线代码生成服务。

若系统未安装对应开发包，可在源码目录执行无 sudo 的本地安装脚本。脚本固定版本
获取并构建全部后端，生成 CVXPYgen C 代码，然后重新构建工作区：

```bash
./scripts/install_ommpc_solvers.sh
cd ~/ws_sensor_combined
colcon build --packages-select geometric_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

不启动 PX4 即可先做重复求解测试。`solver` 只计求解器的 update/solve；`mpc`
还包含流形误差、线性化、QP/OCP 构建和解恢复：

```bash
ros2 run geometric_controller lu_ommpc_benchmark \
  --mode solver --solver all --preset paper --samples 5000 --warmup 500

ros2 run geometric_controller lu_ommpc_benchmark \
  --mode mpc --solver all --preset paper --samples 5000 --warmup 500 \
  --csv /tmp/lu_ommpc_mpc.csv
```

输出包含 mean、p50、p90、p95、p99、p99.9、最大耗时、超一个模型步长的次数、
失败数、原始残差与 KKT 残差。`--solver all` 会在完全相同的状态/参考样本上依次
测试全部路径。新增求解器只需实现
[solver.hpp](include/lu_ommpc/solver.hpp) 的 `QPSolver` 接口并注册，无需修改
OMMPC 建模或 ROS/PX4 节点。

### 飞行数据记录与回放

先正常启动 Iris SITL 和轨迹。设置记录路径后切到模式 6，每次有效位置反馈都会保存
当时的状态、完整参考时域、condensed QP、阶段 OCP 和热启动量：

```bash
ros2 param set /trajectory_offboard_node ommpc.dataset_path \
  /tmp/lu_ommpc_iris_figure8.bin
ros2 param set /trajectory_offboard_node controller_type 6
```

飞完后停止仿真，再对完全相同的飞行输入逐个重建 MPC 并计时：

```bash
ros2 run geometric_controller lu_ommpc_benchmark \
  --mode replay --solver all --preset paper \
  --samples 5000 --warmup 500 \
  --dataset-in /tmp/lu_ommpc_iris_figure8.bin \
  --csv /tmp/lu_ommpc_iris_replay.csv
```

也可用 `--dataset-in old.bin --dataset-out subset.bin --samples 1000` 截取一个可重复
使用的小数据集。二进制数据集来自受控输入，读取器不应处理不可信文件。

### 当前本机结果

QPBuilder 已改成逐阶段累加，只计算 Mu 当前非零的前缀块，不再生成 Hx、完整 Mu、
Qbar 和 Rbar；矩阵工作区与参考时域也会在 N 不变时复用。qpOASES 固定使用
`HST_POSDEF`，HPIPM 构建关闭运行时检查，qpDUNES 持久化工作区并只更新阶段数据。

2026-08-17 在 Mac 上的 Parallels ARM64 虚拟机（10 vCPU、分配16 GiB内存、
Ubuntu 24.04.4）Release 构建中，使用“一求解器一进程、客体CPU 3、三轮重复”的
完整 raw MPC 基准。下表取三轮 mean 的中位数；在线飞行和离线测试均不使用
fallback，因此只用于树莓派前筛选：

| N / dt | qpOASES | qpDUNES OCP | HPIPM OCP | 结论 |
| --- | ---: | ---: | ---: | --- |
| 8 / 0.05 | 27.1 us | 45.1 us | 96.1 us | qpOASES 最快 |
| 20 / 0.05 | 174.3 us | 103.9 us | 238.5 us | qpDUNES 最快 |
| 50 / 0.02 | 2.088 ms | 0.279 ms | 0.665 ms | qpDUNES 最快 |
| 100 / 0.01 | 17.774 ms | 0.805 ms | 1.660 ms | qpOASES 超过 10 ms |

三条路径在每一档内部都以同状态、同参考、同 `N/dt/Q/R/约束` 对高精度 qpOASES
参考逐帧验证，全部 `PASS` 且求解失败为 0。不同档位定义不同 MPC 问题，绝不比较
它们的控制输出。完整单元/回归测试为 125 项、0 失败。另已修复 qpOASES 自带
BLAS replacement 与 OOQP/LAPACK 的全局 `dgemm_` 符号冲突；这是多求解器同进程
共存时会真实导致崩溃的问题。

### 一秒预测时域

新增两个纯 ROS 2 基准预设：

- `one_second`：`N=20, dt=0.05 s`，粗离散的一秒预测；
- `one_second_100hz`：`N=100, dt=0.01 s`，保持100 Hz模型离散的一秒预测。

最新统一生产容差为 `1e-9`。在该设置下 qpDUNES 的 N=100 路径已通过逐帧精度
验证且无求解失败；较宽松的旧容差曾触发它的数值失败，所以不能复用旧结果或为了
速度单独放宽精度。

```bash
ros2 run geometric_controller lu_ommpc_benchmark \
  --mode mpc --solver qpdunes --preset one_second \
  --samples 3000 --warmup 300

ros2 run geometric_controller lu_ommpc_benchmark \
  --mode mpc --solver hpipm_ocp --preset one_second_100hz \
  --samples 500 --warmup 50
```

这张表不能代替 Raspberry Pi Zero 2 W 原机测试。移到树莓派后应使用相同的
Release 构建、参数、样本数，并固定 CPU 核心后至少重复 5 轮；同时记录温度、
CPU 频率和降频状态。当前本机候选为 qpOASES、qpDUNES 和 HPIPM OCP；最终排序
必须以 Zero 2 W 原机的零求解失败和计时结果为前提。完整无仿真步骤与 ARM64 打包说明见
[docs/ommpc_pi_zero2w.md](docs/ommpc_pi_zero2w.md)。TinyMPC cached 仍只作为资源
受限近似实验，不能仅凭耗时替换精确 OMMPC。

本机的完整编译、125 项回归测试、四档独立进程计时、虚拟机环境、
内存占用、结果分析和原始结果目录见
[docs/ommpc_local_benchmark.md](docs/ommpc_local_benchmark.md)。

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
9. J. Lu, Y. Xie, S. Zhang, and J. Wang, “On-Manifold Model Predictive Control
   for Trajectory Tracking on Robotic Systems,” 2023.
   [arXiv:2106.15233](https://arxiv.org/abs/2106.15233)
10. O. Wu, [`ommpc_controller`](https://github.com/OliverWu515/ommpc_controller),
    ROS/PX4 OMMPC reference implementation.

本项目使用的 PX4 固件实现为
[`DuctedFanUAV-Autopilot: df-main`](https://github.com/mengchaoheng/DuctedFanUAV-Autopilot/tree/df-main)。
