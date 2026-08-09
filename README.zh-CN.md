# geometric_controller

[English](README.md) | **简体中文**

这是一个面向 PX4 Offboard 的 ROS 2 轨迹与低层控制器包。控制器内部使用
NED 世界坐标系、FRD 机体系和 SI 单位；RViz 可单独转换为 ENU。

## 控制器

| ID | 名称 | 输出 |
|---:|---|---|
| 1 | `main_geometric` | ROS 物理推力和力矩 |
| 2 | `main_lee` | ROS 物理推力和力矩 |
| 3 | `main_johnson` | ROS 物理推力和力矩 |
| 4 | `main_sun_dfbc` | ROS 物理推力和力矩 |
| 5 | `main_geometric_indi` | ROS Geometric INDI 物理推力和力矩 |
| 6 | `px4_direct` | `TrajectorySetpoint`，使用 PX4 内置串级控制器 |

模式 1–5 设置 `OffboardControlMode.thrust_and_torque=true`，发布
`VehicleThrustSetpoint` 和 `VehicleTorqueSetpoint`，绕过 PX4 的位置、姿态
和角速度控制器，仅使用 PX4 control allocator 与执行器输出。默认启动模式为 5，
且 rate/acceleration INDI 均默认开启；模式 6 可单独用于验证 PX4、模型和轨迹链路。

名称与 `main.m` 的对应关系是：模式 1 对应 `controllerName="geometric"` /
`controllerPDGeometric`，模式 2 对应 `lee`，模式 3 对应 `johnson`，模式 4
对应 `sun_dfbc`，模式 5 对应 `controllerName="geometric_indi"` /
`controllerGeometricINDI`，模式 6 对应 `px4_iris` 的 PX4 内置控制链概念。
原来的模式 0 `legacy_geometric` 是 ROS 早期兼容实现，`main.m` 没有对应算法，
且它的 `ctrl_mode` 只服务于该实现，因此二者已经删除，现有 ID 1–6 不重排。

特别注意，`main.m` 的 `controllerPDGeometric` 和
`controllerGeometricINDI` 是两条独立算法，并不是“后者去掉 INDI 就等于前者”。
前者通过闭环推力轴的一、二阶导数得到期望姿态导数；后者按 Sun
Eq. (14)–(24) 构造期望姿态、参考角速度和角加速度。ROS 保持这个区别：模式 5
关闭某个 INDI 开关时，仅绕过该增量律并直接反演期望力或力矩，仍留在模式 5
自己的 Sun 参考链，绝不调用或回退模式 1。

两种 ROS 实现的核心公式如下。它们接收同一份 `FlatReference`
（位置、速度、加速度、jerk、snap 与 yaw 导数），区别在姿态参考生成和是否使用
已分配反馈：

```text
main_geometric（main.m: controllerPDGeometric）
  ac = ad + Kp(pd-p) + Kv(vd-v)
  Fc = m(g e3-ac)
  对闭环 Fc 的一、二阶导数求 Rd、Omega_d、alpha_d
  alpha_c = KR Log(R'Rd) + KOmega(Omega_d^B-Omega) + alpha_d^B
  tau_c = Omega×J Omega + J alpha_c

main_geometric_indi（论文增量律）
  ac = ad + Kp(pd-p) + Kv(vd-v)
  acceleration INDI 开：Fc = F0 - m(ac-a0)
                    关：Fc = m(g e3-ac)
  用 Fc 和 yaw 求 Rd；用 Sun Eq. (18)-(24) 求 Omega_r、alpha_r
  alpha_c = KR Log(R'Rd) + KOmega(Omega_r-Omega0) + alpha_r
  rate INDI 开：tau_c = tau0 + J(alpha_c-alpha0)
            关：tau_c = Omega×J Omega + J alpha_c
```

ROS 保留 `main.tex/main.m` 的无界 `F0-m(ac-a0)` 和完整
`Log(R'Rd)`。两个“关”分支仍只绕过对应增量律，不调用
`main_geometric`。

### 激进 geometric INDI 的控制分配

此前垂直八字冲高和圆轨迹摇晃的根因是控制分配，不是 Eq.(55)、INDI 反馈滤波器
或 SO(3) Log。当期望力与力矩无法同时实现时，原分配及逐项裁剪没有按照
`main.tex` 所需的优先级保留姿态控制轴，roll/pitch 力矩权限被牺牲，导致
geometric INDI 表现得像发散，即使短时间内位置误差仍可能较小。

PX4 现在使用与参考实现一致的有界 QCAT WLS 分配：强优先 roll/pitch，其次 yaw，
饱和时先让出总推力。allocator 返回的实际有界 wrench 同时进入
`AllocationValue`，因此 geometric INDI 使用的反馈与真正执行命令的分配律一致。
控制器不再为该问题增加低推力 `mg` 下限、力残差裁剪、observer、替代姿态误差
或滤波器补丁。

### 为什么首次切到 PX4 内置控制会顿一下

这不是 INDI 公式造成的。模式 1–5 期间 PX4 收到
`thrust_and_torque=true`，其位置、姿态和角速度控制器都不工作；模式 6 切到
`position=true` 要经过 DDS、Commander 和 `VehicleControlMode` 才重新使能整条
PX4 串级链。PX4 `mc_pos_control` 在被关闭时会清空旧
`TrajectorySetpoint`；重新使能时记录 `_time_position_control_enabled`，比该时刻
更旧的 setpoint 会被当作无效并临时生成 failsafe setpoint。随后还要依次等下一份
`VehicleLocalPosition` 生成姿态目标、下一份姿态样本生成角速度目标、下一份 gyro
样本生成力矩，再由 allocator 接管输出。

ROS 参数切换会在同一个参考定时器回调中发送新的 `OffboardControlMode` 和
`TrajectorySetpoint`，但两个 DDS topic 与 PX4 内部几个模块之间不是原子切换。
这段窗口内 allocator 只能短暂保持上一拍 ROS wrench，或等待 PX4 第一拍有效
wrench，所以视觉上会“顿一下”；首次从完全旁路状态进入 PX4 串级链最明显。
后续连续使用模式 6 时各级已有有效状态和新 setpoint，现象自然减弱。这是一次
控制权交接的若干反馈周期，不是五次多项式过渡，也不是轨迹从头重启——仅切换
`controller_type` 不重置当前轨迹相位。

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

该 patch 增加 `AllocationValue.msg`，并给 `VehicleTorqueSetpoint.msg` 增加
PCA 优先级字段。rate INDI 始终发送该分解；只有 PX4 选择 PCA 分配方法时才会
使用它，WLS 和其他分配方法只使用总 wrench 并忽略分解字段。
`AllocationValue` 用于最终分配 wrench 反馈。

字段顺序和类型必须与 `df-main` 中的 PX4 消息完全一致。修改消息后重新编译：

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs --cmake-clean-cache
source install/setup.bash
colcon build --packages-select geometric_controller --cmake-clean-cache
```

## PX4 DDS 频率

模式 5 使用标准状态话题和 PX4 最终分配 wrench 反馈。取消
`vehicle_attitude`、`vehicle_local_position` 的 DDS 限速，并且不限制
`allocation_value` 的频率：

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

修改后执行 `make px4_sitl`。`unlimited` 取消 DDS 侧的降采样上限，但不会
把源消息升频。控制频率不是参数，而是由反馈事件决定：

- 每个新的 `VehicleAngularVelocity` 样本运行一次 rate INDI 并发布一次 wrench；
- 每个新的 `VehicleLocalPosition` 样本更新一次 acceleration INDI 推力向量，
  随后的角速度周期保持该向量；
- 姿态和 `AllocationValue` 按各自到达频率更新缓存，不要求与角速度逐帧同频。

因此 SITL 自动形成约 250 Hz rate / 125 Hz acceleration 控制，实测硬件自动
形成约 800 Hz rate / 100 Hz acceleration 控制；约 200 Hz 的硬件姿态会在
相邻 rate 更新间复用。调参面板只读显示实测控制与反馈频率，不再提供
`outer_loop_rate_hz` 或 `inner_loop_rate_hz`。
硬件上姿态约 200 Hz 而角速度约 800 Hz 是 PX4 数据链本身的结果：
`vehicle_angular_velocity` 跟随高频 gyro 更新，而 EKF2 的 `vehicle_attitude`
跟随积分后的 `vehicle_imu` 更新；用户记录中后两者分别约 801 Hz 和 201 Hz。
这不要求把 rate 环降到姿态频率，rate 环每拍使用最新姿态即可。
`offboard.setpoint_rate_hz=250` 只刷新轨迹参考、预览和 PX4 直接模式 setpoint，
不再定时触发 ROS wrench 控制计算，因此它不是内环或外环控制频率。
`unlimited` 可减少 DDS 限速造成的等待；不能消除 PX4 内部滤波、调度和
物理执行器延迟。该配置同时把不参与本控制器的高频 DDS 话题降频，例如
`sensor_combined` 为 20 Hz、状态类为 5 Hz、GPS/全局位置为 10 Hz。
若实际 DDS 传输带宽不足，可把 `vehicle_angular_velocity` 限为 250 Hz；
其他控制反馈保持 `unlimited`。

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

### 参考轨迹与 Sun 参考指令

需要区分两层来源：

- `ReferenceTrajectory` 中各轨迹的曲线定义及 `p/v/a/jerk/snap` 解析导数与
  `main.m` 的 `evalFigure8*`、`evalHelixFlip*`、`evalFastCircle` 同构；
- acceleration INDI 根据增量推力向量和参考航向构造 `R_c`，再由 jerk、snap、
  当前姿态/角速度及已实现推力 `T0` 生成 `ω_r、α_r`。这一层采用 Sun
  论文式 (14)–(24)，对应 `main.m` 的 `geometricINDIReferenceCommand()` 和
  `sunFlatnessReferenceRates()`；`main.m` 在这里是实现参照，不是算法来源。

当前默认运行参数并不等于 `main.m` 文件开头的全部数值：ROS 水平八字为
`Ax=Ay=3 m、Hc=6 m`，而当前 `main.m` 为 `2 m、3 m`；ROS YAML 还设置
`trajectory_yaw_lock=true` 固定零航向，而 `main.m` 的水平八字/圆轨迹默认让
航向跟随水平速度。ROS 起飞结束后额外使用 1 秒相位速度平滑，以连接静止起飞段。
因此当前是“相同轨迹公式 + Sun 参考指令 + ROS 飞行参数/起飞连接”，不是逐项
复刻 `main.m` 的一次仿真配置。若要逐项复现实验，应同时设置相同的尺寸、高度、
omega 模式和航向策略；不应为了声称一致而悄悄改变现有飞行配置。

`trajectory_yaw_lock=true` 时，轨迹层明确给出
`ψ=trajectory_yaw_fixed、ψ̇=ψ̈=0`。这只锁定航向参考；Sun 映射中的滚转/俯仰
参考角速度和角加速度仍由 `j_r/s_r`、当前姿态/角速度和 `T0` 计算，并不会被
一并清零。起飞五次多项式也提供解析 jerk 和 snap，之后才送入同一 Sun 映射。

最简结论：acceleration INDI 在 ROS 内复现 PX4 `mc_pos_control` 的 `a0`
二阶低通和 `F0` 延迟插值，但直接复用 PX4 allocator 已经低通的
`allocated_force`；rate INDI 在 ROS 内不复现任何滤波或延迟，直接复用 PX4
传回的 `xyz`、`xyz_derivative` 和已经低通的 `allocated_torque`。

### Acceleration INDI：PX4 与 ROS 对应

PX4 原生 `mc_pos_control`（`MPC_INDI_A_SRC=1`、`MPC_INDI_F_SRC=0`）：

```text
VehicleLocalPosition.ax/ay/az
  → 二阶 LPF(MPC_INDI_A_LP) → a0

最终 actuator setpoint × effectiveness matrix
  → 二阶 LPF(CA_FORCE_CUTOFF) → allocated_force
  → 历史插值[VLP.timestamp_sample-MPC_INDI_F_DLY] → F0

F0、a0 → acceleration INDI
```

本项目 ROS 模式 5：

```text
VehicleLocalPosition.ax/ay/az（DDS 原样传回）
  → ROS 二阶 LPF(indi_acceleration_cutoff_hz) → a0

AllocationValue.allocated_force（已经过 PX4 CA_FORCE_CUTOFF）
  → 在每个 AllocationValue 到达点按当时姿态做 FRD→NED
  → 转换为论文正向 T*b3 并存入 PX4 时间戳历史
  → ROS 历史插值[VLP.timestamp_sample-indi_force_delay_s] → F0

Fc = F0 - mass*(ac-a0)
```

对应参数只有下面这些：

| 功能 | PX4 原生 acceleration INDI | ROS 模式 5 | PX4 Iris / ROS 默认 |
|---|---|---|---:|
| 开关 | `MPC_INDI_ACC_EN` | `indi_acceleration_enabled` | PX4 默认关 / ROS 默认开 |
| 加速度源 | `MPC_INDI_A_SRC=1` | 固定使用 `VehicleLocalPosition.ax/ay/az` | `1` |
| `a0` 二阶低通 | `MPC_INDI_A_LP`，在 `mc_pos_control` 内执行 | `indi_acceleration_cutoff_hz`；`0` 为直通 | `8 Hz / 8 Hz` |
| 力反馈源 | `MPC_INDI_F_SRC=0` | 固定使用物理 `allocated_force` | `0` |
| `F0` 二阶低通 | `CA_FORCE_CUTOFF`，在 PX4 allocator 内执行 | 不重复滤波；直接接收已经滤波的字段 | `8 Hz` |
| `F0` 延迟插值 | `MPC_INDI_F_DLY`，在 `mc_pos_control` 内执行 | `indi_force_delay_s`；`0` 表示不附加延迟 | `0.01 s / 0 s` |
| 质量 | `MPC_MASS` | `mass` | `0.75 kg` |

`MPC_INDI_A_LP` 不会改变 DDS 中的 `VehicleLocalPosition`，因此 ROS 用
`indi_acceleration_cutoff_hz=8` 复现该二阶低通。`CA_FORCE_CUTOFF=8` 已经作用于
`AllocationValue.allocated_force`，ROS 不重复滤波。当前 ROS 默认
`indi_force_delay_s=0`，在加速度样本时刻选择 `F0`，不额外增加配置延迟。

这里使用 `VehicleLocalPosition.timestamp_sample` 不是按 ROS 到达时间猜测。
它与 `AllocationValue.timestamp` 都来自 PX4 同一个 HRT 微秒时钟，所以数值可
直接比较；前者表示 EKF 状态样本时刻，后者表示 allocator 发布反馈的时刻，
它们不是同一个物理事件。该选择直接复刻 PX4
`getDelayedAllocatedThrustAcceleration(vehicle_local_position.timestamp_sample)`：
用共同的 PX4 时基做历史选择/插值，而不使用 ROS 接收时刻。

固定 PX4 `IMU_GYRO_CUTOFF=125`、`IMU_DGYRO_CUTOFF=10`、
`CA_TORQ_CUTOFF=8`、`CA_FORCE_CUTOFF=8` 时，对应的 ROS 参数为：

```yaml
indi_acceleration_cutoff_hz: 8.0
indi_force_delay_s: 0.0
```

这些参数复刻 PX4 的加速度反馈滤波和力延迟；可实现性与控制轴优先级由 WLS
allocator 单独处理。

### Rate INDI：PX4 与 ROS 对应

PX4 原生 `mc_rate_control`：

```text
gyro → PX4 VehicleAngularVelocity 模块
  ├→ xyz             （IMU_GYRO_CUTOFF 及 notch） → ω0
  └→ xyz_derivative  （差分 + IMU_DGYRO_CUTOFF） → α0

最终 actuator setpoint × effectiveness matrix
  → 二阶 LPF(CA_TORQ_CUTOFF) → allocated_torque = τ0

τc = τ0 + J[MC_INDI_*_P*(ωsp-ω0)-α0]
```

本项目 ROS 模式 5 订阅的是同一批 PX4 输出：

```text
VehicleAngularVelocity.xyz            → 直接作为 ω0
VehicleAngularVelocity.xyz_derivative → 直接作为 α0
最新 AllocationValue.allocated_torque → 直接作为 τ0

αc = KR*Log(R'Rd) + KOmega*(ωr-ω0) + αr
τc = τ0 + J*(αc-α0)
```

ROS 对这三个 rate 反馈量不增加滤波、不增加延迟、不维护历史，也不做时间戳
配对。所谓“直接使用”是指 ROS 不再处理；消息在 PX4 发布前已经完成下表中的
处理：

| 功能 | PX4 原生 rate INDI | ROS 模式 5 | Iris 值 |
|---|---|---|---:|
| 开关 | `MC_INDI_RATE_EN` | `indi_rate_enabled` | PX4 默认关 / ROS 默认开 |
| `ω0` 预处理 | `IMU_GYRO_CUTOFF` 和 notch，在 PX4 传感器模块执行 | 直接使用 `xyz` | `125 Hz` |
| `α0` 低通 | `IMU_DGYRO_CUTOFF`，在 PX4 传感器模块执行 | 直接使用 `xyz_derivative` | `10 Hz` |
| `τ0` 二阶低通 | `CA_TORQ_CUTOFF`，在 PX4 allocator 执行 | 直接使用 `allocated_torque` | `8 Hz` |
| 惯量 | `MC_J_X/Y/Z` | `inertia_x/y/z` | `0.0025/0.0021/0.0043 kg·m²` |
| 角速度误差增益 | `MC_INDI_R/P/Y_P` | `KOmega_r/p/y`；ROS 还包含 `KR` 和 `αr` | PX4 `10/10/10`；ROS `20/20/8` |

`IMU_GYRO_CUTOFF`、`IMU_DGYRO_CUTOFF`、`CA_FORCE_CUTOFF` 和
`CA_TORQ_CUTOFF` 都是 PX4 参数，ROS 不读取也不覆盖，但它们直接决定 ROS
收到的反馈带宽。相反，`MPC_INDI_A_LP` 和 `MPC_INDI_F_DLY` 只在 PX4
`mc_pos_control` 内部生效；ROS 必须分别用自己的
`indi_acceleration_cutoff_hz` 和 `indi_force_delay_s` 复现。

`allocated_force/allocated_torque` 是 allocator 根据最终执行器指令和
effectiveness matrix 计算出的物理 wrench，不是力/力矩传感器测量。本项目只用
滤波后的 `allocated_*`，不用 `raw_allocated_*`。

这里不使用 IMU `VehicleAcceleration`，也不依赖 PX4
`AccelerationIndiStatus`。PX4 的 `MC_INDI_RATE_EN` 不参与模式 5：ROS
发布最终 wrench，PX4 rate controller 被绕过。

选择模式 5 时两个 INDI 环默认同时开启；面板也会在每次选择模式 5 时自动勾选
两项。开关仍可独立关闭用于诊断：

1. 两项都关闭：保留模式 5 的 Sun 参考链，直接反演期望力和期望力矩。
2. 只开 `indi_rate_enabled`：直接期望力 + rate INDI。
3. 两项都开：完整 rate + acceleration INDI。

两个开关在解锁前同时开启也已通过 SITL 起飞和周期轨迹验证。

两个开关也可以独立组合；未启用的环不等待对应的 `AllocationValue` 反馈。

到达周期轨迹起点后，相位速度在 1 秒内从零平滑增加到 `omega_value`，使零末速
的起飞五次多项式与水平八字等轨迹的非零初速度连续，避免轨迹释放时的姿态阶跃。

由于 PX4 只有收到 wrench 后才会产生 `AllocationValue`，模式 5 在第一份有限
分配力/力矩到达前，仍由模式 5 自己绕过两个增量律、直接反演并发布最初几拍，
反馈建立后接入 INDI。它从不切到 `main_geometric`；接入后保持 INDI。运行中
没有人为的反馈年龄阈值、rate 反馈时间戳配对、断流超时或自动回退。
`timestamp_sample` 只用于 F_SRC=0 的分配力/加速度历史选择与插值；这不是
“数据新于 100 ms”一类门限。

### PCA 优先级字段

启用 rate INDI 时，模式 5 保留与 PX4 相同的分解：

```text
τ_feedback = τ0 - J*α0
τ_c = J*α_c + τ_feedback
VehicleTorqueSetpoint.xyz = 归一化后的 τ_c
VehicleTorqueSetpoint.xyz_indi_feedback = 归一化后的 τ_feedback
VehicleTorqueSetpoint.xyz_indi_feedback_valid = true
```

两部分使用相同的力矩归一化和最终裁剪比例。ROS 不判断是否启用 PCA；只有 PX4
按机型和分配方法选择 PCA 时（例如已配置的 df4 路径）才消费该分解。当前主要
使用的 iris WLS 路径会忽略它，只使用总力矩。

## 归一化参数

```text
T_normalized = normalizedthrust_constant * T / mass
τ_normalized = diag(normalizedtorque_constant_r,
                    normalizedtorque_constant_p,
                    normalizedtorque_constant_y) τ
```

其中 `normalizedthrust_constant=h/g=m/T_max`。这些比例必须匹配实际
airframe。当前 SITL 数值来自 Iris effectiveness
matrix；更换机型后需要重新核对。

它们与 PX4 allocator 的量之间有直接对应关系：

```text
s_Fz = AllocationValue.force_setpoint_scale[2] = 1/T_max
normalizedthrust_constant = mass * s_Fz = mass/T_max = h/g

D_tau = diag(AllocationValue.torque_setpoint_scale)
normalizedtorque_constant_r/p/y = D_tau 的三个对角元素
```

因此实际发布的是

```text
VehicleThrustSetpoint.xyz = [0, 0, -s_Fz*T]
VehicleTorqueSetpoint.xyz = D_tau*τ
```

`force_setpoint_scale` 和 `torque_setpoint_scale` 是 PX4 control allocator 根据
effectiveness matrix 及其归一化规则生成并随 `AllocationValue` 发布的量，并非
独立的 PX4 参数。当前按照项目要求使用固定用户参数，而不是每拍使用消息中的
scale；Iris 的固定值应与上述消息字段一致。`MPC_THR_HOVER=h` 提供另一种推力
核对关系 `normalizedthrust_constant=MPC_THR_HOVER/g`。在 F_SRC=0 的 INDI
反馈公式中不使用这些 scale；它们只负责把最终物理 wrench 变成 PX4 allocator
接收的无量纲 setpoint。

## 编译和启动
两种启动方式：
1.
```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to geometric_controller
source install/setup.bash
```

一条命令启动 `gz_iris`、Agent、控制节点、RViz 和调参面板：

```bash
ros2 launch geometric_controller sitl_geometric_controller.launch.py
```

2. PX4 SITL 与 Micro XRCE-DDS Agent 分别在其他终端运行：
```bash
make px4_sitl gz_iris
```
```bash
MicroXRCEAgent udp4 -p 8888
```

最后运行本项目：
```bash
ros2 launch geometric_controller geometric_controller.launch.py
```

## 调参面板

面板可修改轨迹、控制器、`Kp/Kv/KR/KOmega`、质量、
惯量和 wrench 归一化参数。数值框不设置人工上下限，参数
正确性由用户负责；后台刷新不会覆盖正在输入的内容，因此可以正常输入
`50` 等多位数值。控制频率栏为只读状态，显示 VLP/acceleration INDI、
gyro/rate INDI、姿态和 AllocationValue 的实测频率。

主要 INDI 参数：

- `indi_rate_enabled`：是否启用转动 INDI。
- `indi_acceleration_enabled`：是否启用平动 INDI。
- `indi_acceleration_cutoff_hz`：EKF 加速度二阶低通，默认 `8 Hz`。
- `indi_force_delay_s`：F_SRC=0 分配力延迟，默认 `0 s`；按加速度采样时刻对齐。
- `Kp_*`、`Kv_*`、`KR_*`、`KOmega_*`：ROS 算法 1 和 5 共用参数接口，
  当前配置为 `Kp=[10,10,10]`、`Kv=[6,6,6]`、
  `KR=[150,150,30]`、`KOmega=[15,15,10]`。

`omega_value` 不是不同轨迹间可直接比较的难度指标。轨迹加速度、jerk、snap
分别按约 `omega²/omega³/omega⁴` 增长；当前 3 m 横向八字的 y 轴还是二倍频。
解析值显示该八字在 `omega=1.2` 的理想最大角速度约 `4.26 rad/s`，到
`omega=1.5` 已约 `8.32 rad/s`、最大倾角约 `70.3°`，所以视觉上必然剧烈。
相反，`fast_circle, omega=2.1` 实测位置误差均值 `0.141 m`、P95 `0.263 m`，
角速度 P95 `6.85 rad/s`，且归一化力矩最大仅 `0.098`。应按轨迹最大
加速度/jerk和归一化 wrench 余量判断，而不是给所有轨迹规定同一个 omega 阈值。

PX4 Iris 的 `MPC_INDI_A_LP=8` 和 `MPC_INDI_F_DLY=0.01` 只供 PX4 原生
acceleration INDI 使用；ROS 独立实现对应处理，不读取这两个 PX4 参数，
当前 ROS 配置的力延迟为零。

运行中可切换：

```bash
ros2 param set /trajectory_offboard_node controller_type 5
ros2 param set /trajectory_offboard_node indi_rate_enabled true
ros2 param set /trajectory_offboard_node indi_acceleration_enabled true
```

## 状态检查

```bash
ros2 topic echo /controller/control_rate_status --once
```

该状态使用节点实际收到并消费的回调计数，能同时看到 VLP、acceleration INDI、
gyro、rate/output、姿态和 allocation 频率，也避免手工处理 PX4 消息版本后缀。

## 参考

- [UAV_Algorithm_Benchmark](https://github.com/mengchaoheng/UAV_Algorithm_Benchmark)
- [DuctedFanUAV-Autopilot `df-main`](https://github.com/mengchaoheng/DuctedFanUAV-Autopilot/tree/df-main)
- [PX4 ROS 2 Offboard Control](https://docs.px4.io/main/en/ros2/offboard_control)
