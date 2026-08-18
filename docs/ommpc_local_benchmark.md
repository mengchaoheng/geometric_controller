# Lu OMMPC 本机测试

本文档用于当前开发机上的测试，不需要 PX4、Gazebo、Micro XRCE-DDS Agent 或飞行仿真。
测试分成三个不同层次，不能混为一个结果：

1. 单元与回归测试：检查建模、坐标方向、求解器共存、在线改参重建和安全回退；
2. 离线正确性验证：在每个固定的 `N/dt` 内，将候选求解器与高精度 qpOASES
   解同一个 QP 并比较；
3. raw MPC 性能：一个求解器一个独立进程，测量不带 ROS 在线安全复核的原始耗时。

不同 `N/dt` 定义不同 MPC 问题，不能比较它们的控制输出是否相同。正确性比较只在
同一个 `N/dt/Q/R/状态/参考/约束` 内进行。

## 1. 编译

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash

colcon build --packages-select geometric_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

性能结果只能使用 `Release` 构建。修改代码、依赖库或编译选项后必须重新编译。

## 2. 单元与回归测试

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon test --packages-select geometric_controller \
  --event-handlers console_direct+

colcon test-result \
  --test-result-base build/geometric_controller \
  --verbose
```

当前基线应为：

```text
125 tests, 0 errors, 0 failures
```

其中包括：

- condensed QP 与阶段累加建模等价；
- qpOASES、DAQP、HPIPM、PIQP、qpSWIFT、OSQP、OOQP、HPIPM OCP、
  qpDUNES 在线切换及参数重建；
- 修改 `N/dt/Q/R/输入界` 后控制量仍与可信解一致；
- 近似求解路径被在线安全门拒绝并回退；
- qpOASES BLAS replacement 与 OOQP/LAPACK 不再发生全局符号冲突。

只要这里有失败，就不要进行飞行或正式性能排名。

## 3. 四档正确性验证

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
source install/setup.bash

OMMPC_CPU=3 \
OMMPC_VALIDATE_SAMPLES=120 \
OMMPC_VALIDATE_WARMUP=20 \
OMMPC_OUTPUT_DIR=/tmp/lu_ommpc_validation \
src/geometric_controller/scripts/validate_ommpc_scales.sh
```

脚本分别验证：

| N | dt | 预测时域 |
|---:|---:|---:|
| 8 | 0.05 s | 0.40 s |
| 20 | 0.05 s | 1.00 s |
| 50 | 0.02 s | 1.00 s |
| 100 | 0.01 s | 1.00 s |

每个求解器都必须满足：

```text
validation=PASS
failures=0
```

验证硬门槛为：

- 相对目标函数误差不超过 `1e-7`；
- 第一个控制量无穷范数误差不超过 `1e-4`；
- 完整决策变量无穷范数误差不超过 `1e-3`；
- primal infeasibility 不超过 `1e-7`；
- 尺度化 projected-gradient KKT 不超过 `1e-4`。

日志写入：

```text
/tmp/lu_ommpc_validation/
```

任何 `FAIL`、非有限残差或求解失败都会使该求解器在对应规模下失去候选资格。

## 4. 候选求解器独立进程性能测试

正式计时不要使用 `--solver all` 作为最终排名。虽然链接符号冲突已经修复，但同一进程
依次运行仍会共享 allocator、缓存和热状态。以下脚本为每个求解器、每轮重复启动新进程。

### N=8, dt=0.05

```bash
OMMPC_SOLVERS="qpoases qpdunes hpipm_ocp" \
OMMPC_SAMPLES=2000 OMMPC_WARMUP=200 OMMPC_REPEATS=5 OMMPC_CPU=3 \
OMMPC_OUTPUT_DIR=/tmp/lu_ommpc_perf/N8_dt005 \
src/geometric_controller/scripts/run_ommpc_isolated.sh \
  --horizon 8 --dt 0.05
```

### N=20, dt=0.05

```bash
OMMPC_SOLVERS="qpoases qpdunes hpipm_ocp" \
OMMPC_SAMPLES=1500 OMMPC_WARMUP=150 OMMPC_REPEATS=5 OMMPC_CPU=3 \
OMMPC_OUTPUT_DIR=/tmp/lu_ommpc_perf/N20_dt005 \
src/geometric_controller/scripts/run_ommpc_isolated.sh \
  --horizon 20 --dt 0.05
```

### N=50, dt=0.02

```bash
OMMPC_SOLVERS="qpoases qpdunes hpipm_ocp" \
OMMPC_SAMPLES=1000 OMMPC_WARMUP=100 OMMPC_REPEATS=5 OMMPC_CPU=3 \
OMMPC_OUTPUT_DIR=/tmp/lu_ommpc_perf/N50_dt002 \
src/geometric_controller/scripts/run_ommpc_isolated.sh \
  --horizon 50 --dt 0.02
```

### N=100, dt=0.01

```bash
OMMPC_SOLVERS="qpoases qpdunes hpipm_ocp" \
OMMPC_SAMPLES=500 OMMPC_WARMUP=50 OMMPC_REPEATS=5 OMMPC_CPU=3 \
OMMPC_OUTPUT_DIR=/tmp/lu_ommpc_perf/N100_dt001 \
src/geometric_controller/scripts/run_ommpc_isolated.sh \
  --horizon 100 --dt 0.01
```

每轮产生一个 `.log` 和一个 `.csv`。重点字段：

- `mean_us`：平均完整 MPC 时间；
- `p99_us`：调度设计更应关注的尾延迟；
- `max_us`：最大抖动，容易受系统抢占影响；
- `failures`：必须为 0；
- `deadline_miss`：必须结合 `dt` 判断，正式候选应为 0。

结果目录：

```text
/tmp/lu_ommpc_perf/
```

## 5. 快速测试单个求解器

下面命令不启动仿真，直接生成确定性的状态和参考序列：

```bash
ros2 run geometric_controller lu_ommpc_benchmark \
  --mode mpc \
  --solver qpoases \
  --preset paper \
  --horizon 8 \
  --dt 0.05 \
  --samples 500 \
  --warmup 50
```

更换 `--solver` 可测试 `qpdunes` 或 `hpipm_ocp`。该命令适合快速检查，不代替
独立进程五轮测试。

## 6. 测试其他精确求解器

```bash
OMMPC_SOLVERS="osqp qpoases daqp piqp qpswift hpipm ooqp hpipm_ocp qpdunes" \
OMMPC_SAMPLES=500 OMMPC_WARMUP=50 OMMPC_REPEATS=1 OMMPC_CPU=3 \
OMMPC_OUTPUT_DIR=/tmp/lu_ommpc_perf/N8_all_exact \
src/geometric_controller/scripts/run_ommpc_isolated.sh \
  --horizon 8 --dt 0.05
```

ProxQP 在统一 `1e-9` 精度下耗时过大，TinyMPC 是近似 LTI 问题，CVXPYgen 是固定
生成结构；它们可以作为研究对照，但不要与精确在线候选混排。

## 7. ROS 在线控制安全状态

这一步需要实际运行 ROS 控制节点；是否连接 PX4 取决于要不要飞行。在线控制只运行
面板所选的求解器，不使用 qpOASES 或其他备用求解器。

```bash
ros2 topic echo /controller/ommpc_status
```

正常运行应长期保持：

```text
"fallback": false
"status": 0
"valid": true
"solver_deadline_miss": false
"callback_deadline_miss": false
```

`fallback` 固定为 `false`。若求解失败、结果非有限或回调超过 10 ms，该帧不发布新的
rates setpoint，日志和状态会直接暴露所选求解器的问题。

`ommpc.dt` 是 OCP 预测网格步长，不是控制周期。按 Lu `main.m`，在线重规划周期
固定为 `Tc=0.01 s`（100 Hz），节点从更快的 `VehicleLocalPosition` 流中使用最新
样本。改变 `N/dt` 会改变预测问题和时域 `N*dt`，但不会改变 100 Hz 控制频率。

本轮 Iris 短飞（qpOASES、无 fallback）已连续通过：`N=8,dt=0.05`、
`N=20,dt=0.05`、`N=40,dt=0.01`；随后在线改变四组 Q/R scale 和输入界后仍保持
100 Hz 稳定轨迹飞行。这里仅是短时冒烟测试，不能代替用户的长时间飞行筛选。

## 8. 当前本机环境与基线

本次结果来自 Mac 上运行的 Parallels ARM64 虚拟机：

| 项目 | 当前环境 |
|---|---|
| 虚拟化平台 | Parallels（宿主机为 Mac） |
| 客体架构 | `aarch64`，64 位 ARM |
| vCPU | 10 个，客体可见 CPU 0-9，1 thread/core |
| 内存 | 虚拟机分配 16 GiB，Ubuntu 实际显示约 15 GiB |
| Swap | 3.8 GiB，本轮仅约 10 MiB 已使用 |
| 操作系统 | Ubuntu 24.04.4 LTS |
| 内核 | 7.0.0-28-generic，PREEMPT_DYNAMIC |
| glibc | 2.39 |
| 构建 | `Release` |
| 计时隔离 | 每个求解器独立进程，固定在客体 vCPU 3 |

Parallels 没有向客体暴露准确 CPU 型号和主频，所以不能用该虚拟机结果推导每 GHz
性能。`taskset -c 3` 只固定客体 vCPU；宿主 macOS/Parallels 仍可能迁移或抢占对应
物理核心。因此 mean/p50 的大幅差距适合筛选，p99/max 的小幅差距不能作为最终结论。

修复 raw OCP 判定和 qpOASES/OOQP BLAS 符号冲突后，重新进行了三轮独立进程测试。
下表给出三轮 `mean_us` 和 `p99_us` 的中位数：

| N / dt | qpOASES mean / p99 | qpDUNES mean / p99 | HPIPM OCP mean / p99 |
|---|---:|---:|---:|
| 8 / 0.05 | 27.1 / 107.4 us | 45.1 / 172.9 us | 96.1 / 311.4 us |
| 20 / 0.05 | 174.3 / 464.4 us | 103.9 / 247.7 us | 238.5 / 664.9 us |
| 50 / 0.02 | 2.088 / 3.228 ms | 0.279 / 0.506 ms | 0.665 / 1.313 ms |
| 100 / 0.01 | 17.774 / 24.855 ms | 0.805 / 1.410 ms | 1.660 / 2.775 ms |

完整日志和 CSV 位于：

```text
/tmp/lu_ommpc_perf_final/
```

### 结果分析

- `N=8`：qpOASES mean 约为 qpDUNES 的 60%，是明确的小规模首选。
- `N=20`：发生交叉，qpDUNES 比 qpOASES 快约 40%，三条路径都远低于 50 ms。
- `N=50`：结构化 OCP 的规模优势明显；qpDUNES 约为 qpOASES 的 1/7.5。
- `N=100`：qpOASES 三轮所有计时样本都超过 10 ms，不能满足 100 Hz；qpDUNES
  mean 约 0.805 ms、p99 约 1.41 ms，HPIPM OCP mean 约 1.66 ms、p99 约 2.78 ms。
- 从 N=8 增长到 N=100，qpOASES mean 增长约 655 倍；qpDUNES 和 HPIPM OCP
  分别只增长约 18 倍和 17 倍。这说明 condensed dense 路径不适合一秒/100步规模，
  阶段 OCP 路径更有扩展潜力。
- HPIPM OCP 的一次 N=20 最大值达到约 17 ms，而同轮 p99 仍约 0.80 ms，属于明显
  的虚拟机调度离群点。虚拟机上的 `max_us` 不能直接解释为求解器算法抖动。

三条候选在四档内部正确性验证均为 `PASS`、`failures=0`。验证比较的是同一档内
同一个 QP，不比较不同 N/dt 的控制输出。

直接运行 standalone 二进制测得 N=100 的最大常驻内存约为：qpOASES 11.8 MiB、
qpDUNES 6.2 MiB、HPIPM OCP 6.9 MiB，且没有 major page fault。说明当前三个候选的
内存本身对 512 MiB Zero 2 W 不是首要风险；Zero 2 W 的 CPU 性能、缓存、系统抢占
和散热降频更可能决定结果。

### 能从虚拟机结果得出的结论

- 可以可靠淘汰本机 N=100 下的 qpOASES 100 Hz 路径；
- 可以确定 qpDUNES 和 HPIPM OCP 的规模增长趋势明显好于 condensed qpOASES；
- 可以把 qpOASES、qpDUNES、HPIPM OCP 作为 Zero 2 W 的三个重点候选。

### 不能从虚拟机结果直接得出的结论

- 不能把本机微秒数按核心数量或主频比例换算成 Zero 2 W 微秒数；
- 不能用虚拟机 p99/max 代替树莓派在温度、降频和 Linux 调度下的尾延迟；
- 不能用离线 QP 正确性代替实际 PX4 闭环轨迹稳定性；
- 10 vCPU 不会加速单次求解：当前每个 benchmark 固定一个 vCPU，求解路径基本是
  单线程。更多 vCPU 只减少其他任务争用的机会。

本机结果只用于筛选；树莓派结论必须在 Zero 2 W 上重新验证和计时。

树莓派步骤见 [ommpc_pi_zero2w.md](ommpc_pi_zero2w.md)。
