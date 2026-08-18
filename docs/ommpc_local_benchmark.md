# Lu OMMPC 本机测试

当前工程保留五个可选基准：默认的 `qpDUNES`、结构化 `HPIPM OCP`、经典
`qpOASES`、常用的 `OSQP` 和 `DAQP`。在线默认是 qpDUNES；其余路径可用于面板切换、
基准和排查，但当前树莓派飞行验证以 qpDUNES 为主，失败时不会互相 fallback。

## 默认和已验证范围

默认参数是：

```text
solver = qpdunes
N      = 20
dt     = 0.05 s
H      = N*dt = 1.0 s
```

树莓派 Zero 2 W 的 180 s、100 Hz 压力测试为 mean 1.824 ms、P99 3.240 ms、
P99.9 5.120 ms、max 8.012 ms、`deadline_miss=0`、`failures=0`。

结合 MATLAB 五轨迹扫描和 Pi 测试，当前只保留以下候选范围：

| 配置 | MATLAB 相对基准改善 | Pi 已测定位 |
|---|---:|---|
| `N=20, dt=0.05` | 0% | 最稳默认 |
| `N=20, dt=0.03` | 约 4.01% | 计算余量优先 |
| `N=25, dt=0.025` | 约 5.92% | 精度/余量折中 |
| `N=30, dt=0.025` | 约 7.40% | 边界对照，不作默认 |

工程上建议把在线范围限制在 `N=20..25`、`dt=0.025..0.05`。`N>=30` 只用于
边界实验；`N>=45`、`dt<=0.01` 和 `dt=0.005` 不再作为飞行候选。`dt` 是预测
网格间隔，控制器仍按 100 Hz（10 ms）刷新；总预测时域为 `H=N*dt`。

## 编译和回归测试

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-select geometric_controller \
  --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select geometric_controller --event-handlers console_direct+
colcon test-result --test-result-base build/geometric_controller --verbose
```

测试应覆盖 SO(3)、Lu 线性化、condensed/OCP 表示分离、五个保留路径的可行性、
在线 `N/dt/Q/R/输入界` 重建和无 fallback 长时间闭环。只要有失败，就不要打包上机。

## 本机性能测试

不需要 PX4 或仿真：

```bash
source install/setup.bash
ros2 run geometric_controller lu_ommpc_benchmark \
  --mode stress --solver qpdunes --horizon 20 --dt 0.05 \
  --rate-hz 100 --duration 10

ros2 run geometric_controller lu_ommpc_benchmark \
  --mode mpc --solver qpdunes --horizon 25 --dt 0.025 \
  --samples 500 --warmup 50

ros2 run geometric_controller lu_ommpc_benchmark \
  --mode solver --solver all --horizon 20 --dt 0.05 \
  --samples 1000 --warmup 100
```

`stress` 是固定 100 Hz、包含 OCP 构造和求解但不发布 PX4 指令的压力测试；
`deadline_miss` 表示单次完整周期超过 10 ms，不能只看平均值。

## 树莓派候选测试

打包后的 ROS bundle 在 Pi 上运行：

```bash
source env.sh
SAMPLES=300 WARMUP=50 scripts/pi_benchmark_matrix.sh
DURATION=30 scripts/pi_matlab_range_system_stress.sh
```

最终长测只需：

```bash
RESULT_DIR="$HOME/qpdunes_n20_dt005" scripts/pi_100hz_system_stress.sh qpdunes 20 0.05 180 100
RESULT_DIR="$HOME/qpdunes_n20_dt003" scripts/pi_100hz_system_stress.sh qpdunes 20 0.03 180 100
RESULT_DIR="$HOME/qpdunes_n25_dt0025" scripts/pi_100hz_system_stress.sh qpdunes 25 0.025 180 100
```

正式候选要求 `failures=0`、`deadline_miss=0`，并同时查看 P99、P99.9 和 max。
压力脚本不发布 PX4 指令；真实飞行前还需在 DDS 有效位置估计时运行 disarmed
节点，确认 `/controller/ommpc_status` 长期 `valid=true`、`fallback=false`。

## qpDUNES 路径检查要点

`OMMPCController` 根据 `solver->buildMode()` 请求 `QPBuildMode::kOcp`，因此在线
路径的 `QPProblem.H` 为空、`ocp.A/B/Q/P/R` 和每阶段输入上下界有效。适配器在
每次 `update()` 更新阶段数据，在改变 N 时重建 qpDUNES workspace，在普通帧保留
热启动；`solve()` 只返回 qpDUNES 的状态和阶段输入。任何非法问题、非有限结果、
求解失败或超过 10 ms 都不调用 fallback，也不发布该帧的新指令。
