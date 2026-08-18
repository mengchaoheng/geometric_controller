# Raspberry Pi Zero 2 W 运行测试

当前发布包包含五个基准：`qpdunes`、`hpipm_ocp`、`qpoases`、`osqp`、`daqp`。默认飞行配置是 `N=20, dt=0.05`；控制刷新仍为
100 Hz，`dt` 只是 OCP 预测网格步长。所有脚本都不启用 fallback。

## 开发机打包（特殊流程）

日常本机开发继续使用 colcon 默认的 **isolated** 布局，不加
`--merge-install`：

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to geometric_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

只有在制作树莓派便携包时，才可以额外生成一个 merged 安装目录。必须使用单独的
`install_pi_merged/`，不能把已有的 `install/` 在两种布局之间来回切换：

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash

# 不指定 packages，编译并收集当前工作空间的全部项目。
colcon build --merge-install \
  --install-base install_pi_merged \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

OMMPC_INSTALL_DIR="$PWD/install_pi_merged" \
  src/geometric_controller/scripts/package_ros_zero2w.sh \
  "$PWD/ommpc_ros_zero2w_final"
```

这样，日常 `install/` 始终是 isolated；`install_pi_merged/` 仅服务于树莓派打包。
若不需要 merged 包，也可以直接用普通 isolated `install/` 打包，脚本默认读取它。

## 启动前

保持一个且仅一个 MicroXRCE-DDS Agent：

```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```

另一个终端：

```bash
cd ~/ommpc_ros_zero2w_final
source env.sh
scripts/pi_dds_check.sh
```

确认 `/fmu/out/vehicle_local_position_v1`、`/fmu/out/vehicle_attitude` 等消息存在。
没有有效位置时可以做压力测试，但不能做真实闭环 OMMPC 飞行。

## 纯求解器/本机系统测试

```bash
SAMPLES=300 WARMUP=50 scripts/pi_benchmark_matrix.sh
```

脚本只测试已保留的候选：`N20/.05`、`N20/.03`、`N25/.025`。它不启动 PX4
控制节点，不发布指令。

若要在同一问题上比较五条求解器路径，直接运行：

```bash
"$OMMPC_BENCHMARK_BIN" \
  --mode solver --solver all --horizon 20 --dt 0.05 \
  --samples 500 --warmup 50
```

这里的 `all` 只并列运行五个后端，不会在某个后端失败时替换它。

在 DDS/ROS2 负载下测试完整 OCP 构造和求解：

```bash
RESULT_DIR="$HOME/qpdunes_n20_dt005" \
  scripts/pi_100hz_system_stress.sh qpdunes 20 0.05 60 100
RESULT_DIR="$HOME/qpdunes_n20_dt003" \
  scripts/pi_100hz_system_stress.sh qpdunes 20 0.03 60 100
RESULT_DIR="$HOME/qpdunes_n25_dt0025" \
  scripts/pi_100hz_system_stress.sh qpdunes 25 0.025 60 100
```

最终确认可将上述 `60` 改为 `180`。要求 `failures=0`、`deadline_miss=0`、
`max_us<10000`，并同时检查 P99/P99.9/max。不要在正式计时期间持续运行交互式
`top`；它会增加调度抖动，结束后查看脚本生成的 `resource.csv`。

## DDS 在线（不上锁、不解锁）

当 PX4 已经有有效位置估计时：

```bash
scripts/pi_online_capture.sh qpdunes 20 0.05 60
```

脚本只启动 `trajectory_offboard_node`，不会自动解锁或起飞。检查生成目录中的
`ommpc_status.log`：应长期 `valid=true`、`fallback=false`、`status=0`，并观察
`resource.csv` 中节点和 Agent 的 CPU/RSS。若位置无效，状态日志为空是预期的，
这时只把 DDS/资源结果作为参考。

## 结束与结果回传

```bash
tar -czf ~/ommpc_pi_results.tar.gz -C ~/ommpc_ros_zero2w_20260818_v15 \
  qpdunes_n20_dt005 qpdunes_n20_dt003 qpdunes_n25_dt0025
```

从 Pi 复制回开发机后，直接把各目录的 `stress.log` 内容发回即可；重点字段为
`mean_us/stddev_us/p99_us/p99.9_us/max_us/deadline_miss/failures`。
