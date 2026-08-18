# Lu OMMPC solver test on Raspberry Pi Zero 2 W

Development-machine testing is documented separately in
[ommpc_local_benchmark.md](ommpc_local_benchmark.md).

This is a solver/controller-computation benchmark only. It does not start ROS,
PX4, Gazebo, a trajectory planner, or a publisher. `dt` is the prediction-model
discretization interval; the program does not emulate a wall-clock feedback rate.

## Compatibility

The supplied binary is built for 64-bit ARM (`aarch64`) on Ubuntu 24.04 with
glibc 2.39. Use a 64-bit Ubuntu 24.04 image on the Zero 2 W. A 32-bit `armhf`
Raspberry Pi OS, Ubuntu 22.04, or another older glibc cannot run this bundle.
ROS is not required for the standalone benchmark.

On the Pi, verify the target before copying results:

```bash
uname -m
. /etc/os-release && echo "$PRETTY_NAME"
ldd --version | head -1
```

Expected: `aarch64`, Ubuntu 24.04, and glibc 2.39 or ABI-compatible newer.

## Install the runtime

```bash
sudo apt update
sudo apt install libblas3 liblapack3 libgfortran5 util-linux
mkdir -p ~/ommpc_test
tar -xzf ~/ommpc_zero2w_aarch64_ubuntu24_safe_v4_20260817.tar.gz -C ~/ommpc_test
cd ~/ommpc_test/ommpc_zero2w_aarch64_ubuntu24_safe_v4_20260817
source env.sh
ldd "$OMMPC_BENCHMARK_BIN"
```

Stop if `ldd` prints `not found`.

## Solver feasibility first

Each candidate is launched in a fresh process and no fallback solver is used.
For the current phase, rank only the solver's `update_us + solve_us`; keep problem
construction separate. Any nonzero `failures`, nonzero solver status, process
crash, or non-finite timing rejects that solver/scale run. Closed-loop suitability
is decided by the preceding Iris flight test, not by the Pi benchmark.

## Stable timing measurements

Power the Pi adequately, close unrelated workloads, and watch temperature and
frequency. If the kernel exposes CPU-frequency controls, use the performance
governor for reproducibility:

```bash
for policy in /sys/devices/system/cpu/cpufreq/policy*; do
  echo performance | sudo tee "$policy/scaling_governor"
done
vcgencmd measure_temp 2>/dev/null || true
```

Run each scale separately. The script pins all runs to CPU 3, starts a new process
for every solver/repetition, discards warm-up samples, and writes one CSV and log
per run.

```bash
export OMMPC_SAMPLES=2000 OMMPC_WARMUP=200 OMMPC_REPEATS=5 OMMPC_CPU=3
export OMMPC_SOLVERS="qpoases qpdunes hpipm_ocp"

OMMPC_OUTPUT_DIR="$HOME/ommpc_results/N8_dt005" \
  ./scripts/run_ommpc_isolated.sh --horizon 8 --dt 0.05

OMMPC_OUTPUT_DIR="$HOME/ommpc_results/N20_dt005" \
  ./scripts/run_ommpc_isolated.sh --horizon 20 --dt 0.05

OMMPC_OUTPUT_DIR="$HOME/ommpc_results/N50_dt002" \
  ./scripts/run_ommpc_isolated.sh --horizon 50 --dt 0.02

OMMPC_SAMPLES=500 OMMPC_WARMUP=50 \
OMMPC_OUTPUT_DIR="$HOME/ommpc_results/N100_dt001" \
  ./scripts/run_ommpc_isolated.sh --horizon 100 --dt 0.01
```

Use `mean_us` for throughput and `p99_us`/`max_us` for scheduling risk. Any
`failures > 0` invalidates the timing result. Lu's online refresh period is fixed
at 10 ms independently of the OCP-grid `dt`; any solver time above 10 ms cannot
fit the current 100 Hz controller.

## Replay recorded Iris problems

Copy a dataset recorded during a stable Iris OMMPC flight into the bundle, then
run every candidate on exactly that sequence. The dataset fixes its own `N/dt`
and problem matrices; do not pass a different horizon override.

```bash
mkdir -p "$HOME/ommpc_results/replay"
export OMMPC_MODE=replay
export OMMPC_SOLVERS="qpoases qpdunes hpipm_ocp"
export OMMPC_SAMPLES=1000 OMMPC_WARMUP=100 OMMPC_REPEATS=5 OMMPC_CPU=3
export OMMPC_OUTPUT_DIR="$HOME/ommpc_results/replay"

./scripts/run_ommpc_isolated.sh \
  --dataset-in "$PWD/data/iris_figure8_125hz_1000.bin"
```

Inspect every `*.log` for `failures=0`, then compare `update_mean_us`,
`solve_mean_us`, p99 and max. Copy the complete result directory back to the
development machine after the run.

## Build and package on the development machine

The current development machine is already ARM64, so its Release binary can run
on the compatible Pi without compiling there:

```bash
cd ~/ws_sensor_combined
source /opt/ros/jazzy/setup.bash
colcon build --packages-select geometric_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
src/geometric_controller/scripts/package_ommpc_zero2w.sh
scp ommpc_zero2w_aarch64_ubuntu24_safe_v4_20260817.tar.gz* pi@PI_ADDRESS:~
```

Verify the checksum on the Pi before extracting:

```bash
sha256sum -c ommpc_zero2w_aarch64_ubuntu24_safe_v4_20260817.tar.gz.sha256
```
