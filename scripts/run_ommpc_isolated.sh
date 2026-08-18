#!/usr/bin/env bash
set -euo pipefail

# Run one solver per process so allocator state and solver-global state cannot leak
# between measurements. Extra CLI arguments (for example --horizon 20 --dt 0.05)
# are passed verbatim to lu_ommpc_benchmark.
mode="${OMMPC_MODE:-mpc}"
preset="${OMMPC_PRESET:-paper}"
samples="${OMMPC_SAMPLES:-2000}"
warmup="${OMMPC_WARMUP:-200}"
repeats="${OMMPC_REPEATS:-3}"
cpu="${OMMPC_CPU:-3}"
solvers="${OMMPC_SOLVERS:-qpoases hpipm_ocp qpdunes}"
output_dir="${OMMPC_OUTPUT_DIR:-/tmp/lu_ommpc_isolated}"
benchmark_bin="${OMMPC_BENCHMARK_BIN:-}"

mkdir -p "${output_dir}"

if [[ -n "${benchmark_bin}" ]]; then
  benchmark=("${benchmark_bin}")
else
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 is unavailable; set OMMPC_BENCHMARK_BIN to the standalone benchmark" >&2
    exit 1
  fi
  benchmark=(ros2 run geometric_controller lu_ommpc_benchmark)
fi

read -r -a solver_list <<< "${solvers}"
for ((round = 1; round <= repeats; ++round)); do
  for solver in "${solver_list[@]}"; do
    stem="${output_dir}/${solver}_round${round}"
    echo "round=${round} solver=${solver} cpu=${cpu}"
    if [[ -r /sys/class/thermal/thermal_zone0/temp ]]; then
      echo "temperature_mC=$(< /sys/class/thermal/thermal_zone0/temp)"
    fi
    taskset -c "${cpu}" "${benchmark[@]}" \
      --mode "${mode}" --solver "${solver}" --preset "${preset}" \
      --samples "${samples}" --warmup "${warmup}" --csv "${stem}.csv" \
      "$@" 2>&1 | tee "${stem}.log"
  done
done

