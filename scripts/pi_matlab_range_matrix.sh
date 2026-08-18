#!/usr/bin/env bash
set -euo pipefail

# Measure the retained candidate range identified by the MATLAB scan. The
# default mode is standalone MPC timing; set MODE=solver to isolate update/solve.
script_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${script_root}/env.sh"

mode="${MODE:-mpc}"
samples="${SAMPLES:-300}"
warmup="${WARMUP:-50}"
result_dir="${RESULT_DIR:-${OMMPC_PI_ROOT}/matlab_range_$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "${result_dir}"

run_case() {
  local tag="$1" solver="$2" horizon="$3" dt="$4"
  echo
  echo "===== ${tag}: mode=${mode} solver=${solver} N=${horizon} dt=${dt} ====="
  "${OMMPC_BENCHMARK_BIN}" \
    --mode "${mode}" --solver "${solver}" \
    --horizon "${horizon}" --dt "${dt}" \
    --samples "${samples}" --warmup "${warmup}" \
    --csv "${result_dir}/${tag}.csv" | tee "${result_dir}/${tag}.txt"
}

if [[ "${mode}" != "mpc" && "${mode}" != "solver" ]]; then
  echo "MODE must be mpc or solver (got: ${mode})" >&2
  exit 2
fi

# Practical validated range: N=20..25, dt=0.025..0.05. N=30,dt=.025 is
# retained only as a boundary comparison, not as the default flight setting.
run_case n20_dt005_qpdunes qpdunes 20 0.05
run_case n20_dt003_qpdunes qpdunes 20 0.03
run_case n25_dt004_qpdunes qpdunes 25 0.04
run_case n25_dt0025_qpdunes qpdunes 25 0.025
run_case n30_dt0025_qpdunes qpdunes 30 0.025

echo
echo "Results written to ${result_dir}"
