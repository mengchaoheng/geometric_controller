#!/usr/bin/env bash
set -eo pipefail

script_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${script_root}/env.sh"

samples="${SAMPLES:-100}"
warmup="${WARMUP:-20}"
solver_samples="${SOLVER_SAMPLES:-50}"
solver_warmup="${SOLVER_WARMUP:-10}"
result_dir="${RESULT_DIR:-${OMMPC_PI_ROOT}/frontier_$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "${result_dir}"

run_case() {
  local mode="$1" tag="$2" solver="$3" horizon="$4" dt="$5" n_samples="$6" n_warmup="$7"
  echo
  echo "===== ${tag}: mode=${mode} solver=${solver} N=${horizon} dt=${dt} samples=${n_samples} warmup=${n_warmup} ====="
  "${OMMPC_BENCHMARK_BIN}" \
    --mode "${mode}" --solver "${solver}" \
    --horizon "${horizon}" --dt "${dt}" \
    --samples "${n_samples}" --warmup "${n_warmup}" \
    --csv "${result_dir}/${tag}.csv" | tee "${result_dir}/${tag}.txt"
}

# Full MPC is used for the retained Pareto candidates. This includes
# manifold/linearization, OCP building, solver update and solve.
run_case mpc qpdunes_n20_dt005 qpdunes 20 0.05 "${samples}" "${warmup}"
run_case mpc qpdunes_n20_dt003 qpdunes 20 0.03 "${samples}" "${warmup}"
run_case mpc qpdunes_n25_dt0025 qpdunes 25 0.025 "${samples}" "${warmup}"

echo
echo "Results written to ${result_dir}"
