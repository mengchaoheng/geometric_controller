#!/usr/bin/env bash
set -eo pipefail

script_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${script_root}/env.sh"

samples="${SAMPLES:-300}"
warmup="${WARMUP:-50}"
result_dir="${RESULT_DIR:-${OMMPC_PI_ROOT}/results_$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "${result_dir}"

run_case() {
  local tag="$1" solver="$2" horizon="$3" dt="$4"
  echo
  echo "===== ${tag}: solver=${solver} N=${horizon} dt=${dt} ====="
  "${OMMPC_BENCHMARK_BIN}" \
    --mode solver --solver "${solver}" --horizon "${horizon}" --dt "${dt}" \
    --samples "${samples}" --warmup "${warmup}" \
    --csv "${result_dir}/${tag}.csv" | tee "${result_dir}/${tag}.txt"
}

# Candidate configurations retained after the MATLAB/Pi scan. This matrix uses
# the flight default qpDUNES; compare all five retained backends with
# `lu_ommpc_benchmark --solver all` or run_ommpc_isolated.sh.
run_case n20_dt005_qpdunes qpdunes 20 0.05
run_case n20_dt003_qpdunes qpdunes 20 0.03
run_case n25_dt0025_qpdunes qpdunes 25 0.025

echo
echo "Results written to ${result_dir}"
