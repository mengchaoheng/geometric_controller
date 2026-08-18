#!/usr/bin/env bash
set -u

# Validate exact candidate paths against a high-accuracy qpOASES reference.
# Each solver/scale is a fresh process. A nonzero final status means at least one
# candidate was rejected; its log contains the failed quality metric.
cpu="${OMMPC_CPU:-3}"
samples="${OMMPC_VALIDATE_SAMPLES:-120}"
warmup="${OMMPC_VALIDATE_WARMUP:-20}"
output_dir="${OMMPC_OUTPUT_DIR:-/tmp/lu_ommpc_validation}"
benchmark_bin="${OMMPC_BENCHMARK_BIN:-}"
solvers="${OMMPC_VALIDATE_SOLVERS:-qpoases hpipm_ocp qpdunes}"
status=0

mkdir -p "${output_dir}"
if [[ -n "${benchmark_bin}" ]]; then
  benchmark=("${benchmark_bin}")
else
  benchmark=(ros2 run geometric_controller lu_ommpc_benchmark)
fi
read -r -a solver_list <<< "${solvers}"

scales=("8 0.05" "20 0.05" "50 0.02" "100 0.01")
for scale in "${scales[@]}"; do
  read -r horizon dt <<< "${scale}"
  for solver in "${solver_list[@]}"; do
    log="${output_dir}/N${horizon}_dt${dt}_${solver}.log"
    echo "validate N=${horizon} dt=${dt} solver=${solver}"
    taskset -c "${cpu}" "${benchmark[@]}" --mode validate --solver "${solver}" \
      --preset paper --horizon "${horizon}" --dt "${dt}" \
      --samples "${samples}" --warmup "${warmup}" 2>&1 | tee "${log}"
    result=${PIPESTATUS[0]}
    if ((result != 0)); then
      status=2
    fi
  done
done
exit "${status}"

