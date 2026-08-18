#!/usr/bin/env bash
set -euo pipefail

# Full-system stress for the retained candidate range. Each case starts and
# stops one headless ROS2 controller, so DDS/controller load is included.
script_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${script_root}/env.sh"

duration="${DURATION:-30}"
rate_hz="${RATE_HZ:-100}"
result_root="${RESULT_ROOT:-${OMMPC_PI_ROOT}/matlab_range_stress_$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "${result_root}"

run_case() {
  local tag="$1" solver="$2" horizon="$3" dt="$4"
  echo
  echo "===== ${tag}: solver=${solver} N=${horizon} dt=${dt} duration=${duration}s ====="
  RESULT_DIR="${result_root}/${tag}" \
    OMMPC_TOPIC_MONITORS=0 \
    scripts/pi_100hz_system_stress.sh "${solver}" "${horizon}" "${dt}" \
      "${duration}" "${rate_hz}"
}

run_case n20_dt005_qpdunes qpdunes 20 0.05
run_case n20_dt003_qpdunes qpdunes 20 0.03
run_case n25_dt004_qpdunes qpdunes 25 0.04
run_case n25_dt0025_qpdunes qpdunes 25 0.025
run_case n30_dt0025_qpdunes qpdunes 30 0.025

echo
echo "Results written to ${result_root}"
