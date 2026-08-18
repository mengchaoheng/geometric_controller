#!/usr/bin/env bash
set -eo pipefail

script_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${script_root}/env.sh"

solver="${1:-qpdunes}"
horizon="${2:-20}"
dt="${3:-0.05}"
duration="${4:-60}"
rate_hz="${5:-100}"
result_dir="${RESULT_DIR:-${OMMPC_PI_ROOT}/stress_$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "${result_dir}"
param_file="${OMMPC_PI_ROOT}/install/share/geometric_controller/config/controller.yaml"
if [[ ! -x "${OMMPC_TRAJECTORY_BIN}" ]]; then
  echo "Missing trajectory node: ${OMMPC_TRAJECTORY_BIN}" >&2
  exit 1
fi

echo "100 Hz system stress: solver=${solver} N=${horizon} dt=${dt} duration=${duration}s"
echo "The stress process computes OMMPC at a fixed rate but does not publish PX4 commands."
topic_monitors="${OMMPC_TOPIC_MONITORS:-1}"
if [[ "${topic_monitors}" == "0" ]]; then
  echo "Diagnostic ROS2 topic monitors disabled; resource.csv remains enabled."
fi

setsid "${OMMPC_TRAJECTORY_BIN}" --ros-args \
  --params-file "${param_file}" \
  -p controller_type:=6 \
  -p ommpc.solver:="${solver}" \
  -p ommpc.N:="${horizon}" \
  -p ommpc.dt:="${dt}" \
  -p offboard.enabled:=true \
  -p offboard.auto_start:=false \
  -p offboard.arm_on_start:=false \
  -p offboard.takeoff_before_trajectory:=false \
  > "${result_dir}/node.log" 2>&1 &
node_pid=$!

"${OMMPC_BENCHMARK_BIN}" \
  --mode stress --solver "${solver}" \
  --horizon "${horizon}" --dt "${dt}" \
  --rate-hz "${rate_hz}" --duration "${duration}" \
  --csv "${result_dir}/stress.csv" \
  > "${result_dir}/stress.log" 2>&1 &
stress_pid=$!
agent_pid="$(pgrep -n -x MicroXRCEAgent || true)"

echo "timestamp,pid,comm,pcpu,pmem,rss_kb,vsz_kb,etime" > "${result_dir}/resource.csv"
(
  while kill -0 "${node_pid}" 2>/dev/null || kill -0 "${stress_pid}" 2>/dev/null; do
    stamp="$(date -u +%Y-%m-%dT%H:%M:%S.%3NZ)"
    for pid in "${node_pid}" "${stress_pid}" "${agent_pid}"; do
      if [[ "${pid}" =~ ^[0-9]+$ ]]; then
        ps -p "${pid}" -o pid=,comm=,pcpu=,pmem=,rss=,vsz=,etime= | \
          awk -v stamp="${stamp}" \
            '{printf "%s,%s,%s,%s,%s,%s,%s,%s\n", stamp, $1, $2, $3, $4, $5, $6, $7}' \
          >> "${result_dir}/resource.csv" || true
      fi
    done
    sleep 0.5
  done
) &
resource_pid=$!

cleanup() {
  # Stop topic-monitor helper groups first.  They are diagnostic only and
  # must never keep a completed stress run alive.
  for pid in "${rate_pid:-}" "${status_pid:-}" "${hz_pid:-}"; do
    if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  if kill -0 "${stress_pid}" 2>/dev/null; then
    kill -INT "${stress_pid}" 2>/dev/null || true
  fi
  if kill -0 "${node_pid}" 2>/dev/null; then
    # ros2 run is a Python launcher with the C++ node as a child. Kill the
    # process group so the child cannot survive the stress script.
    kill -INT -- "-${node_pid}" 2>/dev/null || kill -INT "${node_pid}" 2>/dev/null || true
    sleep 0.2
    kill -TERM -- "-${node_pid}" 2>/dev/null || true
  fi
  wait "${stress_pid}" 2>/dev/null || true
  wait "${node_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ "${topic_monitors}" != "0" ]]; then
  setsid timeout "${duration}s" ros2 topic echo /controller/control_rate_status \
    > "${result_dir}/control_rate_status.log" 2>&1 &
  rate_pid=$!
  setsid timeout "${duration}s" ros2 topic echo /controller/ommpc_status \
    > "${result_dir}/ommpc_status.log" 2>&1 &
  status_pid=$!
  setsid timeout "${duration}s" ros2 topic hz /fmu/in/vehicle_rates_setpoint \
    > "${result_dir}/rates_hz.log" 2>&1 &
  hz_pid=$!
else
  : > "${result_dir}/control_rate_status.log"
  : > "${result_dir}/ommpc_status.log"
  : > "${result_dir}/rates_hz.log"
fi

# The benchmark is the timed operation.  Once it exits, diagnostic topic
# monitors are stopped immediately; waiting for their timeout can otherwise
# make a finished test appear hung on headless Pi systems.
wait "${stress_pid}" 2>/dev/null || true
for pid in "${rate_pid}" "${status_pid}" "${hz_pid}"; do
  if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  fi
done
wait "${rate_pid}" 2>/dev/null || true
wait "${status_pid}" 2>/dev/null || true
wait "${hz_pid}" 2>/dev/null || true
if kill -0 "${resource_pid}" 2>/dev/null; then
  kill "${resource_pid}" 2>/dev/null || true
fi
wait "${resource_pid}" 2>/dev/null || true

echo "Stress results written to ${result_dir}"
echo "Inspect stress.log/stress.csv and resource.csv."
