#!/usr/bin/env bash
set -eo pipefail

script_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${script_root}/env.sh"

solver="${1:-qpdunes}"
horizon="${2:-20}"
dt="${3:-0.05}"
param_file="${OMMPC_PI_ROOT}/install/share/geometric_controller/config/controller.yaml"

case "${solver}" in
  qpdunes|hpipm_ocp|qpoases|osqp|daqp) ;;
  *) echo "Unsupported solver name: ${solver}" >&2; exit 2 ;;
esac
if (( horizon < 1 || horizon > 100 )); then
  echo "N must be in 1..100" >&2; exit 2
fi

echo "Starting LU OMMPC diagnostics: solver=${solver} N=${horizon} dt=${dt}"
echo "auto_start=false and arm_on_start=false: PX4 will not be armed by this command."
echo "The node still consumes DDS feedback and publishes rate setpoints; PX4 ignores them outside Offboard."

exec ros2 run geometric_controller trajectory_offboard_node --ros-args \
  --params-file "${param_file}" \
  -p controller_type:=6 \
  -p ommpc.solver:="${solver}" \
  -p ommpc.N:="${horizon}" \
  -p ommpc.dt:="${dt}" \
  -p offboard.enabled:=true \
  -p offboard.auto_start:=false \
  -p offboard.arm_on_start:=false \
  -p offboard.takeoff_before_trajectory:=false
