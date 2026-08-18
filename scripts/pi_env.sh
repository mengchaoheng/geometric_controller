#!/usr/bin/env bash
set -eo pipefail

_ommpc_pi_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
# ROS 2 Jazzy's generated setup scripts use a few optional variables without
# `${var:-...}`.  They are not compatible with Bash nounset (`set -u`).
# Temporarily relax nounset while sourcing ROS and the merged workspace, then
# restore the caller's setting.
_ommpc_pi_restore_nounset=0
case "$-" in
  *u*) _ommpc_pi_restore_nounset=1; set +u ;;
esac
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed at /opt/ros/jazzy on this Pi." >&2
  if (( _ommpc_pi_restore_nounset )); then set -u; fi
  return 1 2>/dev/null || exit 1
fi
source /opt/ros/jazzy/setup.bash
source "${_ommpc_pi_root}/install/setup.bash"
export OMMPC_PI_ROOT="${_ommpc_pi_root}"
export OMMPC_BENCHMARK_BIN="${_ommpc_pi_root}/install/lib/geometric_controller/lu_ommpc_benchmark"
export OMMPC_TRAJECTORY_BIN="${_ommpc_pi_root}/install/lib/geometric_controller/trajectory_offboard_node"
if (( _ommpc_pi_restore_nounset )); then set -u; fi
unset _ommpc_pi_restore_nounset
unset _ommpc_pi_root
