#!/usr/bin/env bash
set -euo pipefail

# Package an already-built ROS 2 workspace for an aarch64 Raspberry Pi.
#
# Normal development builds use colcon's default isolated layout in install/.
# A merged layout is optional and is used only to make a compact Pi bundle.  If
# desired, build it in a separate install_pi_merged/ directory and pass that
# directory through OMMPC_INSTALL_DIR; never reuse the normal install/ directory
# with two different layouts.
#
# The Pi must already provide the same ROS 2 Jazzy runtime and Micro XRCE-DDS
# Agent.  This archive carries the workspace packages and their generated PX4
# message types, but does not copy /opt/ros.

package_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="$(cd "${package_root}/../.." && pwd)"
install_dir="${OMMPC_INSTALL_DIR:-${workspace_root}/install}"
bundle="${1:-${workspace_root}/ommpc_ros_zero2w_20260818}"
archive="${bundle}.tar.gz"

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "This bundle must be built on aarch64 (current: $(uname -m))." >&2
  exit 1
fi
if [[ ! -d "${install_dir}" || ! -f "${install_dir}/setup.bash" ]]; then
  echo "Missing ROS 2 install at ${install_dir}. Build the workspace before packaging." >&2
  exit 1
fi
if [[ -e "${bundle}" || -e "${archive}" || -e "${archive}.sha256" ]]; then
  echo "Refusing to overwrite an existing bundle: ${bundle}" >&2
  exit 1
fi

install -d "${bundle}/scripts" "${bundle}/docs"
cp -a "${install_dir}" "${bundle}/install"
install -m 0755 "${package_root}/scripts/pi_env.sh" "${bundle}/env.sh"
install -m 0755 "${package_root}/scripts/pi_benchmark_matrix.sh" "${bundle}/scripts/"
install -m 0755 "${package_root}/scripts/pi_frontier_matrix.sh" "${bundle}/scripts/"
install -m 0755 "${package_root}/scripts/pi_matlab_range_matrix.sh" "${bundle}/scripts/"
install -m 0755 "${package_root}/scripts/pi_matlab_range_system_stress.sh" "${bundle}/scripts/"
install -m 0755 "${package_root}/scripts/pi_dds_check.sh" "${bundle}/scripts/"
install -m 0755 "${package_root}/scripts/pi_run_ommpc.sh" "${bundle}/scripts/"
install -m 0755 "${package_root}/scripts/pi_online_capture.sh" "${bundle}/scripts/"
install -m 0755 "${package_root}/scripts/pi_100hz_system_stress.sh" "${bundle}/scripts/"
install -m 0644 "${package_root}/docs/ommpc_pi_runtime_test.md" "${bundle}/docs/"

{
  install_layout="unknown"
  if [[ -f "${install_dir}/.colcon_install_layout" ]]; then
    install_layout="$(<"${install_dir}/.colcon_install_layout")"
  fi
  echo "bundle_root=$(basename "${bundle}")"
  echo "install_layout=${install_layout}"
  echo "install_source=${install_dir}"
  echo "built_on=$(uname -a)"
  echo "built_at=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "packages="
  (cd "${workspace_root}" && colcon list --names-only 2>/dev/null || true)
} > "${bundle}/MANIFEST.txt"

tar -C "$(dirname "${bundle}")" -czf "${archive}" "$(basename "${bundle}")"
# Write a portable checksum: sha256sum -c must be run on the Pi, so the
# checksum file must contain the archive basename rather than this VM's path.
(cd "$(dirname "${archive}")" && sha256sum "$(basename "${archive}")") > "${archive}.sha256"
echo "archive=${archive}"
cat "${archive}.sha256"
