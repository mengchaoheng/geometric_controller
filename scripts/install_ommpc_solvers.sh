#!/usr/bin/env bash

# Prepare the qpDUNES dependency used by the online default. The package keeps
# five solver adapters (qpDUNES, HPIPM OCP, qpOASES, OSQP and DAQP); the latter four
# are intentionally kept as standard reference baselines. Their pinned local
# libraries must also be present before CMake can build the package.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd -- "${script_dir}/.." && pwd)"
deps_dir="${package_dir}/.ommpc_deps"
source_dir="${deps_dir}/src"
build_dir="${deps_dir}/build"

mkdir -p "${source_dir}" "${build_dir}"

clone_commit()
{
  local repository="$1"
  local directory="$2"
  local commit="$3"
  if [[ ! -d "${directory}/.git" ]]; then
    git clone --filter=blob:none --no-checkout "${repository}" "${directory}"
  fi
  if [[ "$(git -C "${directory}" rev-parse HEAD 2>/dev/null || true)" != "${commit}" ]]; then
    git -C "${directory}" fetch --depth 1 origin "${commit}"
  fi
  git -C "${directory}" checkout --detach "${commit}"
}

clone_commit \
  https://github.com/jfrasch/qpDUNES.git \
  "${source_dir}/qpdunes-next" \
  665cbaac32be1a2477a991722ad99b2bb84d0631

cmake -S "${source_dir}/qpdunes-next" -B "${build_dir}/qpdunes-next" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${deps_dir}/qpdunes"
cmake --build "${build_dir}/qpdunes-next" --parallel
cmake --install "${build_dir}/qpdunes-next"

required_paths=(
  "${deps_dir}/qpdunes/lib/libqpdunes.a"
  "${deps_dir}/qpoases/lib/libqpOASES.a"
  "${deps_dir}/daqp/lib/libdaqpstat.a"
  "${deps_dir}/hpipm/lib/libhpipm.a"
  "${deps_dir}/blasfeo/lib/libblasfeo.a"
  "${deps_dir}/opt/ros/jazzy/lib/libosqp.a"
)
missing=0
for path in "${required_paths[@]}"; do
  if [[ ! -f "${path}" ]]; then
    echo "missing retained solver dependency: ${path}" >&2
    missing=1
  fi
done
if (( missing != 0 )); then
  echo "Install the pinned qpOASES/DAQP/HPIPM/BLASFEO/OSQP packages before building." >&2
  exit 1
fi

echo "Installed qpDUNES under ${deps_dir}/qpdunes"
echo "Retained build-time libraries also required: hpipm, blasfeo, qpoases, osqp and daqp"
