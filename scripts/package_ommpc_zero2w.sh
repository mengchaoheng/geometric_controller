#!/usr/bin/env bash
set -euo pipefail

package_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="$(cd "${package_root}/../.." && pwd)"
bundle="${1:-${workspace_root}/ommpc_zero2w_aarch64_ubuntu24_safe_v4_20260817}"
benchmark="${workspace_root}/build/geometric_controller/lu_ommpc_benchmark"
cpg_library="${workspace_root}/build/geometric_controller/liblu_ommpc_cpg.so"

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "this bundle must be built on aarch64 (current: $(uname -m))" >&2
  exit 1
fi
if [[ -e "${bundle}" || -e "${bundle}.tar.gz" ]]; then
  echo "refusing to overwrite existing bundle: ${bundle}" >&2
  exit 1
fi
if [[ ! -x "${benchmark}" || ! -r "${cpg_library}" ]]; then
  echo "Release build artifacts are missing; build geometric_controller first" >&2
  exit 1
fi

install -d "${bundle}/bin" "${bundle}/lib" "${bundle}/scripts" "${bundle}/data"
install -m 0755 "${benchmark}" "${bundle}/bin/lu_ommpc_benchmark"
install -m 0644 "${cpg_library}" "${bundle}/lib/liblu_ommpc_cpg.so"
install -m 0755 "${package_root}/scripts/ommpc_bundle_env.sh" "${bundle}/env.sh"
install -m 0755 "${package_root}/scripts/run_ommpc_isolated.sh" "${bundle}/scripts/"
install -m 0755 "${package_root}/scripts/validate_ommpc_scales.sh" "${bundle}/scripts/"
install -m 0644 "${package_root}/docs/ommpc_pi_zero2w.md" "${bundle}/README.md"
install -m 0644 "${package_root}/docs/ommpc_local_benchmark.md" "${bundle}/"
if [[ -r "${workspace_root}/ommpc_data/iris_figure8_125hz_1000.bin" ]]; then
  install -m 0644 "${workspace_root}/ommpc_data/iris_figure8_125hz_1000.bin" "${bundle}/data/"
fi

tar -C "$(dirname "${bundle}")" -czf "${bundle}.tar.gz" "$(basename "${bundle}")"
sha256sum "${bundle}.tar.gz" > "${bundle}.tar.gz.sha256"
echo "bundle=${bundle}.tar.gz"
cat "${bundle}.tar.gz.sha256"
