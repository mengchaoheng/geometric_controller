#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
DEPS_DIR="${PACKAGE_DIR}/.ommpc_deps"
SOURCE_DIR="${DEPS_DIR}/src"
BUILD_DIR="${DEPS_DIR}/build"
DEB_DIR="${DEPS_DIR}/packages"

mkdir -p "${DEB_DIR}" "${SOURCE_DIR}" "${BUILD_DIR}"

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
  # A --no-checkout clone can already point at the requested commit while
  # still having an empty worktree.  Always materialize the pinned revision.
  git -C "${directory}" checkout --detach "${commit}"
}

(
  cd "${DEB_DIR}"
  apt download ros-jazzy-osqp-vendor ros-jazzy-proxsuite libsimde-dev
)
for package_file in "${DEB_DIR}"/*.deb; do
  dpkg-deb --extract "${package_file}" "${DEPS_DIR}"
done

clone_commit https://github.com/coin-or/qpOASES.git \
  "${SOURCE_DIR}/qpOASES" 680f18c8ef0018a120e1604b769f056e8368df97
clone_commit https://github.com/darnstrom/daqp.git \
  "${SOURCE_DIR}/daqp" fa2bb38d57562f1b8835a7ee0789c67cad70321b
clone_commit https://github.com/PREDICT-EPFL/piqp.git \
  "${SOURCE_DIR}/piqp" 2f16417cde0628928ad0103db4ea84b09a76552b
clone_commit https://github.com/qpSWIFT/qpSWIFT.git \
  "${SOURCE_DIR}/qpswift" 24608b671d0e7ecde4d14ee8530d1656d6940fd1
clone_commit https://github.com/giaf/blasfeo.git \
  "${SOURCE_DIR}/blasfeo" 0ab5db3259c009ea62318a5e35622fe6de7ae554
clone_commit https://github.com/giaf/hpipm.git \
  "${SOURCE_DIR}/hpipm" 01786b5abcf26b321eeed1c12ccd10cdad0212fc
clone_commit https://github.com/emgertz/OOQP.git \
  "${SOURCE_DIR}/ooqp" 460ba0eb849eadaa29c4d4d88a9ce3554f3ce165
clone_commit https://github.com/TinyMPC/TinyMPC.git \
  "${SOURCE_DIR}/tinympc-next" 8b65e27a142000540b06d0264073ce3c9b2dad17
clone_commit https://github.com/jfrasch/qpDUNES.git \
  "${SOURCE_DIR}/qpdunes-next" 665cbaac32be1a2477a991722ad99b2bb84d0631

if git -C "${SOURCE_DIR}/tinympc-next" apply --check \
  "${PACKAGE_DIR}/patches/tinympc-system-eigen.patch" 2>/dev/null; then
  git -C "${SOURCE_DIR}/tinympc-next" apply \
    "${PACKAGE_DIR}/patches/tinympc-system-eigen.patch"
fi
if git -C "${SOURCE_DIR}/hpipm" apply --check \
  "${PACKAGE_DIR}/patches/hpipm-disable-runtime-checks.patch" 2>/dev/null; then
  git -C "${SOURCE_DIR}/hpipm" apply \
    "${PACKAGE_DIR}/patches/hpipm-disable-runtime-checks.patch"
fi

cmake -S "${SOURCE_DIR}/qpOASES" -B "${BUILD_DIR}/qpoases" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${DEPS_DIR}/qpoases" \
  -DQPOASES_AVOID_LA_NAMING_CONFLICTS=ON -DQPOASES_BUILD_EXAMPLES=OFF
cmake --build "${BUILD_DIR}/qpoases" --parallel
cmake --install "${BUILD_DIR}/qpoases"

cmake -S "${SOURCE_DIR}/daqp" -B "${BUILD_DIR}/daqp" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${DEPS_DIR}/daqp" \
  -DEIGEN=ON -DPROFILING=OFF
cmake --build "${BUILD_DIR}/daqp" --parallel
cmake --install "${BUILD_DIR}/daqp"

cmake -S "${SOURCE_DIR}/piqp" -B "${BUILD_DIR}/piqp" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${DEPS_DIR}/piqp" \
  -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF \
  -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_MATLAB_INTERFACE=OFF \
  -DBUILD_OCTAVE_INTERFACE=OFF -DBUILD_C_INTERFACE=OFF \
  -DBUILD_WITH_TEMPLATE_INSTANTIATION=ON
cmake --build "${BUILD_DIR}/piqp" --parallel
cmake --install "${BUILD_DIR}/piqp"

cmake -S "${SOURCE_DIR}/qpswift" -B "${BUILD_DIR}/qpswift" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${DEPS_DIR}/qpswift" \
  -DQPTESTS=OFF -DQPSHAREDLIB=OFF -DQPDEMOS=OFF
cmake --build "${BUILD_DIR}/qpswift" --parallel
cmake --install "${BUILD_DIR}/qpswift"

cmake -S "${SOURCE_DIR}/tinympc-next" -B "${BUILD_DIR}/tinympc-next" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}/tinympc-next" --target tinympcstatic --parallel

cmake -S "${SOURCE_DIR}/qpdunes-next" -B "${BUILD_DIR}/qpdunes-next" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${DEPS_DIR}/qpdunes"
cmake --build "${BUILD_DIR}/qpdunes-next" --parallel
cmake --install "${BUILD_DIR}/qpdunes-next"

python3 -m venv "${DEPS_DIR}/cvxpygen-venv"
"${DEPS_DIR}/cvxpygen-venv/bin/pip" install "cvxpygen==1.0.0"
"${DEPS_DIR}/cvxpygen-venv/bin/python" \
  "${PACKAGE_DIR}/scripts/generate_ommpc_cvxpygen.py" \
  --output "${DEPS_DIR}/cvxpygen_ommpc"

BLASFEO_TARGET=GENERIC
OOQP_CONFIGURE_TARGET=()
if [[ "$(uname -m)" == "aarch64" ]]; then
  BLASFEO_TARGET=ARMV8A_ARM_CORTEX_A53
  OOQP_CONFIGURE_TARGET=(
    --build=aarch64-unknown-linux-gnu --host=aarch64-unknown-linux-gnu)
fi
cmake -S "${SOURCE_DIR}/blasfeo" -B "${BUILD_DIR}/blasfeo" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${DEPS_DIR}/blasfeo" \
  -DTARGET="${BLASFEO_TARGET}" -DLA=HIGH_PERFORMANCE \
  -DBLASFEO_EXAMPLES=OFF -DBLASFEO_TESTING=OFF \
  -DBLASFEO_BENCHMARKS=OFF -DBUILD_SHARED_LIBS=OFF
cmake --build "${BUILD_DIR}/blasfeo" --parallel
cmake --install "${BUILD_DIR}/blasfeo"

cmake -S "${SOURCE_DIR}/hpipm" -B "${BUILD_DIR}/hpipm" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${DEPS_DIR}/hpipm" \
  -DTARGET=GENERIC -DBLASFEO_PATH="${DEPS_DIR}/blasfeo" \
  -DBLASFEO_INCLUDE_DIR="${DEPS_DIR}/blasfeo/include" \
  -DHPIPM_TESTING=OFF -DBUILD_SHARED_LIBS=OFF
cmake --build "${BUILD_DIR}/hpipm" --parallel
cmake --install "${BUILD_DIR}/hpipm"

(
  cd "${SOURCE_DIR}/ooqp"
  CFLAGS="-O3 -DNDEBUG -fPIC" CXXFLAGS="-O3 -DNDEBUG -fPIC" \
    ./configure "${OOQP_CONFIGURE_TARGET[@]}" --prefix="${DEPS_DIR}/ooqp"
  make --jobs "$(getconf _NPROCESSORS_ONLN)" qpgen-dense-gondzio.exe
  make --jobs "$(getconf _NPROCESSORS_ONLN)" \
    lib/libooqpgendense.a lib/libooqpgondzio.a \
    lib/libooqpdense.a lib/libooqpbase.a
  make install_headers
)
mkdir -p "${DEPS_DIR}/ooqp/lib"
install -m 644 \
  "${SOURCE_DIR}/ooqp/lib/libooqpgendense.a" \
  "${SOURCE_DIR}/ooqp/lib/libooqpgondzio.a" \
  "${SOURCE_DIR}/ooqp/lib/libooqpdense.a" \
  "${SOURCE_DIR}/ooqp/lib/libooqpbase.a" \
  "${DEPS_DIR}/ooqp/lib/"

echo "Installed condensed solvers plus HPIPM OCP, qpDUNES and TinyMPC under ${DEPS_DIR}"
