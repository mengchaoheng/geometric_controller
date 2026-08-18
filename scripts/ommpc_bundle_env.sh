#!/usr/bin/env bash

_ommpc_bundle_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export OMMPC_BENCHMARK_BIN="${_ommpc_bundle_root}/bin/lu_ommpc_benchmark"
export LD_LIBRARY_PATH="${_ommpc_bundle_root}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
unset _ommpc_bundle_root

