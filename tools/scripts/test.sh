#!/usr/bin/env bash
# Builds host-debug and runs all host unit tests (T-00).
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
tof_require ctest

"$TOF_ROOT/tools/scripts/build_host.sh"
ctest --preset host-debug "$@"
