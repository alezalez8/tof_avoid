#!/usr/bin/env bash
# Configures and builds the host-debug preset (core + unit tests).
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
tof_require cmake ninja gcc

cmake --preset host-debug
cmake --build --preset host-debug
