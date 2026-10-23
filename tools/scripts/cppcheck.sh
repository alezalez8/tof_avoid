#!/usr/bin/env bash
# Static analysis of own code (SW-09). Any finding fails the script.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
tof_require cppcheck

cppcheck --version
cppcheck \
    --std=c11 \
    --enable=warning,style,performance,portability \
    --error-exitcode=1 \
    --inline-suppr \
    --quiet \
    --template=gcc \
    --suppress=missingIncludeSystem \
    -I core/include \
    core app drivers bsp tests/host tools/params_md

echo "cppcheck: no findings"
