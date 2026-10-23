#!/usr/bin/env bash
# Builds host-coverage, runs tests and reports line coverage of core/ (SW-02).
# Fails if coverage is below TOF_COVERAGE_MIN percent (default 90).
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
tof_activate_venv
tof_require cmake ninja gcc ctest gcovr

COV_MIN="${TOF_COVERAGE_MIN:-90}"
BUILD_DIR="$TOF_ROOT/build/host-coverage"
REPORT_DIR="$BUILD_DIR/coverage"

cmake --preset host-coverage
cmake --build --preset host-coverage
# Stale counters from previous runs would inflate the result.
find "$BUILD_DIR" -name '*.gcda' -delete
ctest --preset host-coverage

mkdir -p "$REPORT_DIR"
GCOVR_ARGS=(--root "$TOF_ROOT" --filter "$TOF_ROOT/core/" --object-directory "$BUILD_DIR")

gcovr "${GCOVR_ARGS[@]}" --html-details "$REPORT_DIR/index.html"
gcovr "${GCOVR_ARGS[@]}" --print-summary --fail-under-line "$COV_MIN"

echo "HTML report: $REPORT_DIR/index.html"
echo "Coverage threshold for core/: ${COV_MIN}% lines - OK"
