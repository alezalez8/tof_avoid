#!/usr/bin/env bash
# E3: every SITL scenario (TZ section 13) in order, then the PASS/FAIL table and the reports.
#   tools/emulator/run_all.sh            all scenarios
#   tools/emulator/run_all.sh T26 T12    only these files (test_<id>.py)
# One SITL instance (-I 5) at a time; logs in build/sitl_run/<scenario>/, results in build/sitl_run/results/.
# Exit code 1 if a strict scenario failed or a run broke.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/../scripts/_common.sh"
tof_activate_venv
tof_require python3

ORDER=(T12 T13 T14 T15 T16 T18 T19 T26)
if [ $# -gt 0 ]; then ORDER=("$@"); fi

if [ ! -x "$TOF_ROOT/build/host-debug/tools/emulator/tof_sil/tof_sil" ]; then
    "$TOF_ROOT/tools/scripts/build_host.sh" >/dev/null
fi

RESULTS="$TOF_ROOT/build/sitl_run/results"
mkdir -p "$RESULTS"
rc=0
for t in "${ORDER[@]}"; do
    f="$HERE/scenarios/test_${t}.py"
    if [ ! -f "$f" ]; then
        echo "== $t: no scenario file" >&2
        rc=1
        continue
    fi
    rm -f "$RESULTS/T-${t#T}"*.json
    echo "== $t"
    (cd "$HERE/scenarios" && python3 -m pytest -q -p no:cacheprovider "$f" 2>&1 | tail -n 3)
done

python3 "$HERE/make_reports.py" --summary --write || rc=1
exit $rc
