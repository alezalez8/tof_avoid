#!/usr/bin/env bash
# Regenerates docs/params.md from the parameter table (SW-07a).
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

"$TOF_ROOT/tools/scripts/build_host.sh" >/dev/null
"$TOF_ROOT/build/host-debug/tests/host/params_md" > "$TOF_ROOT/docs/params.md"
echo "docs/params.md updated"
