# Shared setup for host scripts (WSL2). Source it, do not execute.
set -euo pipefail

TOF_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TOF_VENV="${TOF_VENV:-$HOME/venvs/tof_avoid}"
cd "$TOF_ROOT"

# Activates the project venv for scripts that need Python tools.
tof_activate_venv() {
    if [ -f "$TOF_VENV/bin/activate" ]; then
        # shellcheck disable=SC1091
        source "$TOF_VENV/bin/activate"
    else
        echo "warning: venv $TOF_VENV not found, using system Python tools" >&2
    fi
}

tof_require() {
    local tool
    for tool in "$@"; do
        if ! command -v "$tool" >/dev/null 2>&1; then
            echo "error: '$tool' not found in PATH" >&2
            exit 127
        fi
    done
}
