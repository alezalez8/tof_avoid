"""pytest setup for the SITL scenarios: one SITL instance (5) at a time, so tests run sequentially."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import harness as H  # noqa: E402


@pytest.fixture(scope="session", autouse=True)
def tof_sil_built():
    if not H.tof_sil_bin().exists():
        pytest.exit("tof_sil not built: run tools/scripts/build_host.sh", returncode=2)
    if not (H.ardupilot_dir() / "build" / "sitl" / "bin" / "arducopter").exists():
        pytest.exit("ArduCopter SITL binary not found (TOF_ARDUPILOT)", returncode=2)
