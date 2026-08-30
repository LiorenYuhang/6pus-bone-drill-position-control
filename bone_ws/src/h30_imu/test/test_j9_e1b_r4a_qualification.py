from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import pytest

PKG = Path(__file__).resolve().parents[1]
WS_ROOT = PKG.parent
for pattern in ["*/lib/python3.12/site-packages", "*/local/lib/python3.12/dist-packages"]:
    for p in (WS_ROOT / "install").glob(pattern):
        if str(p) not in sys.path:
            sys.path.insert(0, str(p))
SCRIPT = PKG / "scripts" / "j9_e1b_r4a_feedback_fk_qualification.py"
spec = importlib.util.spec_from_file_location("j9_e1b_r4a_feedback_fk_qualification", SCRIPT)
r4a = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = r4a
spec.loader.exec_module(r4a)


def test_hardware_write_accounting_full_run():
    final = {
        "home_results": [
            {"run_index": 1, "home": {"total_cycles": 1038}},
            {"run_index": 2, "home": {"total_cycles": 1038}},
        ],
        "repeats": [
            {"run_index": 1, "motor_summary": {"cycles_completed": 3288}},
            {"run_index": 2, "motor_summary": {"cycles_completed": 3288}},
        ],
        "park_result": {"total_cycles": 1049},
    }
    total = r4a.compute_total_hardware_writes(final)
    # home 1: 6 + 1038*6 + 6 = 6240
    # home 2: 6 + 1038*6 + 6 = 6240
    # rep 1:  6 + 3288*6 + 6 = 19740
    # rep 2:  6 + 3288*6 + 6 = 19740
    # park:   6 + 1049*6 + 6 = 6306
    # Total = 6240 + 6240 + 19740 + 19740 + 6306 = 58266
    expected = (6 + 1038*6 + 6)*2 + (6 + 3288*6 + 6)*2 + (6 + 1049*6 + 6)
    assert total == expected
    assert total == 58266


def test_hardware_write_accounting_aborted_run():
    final = {
        "home_results": [
            {"run_index": 1, "home": {"total_cycles": 1038}},
        ],
        "repeats": [
            {"run_index": 1, "motor_summary": {"cycles_completed": 3287, "cycles_attempted": 3288}},
        ],
        "park_result": {"total_cycles": 1049},
    }
    total = r4a.compute_total_hardware_writes(final)
    # home: 6240
    # rep 1: 6 + 3287*6 + 6 = 19734
    # park: 6306
    # Total = 6240 + 19734 + 6306 = 32280
    expected = (6 + 1038*6 + 6) + (6 + 3287*6 + 6) + (6 + 1049*6 + 6)
    assert total == expected
    assert total == 32280
    assert total > 0


def test_qualification_segments_structure():
    segs = r4a.qualification_segments(1)
    assert len(segs) == 6
    roles = [s.role for s in segs]
    assert roles == ["stationary", "roll", "pitch", "yaw", "combined", "home_return"]
    total_duration = sum(s.duration_s for s in segs)
    assert total_duration == 90.0
