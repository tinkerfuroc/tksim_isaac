import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from step_profile_summary import summarize  # noqa: E402


def _line(wall_time, sim_time, **ms):
    buckets = {"physics": 10.0, "publish": 1.0, "kit_pump": 30.0, "cameras": 5.0,
               "spin": 1.0, "unaccounted": 3.0, "wall": 50.0}
    buckets.update(ms)
    return json.dumps({"step_profile": {"wall_time": wall_time, "sim_time": sim_time,
                                        "cycles": 10, "ms_per_cycle": buckets}})


def test_summarize_skips_warmup_and_takes_medians():
    lines = [
        "step profiling on: reporting every 10 camera cycles",   # non-JSON noise
        _line(1000.0, 0.0, kit_pump=200.0),                        # warm-up, skipped
        _line(1040.0, 8.0, kit_pump=30.0),
        _line(1041.0, 8.8, kit_pump=32.0),
        _line(1042.0, 9.6, kit_pump=34.0),
    ]
    out = summarize(lines, skip_first_s=30.0)
    assert out["windows"] == 3
    assert out["median_ms_per_cycle"]["kit_pump"] == 32.0
    assert out["median_ms_per_cycle"]["wall"] == 50.0
    # 10 cycles of sim per window: 0.8 s sim per 1.0 s wall between windows
    assert abs(out["rtf_estimate"] - 0.8) < 1e-6


def test_summarize_empty_is_explicit():
    out = summarize([], skip_first_s=0.0)
    assert out["windows"] == 0 and out["rtf_estimate"] is None
