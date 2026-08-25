import sys
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from validation.gpsr_run_recorder import FrameSink  # noqa: E402


def _rgb(w, h, val):  # solid-colour rgb8 buffer
    return bytes([val, 0, 0]) * (w * h)


def test_sink_saves_first_and_respects_interval(tmp_path):
    s = FrameSink(tmp_path, "arena", interval_s=1.0, max_frames=10)
    assert s.offer(0.0, _rgb(4, 3, 200), 4, 3) is not None
    assert s.offer(0.5, _rgb(4, 3, 200), 4, 3) is None
    p = s.offer(1.05, _rgb(4, 3, 100), 4, 3)
    assert p is not None and p.name.startswith("0001_1050")
    img = Image.open(p)
    assert img.size == (4, 3)


def test_sink_caps_frames(tmp_path):
    s = FrameSink(tmp_path, "head", interval_s=0.0, max_frames=2)
    assert s.offer(0.0, _rgb(2, 2, 1), 2, 2)
    assert s.offer(1.0, _rgb(2, 2, 1), 2, 2)
    assert s.offer(2.0, _rgb(2, 2, 1), 2, 2) is None


def test_meta_summary(tmp_path):
    s = FrameSink(tmp_path, "arena", interval_s=1.0, max_frames=10)
    s.offer(2.0, _rgb(2, 2, 1), 2, 2)
    s.offer(3.5, _rgb(2, 2, 1), 2, 2)
    assert s.summary() == {"frames": 2, "first_stamp": 2.0, "last_stamp": 3.5}
