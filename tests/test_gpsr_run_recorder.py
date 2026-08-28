import json
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


def test_sink_appends_index_line_for_accepted_frames_only(tmp_path):
    s = FrameSink(tmp_path, "head", interval_s=1.0, max_frames=10)
    accepted = s.offer(0.0, _rgb(2, 2, 1), 2, 2, wall_iso="2026-08-28T10:52:33.579091+00:00")
    rejected = s.offer(0.5, _rgb(2, 2, 1), 2, 2, wall_iso="2026-08-28T10:52:34.000000+00:00")
    assert accepted is not None
    assert rejected is None

    index_path = tmp_path / "frames" / "index.jsonl"
    lines = index_path.read_text().splitlines()
    assert len(lines) == 1
    entry = json.loads(lines[0])
    assert entry == {
        "label": "head",
        "file": f"frames/head/{accepted.name}",
        "stamp_s": 0.0,
        "wall": "2026-08-28T10:52:33.579091+00:00",
    }


def test_sink_index_wall_defaults_to_none(tmp_path):
    s = FrameSink(tmp_path, "arena", interval_s=0.0, max_frames=10)
    s.offer(0.0, _rgb(2, 2, 1), 2, 2)

    index_path = tmp_path / "frames" / "index.jsonl"
    entry = json.loads(index_path.read_text().splitlines()[0])
    assert entry["wall"] is None


def test_sink_index_shared_across_labels(tmp_path):
    a = FrameSink(tmp_path, "arena", interval_s=0.0, max_frames=10)
    h = FrameSink(tmp_path, "head", interval_s=0.0, max_frames=10)
    a.offer(0.0, _rgb(2, 2, 1), 2, 2)
    h.offer(0.0, _rgb(2, 2, 1), 2, 2)

    index_path = tmp_path / "frames" / "index.jsonl"
    lines = index_path.read_text().splitlines()
    assert len(lines) == 2
    labels = {json.loads(line)["label"] for line in lines}
    assert labels == {"arena", "head"}
