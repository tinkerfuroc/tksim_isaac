import sys
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.contact_sheet import build_sheet, sample_evenly, _stamp_from_name  # noqa: E402


def test_sample_evenly():
    assert sample_evenly([1, 2, 3], 5) == [1, 2, 3]
    assert sample_evenly(list(range(10)), 4) == [0, 3, 6, 9]
    assert sample_evenly([], 4) == []


def _mk_frames(d, label, n, size=(32, 18)):
    p = d / "frames" / label
    p.mkdir(parents=True)
    for i in range(n):
        Image.new("RGB", size, (i * 10 % 255, 80, 80)).save(p / f"{i:04d}_{i*1000}.jpg")


def test_build_sheet_two_rows(tmp_path):
    _mk_frames(tmp_path, "arena", 20)
    _mk_frames(tmp_path, "head", 3)
    meta = {
        "id": "c001",
        "text": "go to the kitchen table",
        "verdict": "PASS",
        "seconds": 93.2,
        "tier": "T2",
    }
    out = build_sheet(tmp_path, meta, tmp_path / "sheet.jpg", columns=12)
    img = Image.open(out)
    assert img.width == 12 * 320
    assert img.height > 88 + 2 * 18  # header + two captioned rows


def test_build_sheet_missing_label_degrades(tmp_path):
    _mk_frames(tmp_path, "head", 2)
    out = build_sheet(tmp_path, {"id": "x", "verdict": "ERROR"}, tmp_path / "s.jpg")
    assert out.exists()  # no crash; arena row is a placeholder band


def test_stamp_from_name_parses_seq_ms_filename():
    assert _stamp_from_name("0007_1500.jpg") == "1.5"


def test_stamp_from_name_degrades_on_unparseable_name():
    # No trailing `_<ms>` integer -- must not raise (C3's "never a crash").
    assert _stamp_from_name("not-a-frame-name.jpg") == "?"
    assert _stamp_from_name("frame.jpg") == "?"


def test_build_sheet_survives_unparseable_frame_filename(tmp_path):
    # A stray non-conforming file in frames/head/ must not crash the
    # builder -- it degrades to a "t=?s" caption instead.
    p = tmp_path / "frames" / "head"
    p.mkdir(parents=True)
    Image.new("RGB", (32, 18), (10, 80, 80)).save(p / "not-a-frame-name.jpg")
    out = build_sheet(tmp_path, {"id": "x", "verdict": "ERROR"}, tmp_path / "s.jpg")
    assert out.exists()
