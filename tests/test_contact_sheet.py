import sys
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.contact_sheet import build_sheet, sample_evenly  # noqa: E402


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
