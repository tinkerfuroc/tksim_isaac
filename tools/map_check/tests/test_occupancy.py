import io
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from map_check.occupancy import (MapFileError, cell_center, closed_outlines,
                                 load_map_dir, parse_map)

REAL_YAML = Path("/home/tinker/tk25_ws/src/tk26_navigation/src/navigation_bringup/maps/"
                 "robocup_home_2026_measured_5cm.yaml")


def _yaml(image="m.pgm", negate=0, res=0.05):
    return (f"image: {image}\nresolution: {res}\norigin: [-1.0, -2.0, 0.0]\n"
            f"negate: {negate}\noccupied_thresh: 0.65\nfree_thresh: 0.25\n").encode()


def _encode(arr, mode="L", fmt="PPM"):
    buf = io.BytesIO()
    Image.fromarray(arr, mode).save(buf, format=fmt)
    return buf.getvalue()


ROW = np.array([[0, 80, 254]] * 3, dtype=np.uint8)


def test_trinary_threshold():
    m = parse_map(_yaml(), {"m.pgm": _encode(ROW)})
    assert m.occupied.tolist() == [[True, True, False]] * 3


def test_negate():
    m = parse_map(_yaml(negate=1), {"m.pgm": _encode(ROW)})
    assert m.occupied.tolist() == [[False, False, True]] * 3


def test_png_and_rgba():
    ref = parse_map(_yaml(), {"m.pgm": _encode(ROW)}).occupied
    png = parse_map(_yaml("m.png"), {"m.png": _encode(ROW, "L", "PNG")})
    rgba_arr = np.dstack([ROW, ROW, ROW, np.zeros_like(ROW)])
    rgba = parse_map(_yaml("m.png"), {"m.png": _encode(rgba_arr, "RGBA", "PNG")})
    assert (png.occupied == ref).all() and (rgba.occupied == ref).all()
    assert png.image_name == "m.png" and png.resolution == 0.05
    assert png.origin == (-1.0, -2.0)
    assert png.meta["negate"] == 0
    assert png.yaml_bytes == _yaml("m.png")


def test_missing_image():
    with pytest.raises(MapFileError, match="foo.pgm"):
        parse_map(_yaml("foo.pgm"), {})


def test_relative_image_path():
    data = _encode(ROW)
    m = parse_map(_yaml("sub/foo.pgm"), {"foo.pgm": data})
    assert m.image_name == "foo.pgm" and m.image_bytes == data


def test_bad_yaml():
    with pytest.raises(MapFileError):
        parse_map(b"just: [nope", {})
    with pytest.raises(MapFileError, match="resolution"):
        parse_map(b"image: a.pgm\n", {"a.pgm": b""})


def test_cell_center():
    m = parse_map(_yaml(), {"m.pgm": _encode(ROW)})
    big = type(m)(occupied=np.zeros((290, 370), bool), resolution=0.05,
                  origin=(-2.5, -2.5), meta={}, image_name="x",
                  yaml_bytes=b"", image_bytes=b"")
    x, y = cell_center(big, 289, 0)
    assert (x, y) == pytest.approx((-2.475, -2.475))


def test_closed_outline():
    occ = np.zeros((10, 10), bool)
    occ[2:6, 3:7] = True
    occ[3:5, 4:6] = False
    m = parse_map(_yaml(), {"m.pgm": _encode((~occ * 255).astype(np.uint8))})
    labels, outlines = closed_outlines(m)
    assert labels.dtype == np.int32 and len(outlines) == 1
    o = outlines[0]
    assert o.id == 1 and o.area_m2 == pytest.approx(4 * 0.05 ** 2)
    pts = [cell_center(m, r, c) for r in (3, 4) for c in (4, 5)]
    assert o.cx == pytest.approx(np.mean([p[0] for p in pts]))
    assert o.cy == pytest.approx(np.mean([p[1] for p in pts]))
    assert (labels == 1).sum() == 4 and (labels == 0).sum() == 96


def test_real_map():
    if not REAL_YAML.exists():
        pytest.skip("measured map absent")
    m = load_map_dir(REAL_YAML)
    assert m.occupied.shape == (290, 370) and m.resolution == 0.05
    _, outlines = closed_outlines(m)
    assert len(outlines) == 16
    assert sorted(round(o.area_m2, 2) for o in outlines) == [
        0.06, 0.06, 0.06, 0.09, 0.12, 0.28, 0.35, 0.36, 0.36, 0.54, 0.54,
        0.75, 0.76, 1.28, 1.33, 1.53]
