"""Give AMCL a surface-only copy of the arena map.

The arena map (tools/tinker_sim_deploy/arena_map.py) is rasterized from
collider footprints, so each obstacle is a SOLID block of occupied cells.
That is right for the costmaps -- the robot must not plan into furniture --
but wrong for AMCL: its likelihood field scores a scan hit only by the
distance to the nearest occupied cell, so a hit pushed deep inside a block
costs nothing. With the 165 deg front-only scan, nothing behind the robot
pushes back, and the estimate slides forward into walls and furniture
(2026-10-10, vs Isaac ground truth: likelihood optimum +0.07 m ahead of the
true pose; with this surface-only map +0.02 m; live AMCL error mean 0.13 m ->
0.09 m). A SLAM-built map -- what AMCL sees on the real robot -- records only
obstacle surfaces, which is what this produces: occupied cells that border a
non-occupied cell stay occupied, interiors become unknown.

The source map is left untouched; the copy is written per user and ROS domain
like the nav params overlay, and served to AMCL by its own map_server.
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

import yaml

OCCUPIED = 0
# map_server trinary reads p = (255 - v) / 255 as unknown between the yaml's
# free_thresh (0.25) and occupied_thresh (0.65); 128 is p = 0.50. (map_saver's
# usual 205 is p = 0.196 -- FREE here.)
UNKNOWN = 128
_OCCUPIED_BELOW = 128


def _read_pgm(pgm: bytes) -> tuple[int, int, int, bytearray]:
    """(width, height, maxval, data) of a binary P5 PGM, comments allowed."""
    fields: list[bytes] = []
    pos = 0
    while len(fields) < 4:
        while pgm[pos:pos + 1].isspace():
            pos += 1
        if pgm[pos:pos + 1] == b"#":
            pos = pgm.index(b"\n", pos) + 1
            continue
        end = pos
        while not pgm[end:end + 1].isspace():
            end += 1
        fields.append(pgm[pos:end])
        pos = end
    if fields[0] != b"P5":
        raise ValueError("not a binary (P5) PGM")
    width, height, maxval = (int(v) for v in fields[1:4])
    data = bytearray(pgm[pos + 1:pos + 1 + width * height])
    if len(data) != width * height:
        raise ValueError("PGM data shorter than its header")
    return width, height, maxval, data


def surface_only_pgm(pgm: bytes) -> bytes:
    """Keep each obstacle's outline; mark its interior unknown.

    An occupied cell stays occupied if any 4-neighbour is not occupied (or it
    is on the image edge); otherwise it becomes UNKNOWN.
    """
    width, height, maxval, data = _read_pgm(pgm)
    occupied = [v < _OCCUPIED_BELOW for v in data]
    out = bytearray(data)
    for row in range(1, height - 1):
        base = row * width
        for col in range(1, width - 1):
            i = base + col
            if (
                occupied[i]
                and occupied[i - 1]
                and occupied[i + 1]
                and occupied[i - width]
                and occupied[i + width]
            ):
                out[i] = UNKNOWN
    return b"P5\n%d %d\n%d\n" % (width, height, maxval) + bytes(out)


def write_amcl_map(source_yaml: Path, destination_dir: Path) -> Path:
    """Write the surface-only copy of *source_yaml*'s map; return its yaml."""
    source_yaml = Path(source_yaml)
    meta = yaml.safe_load(source_yaml.read_text(encoding="utf-8"))
    image = Path(meta["image"])
    if not image.is_absolute():
        image = source_yaml.parent / image
    destination_dir = Path(destination_dir)
    destination_dir.mkdir(parents=True, exist_ok=True)
    out_image = destination_dir / "amcl_map.pgm"
    out_image.write_bytes(surface_only_pgm(image.read_bytes()))
    meta["image"] = str(out_image.resolve())
    out_yaml = destination_dir / "amcl_map.yaml"
    out_yaml.write_text(yaml.safe_dump(meta, sort_keys=False), encoding="utf-8")
    return out_yaml


def default_amcl_map_dir() -> Path:
    """Per-user, per-ROS-domain scratch dir (several domains share the host)."""
    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    return Path(tempfile.gettempdir()) / f"tinker_sim_amcl_map.uid{os.getuid()}.domain{domain}"
