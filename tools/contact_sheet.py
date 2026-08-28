#!/usr/bin/env python3
"""Contact-sheet builder for GPSR tier-2 runs.

Condenses the JPEG frames a run recorder captured (see
``validation/gpsr_run_recorder.py``) into one reviewable JPEG in PORTRAIT
orientation: a header band summarising the run, a label band naming the
camera columns, then one column per camera label with an evenly sampled
run of frames flowing top-to-bottom, each tile captioned with its
timestamp. (The original layout was one very wide horizontal strip per
camera; the user asked for vertical sheets, which also scroll naturally
in an editor or on a phone.)

Pure PIL, no ROS.
"""
from __future__ import annotations

import argparse
import json
import sys
import textwrap
from pathlib import Path
from typing import Sequence

from PIL import Image, ImageDraw, ImageFont

TILE_W = 320
CAPTION_H = 18
HEADER_H = 88
LABEL_BAND_H = 18
PLACEHOLDER_H = 40
JPEG_QUALITY = 80
# "arena" stays first even though the arena observer camera is parked as a
# known issue (CUDA-700 under full-stack load; see docs/developer-log.md,
# "2026-08-26 -- GPSR recorded sim battery bring-up") and battery runs
# record head-camera only -- every sheet built from a battery run therefore
# shows a permanent grey "no arena frames captured" placeholder band for
# this row. That is the ledger's ruling, not a bug in this module.
ROW_LABELS = ("arena", "head")
# Portrait sheets are two tiles (640 px) wide; the default PIL bitmap font
# runs ~6 px per character, so wrap the command text to fit that width.
TEXT_WRAP_WIDTH = 96

VERDICT_COLORS = {
    "PASS": "#2e7d32",
    "FAIL": "#c62828",
    "TIMEOUT": "#ef6c00",
    "ERROR": "#616161",
}
DEFAULT_VERDICT_COLOR = "#455a64"


def sample_evenly(items: Sequence, k: int) -> list:
    """Return up to `k` items from `items`, evenly spaced, order-preserving, unique."""
    n = len(items)
    if n == 0 or k <= 0:
        return []
    if n <= k:
        return list(items)
    if k == 1:
        return [items[0]]
    indices = []
    seen = set()
    for i in range(k):
        j = round(i * (n - 1) / (k - 1))
        if j not in seen:
            seen.add(j)
            indices.append(j)
    return [items[j] for j in indices]


def _stamp_from_name(name: str) -> str:
    """Parse the `_<ms>` suffix from a `<seq>_<ms>.jpg` frame filename into a
    seconds string. Never raises: a file that doesn't match `<seq>_<ms>.jpg`
    (unparseable trailing `_<ms>`) degrades to "?" rather than crashing the
    builder, matching C3's "never a crash" promise -- honoured elsewhere for
    missing rows, but this parse had no guard.
    """
    stem = name.rsplit(".", 1)[0]
    ms_str = stem.split("_")[-1]
    try:
        seconds = int(ms_str) / 1000.0
    except ValueError:
        return "?"
    text = f"{seconds:.3f}".rstrip("0").rstrip(".")
    return text if text else "0"


def _round_seconds(value) -> int:
    try:
        return round(float(value))
    except (TypeError, ValueError):
        return 0


def _wrapped_text_lines(text: str, max_lines: int) -> list:
    lines = textwrap.wrap(str(text), width=TEXT_WRAP_WIDTH) or [""]
    return lines[:max_lines]


def _load_label_files(run_dir: Path, label: str) -> list:
    frames_dir = run_dir / "frames" / label
    if not frames_dir.is_dir():
        return []
    return sorted(frames_dir.glob("*.jpg"))


def _draw_header(draw: ImageDraw.ImageDraw, font, width: int, meta: dict) -> None:
    verdict = str(meta.get("verdict", ""))
    color = VERDICT_COLORS.get(verdict, DEFAULT_VERDICT_COLOR)
    run_id = meta.get("id", "")
    seconds = _round_seconds(meta.get("seconds", 0))
    tier = meta.get("tier", "")

    line1 = f"{run_id}  [{verdict}]  {seconds}s  {tier}"
    draw.text((8, 6), line1, fill=color, font=font)

    for i, line in enumerate(_wrapped_text_lines(meta.get("text", ""), max_lines=2)):
        draw.text((8, 6 + 20 * (i + 1)), line, fill="black", font=font)

    draw.line([(0, HEADER_H - 1), (width, HEADER_H - 1)], fill="#cccccc")


def build_sheet(run_dir: Path, meta: dict, out: Path, columns: int = 12) -> Path:
    """Build a PORTRAIT sheet: one column per camera, time flowing downward.

    ``columns`` keeps its historical name for caller compatibility (the
    battery's ``--sheet-cmd`` and older scripts) but now means the number of
    evenly sampled frames per camera column — i.e. the number of tile rows.
    """
    run_dir = Path(run_dir)
    out = Path(out)
    n_samples = columns
    width = len(ROW_LABELS) * TILE_W

    # Sample every camera first; missing cameras keep a column of grey
    # placeholder tiles so the sheet never crashes and the reviewer sees the
    # absence explicitly.
    cols = []
    for label in ROW_LABELS:
        files = _load_label_files(run_dir, label)
        tiles = []
        for f in sample_evenly(files, n_samples):
            img = Image.open(f).convert("RGB")
            w, h = img.size
            tile_h = max(1, round(TILE_W * h / w))
            img = img.resize((TILE_W, tile_h))
            tiles.append((img, _stamp_from_name(f.name)))
        cols.append({"label": label, "tiles": tiles})

    n_rows = max((len(c["tiles"]) for c in cols), default=0)
    # Per-tile-row height: the tallest tile across the cameras in that row
    # (cameras may differ in aspect ratio), plus the caption band.
    row_heights = []
    for i in range(n_rows):
        max_img_h = PLACEHOLDER_H
        for c in cols:
            if i < len(c["tiles"]):
                max_img_h = max(max_img_h, c["tiles"][i][0].height)
        row_heights.append(max_img_h + CAPTION_H)

    total_h = HEADER_H + LABEL_BAND_H + (sum(row_heights) if row_heights else PLACEHOLDER_H)
    sheet = Image.new("RGB", (width, total_h), "white")
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.load_default()

    _draw_header(draw, font, width, meta)

    # Camera-name band under the header, one label per column.
    band_y = HEADER_H
    draw.rectangle([0, band_y, width, band_y + LABEL_BAND_H], fill="#eeeeee")
    for k, c in enumerate(cols):
        draw.text((k * TILE_W + 4, band_y + 3), c["label"], fill="black", font=font)
    draw.line(
        [(0, band_y + LABEL_BAND_H - 1), (width, band_y + LABEL_BAND_H - 1)],
        fill="#cccccc",
    )

    if n_rows == 0:
        y = band_y + LABEL_BAND_H
        draw.rectangle([0, y, width, y + PLACEHOLDER_H], fill="#bdbdbd")
        draw.text(
            (8, y + PLACEHOLDER_H // 2 - 6),
            "no frames captured",
            fill="black",
            font=font,
        )
    else:
        y = band_y + LABEL_BAND_H
        for i in range(n_rows):
            row_h = row_heights[i]
            for k, c in enumerate(cols):
                x = k * TILE_W
                if i < len(c["tiles"]):
                    img, stamp = c["tiles"][i]
                    sheet.paste(img, (x, y))
                    caption_y = y + row_h - CAPTION_H
                    draw.rectangle(
                        [x, caption_y, x + TILE_W, caption_y + CAPTION_H],
                        fill="#222222",
                    )
                    draw.text(
                        (x + 4, caption_y + 2), f"t={stamp}s", fill="white", font=font
                    )
                else:
                    draw.rectangle([x, y, x + TILE_W, y + row_h], fill="#bdbdbd")
                    draw.text(
                        (x + 8, y + row_h // 2 - 6),
                        f"no {c['label']} frame",
                        fill="black",
                        font=font,
                    )
            y += row_h

    out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out, "JPEG", quality=JPEG_QUALITY)
    return out


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build a contact sheet for a GPSR run.")
    parser.add_argument("--run-dir", required=True, help="Run directory containing frames/<label>/*.jpg.")
    parser.add_argument("--meta", required=True, help="Path to run.json (id/text/template/... metadata).")
    parser.add_argument("--out", required=True, help="Output sheet JPEG path.")
    parser.add_argument(
        "--columns",
        type=int,
        default=12,
        help="Sampled frames per camera column (tile rows); historical name.",
    )
    return parser


def main(argv=None) -> int:
    args = _build_arg_parser().parse_args(argv)
    meta = json.loads(Path(args.meta).read_text())
    build_sheet(Path(args.run_dir), meta, Path(args.out), columns=args.columns)
    return 0


if __name__ == "__main__":
    sys.exit(main())
