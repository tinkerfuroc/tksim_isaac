#!/usr/bin/env python3
"""Contact-sheet builder for GPSR tier-2 runs.

Condenses the JPEG frames a run recorder captured (see
``validation/gpsr_run_recorder.py``) into one reviewable JPEG: a header band
summarising the run, then one row per camera label showing an evenly sampled
strip of frames with per-tile timestamp captions.

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
PLACEHOLDER_H = 40
JPEG_QUALITY = 80
ROW_LABELS = ("arena", "head")
TEXT_WRAP_WIDTH = 110

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
    """Parse the `_<ms>` suffix from a `<seq>_<ms>.jpg` frame filename into a seconds string."""
    stem = name.rsplit(".", 1)[0]
    ms_str = stem.split("_")[-1]
    seconds = int(ms_str) / 1000.0
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
    run_dir = Path(run_dir)
    out = Path(out)
    width = columns * TILE_W

    rows = []
    total_h = HEADER_H
    for label in ROW_LABELS:
        files = _load_label_files(run_dir, label)
        if not files:
            rows.append({"label": label, "kind": "placeholder", "height": PLACEHOLDER_H})
            total_h += PLACEHOLDER_H
            continue

        tiles = []
        max_img_h = 0
        for f in sample_evenly(files, columns):
            img = Image.open(f).convert("RGB")
            w, h = img.size
            tile_h = max(1, round(TILE_W * h / w))
            img = img.resize((TILE_W, tile_h))
            tiles.append((img, _stamp_from_name(f.name)))
            max_img_h = max(max_img_h, tile_h)

        row_h = max_img_h + CAPTION_H
        rows.append({"label": label, "kind": "frames", "tiles": tiles, "height": row_h})
        total_h += row_h

    sheet = Image.new("RGB", (width, total_h), "white")
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.load_default()

    _draw_header(draw, font, width, meta)

    y = HEADER_H
    for row in rows:
        if row["kind"] == "placeholder":
            draw.rectangle([0, y, width, y + row["height"]], fill="#bdbdbd")
            draw.text(
                (8, y + row["height"] // 2 - 6),
                f"no {row['label']} frames captured",
                fill="black",
                font=font,
            )
            y += row["height"]
            continue

        x = 0
        for img, stamp in row["tiles"]:
            sheet.paste(img, (x, y))
            caption_y = y + row["height"] - CAPTION_H
            draw.rectangle([x, caption_y, x + TILE_W, caption_y + CAPTION_H], fill="#222222")
            draw.text((x + 4, caption_y + 2), f"t={stamp}s", fill="white", font=font)
            x += TILE_W
        y += row["height"]

    out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out, "JPEG", quality=JPEG_QUALITY)
    return out


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build a contact sheet for a GPSR run.")
    parser.add_argument("--run-dir", required=True, help="Run directory containing frames/<label>/*.jpg.")
    parser.add_argument("--meta", required=True, help="Path to run.json (id/text/template/... metadata).")
    parser.add_argument("--out", required=True, help="Output sheet JPEG path.")
    parser.add_argument("--columns", type=int, default=12, help="Tiles per row.")
    return parser


def main(argv=None) -> int:
    args = _build_arg_parser().parse_args(argv)
    meta = json.loads(Path(args.meta).read_text())
    build_sheet(Path(args.run_dir), meta, Path(args.out), columns=args.columns)
    return 0


if __name__ == "__main__":
    sys.exit(main())
