#!/usr/bin/env python3
"""Contact-sheet builder for GPSR tier-2 runs.

Primary layout is EVENT-DRIVEN: each row is one completed milestone BT
node (a NAV/VISION/AUDIO/MANIP action reported by the run's telemetry --
see ``tools/sheet_events.py``), paired with the arena/head frames closest
to when it finished and a label block describing it. This makes the
sheet a legible narrative of what the robot actually did, in the order it
did it, rather than an arbitrary time strip.

When a run has no telemetry (older runs, or telemetry absent/corrupt),
the sheet degrades to the original PORTRAIT time-strip layout: a header
band summarising the run, a label band naming the camera columns, then
one column per camera label with an evenly sampled run of frames flowing
top-to-bottom, each tile captioned with its timestamp. That fallback
layout, and its behaviour, is unchanged from before this module gained
event-driven rows.

Pure PIL, no ROS. Never raises: any absent/corrupt input degrades event
layout -> fallback layout -> placeholder tiles, never a crash.
"""
from __future__ import annotations

import argparse
import datetime
import json
import sys
import textwrap
from pathlib import Path
from typing import Optional, Sequence

from PIL import Image, ImageDraw, ImageFont

# `tools` has no __init__.py -- it works as a namespace package when the
# repo root is on sys.path (which pytest arranges for `from tools.contact_sheet
# import ...`), but running this file directly as `python3 tools/contact_sheet.py`
# only puts tools/ itself on sys.path[0]. Add the repo root explicitly so both
# invocation styles resolve the sibling `tools.sheet_events` module.
_REPO_ROOT = str(Path(__file__).resolve().parent.parent)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from tools import sheet_events  # noqa: E402

TILE_W = 320
CAPTION_H = 18
HEADER_H = 88
LABEL_BAND_H = 18
PLACEHOLDER_H = 40
JPEG_QUALITY = 80

# --- event-driven layout constants --------------------------------------
EVENT_ROW_CAP = 40
LABEL_BLOCK_W = TILE_W
LABEL_LINE_H = 14
LABEL_PAD = 6
LABEL_WRAP_WIDTH = 46
INFO_MAX_LINES = 4
KIND_COLORS = {
    "NAV": "#1565c0",
    "VISION": "#6a1b9a",
    "AUDIO": "#2e7d32",
    "MANIP": "#ef6c00",
}
DEFAULT_KIND_COLOR = "#212121"
FAILURE_TINT = "#ffebee"
EVENT_COL_LABELS = ("arena", "head", "event")
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


def select_event_rows(milestones: Sequence, cap: int = EVENT_ROW_CAP) -> list:
    """Return up to `cap` milestones, evenly sampled (order-preserving).

    Pure passthrough to `sample_evenly` under a name that documents intent
    at the call site: this is the row-count cap for the event-driven sheet
    layout, not a generic sampling utility.
    """
    return sample_evenly(list(milestones), cap)


def _stamp_seconds(name: str) -> Optional[float]:
    """Parse the `_<ms>` suffix from a `<seq>_<ms>.jpg` frame filename into
    seconds. Never raises: returns None on an unparseable name.
    """
    stem = name.rsplit(".", 1)[0]
    ms_str = stem.split("_")[-1]
    try:
        return int(ms_str) / 1000.0
    except ValueError:
        return None


def _stamp_from_name(name: str) -> str:
    """Parse the `_<ms>` suffix from a `<seq>_<ms>.jpg` frame filename into a
    seconds string. Never raises: a file that doesn't match `<seq>_<ms>.jpg`
    (unparseable trailing `_<ms>`) degrades to "?" rather than crashing the
    builder, matching C3's "never a crash" promise -- honoured elsewhere for
    missing rows, but this parse had no guard.
    """
    seconds = _stamp_seconds(name)
    if seconds is None:
        return "?"
    text = f"{seconds:.3f}".rstrip("0").rstrip(".")
    return text if text else "0"


def _parse_iso(value) -> Optional[datetime.datetime]:
    """Parse an ISO-8601 wall-clock string into a tz-aware datetime.

    Never raises: None/non-str/unparseable input yields None. Handles the
    trailing "Z" UTC suffix that `datetime.fromisoformat` only accepts
    natively from Python 3.11 -- this codebase runs on 3.10.
    """
    if not isinstance(value, str) or not value:
        return None
    text = value[:-1] + "+00:00" if value.endswith("Z") else value
    try:
        return datetime.datetime.fromisoformat(text)
    except ValueError:
        return None


def _round_seconds(value) -> int:
    try:
        return round(float(value))
    except (TypeError, ValueError):
        return 0


def _wrapped_text_lines(text: str, max_lines: int, width: int = TEXT_WRAP_WIDTH) -> list:
    lines = textwrap.wrap(str(text), width=width) or [""]
    return lines[:max_lines]


def _load_label_files(run_dir: Path, label: str) -> list:
    frames_dir = run_dir / "frames" / label
    if not frames_dir.is_dir():
        return []
    return sorted(frames_dir.glob("*.jpg"))


def _load_recorder_meta(run_dir: Path) -> dict:
    path = run_dir / "recorder-meta.json"
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def _load_frame_index(run_dir: Path) -> dict:
    """Read frames/index.jsonl into {label: [{"wall_dt": datetime, "file": str}, ...]}.

    Lines with a null/unparseable "wall" are unusable for exact matching
    and are skipped (interpolation is the fallback for those). Never
    raises: an absent/corrupt index yields {}.
    """
    index_path = run_dir / "frames" / "index.jsonl"
    by_label: dict = {}
    try:
        text = index_path.read_text()
    except OSError:
        return by_label
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            entry = json.loads(line)
        except ValueError:
            continue
        if not isinstance(entry, dict):
            continue
        label = entry.get("label")
        file_ = entry.get("file")
        wall_dt = _parse_iso(entry.get("wall"))
        if not label or not file_ or wall_dt is None:
            continue
        by_label.setdefault(label, []).append({"wall_dt": wall_dt, "file": file_})
    return by_label


def _pick_frame_from_index(run_dir: Path, entries: list, event_dt: datetime.datetime) -> Optional[Path]:
    if not entries or event_dt is None:
        return None
    best = min(entries, key=lambda e: abs((e["wall_dt"] - event_dt).total_seconds()))
    path = run_dir / best["file"]
    return path if path.is_file() else None


def _interpolate_frame(
    run_dir: Path, label: str, event_dt: datetime.datetime, recorder_meta: dict
) -> Optional[Path]:
    """Fallback frame pick when frames/index.jsonl is missing/unusable.

    stamp ~= first_stamp + (wall_event - started_wall)/(ended_wall -
    started_wall) * (last_stamp - first_stamp), clamped to [0, 1] before
    scaling, then nearest frame file by its filename stamp.
    """
    if event_dt is None:
        return None
    labels = recorder_meta.get("labels")
    if not isinstance(labels, dict):
        return None
    label_meta = labels.get(label)
    if not isinstance(label_meta, dict):
        return None
    first_stamp = label_meta.get("first_stamp")
    last_stamp = label_meta.get("last_stamp")
    started_wall = _parse_iso(recorder_meta.get("started_wall"))
    ended_wall = _parse_iso(recorder_meta.get("ended_wall"))
    if not isinstance(first_stamp, (int, float)) or not isinstance(last_stamp, (int, float)):
        return None
    if started_wall is None or ended_wall is None:
        return None
    total = (ended_wall - started_wall).total_seconds()
    if total <= 0:
        return None
    frac = (event_dt - started_wall).total_seconds() / total
    frac = max(0.0, min(1.0, frac))
    target_stamp = first_stamp + frac * (last_stamp - first_stamp)

    scored = [(f, _stamp_seconds(f.name)) for f in _load_label_files(run_dir, label)]
    scored = [(f, s) for f, s in scored if s is not None]
    if not scored:
        return None
    return min(scored, key=lambda fs: abs(fs[1] - target_stamp))[0]


def _select_event_frame(
    run_dir: Path, label: str, event_dt: datetime.datetime, index_by_label: dict, recorder_meta: dict
) -> Optional[Path]:
    entries = index_by_label.get(label)
    if entries:
        picked = _pick_frame_from_index(run_dir, entries, event_dt)
        if picked is not None:
            return picked
    return _interpolate_frame(run_dir, label, event_dt, recorder_meta)


def _load_event_tile(
    run_dir: Path, label: str, event_dt: datetime.datetime, index_by_label: dict, recorder_meta: dict
) -> Optional[Image.Image]:
    path = _select_event_frame(run_dir, label, event_dt, index_by_label, recorder_meta)
    if path is None:
        return None
    try:
        img = Image.open(path).convert("RGB")
    except (OSError, ValueError):
        return None
    w, h = img.size
    if w <= 0 or h <= 0:
        return None
    tile_h = max(1, round(TILE_W * h / w))
    return img.resize((TILE_W, tile_h))


def _label_block_height(info_line_count: int) -> int:
    return LABEL_PAD * 2 + LABEL_LINE_H * (2 + info_line_count)


def _draw_label_block(draw: ImageDraw.ImageDraw, font, x: int, y: int, h: int, event) -> None:
    if event.status == "FAILURE":
        draw.rectangle([x, y, x + LABEL_BLOCK_W, y + h], fill=FAILURE_TINT)
    dt = _parse_iso(event.wall)
    time_str = dt.strftime("%H:%M:%S") if dt is not None else "??:??:??"
    kind_color = KIND_COLORS.get(event.kind, DEFAULT_KIND_COLOR)
    draw.text((x + LABEL_PAD, y + LABEL_PAD), f"{time_str}  {event.kind}", fill=kind_color, font=font)
    draw.text(
        (x + LABEL_PAD, y + LABEL_PAD + LABEL_LINE_H),
        f"{event.name}  {event.status}",
        fill="black",
        font=font,
    )
    info_lines = (
        _wrapped_text_lines(event.info, INFO_MAX_LINES, width=LABEL_WRAP_WIDTH) if event.info else []
    )
    for i, line in enumerate(info_lines):
        draw.text(
            (x + LABEL_PAD, y + LABEL_PAD + LABEL_LINE_H * (2 + i)),
            line,
            fill="#333333",
            font=font,
        )


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
    """Build a GPSR run contact sheet. Signature is unchanged for callers.

    Event-driven layout (one row per completed milestone BT node, each with
    its nearest arena/head frames and a label block) when the run has
    telemetry; degrades to the original PORTRAIT time-strip layout
    otherwise. Never raises: any failure while building the event layout
    falls back to the time-strip layout rather than propagating.
    """
    run_dir = Path(run_dir)
    out = Path(out)

    milestones, _judge_events, _tmeta = sheet_events.load_run_telemetry(run_dir)
    if milestones:
        try:
            return _build_event_sheet(run_dir, meta, out, milestones)
        except Exception:
            # Never crash the bench: fall through to the time-strip layout.
            pass

    return _build_fallback_sheet(run_dir, meta, out, columns)


def _build_event_sheet(run_dir: Path, meta: dict, out: Path, milestones: list) -> Path:
    """Build the event-driven layout: one row per milestone, 3 tiles wide
    (arena frame | head frame | label block), 960px total.
    """
    width = len(EVENT_COL_LABELS) * TILE_W
    index_by_label = _load_frame_index(run_dir)
    recorder_meta = _load_recorder_meta(run_dir)

    rows = []
    for event in select_event_rows(milestones, EVENT_ROW_CAP):
        event_dt = _parse_iso(event.wall)
        tiles = {
            label: _load_event_tile(run_dir, label, event_dt, index_by_label, recorder_meta)
            for label in ROW_LABELS
        }
        info_lines = (
            _wrapped_text_lines(event.info, INFO_MAX_LINES, width=LABEL_WRAP_WIDTH) if event.info else []
        )
        row_h = max(_label_block_height(len(info_lines)), PLACEHOLDER_H)
        for label in ROW_LABELS:
            tile = tiles[label]
            if tile is not None:
                row_h = max(row_h, tile.height)
        rows.append({"event": event, "tiles": tiles, "info_lines": info_lines, "row_h": row_h})

    total_h = HEADER_H + LABEL_BAND_H + (sum(r["row_h"] for r in rows) if rows else PLACEHOLDER_H)
    sheet = Image.new("RGB", (width, total_h), "white")
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.load_default()

    _draw_header(draw, font, width, meta)

    band_y = HEADER_H
    draw.rectangle([0, band_y, width, band_y + LABEL_BAND_H], fill="#eeeeee")
    for k, label in enumerate(EVENT_COL_LABELS):
        draw.text((k * TILE_W + 4, band_y + 3), label, fill="black", font=font)
    draw.line(
        [(0, band_y + LABEL_BAND_H - 1), (width, band_y + LABEL_BAND_H - 1)],
        fill="#cccccc",
    )

    y = band_y + LABEL_BAND_H
    if not rows:
        draw.rectangle([0, y, width, y + PLACEHOLDER_H], fill="#bdbdbd")
        draw.text((8, y + PLACEHOLDER_H // 2 - 6), "no milestones recorded", fill="black", font=font)
    else:
        for row in rows:
            row_h = row["row_h"]
            for k, label in enumerate(ROW_LABELS):
                x = k * TILE_W
                tile = row["tiles"][label]
                if tile is not None:
                    sheet.paste(tile, (x, y))
                    if tile.height < row_h:
                        draw.rectangle([x, y + tile.height, x + TILE_W, y + row_h], fill="#fafafa")
                else:
                    draw.rectangle([x, y, x + TILE_W, y + row_h], fill="#bdbdbd")
                    draw.text(
                        (x + 8, y + row_h // 2 - 6),
                        f"no {label} frame",
                        fill="black",
                        font=font,
                    )
            _draw_label_block(draw, font, len(ROW_LABELS) * TILE_W, y, row_h, row["event"])
            y += row_h

    out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out, "JPEG", quality=JPEG_QUALITY)
    return out


def _build_fallback_sheet(run_dir: Path, meta: dict, out: Path, columns: int = 12) -> Path:
    """Build a PORTRAIT sheet: one column per camera, time flowing downward.

    ``columns`` keeps its historical name for caller compatibility (the
    battery's ``--sheet-cmd`` and older scripts) but now means the number of
    evenly sampled frames per camera column — i.e. the number of tile rows.
    """
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
