"""Integrated visual contact sheets (ROS-free, Python 3.12).

Renders deterministic ``contact-sheet-integrated-agent.png`` /
``contact-sheet-integrated-user.png`` from the integrated qualification captures.

Authorization contract
----------------------
- Source captures must already be represented by exact path+digest+event/frame
  metadata in ``suite_dir/evidence-index.json`` (the index used to authorize a
  sheet).  ``build_contact_sheet`` reads that index and raises ``ValueError``
  for blank, transparent, unindexed, stale/mismatched, missing, or unbound
  captures.
- Source images are selected from the bound keyframe/index entries under
  ``visual/source/*.png``; the CLI never reconstructs ``captures/{event}.png``.
- Every visual event binds to exact scenario/attempt/execution-request plus
  ``(frame_index, timestamp)`` metadata from the index.  PlanningScene/action/
  screenshots are diagnostic only and are never physical pass authority.
- Deterministic semantic metadata (role, event list, source capture records,
  explicit reviewed state) is embedded in each PNG's text chunks so Gate F can
  verify sheet semantics from the sheet bytes themselves.
- Path traversal, symlink escape, duplicate canonical paths/events,
  output-as-input, and files changing during rendering are rejected.
- Rendering is deterministic: identical inputs produce identical PNG bytes.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import os
import sys
import tempfile
from pathlib import Path
from typing import Any, Mapping, Sequence

from PIL import Image, ImageDraw, ImageFont, ImageOps, PngImagePlugin

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from validation.integrated_evidence_index import (  # noqa: E402
    CANCEL_EVENTS,
    INDEX_NAME,
    REQUIRED_POSITIVE_EVENTS,
    SAFETY_EVENTS,
    SUMMARY_NAME,
)

AGENT_NAME = "contact-sheet-integrated-agent.png"
USER_NAME = "contact-sheet-integrated-user.png"
IMAGE_SIZE = (960, 540)
REPORT_REVISION = "2026-08-04"
_FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/dejavu/DejaVuSans.ttf",
)

#: Recognized output names that a sheet must never overwrite.
_PROTECTED_OUTPUT_NAMES = {INDEX_NAME, SUMMARY_NAME, AGENT_NAME, USER_NAME}


def _find_font(size: int) -> ImageFont.ImageFont:
    for candidate in _FONT_CANDIDATES:
        if Path(candidate).is_file():
            return ImageFont.truetype(candidate, size=size)
    return ImageFont.load_default()


def _image_stats(data: bytes) -> dict[str, float] | None:
    """Return blank/transparent statistics for exact PNG bytes, or None if corrupt."""
    try:
        with Image.open(io.BytesIO(data)) as image:
            image.load()
    except Exception:
        return None
    rgba = image.convert("RGBA")
    sample_width = min(rgba.width, 240)
    sample_height = min(rgba.height, 135)
    rgba = rgba.resize((sample_width, sample_height), Image.Resampling.BILINEAR)
    pixels = list(rgba.get_flattened_data())
    if not pixels:
        return {"opaque_ratio": 0.0, "luminance_variance": 0.0, "foreground_ratio": 0.0, "blank": True}
    opaque = [pixel for pixel in pixels if pixel[3] >= 8]
    opaque_ratio = len(opaque) / len(pixels)
    if not opaque:
        return {"opaque_ratio": opaque_ratio, "luminance_variance": 0.0, "foreground_ratio": 0.0, "blank": True}
    luminance = [0.2126 * p[0] + 0.7152 * p[1] + 0.0722 * p[2] for p in opaque]
    mean = sum(luminance) / len(luminance)
    variance = sum((value - mean) ** 2 for value in luminance) / len(luminance)
    width, height = rgba.size
    border: list[tuple[int, int, int]] = []
    interior: list[tuple[int, int, int]] = []
    border_width = max(2, min(width, height) // 100)
    for index, pixel in enumerate(pixels):
        if pixel[3] < 8:
            continue
        x = index % width
        y = index // width
        rgb = pixel[:3]
        if x < border_width or x >= width - border_width or y < border_width or y >= height - border_width:
            border.append(rgb)
        else:
            interior.append(rgb)
    border_mean = tuple(sum(pixel[channel] for pixel in border) / max(1, len(border)) for channel in range(3)) if border else (0, 0, 0)
    different = sum(1 for pixel in interior if sum((pixel[channel] - border_mean[channel]) ** 2 for channel in range(3)) ** 0.5 >= 12.0)
    foreground_ratio = different / max(1, len(interior))
    blank = opaque_ratio < 0.98 or variance < 0.75 or foreground_ratio < 0.001
    return {
        "opaque_ratio": round(opaque_ratio, 6),
        "luminance_variance": round(variance, 6),
        "foreground_ratio": round(foreground_ratio, 6),
        "blank": blank,
    }


def _atomic_png(path: Path, image: Image.Image, pnginfo: PngImagePlugin.PngInfo | None = None) -> None:
    """Atomically write a PNG (fsync file and parent directory)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as stream:
            image.save(stream, format="PNG", optimize=False, compress_level=9, pnginfo=pnginfo)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        dir_fd = os.open(str(path.parent), os.O_DIRECTORY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
    except BaseException:
        try:
            os.close(fd)
        except OSError:
            pass
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _load_index(suite_dir: Path) -> dict[str, Any]:
    index_path = suite_dir / INDEX_NAME
    if not index_path.is_file():
        raise ValueError(f"missing evidence index: {index_path}")
    try:
        index = json.loads(index_path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as error:
        raise ValueError(f"corrupt evidence index: {index_path}: {error}")
    if not isinstance(index, Mapping):
        raise ValueError(f"invalid evidence index: {index_path}")
    return index


def _index_by_path(index: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    files = index.get("files")
    if not isinstance(files, list):
        return {}
    by_path: dict[str, Mapping[str, Any]] = {}
    for entry in files:
        if isinstance(entry, Mapping) and isinstance(entry.get("path"), str):
            if entry["path"] in by_path:
                raise ValueError(f"duplicate canonical path in index: {entry['path']}")
            by_path[entry["path"]] = entry
    return by_path


def _canonical_metadata(rows: Sequence[Mapping[str, Any]]) -> Mapping[str, Any]:
    """Deterministic embedded sheet metadata (F1.6)."""
    return {
        "schema_version": 1,
        "report_revision": REPORT_REVISION,
        "role": rows[0]["role"] if rows else None,
        "diagnostic_only": True,
        "reviewed": False,
        "events": [row["event"] for row in rows],
        "captures": [
            {
                "path": row["rel"],
                "sha256": row["sha256"],
                "scenario": row["scenario"],
                "attempt": row["attempt"],
                "execution_request": row["execution_request"],
                "frame_index": row["frame_index"],
                "timestamp": row["timestamp"],
                "camera": row["camera"],
                "event": row["event"],
            }
            for row in rows
        ],
        "captures_sha256": [row["sha256"] for row in rows],
    }


def _render_sheet(rows: Sequence[Mapping[str, Any]], *, user: bool) -> Image.Image:
    """Deterministic grid render; identical inputs -> identical bytes."""
    count = len(rows)
    cell_width, cell_height = 320, 240
    label_width, header_height, margin = 180, 64, 16
    footer_height = 96
    width = margin * 2 + label_width + cell_width * count
    height = header_height + margin + cell_height + footer_height
    canvas = Image.new("RGB", (width, height), (10, 14, 20))
    draw = ImageDraw.Draw(canvas)
    title_font = _find_font(24)
    small_font = _find_font(14)
    draw.text(
        (margin, 12),
        f"Tinker integrated manipulation visual evidence - {'user' if user else 'agent'}",
        font=title_font,
        fill=(245, 245, 245),
    )
    y = header_height + margin
    for index, row in enumerate(rows):
        x = margin + label_width + index * cell_width
        image = row["image"].convert("RGB")
        fitted = ImageOps.contain(image, (cell_width - 8, cell_height - 40))
        canvas.paste(fitted, (x + (cell_width - fitted.width) // 2, y + 40 + (cell_height - 40 - fitted.height) // 2))
        draw.rectangle((x, y, x + cell_width - 1, y + cell_height - 1), outline=(55, 175, 90), width=3)
        draw.text((x + 6, y + 6), str(row["event"]), font=small_font, fill=(245, 245, 245))
        detail = f"frame {row['frame_index']}  t={float(row['timestamp']):g}"
        draw.text((x + 6, y + cell_height - 24), detail, font=small_font, fill=(225, 225, 225))
        draw.text((x + 6, y + 24), f"{row['scenario']} / {row['attempt']}", font=small_font, fill=(200, 200, 200))
    draw.text(
        (margin, y + cell_height + 12),
        "diagnostic only - event metadata bound from evidence-index.json (never physical pass authority)",
        font=small_font,
        fill=(225, 225, 225),
    )
    return canvas


def build_contact_sheet(
    suite_dir: Path,
    image_paths: Sequence[Path],
    output: Path,
    *,
    user: bool = False,
) -> dict[str, Any]:
    """Render one deterministic integrated contact sheet from indexed captures.

    Every capture must already be represented by exact path+digest+event/frame
    metadata in ``suite_dir/evidence-index.json``.  Blank/transparent, unindexed,
    stale/mismatched, missing, unbound, out-of-suite, duplicate, and
    output-as-input inputs fail closed with ``ValueError``.
    """
    suite_resolved = Path(suite_dir).resolve()
    output_resolved = Path(output).resolve()
    index = _load_index(suite_resolved)
    by_path = _index_by_path(index)

    try:
        rel_output = output_resolved.relative_to(suite_resolved).as_posix()
    except ValueError:
        rel_output = None
    if rel_output is not None:
        name = rel_output.rsplit("/", 1)[-1]
        expected = USER_NAME if user else AGENT_NAME
        if name == expected:
            # F2.9: a sheet may always regenerate its own expected output path.
            pass
        elif name in _PROTECTED_OUTPUT_NAMES:
            # F2.9: the sibling sheet, the index, or the summary are protected.
            raise ValueError(f"output-as-input: refusing to overwrite protected artifact {rel_output}")
        else:
            # F3.7: reject output equal to ANY indexed artifact (journal,
            # manifest, verdict, raw/evaluator/drain, rosbag file, capture,
            # metadata) -- never only captures.  Rejected before any overwrite.
            indexed = by_path.get(rel_output)
            if indexed is not None:
                raise ValueError(
                    f"output-as-input: refusing to overwrite indexed evidence artifact {rel_output}"
                )

    rows: list[dict[str, Any]] = []
    seen_paths: set[str] = set()
    seen_events: set[str] = set()
    for raw in image_paths:
        image_path = Path(raw).resolve()
        try:
            rel = image_path.relative_to(suite_resolved).as_posix()
        except ValueError:
            raise ValueError(f"path traversal or symlink escape outside suite: {image_path}")
        if rel in seen_paths:
            raise ValueError(f"duplicate capture path: {rel}")
        seen_paths.add(rel)
        if output_resolved == image_path:
            raise ValueError(f"output-as-input: refusing to overwrite source capture {rel}")
        if rel in (INDEX_NAME, SUMMARY_NAME):
            raise ValueError(f"output-as-input: refusing to overwrite protected artifact {rel}")
        entry = by_path.get(rel)
        if entry is None:
            raise ValueError(f"unindexed capture: {rel} is not present in {INDEX_NAME}")
        event = entry.get("event")
        if not entry.get("bound") or not isinstance(event, str) or not event:
            raise ValueError(f"capture missing bound event metadata: {rel}")
        if event in seen_events:
            raise ValueError(f"duplicate event binding: {event}")
        seen_events.add(event)
        scenario = entry.get("scenario")
        attempt = entry.get("attempt")
        if not scenario or not attempt:
            raise ValueError(f"capture missing scenario/attempt binding: {rel}")
        for required_field in ("execution_request", "frame_index", "timestamp"):
            if entry.get(required_field) is None:
                raise ValueError(f"capture missing {required_field} binding: {rel}")
        if entry.get("physics_bound") is not True:
            raise ValueError(f"capture missing physics cross-bind: {rel}")
        data = Path(image_path).read_bytes()
        stats = _image_stats(data)
        if stats is None:
            raise ValueError(f"corrupt or invalid image capture: {rel}")
        if stats["blank"]:
            raise ValueError(f"blank or transparent capture: {rel}")
        digest = hashlib.sha256(data).hexdigest()
        if digest != entry.get("sha256"):
            raise ValueError(f"stale or mismatched capture (digest differs from index): {rel}")
        image = Image.open(io.BytesIO(data))
        image.load()
        rows.append(
            {
                "image": image,
                "event": event,
                "scenario": scenario,
                "attempt": attempt,
                "execution_request": entry.get("execution_request"),
                "frame_index": entry.get("frame_index"),
                "timestamp": entry.get("timestamp"),
                "camera": entry.get("camera"),
                "sha256": digest,
                "rel": rel,
                "role": "user" if user else "agent",
            }
        )
    if not rows:
        raise ValueError("contact sheet requires at least one bound capture")

    canvas = _render_sheet(rows, user=user)
    metadata = _canonical_metadata(rows)
    pnginfo = PngImagePlugin.PngInfo()
    pnginfo.add_text("tinker.qualification.metadata", json.dumps(metadata, sort_keys=True))
    _atomic_png(output_resolved, canvas, pnginfo=pnginfo)
    return {
        "schema_version": 1,
        "kind": "integrated-contact-sheet",
        "role": "user" if user else "agent",
        "output": str(output_resolved),
        "scenario": rows[0]["scenario"],
        "attempt": rows[0]["attempt"],
        "events": [row["event"] for row in rows],
        "paths": [row["rel"] for row in rows],
        "captures_sha256": [row["sha256"] for row in rows],
        "diagnostic_only": True,
        "reviewed": False,
        "report_revision": REPORT_REVISION,
    }


def _read_sheet_metadata(path: Path) -> Mapping[str, Any] | None:
    """Read embedded deterministic metadata from a sheet PNG (F1.6)."""
    try:
        with Image.open(path) as image:
            text = image.text
    except Exception:
        return None
    value = text.get("tinker.qualification.metadata") if isinstance(text, Mapping) else None
    if not isinstance(value, str):
        return None
    try:
        parsed = json.loads(value)
    except (ValueError, json.JSONDecodeError):
        return None
    if not isinstance(parsed, Mapping):
        return None
    return parsed


def _validate_sheet_image(path: Path) -> tuple[bool, str]:
    """Validate a sheet PNG is a real RGB/RGBA image with sane dimensions."""
    try:
        with Image.open(path) as image:
            image.load()
            if image.mode not in ("RGB", "RGBA"):
                return False, f"unsupported mode {image.mode}"
            if image.width < 32 or image.height < 32:
                return False, "dimensions too small"
            stats = _image_stats(path.read_bytes())
            if stats is None:
                return False, "corrupt PNG"
            if stats["blank"]:
                return False, "blank or transparent"
    except Exception as error:
        return False, str(error)
    return True, ""


def _canonical_suite_event_order(index: Mapping[str, Any]) -> list[str]:
    """Return the canonical required suite event sequence for the present kinds.

    Mirrors the Gate-F validator's required order:
    ``REQUIRED_POSITIVE_EVENTS + CANCEL_EVENTS + SAFETY_EVENTS`` for the kinds
    present in the suite (F4.3).  A production CLI sheet embeds exactly this
    ordered event sequence so ``validate_gate_f`` accepts a real generated sheet.
    """
    kinds = set(index.get("scenario_kinds") or [])
    order: list[str] = []
    if "positive" in kinds:
        order.extend(REQUIRED_POSITIVE_EVENTS)
    if "cancel" in kinds:
        order.extend(CANCEL_EVENTS)
    if "safety" in kinds:
        order.extend(SAFETY_EVENTS)
    return order


def _all_bound_capture_entries(suite_dir: Path) -> list[dict[str, Any]]:
    """Select one bound capture per visual event in the canonical suite order.

    Bound captures are ordered by the required suite event sequence
    (positive -> cancel -> safety), not by index path order; within an event the
    deterministic camera/path ordering is preserved.  Unknown visual event
    identities (not part of the canonical required suite for the present kinds)
    are rejected instead of silently placed, and an event whose bound captures
    carry conflicting scenario/attempt identities is rejected as a duplicate
    event identity.
    """
    index = _load_index(suite_dir)
    order = _canonical_suite_event_order(index)
    known = set(order)
    entries: list[dict[str, Any]] = []
    by_event: dict[str, list[dict[str, Any]]] = {}
    for entry in index.get("files", []):
        if (
            isinstance(entry, Mapping)
            and entry.get("category") == "capture"
            and entry.get("bound")
            and isinstance(entry.get("event"), str)
        ):
            event = entry["event"]
            if event not in known:
                raise ValueError(f"unknown visual event identity in evidence index: {event!r}")
            by_event.setdefault(event, []).append(dict(entry))
    for event in order:
        captures = by_event.get(event, [])
        if not captures:
            continue
        scenarios = {capture.get("scenario") for capture in captures}
        attempts = {capture.get("attempt") for capture in captures}
        if len(scenarios) != 1 or len(attempts) != 1:
            raise ValueError(
                f"duplicate visual event identity in evidence index: {event!r} "
                f"is bound to multiple scenario/attempt transactions"
            )
        # Deterministic camera/path ordering within the event.
        captures.sort(key=lambda capture: (capture.get("camera") or "", capture["path"]))
        entries.append(captures[0])
    return entries


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite-dir", required=True, type=Path)
    parser.add_argument("--agent", type=Path, default=None, help="agent sheet output path")
    parser.add_argument("--user", type=Path, default=None, help="user sheet output path")
    parser.add_argument("--events", nargs="*", default=None, help="subset of events (default: all bound events)")
    args = parser.parse_args(argv)
    suite_dir = Path(args.suite_dir).resolve()
    events = set(args.events) if args.events is not None else None
    entries = _all_bound_capture_entries(suite_dir)
    if events is not None:
        entries = [entry for entry in entries if entry["event"] in events]
    if not entries:
        raise SystemExit("no bound capture events found in evidence index")
    paths = [suite_dir / entry["path"] for entry in entries]
    agent_output = Path(args.agent) if args.agent else suite_dir / AGENT_NAME
    user_output = Path(args.user) if args.user else suite_dir / USER_NAME
    if args.agent or args.user:
        if args.agent:
            build_contact_sheet(suite_dir, paths, output=agent_output)
        if args.user:
            build_contact_sheet(suite_dir, paths, output=user_output, user=True)
    else:
        build_contact_sheet(suite_dir, paths, output=agent_output)
        build_contact_sheet(suite_dir, paths, output=user_output, user=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
