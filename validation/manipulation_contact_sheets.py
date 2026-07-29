"""Validate manipulation visual evidence and render deterministic contact sheets.

This module deliberately has no Isaac, ROS, or runner dependencies.  A gate
attempt is a directory containing ``visual-keyframes.json`` and source PNGs;
the visual evidence writer consumes that directory after the runtime has
finished.  ``--suite-dir`` accepts a directory containing one such attempt
per manipulation gate.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from PIL import Image, ImageDraw, ImageFont, ImageOps


IMAGE_SIZE = (960, 540)
EXPECTED_CAMERAS = ("overview", "manipulation_closeup")
GATE_EVENTS: dict[str, tuple[str, ...]] = {
    "free-space-fjt": ("start", "outbound-apex", "return-arrival", "terminal"),
    "safety-stop": ("moving", "effective-stop", "velocity-compliant", "post-clear"),
    "free-gripper": ("open-start", "closed", "reopening", "open-terminal"),
    "obstructed-gripper": ("pre-close", "bilateral-contact", "stalled-result", "terminal"),
    "arm-collision": ("approach", "first-contact", "velocity-compliant", "terminal"),
    "retention": ("bilateral-grasp", "lift-threshold", "translation-threshold", "stable-hold"),
}
EXECUTION_NON_CHECKPOINT_EVENTS = frozenset(
    {
        "gate_started",
        "action_goal_sent",
        "action_goal_response",
        "action_feedback",
        "action_result",
        "action_cancel_requested",
        "safety_asserted",
        "safety_cleared",
        "executor_error",
    }
)
DEFAULT_PHYSICS_FRAME_S = 1.0 / 150.0
# This is intentionally a frame-count contract, not a configurable seconds
# timeout.  It reflects the bounded ROS/render polling handoff used by the
# producer and prevents stale images from being accepted by omission.
MAX_CAPTURE_LATENCY_FRAMES = 4
GENERATED_NAMES = {
    "contact-sheet-diagnostic.png",
    "contact-sheet-agent.png",
    "contact-sheet-user.png",
}
RESULT_NAME = "visual-evidence-result.json"
_IMAGE_KEYS = ("path", "file", "filename", "image", "image_path", "source", "png")
_FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/dejavu/DejaVuSans.ttf",
)


def _canonical_gate(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip().lower().replace("_", "-").replace(" ", "-")
    return text if text in GATE_EVENTS else text or None


def _canonical_event(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip().lower().replace("_", "-").replace(" ", "-")
    return text or None


def _canonical_camera(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip().lower().replace("-", "_").replace(" ", "_")
    if text in {"closeup", "manipulation", "manipulation_close_up"}:
        return "manipulation_closeup"
    return text or None


def _finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _integer(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    try:
        result = int(value)
    except (TypeError, ValueError):
        return None
    return result


def _atomic_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=True) + "\n").encode("utf-8")
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def _read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _find_font(size: int) -> ImageFont.ImageFont:
    for candidate in _FONT_CANDIDATES:
        if Path(candidate).is_file():
            return ImageFont.truetype(candidate, size=size)
    return ImageFont.load_default()


def _text(draw: ImageDraw.ImageDraw, xy: tuple[int, int], value: str, font: ImageFont.ImageFont, fill: Any, max_width: int) -> None:
    """Draw one line with deterministic ellipsis, never outside its cell."""
    value = " ".join(str(value).split())
    if not value:
        return
    if draw.textbbox((0, 0), value, font=font)[2] <= max_width:
        draw.text(xy, value, font=font, fill=fill)
        return
    suffix = "..."
    while value and draw.textbbox((0, 0), value + suffix, font=font)[2] > max_width:
        value = value[:-1]
    draw.text(xy, value.rstrip() + suffix, font=font, fill=fill)


def _wrap_lines(value: str, font: ImageFont.ImageFont, max_width: int, limit: int = 4) -> list[str]:
    words = str(value).split()
    lines: list[str] = []
    current = ""
    for word in words:
        candidate = f"{current} {word}".strip()
        if current and ImageDraw.Draw(Image.new("RGB", (1, 1))).textbbox((0, 0), candidate, font=font)[2] > max_width:
            lines.append(current)
            current = word
        else:
            current = candidate
    if current:
        lines.append(current)
    if len(lines) > limit:
        lines = lines[:limit]
        last = lines[-1]
        while last and ImageDraw.Draw(Image.new("RGB", (1, 1))).textbbox((0, 0), last + "...", font=font)[2] > max_width:
            last = last[:-1]
        lines[-1] = last.rstrip() + "..."
    return lines


def _status_color(status: str) -> tuple[int, int, int]:
    status = status.lower()
    if status in {"valid", "pass", "passed", "verified-pass", "success"}:
        return (55, 175, 90)
    if status in {"evidence-invalid", "invalid", "fail", "failed", "verified-fail", "error"}:
        return (220, 65, 65)
    return (224, 155, 45)


@dataclass(frozen=True)
class Keyframe:
    gate: str | None
    event: str | None
    camera: str | None
    path: Path | None
    relative_path: str
    sim_time: float | None
    raw_frame_id: int | None
    raw_frame_index: int | None
    source: Mapping[str, Any]


@dataclass(frozen=True)
class Evidence:
    root: Path
    gate: str
    keyframes_path: Path
    frames: tuple[Keyframe, ...]
    physics_frame_s: float | None
    verdict: Mapping[str, Any]
    diagnostics: tuple[dict[str, Any], ...]
    by_identity: Mapping[tuple[str, str], Keyframe]
    status: str


def _image_path(root: Path, keyframes_path: Path, value: Any) -> tuple[Path | None, str]:
    raw = str(value).strip()
    relative = raw.replace("\\", "/")
    candidate = Path(raw)
    candidates = [candidate] if candidate.is_absolute() else [keyframes_path.parent / candidate, root / candidate, root / "visual" / candidate]
    for item in candidates:
        if item.is_file():
            return item.resolve(), relative
    return None, relative


def _record_image(record: Mapping[str, Any]) -> Any:
    for key in _IMAGE_KEYS:
        if key in record and isinstance(record[key], (str, Path)):
            return record[key]
    return None


def _collect_records(node: Any, context: Mapping[str, Any] | None = None) -> list[dict[str, Any]]:
    context = dict(context or {})
    records: list[dict[str, Any]] = []
    if isinstance(node, list):
        for item in node:
            records.extend(_collect_records(item, context))
        return records
    if not isinstance(node, Mapping):
        return records
    record = dict(context)
    for key, target in (("gate", "gate"), ("gate_name", "gate"), ("event", "event"), ("camera", "camera")):
        if key in node:
            record[target] = node[key]
    if _record_image(node) is not None:
        for target in ("gate", "event", "camera"):
            if target in record and not any(key in node for key in (target, f"{target}_name")):
                record[f"__inherited_{target}"] = True
        record.update(node)
        records.append(record)
        return records
    for key, value in node.items():
        next_context = dict(record)
        canonical_key = _canonical_gate(key) or _canonical_event(key) or _canonical_camera(key)
        if canonical_key in GATE_EVENTS:
            next_context["gate"] = canonical_key
        elif canonical_key in EXPECTED_CAMERAS:
            next_context["camera"] = canonical_key
        elif canonical_key:
            next_context["event"] = canonical_key
        records.extend(_collect_records(value, next_context))
    return records


def _physics_frame(payload: Any) -> float | None:
    candidates: list[Any] = []
    if isinstance(payload, Mapping):
        for key in ("physics_frame_s", "physics_frame_period_s", "physics_dt", "physics_step_s", "frame_period_s"):
            candidates.append(payload.get(key))
        for key in ("metadata", "config", "timing"):
            nested = payload.get(key)
            if isinstance(nested, Mapping):
                candidates.extend(nested.get(name) for name in ("physics_frame_s", "physics_frame_period_s", "physics_dt", "physics_step_s"))
    for candidate in candidates:
        value = _finite_float(candidate)
        if value is not None and value > 0:
            return value
    return None


def _capture_latency_contract(payload: Any, diagnostics: list[dict[str, Any]]) -> int | None:
    contract = payload.get("capture_latency_contract") if isinstance(payload, Mapping) else None
    if not isinstance(contract, Mapping):
        diagnostics.append({"code": "missing-capture-latency-contract"})
        return None
    if contract.get("unit") != "physics_frames":
        diagnostics.append({"code": "invalid-capture-latency-unit", "actual": contract.get("unit")})
    max_frames = _integer(contract.get("max_frames"))
    if max_frames != MAX_CAPTURE_LATENCY_FRAMES:
        diagnostics.append({
            "code": "invalid-capture-latency-contract",
            "actual_max_frames": max_frames,
            "expected_max_frames": MAX_CAPTURE_LATENCY_FRAMES,
        })
        return None
    if contract.get("basis") != "raw_frame_index-requested_physics_frame_index":
        diagnostics.append({"code": "invalid-capture-latency-basis", "actual": contract.get("basis")})
    return max_frames


def _gate_from_payload(payload: Any) -> str | None:
    if isinstance(payload, Mapping):
        for key in ("gate", "gate_name", "selected_gate"):
            gate = _canonical_gate(payload.get(key))
            if gate:
                return gate
    return None


def _field(record: Mapping[str, Any], names: Sequence[str]) -> Any:
    for name in names:
        if name in record:
            return record[name]
    return None


def _normalize_frame(root: Path, keyframes_path: Path, record: Mapping[str, Any], payload_gate: str | None) -> Keyframe:
    path_value = _record_image(record)
    path, relative = _image_path(root, keyframes_path, path_value) if path_value is not None else (None, "")
    gate = _canonical_gate(_field(record, ("gate", "gate_name"))) or payload_gate
    event = _canonical_event(_field(record, ("event", "event_name")))
    camera = _canonical_camera(_field(record, ("camera", "camera_name")))
    timestamp = _field(record, ("sim_time", "simulated_time", "simulated_timestamp", "timestamp", "time_s"))
    raw_id = _field(record, ("raw_frame_id", "frame_id", "raw_id"))
    raw_index = _field(record, ("raw_frame_index", "frame_index", "raw_index", "index"))
    return Keyframe(
        gate=gate,
        event=event,
        camera=camera,
        path=path,
        relative_path=relative,
        sim_time=_finite_float(timestamp),
        raw_frame_id=_integer(raw_id),
        raw_frame_index=_integer(raw_index),
        source=record,
    )


def _load_verdict(root: Path, gate: str) -> Mapping[str, Any]:
    candidates = ("gate-verdict.json", "gate_result.json", "gate_results.json", "result.json")
    for name in candidates:
        path = root / name
        if not path.is_file():
            continue
        try:
            value = _read_json(path)
        except (OSError, ValueError, json.JSONDecodeError):
            return {"status": "evidence-invalid", "diagnostics": [f"unable to decode {name}"]}
        if not isinstance(value, Mapping):
            continue
        if gate in value and isinstance(value[gate], Mapping):
            return value[gate]
        for key in ("verdict", "gate_verdict", "result", "gates", "gate_results"):
            nested = value.get(key)
            if isinstance(nested, Mapping) and gate in nested and isinstance(nested[gate], Mapping):
                return nested[gate]
        if isinstance(value.get("verdict"), Mapping):
            return value["verdict"]
        return value
    return {"status": "unverified", "pass": False}


def _status(verdict: Mapping[str, Any]) -> str:
    raw = verdict.get("status")
    if raw is not None:
        return str(raw).lower()
    if verdict.get("pass") is True or verdict.get("verified") is True:
        return "verified-pass"
    return "verified-fail" if verdict.get("pass") is False else "unverified"


def _display_status(evidence: Evidence) -> str:
    return evidence.status if evidence.status != "valid" else _status(evidence.verdict)


def _image_stats(image: Image.Image) -> tuple[bool, dict[str, float]]:
    rgba = image.convert("RGBA")
    # A small, fixed raster is sufficient for blank/foreground detection and
    # keeps a six-gate suite practical while preserving deterministic results.
    sample_width = min(rgba.width, 240)
    sample_height = min(rgba.height, 135)
    rgba = rgba.resize((sample_width, sample_height), Image.Resampling.BILINEAR)
    pixels = list(rgba.get_flattened_data())
    if not pixels:
        return True, {"luminance_variance": 0.0, "foreground_ratio": 0.0, "opaque_ratio": 0.0}
    opaque = [pixel for pixel in pixels if pixel[3] >= 8]
    opaque_ratio = len(opaque) / len(pixels)
    if not opaque:
        return True, {"luminance_variance": 0.0, "foreground_ratio": 0.0, "opaque_ratio": opaque_ratio}
    luminance = [(0.2126 * p[0] + 0.7152 * p[1] + 0.0722 * p[2]) for p in opaque]
    mean = sum(luminance) / len(luminance)
    variance = sum((value - mean) ** 2 for value in luminance) / len(luminance)
    width, height = rgba.size
    border: list[tuple[int, int, int]] = []
    interior: list[tuple[int, int, int]] = []
    border_width = max(2, min(width, height) // 100)
    for y, pixel_row in enumerate(rgba.get_flattened_data()):
        x = y % width
        row = y // width
        rgb = pixel_row[:3]
        if x < border_width or x >= width - border_width or row < border_width or row >= height - border_width:
            border.append(rgb)
        elif pixel_row[3] >= 8:
            interior.append(rgb)
    border_mean = tuple(sum(pixel[channel] for pixel in border) / max(1, len(border)) for channel in range(3))
    different = sum(1 for pixel in interior if sum((pixel[channel] - border_mean[channel]) ** 2 for channel in range(3)) ** 0.5 >= 12.0)
    foreground_ratio = different / max(1, len(interior))
    blank = opaque_ratio < 0.98 or variance < 0.75 or foreground_ratio < 0.001
    return blank, {"luminance_variance": round(variance, 6), "foreground_ratio": round(foreground_ratio, 6), "opaque_ratio": round(opaque_ratio, 6)}


def _all_pngs(root: Path) -> set[Path]:
    return {path.resolve() for path in root.rglob("*.png") if path.name not in GENERATED_NAMES}


def _read_jsonl(path: Path, kind: str, diagnostics: list[dict[str, Any]]) -> list[Mapping[str, Any]]:
    if not path.is_file():
        diagnostics.append({"code": f"missing-{kind}-journal", "path": str(path)})
        return []
    records: list[Mapping[str, Any]] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        diagnostics.append({"code": f"unreadable-{kind}-journal", "path": str(path), "detail": str(error)})
        return []
    for line_number, line in enumerate(lines, 1):
        if not line.strip():
            diagnostics.append({"code": f"blank-{kind}-journal-record", "line": line_number})
            continue
        try:
            value = json.loads(line)
        except (ValueError, json.JSONDecodeError) as error:
            diagnostics.append({"code": f"corrupt-{kind}-journal-record", "line": line_number, "detail": str(error)})
            continue
        if not isinstance(value, Mapping):
            diagnostics.append({"code": f"invalid-{kind}-journal-record", "line": line_number})
            continue
        records.append(value)
    if not records:
        diagnostics.append({"code": f"empty-{kind}-journal", "path": str(path)})
    return records


def _journal_sequence(record: Mapping[str, Any]) -> int | None:
    return _integer(record.get("sequence"))


def _journal_event(record: Mapping[str, Any]) -> str | None:
    value = record.get("event")
    return value if isinstance(value, str) and value else None


def _journal_timestamp(record: Mapping[str, Any]) -> float | None:
    return _finite_float(_field(record, ("simulated_timestamp", "requested_simulated_timestamp", "requested_timestamp")))


def _validate_journals(
    root: Path,
    gate: str,
    expected_events: Sequence[str],
    diagnostics: list[dict[str, Any]],
) -> tuple[dict[str, Mapping[str, Any]], dict[str, Mapping[str, Any]]]:
    execution_path = root / "gate-execution.jsonl"
    request_path = root / "visual-capture-requests.jsonl"
    execution_records = _read_jsonl(execution_path, "gate-execution", diagnostics)
    request_records = _read_jsonl(request_path, "visual-capture-request", diagnostics)
    execution_by_event: dict[str, Mapping[str, Any]] = {}
    previous_sequence = 0
    checkpoint_events: list[str] = []
    for index, record in enumerate(execution_records):
        event = _journal_event(record)
        sequence = _journal_sequence(record)
        if record.get("gate") != gate:
            diagnostics.append({"code": "execution-journal-gate-mismatch", "record": index, "actual": record.get("gate"), "expected": gate})
        if sequence is None or sequence <= 0:
            diagnostics.append({"code": "invalid-execution-journal-sequence", "record": index})
        elif sequence <= previous_sequence:
            diagnostics.append({"code": "reordered-execution-journal", "record": index, "sequence": sequence})
        previous_sequence = max(previous_sequence, sequence or 0)
        if event is None:
            diagnostics.append({"code": "missing-execution-journal-event", "record": index})
        elif event in expected_events:
            checkpoint_events.append(event)
            if event in execution_by_event:
                diagnostics.append({"code": "duplicate-execution-checkpoint", "event": event})
            else:
                execution_by_event[event] = record
            if _journal_timestamp(record) is None:
                diagnostics.append({"code": "missing-execution-checkpoint-timestamp", "event": event})
        elif event not in EXECUTION_NON_CHECKPOINT_EVENTS:
            diagnostics.append({"code": "extra-execution-checkpoint", "record": index, "event": event})
    if checkpoint_events != list(expected_events):
        diagnostics.append({"code": "execution-checkpoint-order-mismatch", "actual": checkpoint_events, "expected": list(expected_events)})
    if len(checkpoint_events) != len(expected_events):
        diagnostics.append({"code": "execution-checkpoint-count-mismatch", "actual": len(checkpoint_events), "expected": len(expected_events)})

    request_by_event: dict[str, Mapping[str, Any]] = {}
    previous_request_sequence = 0
    request_events: list[str] = []
    for index, record in enumerate(request_records):
        event = _journal_event(record)
        sequence = _journal_sequence(record)
        if record.get("gate") != gate:
            diagnostics.append({"code": "visual-request-journal-gate-mismatch", "record": index, "actual": record.get("gate"), "expected": gate})
        if sequence is None or sequence <= 0:
            diagnostics.append({"code": "invalid-visual-request-sequence", "record": index})
        elif sequence != previous_request_sequence + 1:
            diagnostics.append({"code": "reordered-visual-request-journal", "record": index, "sequence": sequence})
        previous_request_sequence = sequence or previous_request_sequence
        if event not in expected_events:
            diagnostics.append({"code": "extra-visual-request", "record": index, "event": event})
            continue
        request_events.append(event)
        if event in request_by_event:
            diagnostics.append({"code": "duplicate-visual-request", "event": event})
        else:
            request_by_event[event] = record
        if _journal_timestamp(record) is None:
            diagnostics.append({"code": "missing-visual-request-timestamp", "event": event})
        source_sequence = _integer(record.get("source_execution_event_sequence"))
        execution = execution_by_event.get(event)
        if execution is not None and source_sequence != _journal_sequence(execution):
            diagnostics.append({"code": "visual-request-execution-binding-mismatch", "event": event})
    if request_events != list(expected_events):
        diagnostics.append({"code": "visual-request-order-mismatch", "actual": request_events, "expected": list(expected_events)})
    if len(request_events) != len(expected_events):
        diagnostics.append({"code": "visual-request-count-mismatch", "actual": len(request_events), "expected": len(expected_events)})
    return execution_by_event, request_by_event


def validate_attempt(attempt_dir: Path, gate: str | None = None) -> Evidence:
    root = attempt_dir.resolve()
    diagnostics: list[dict[str, Any]] = []
    keyframes_path = root / "visual-keyframes.json"
    payload: Any = None
    if not keyframes_path.is_file():
        diagnostics.append({"code": "missing-keyframes", "path": str(keyframes_path)})
        payload = {}
    else:
        try:
            payload = _read_json(keyframes_path)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            diagnostics.append({"code": "corrupt-keyframes", "detail": str(error)})
            payload = {}
    inferred_gate = _canonical_gate(gate) or _gate_from_payload(payload)
    records = [_normalize_frame(root, keyframes_path, record, _gate_from_payload(payload)) for record in _collect_records(payload)]
    gates = {frame.gate for frame in records if frame.gate}
    if inferred_gate is None and len(gates) == 1:
        inferred_gate = next(iter(gates))
    if inferred_gate is None:
        diagnostics.append({"code": "missing-gate"})
        inferred_gate = "unknown"
    if inferred_gate not in GATE_EVENTS:
        diagnostics.append({"code": "unknown-gate", "gate": inferred_gate})
        expected_events: tuple[str, ...] = ()
    else:
        expected_events = GATE_EVENTS[inferred_gate]
    if any(item != inferred_gate for item in gates):
        diagnostics.append({"code": "gate-identity-mismatch", "gates": sorted(str(item) for item in gates)})
    physics_frame_s = _physics_frame(payload)
    if physics_frame_s is None:
        diagnostics.append({"code": "missing-physics-frame", "detail": "visual-keyframes.json must configure physics_frame_s"})
    max_capture_latency_frames = _capture_latency_contract(payload, diagnostics)
    by_identity: dict[tuple[str, str], Keyframe] = {}
    execution_by_event, request_by_event = _validate_journals(root, inferred_gate, expected_events, diagnostics)
    referenced: set[Path] = set()
    for number, frame in enumerate(records):
        identity = (frame.event or "", frame.camera or "")
        raw_keyframe_gate = _field(frame.source, ("gate", "gate_name"))
        raw_keyframe_event = _field(frame.source, ("event", "event_name"))
        if raw_keyframe_gate != inferred_gate or frame.source.get("__inherited_gate"):
            diagnostics.append({"code": "keyframe-gate-journal-mismatch", "record": number, "actual": raw_keyframe_gate, "expected": inferred_gate})
        if raw_keyframe_event != frame.event or frame.source.get("__inherited_event"):
            diagnostics.append({"code": "noncanonical-keyframe-event", "record": number, "actual": raw_keyframe_event, "expected": frame.event})
        if frame.gate != inferred_gate:
            diagnostics.append({"code": "gate-identity-mismatch", "record": number, "actual": frame.gate, "expected": inferred_gate})
        if frame.event not in expected_events:
            diagnostics.append({"code": "unexpected-event", "record": number, "event": frame.event})
        if frame.camera not in EXPECTED_CAMERAS:
            diagnostics.append({"code": "unexpected-camera", "record": number, "camera": frame.camera})
        if identity in by_identity:
            diagnostics.append({"code": "duplicate-keyframe", "event": frame.event, "camera": frame.camera})
        else:
            by_identity[identity] = frame
        if frame.sim_time is None:
            diagnostics.append({"code": "invalid-simulated-timestamp", "record": number})
        requested_time = _finite_float(_field(frame.source, ("requested_simulated_timestamp", "requested_timestamp")))
        if requested_time is None:
            diagnostics.append({"code": "missing-requested-simulated-timestamp", "record": number})
        elif frame.sim_time is not None and physics_frame_s is not None and max_capture_latency_frames is not None:
            skew = abs(frame.sim_time - requested_time)
            limit_s = max_capture_latency_frames * physics_frame_s
            if skew > limit_s + 1e-12:
                diagnostics.append({"code": "capture-timestamp-skew", "record": number, "skew_s": skew, "limit_s": limit_s, "limit_frames": max_capture_latency_frames})
        requested_frame_index = _integer(frame.source.get("requested_physics_frame_index"))
        capture_latency_frames = _integer(frame.source.get("capture_latency_frames"))
        declared_max_frames = _integer(frame.source.get("max_capture_latency_frames"))
        if requested_frame_index is None:
            diagnostics.append({"code": "missing-requested-physics-frame-index", "record": number})
        if capture_latency_frames is None:
            diagnostics.append({"code": "missing-capture-latency-frames", "record": number})
        if declared_max_frames != max_capture_latency_frames:
            diagnostics.append({"code": "keyframe-capture-latency-contract-mismatch", "record": number, "actual": declared_max_frames, "expected": max_capture_latency_frames})
        if requested_frame_index is not None and capture_latency_frames is not None and frame.raw_frame_index is not None:
            derived_latency = frame.raw_frame_index - requested_frame_index
            if capture_latency_frames != derived_latency:
                diagnostics.append({"code": "capture-latency-frame-mismatch", "record": number, "declared": capture_latency_frames, "derived": derived_latency})
        if capture_latency_frames is not None and max_capture_latency_frames is not None and not 0 <= capture_latency_frames <= max_capture_latency_frames:
            diagnostics.append({"code": "capture-latency-out-of-bounds", "record": number, "latency_frames": capture_latency_frames, "max_frames": max_capture_latency_frames})
        request_sequence = _integer(_field(frame.source, ("request_sequence", "visual_request_sequence")))
        execution_sequence = _integer(_field(frame.source, ("execution_event_sequence", "source_execution_event_sequence")))
        if request_sequence is None or request_sequence <= 0:
            diagnostics.append({"code": "missing-visual-request-sequence", "record": number})
        if execution_sequence is None or execution_sequence <= 0:
            diagnostics.append({"code": "missing-execution-event-sequence", "record": number})
        expected_execution = execution_by_event.get(frame.event or "")
        expected_request = request_by_event.get(frame.event or "")
        if expected_execution is None or expected_request is None:
            diagnostics.append({"code": "unbound-keyframe-journal-event", "record": number, "event": frame.event})
        else:
            expected_execution_sequence = _journal_sequence(expected_execution)
            expected_request_sequence = _journal_sequence(expected_request)
            expected_timestamp = _journal_timestamp(expected_execution)
            request_timestamp = _journal_timestamp(expected_request)
            requested_fields = _finite_float(_field(frame.source, ("requested_simulated_timestamp", "requested_timestamp")))
            if execution_sequence != expected_execution_sequence:
                diagnostics.append({"code": "keyframe-execution-journal-mismatch", "record": number, "event": frame.event})
            if request_sequence != expected_request_sequence:
                diagnostics.append({"code": "keyframe-request-journal-mismatch", "record": number, "event": frame.event})
            if requested_fields != expected_timestamp or requested_fields != request_timestamp:
                diagnostics.append({"code": "keyframe-requested-timestamp-journal-mismatch", "record": number, "event": frame.event})
        if frame.raw_frame_id is None and frame.raw_frame_index is None:
            diagnostics.append({"code": "missing-raw-frame-id-index", "record": number})
        if frame.path is None:
            diagnostics.append({"code": "missing-source-image", "record": number, "path": frame.relative_path})
            continue
        referenced.add(frame.path)
        try:
            with Image.open(frame.path) as image:
                image.load()
                if image.size != IMAGE_SIZE:
                    diagnostics.append({"code": "invalid-dimensions", "path": frame.relative_path, "actual": list(image.size), "expected": list(IMAGE_SIZE)})
                if image.mode not in {"RGB", "RGBA"}:
                    diagnostics.append({"code": "invalid-mode", "path": frame.relative_path, "actual": image.mode})
                blank, stats = _image_stats(image)
                if blank:
                    diagnostics.append({"code": "blank-or-transparent", "path": frame.relative_path, "stats": stats})
        except (OSError, ValueError, SyntaxError) as error:
            diagnostics.append({"code": "corrupt-image", "path": frame.relative_path, "detail": str(error)})
    for path in sorted(_all_pngs(root) - referenced):
        diagnostics.append({"code": "unindexed-source-image", "path": path.relative_to(root).as_posix()})
    expected_identities = {(event, camera) for event in expected_events for camera in EXPECTED_CAMERAS}
    for identity in sorted(expected_identities - set(by_identity)):
        diagnostics.append({"code": "missing-keyframe", "event": identity[0], "camera": identity[1]})
    valid_frames = [frame for frame in records if frame.sim_time is not None]
    for camera in EXPECTED_CAMERAS:
        times = [frame.sim_time for frame in valid_frames if frame.camera == camera]
        if any(right < left for left, right in zip(times, times[1:])):
            diagnostics.append({"code": "non-monotonic-simulated-timestamps", "camera": camera})
    if physics_frame_s is not None:
        for event in expected_events:
            times = [by_identity[(event, camera)].sim_time for camera in EXPECTED_CAMERAS if (event, camera) in by_identity and by_identity[(event, camera)].sim_time is not None]
            if len(times) == 2 and abs(times[0] - times[1]) > physics_frame_s + 1e-12:
                diagnostics.append({"code": "timestamp-skew", "event": event, "skew_s": abs(times[0] - times[1]), "limit_s": physics_frame_s})
    verdict = _load_verdict(root, inferred_gate)
    for camera in EXPECTED_CAMERAS:
        camera_frames = [frame for frame in records if frame.camera == camera]
        for field_name, values in (
            ("raw_frame_id", [frame.raw_frame_id for frame in camera_frames]),
            ("raw_frame_index", [frame.raw_frame_index for frame in camera_frames]),
        ):
            present = [value for value in values if value is not None]
            if present and any(right <= left for left, right in zip(present, present[1:])):
                diagnostics.append({"code": "stale-or-regressing-raw-frame", "camera": camera, "field": field_name})
    request_order: list[int] = []
    execution_order: list[int] = []
    for event in expected_events:
        event_frames = [by_identity.get((event, camera)) for camera in EXPECTED_CAMERAS]
        event_frames = [frame for frame in event_frames if frame is not None]
        request_values = [_integer(_field(frame.source, ("request_sequence", "visual_request_sequence"))) for frame in event_frames]
        execution_values = [_integer(_field(frame.source, ("execution_event_sequence", "source_execution_event_sequence"))) for frame in event_frames]
        if len(set(value for value in request_values if value is not None)) > 1:
            diagnostics.append({"code": "visual-request-sequence-mismatch", "event": event})
        if len(set(value for value in execution_values if value is not None)) > 1:
            diagnostics.append({"code": "execution-event-sequence-mismatch", "event": event})
        if request_values and request_values[0] is not None:
            request_order.append(request_values[0])
        if execution_values and execution_values[0] is not None:
            execution_order.append(execution_values[0])
    if any(right <= left for left, right in zip(request_order, request_order[1:])):
        diagnostics.append({"code": "non-monotonic-visual-request-sequence"})
    if any(right <= left for left, right in zip(execution_order, execution_order[1:])):
        diagnostics.append({"code": "non-monotonic-execution-event-sequence"})
    return Evidence(root, inferred_gate, keyframes_path, tuple(records), physics_frame_s, verdict, tuple(diagnostics), by_identity, "valid" if not diagnostics else "evidence-invalid")


def _fit_source(image: Image.Image | None, size: tuple[int, int]) -> Image.Image:
    canvas = Image.new("RGB", size, (18, 22, 28))
    if image is None:
        return canvas
    image = image.convert("RGB")
    fitted = ImageOps.contain(image, size)
    canvas.paste(fitted, ((size[0] - fitted.width) // 2, (size[1] - fitted.height) // 2))
    return canvas


def _open_frame(evidence: Evidence, event: str, camera: str) -> Image.Image | None:
    frame = evidence.by_identity.get((event, camera))
    if frame is None or frame.path is None:
        return None
    try:
        with Image.open(frame.path) as image:
            image.load()
            return image.copy()
    except (OSError, ValueError, SyntaxError):
        return None


def _draw_cell(draw: ImageDraw.ImageDraw, image: Image.Image | None, box: tuple[int, int, int, int], title: str, status: str, font: ImageFont.ImageFont, small: ImageFont.ImageFont, detail: str = "") -> None:
    x, y, width, height = box
    image_box = (x, y + 22, width, height - 22)
    # Kept as a small drawing primitive for callers that already pasted the
    # fitted image; `_paste_cell` is used by the public renderers.
    draw.rectangle((x, y, x + width - 1, y + height - 1), outline=_status_color(status), width=4)
    _text(draw, (x + 6, y + 3), title, small, (245, 245, 245), width - 12)
    if detail:
        _text(draw, (x + 6, y + height - 18), detail, small, (225, 225, 225), width - 12)


def _paste_cell(canvas: Image.Image, image: Image.Image | None, box: tuple[int, int, int, int], title: str, status: str, font: ImageFont.ImageFont, small: ImageFont.ImageFont, detail: str = "") -> None:
    x, y, width, height = box
    canvas.paste(_fit_source(image, (width, height - 22)), (x, y + 22))
    draw = ImageDraw.Draw(canvas)
    draw.rectangle((x, y, x + width - 1, y + height - 1), outline=_status_color(status), width=4)
    _text(draw, (x + 6, y + 3), title, font, (245, 245, 245), width - 12)
    if detail:
        _text(draw, (x + 6, y + height - 18), detail, small, (225, 225, 225), width - 12)


def render_gate_sheet(evidence: Evidence) -> Path:
    events = GATE_EVENTS.get(evidence.gate, ())
    cell_width, cell_height = 480, 306
    margin, label_width, header_height = 16, 170, 36
    canvas = Image.new("RGB", (margin * 2 + label_width + cell_width * 4, header_height + cell_height * 2 + margin * 2), (10, 14, 20))
    draw = ImageDraw.Draw(canvas)
    title_font, label_font, small_font = _find_font(24), _find_font(18), _find_font(14)
    display_status = _display_status(evidence)
    _text(draw, (margin, 8), f"{evidence.gate}  |  {display_status.upper()}", title_font, (245, 245, 245), canvas.width - margin * 2)
    for row, camera in enumerate(EXPECTED_CAMERAS):
        y = header_height + margin + row * cell_height
        _text(draw, (margin, y + 8), camera, label_font, (225, 225, 225), label_width - 10)
        for column, event in enumerate(events):
            x = margin + label_width + column * cell_width
            frame = evidence.by_identity.get((event, camera))
            detail = "missing" if frame is None else f"t={frame.sim_time:g}" if frame.sim_time is not None else "invalid time"
            _paste_cell(canvas, _open_frame(evidence, event, camera), (x, y, cell_width, cell_height), event, display_status, title_font, small_font, detail)
    path = evidence.root / "contact-sheet-diagnostic.png"
    canvas.save(path, format="PNG", optimize=False, compress_level=9)
    return path


def _metrics_text(evidence: Evidence) -> str:
    verdict = evidence.verdict
    metrics = verdict.get("metrics") if isinstance(verdict.get("metrics"), Mapping) else {}
    pieces = [f"status: {_display_status(evidence)}", f"evidence: {evidence.status}"]
    for key in sorted(metrics)[:6]:
        value = metrics[key]
        if isinstance(value, (str, int, float, bool)):
            pieces.append(f"{key}: {value}")
    if evidence.diagnostics:
        pieces.append(f"diagnostics: {len(evidence.diagnostics)}")
    return " | ".join(pieces)


def _suite_candidates(root: Path) -> dict[str, Path]:
    found: dict[str, Path] = {}
    for keyframes in sorted(root.rglob("visual-keyframes.json")):
        if keyframes.parent == root:
            continue
        try:
            payload = _read_json(keyframes)
        except (OSError, ValueError, json.JSONDecodeError):
            payload = {}
        gate = _gate_from_payload(payload)
        if gate is None:
            records = _collect_records(payload)
            gates = {_canonical_gate(item.get("gate")) for item in records if isinstance(item, Mapping)}
            gates.discard(None)
            if len(gates) == 1:
                gate = next(iter(gates))
        if gate is None:
            gate = _canonical_gate(keyframes.parent.name)
        if gate in GATE_EVENTS and gate not in found:
            found[gate] = keyframes.parent
    return found


def _suite_sheet(evidences: Mapping[str, Evidence], user: bool, output_root: Path) -> Path:
    gates = tuple(GATE_EVENTS)
    cell_width, cell_height = (300, 202) if user else (300, 220)
    label_width, panel_width, margin, header = 190, 390, 16, 48
    width = margin * 2 + label_width + cell_width * 4 + (0 if user else panel_width)
    height = header + margin + len(gates) * (cell_height + margin)
    canvas = Image.new("RGB", (width, height), (10, 14, 20))
    draw = ImageDraw.Draw(canvas)
    title_font, label_font, small_font = _find_font(26), _find_font(18), _find_font(13)
    _text(draw, (margin, 10), "Tinker manipulation visual evidence", title_font, (245, 245, 245), width - margin * 2)
    primary_camera = {"free-space-fjt": "overview", "safety-stop": "overview"}
    for row, gate in enumerate(gates):
        evidence = evidences.get(gate)
        y = header + margin + row * (cell_height + margin)
        status = _display_status(evidence) if evidence else "evidence-invalid"
        draw.rectangle((margin, y, width - margin - 1, y + cell_height - 1), outline=_status_color(status), width=3)
        _text(draw, (margin + 8, y + 8), gate, label_font, (245, 245, 245), label_width - 16)
        _text(draw, (margin + 8, y + 34), status.upper(), small_font, _status_color(status), label_width - 16)
        if evidence is None:
            events = GATE_EVENTS[gate]
            for column, event in enumerate(events):
                _paste_cell(canvas, None, (margin + label_width + column * cell_width, y, cell_width, cell_height), event, status, label_font, small_font, "missing gate")
            continue
        camera = primary_camera.get(gate, "manipulation_closeup")
        for column, event in enumerate(GATE_EVENTS[gate]):
            x = margin + label_width + column * cell_width
            frame = evidence.by_identity.get((event, camera))
            detail = "missing" if frame is None else (f"frame {frame.raw_frame_id if frame.raw_frame_id is not None else frame.raw_frame_index}")
            _paste_cell(canvas, _open_frame(evidence, event, camera), (x, y, cell_width, cell_height), event, status, label_font, small_font, detail)
        if not user:
            panel_x = margin + label_width + cell_width * 4
            draw.rectangle((panel_x, y, panel_x + panel_width - 1, y + cell_height - 1), fill=(25, 31, 40))
            lines = _wrap_lines(_metrics_text(evidence), small_font, panel_width - 20, 8)
            for index, line in enumerate(lines):
                _text(draw, (panel_x + 10, y + 10 + index * 21), line, small_font, (235, 235, 235), panel_width - 20)
            draw.rectangle((panel_x, y, panel_x + panel_width - 1, y + cell_height - 1), outline=_status_color(status), width=3)
    path = output_root / ("contact-sheet-user.png" if user else "contact-sheet-agent.png")
    canvas.save(path, format="PNG", optimize=False, compress_level=9)
    return path


def process_attempt(attempt_dir: Path, gate: str | None = None) -> dict[str, Any]:
    evidence = validate_attempt(attempt_dir, gate)
    generated: list[Path] = []
    diagnostics = list(evidence.diagnostics)
    try:
        generated.append(render_gate_sheet(evidence))
    except (OSError, ValueError) as error:
        diagnostics.append({"code": "sheet-generation-failed", "detail": str(error)})
    result = {
        "schema_version": 1,
        "mode": "attempt",
        "attempt_dir": str(evidence.root),
        "gate": evidence.gate,
        "status": "valid" if not diagnostics else "evidence-invalid",
        "diagnostics": diagnostics,
    }
    _atomic_json(evidence.root / RESULT_NAME, result)
    return result


def process_suite(suite_dir: Path) -> dict[str, Any]:
    root = suite_dir.resolve()
    candidates = _suite_candidates(root)
    evidences: dict[str, Evidence] = {}
    diagnostics: list[dict[str, Any]] = []
    for gate in GATE_EVENTS:
        if gate not in candidates:
            diagnostics.append({"code": "missing-gate-attempt", "gate": gate})
            continue
        evidences[gate] = validate_attempt(candidates[gate], gate)
        diagnostics.extend({"gate": gate, **item} for item in evidences[gate].diagnostics)
    generated: list[Path] = []
    try:
        generated.extend([_suite_sheet(evidences, user=False, output_root=root), _suite_sheet(evidences, user=True, output_root=root)])
    except (OSError, ValueError) as error:
        diagnostics.append({"code": "sheet-generation-failed", "detail": str(error)})
    result = {
        "schema_version": 1,
        "mode": "suite",
        "suite_dir": str(root),
        "status": "valid" if not diagnostics and len(evidences) == len(GATE_EVENTS) else "evidence-invalid",
        "diagnostics": diagnostics,
        "gates": {gate: {"status": evidence.status, "diagnostic_count": len(evidence.diagnostics)} for gate, evidence in sorted(evidences.items())},
    }
    _atomic_json(root / RESULT_NAME, result)
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--attempt-dir", type=Path)
    group.add_argument("--suite-dir", type=Path)
    parser.add_argument("--gate", choices=tuple(GATE_EVENTS), help="optional gate identity for attempt mode")
    args = parser.parse_args(argv)
    if args.suite_dir is not None and args.gate:
        parser.error("--gate is only valid with --attempt-dir")
    result = process_attempt(args.attempt_dir, args.gate) if args.attempt_dir else process_suite(args.suite_dir)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "valid" else 1


if __name__ == "__main__":
    raise SystemExit(main())
