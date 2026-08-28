#!/usr/bin/env python3
"""Telemetry event extraction for event-driven GPSR contact sheets.

Reads the ``events.jsonl`` telemetry stream a GPSR run's orchestrator
writes to ``<run_dir>/debug/gpsr-*/events.jsonl`` and extracts two
timelines from it:

- milestone events: the robot's own actions (nav / vision / audio /
  manipulation) that a contact sheet would want to caption frames with.
- judge events: the tree's precondition/postcondition/supervisor/replan/
  correction bookkeeping, useful for explaining *why* a run failed.

Pure stdlib, no ROS. Never raises on absent or corrupt input -- callers
(the contact-sheet builder) always get back a valid, possibly empty,
result.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional


@dataclass
class MilestoneEvent:
    wall: str  # occurred_at ISO timestamp
    kind: str  # "NAV" | "VISION" | "AUDIO" | "MANIP"
    name: str  # human node name from tree.generated
    status: str  # "SUCCESS" | "FAILURE"
    info: str  # feedback, trimmed


@dataclass
class JudgeEvent:
    wall: str
    kind: str  # "PRECONDITION" | "POSTCONDITION" | "SUPERVISOR" | "REPLAN" | "CORRECTION"
    name: str
    status: str
    info: str


_VISION_PREFIXES = (
    "scan ",
    "scan to",
    "vlm count",
    "count detections",
    "detect ",
    "track ",
)
_AUDIO_PREFIXES = ("announce", "listen", "say ")
_MANIP_SUBSTRINGS = ("arm", "grasp", "tuck", "gripper", "place")

_ANNOUNCE_PREFIX = "Finished announcing "

_TERMINAL_STATUSES = ("SUCCESS", "FAILURE")


def _classify_milestone(name: str) -> Optional[str]:
    low = name.lower()
    if "keepalive" in low:
        return None
    if low.startswith("goto target"):
        return "NAV"
    if low.startswith(_VISION_PREFIXES) or low == "turn pantilt":
        return "VISION"
    if low.startswith(_AUDIO_PREFIXES):
        return "AUDIO"
    if any(sub in low for sub in _MANIP_SUBSTRINGS):
        return "MANIP"
    return None


def _classify_judge(name: str) -> Optional[str]:
    low = name.lower()
    if low.startswith("precondition gate"):
        return "PRECONDITION"
    if low.startswith("postcondition gate"):
        return "POSTCONDITION"
    if low.startswith("supervisor barrier"):
        return "SUPERVISOR"
    if "correction" in low and low != "reset correction":
        return "CORRECTION"
    return None


def _milestone_info(kind: str, feedback: str) -> str:
    text = (feedback or "").strip()
    if kind == "AUDIO":
        if text.startswith(_ANNOUNCE_PREFIX):
            text = text[len(_ANNOUNCE_PREFIX):].strip()
        if text.endswith("."):
            text = text[:-1]
    return text


def _newest_events_file(run_dir: Path) -> Optional[Path]:
    debug_dir = run_dir / "debug"
    if not debug_dir.is_dir():
        return None
    try:
        candidates = sorted(p for p in debug_dir.glob("gpsr-*") if p.is_dir())
    except OSError:
        return None
    if not candidates:
        return None
    events_file = candidates[-1] / "events.jsonl"
    if not events_file.is_file():
        return None
    return events_file


def load_run_telemetry(run_dir: Path) -> tuple[list[MilestoneEvent], list[JudgeEvent], dict]:
    """Extract milestone/judge events and run meta from a GPSR run's telemetry.

    Returns (milestones, judge_events, meta) where meta has keys
    "trajectory_id" (str|None), "tree_revisions" (int, max tree_revision
    seen across tree.generated events), and "run_finished" (the payload
    dict of the run.finished event, or {} if none seen).

    Never raises: any absence or corruption of the telemetry (no debug
    dir, no gpsr-* subdir, no events.jsonl, unparseable lines) yields
    ([], [], {}). Individual bad lines within an otherwise-good file are
    skipped rather than aborting the whole file.
    """
    empty: tuple[list[MilestoneEvent], list[JudgeEvent], dict] = ([], [], {})

    try:
        events_file = _newest_events_file(Path(run_dir))
    except OSError:
        return empty
    if events_file is None:
        return empty

    try:
        raw_text = events_file.read_text()
    except OSError:
        return empty

    milestones: list[MilestoneEvent] = []
    judge_events: list[JudgeEvent] = []
    name_map: dict[str, str] = {}
    trajectory_id: Optional[str] = None
    max_revision = 0
    run_finished: dict[str, Any] = {}

    for line in raw_text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            event = json.loads(line)
        except (json.JSONDecodeError, ValueError):
            continue
        if not isinstance(event, dict):
            continue

        if trajectory_id is None:
            tid = event.get("trajectory_id")
            if isinstance(tid, str):
                trajectory_id = tid

        event_type = event.get("event_type")
        occurred_at = event.get("occurred_at")
        payload = event.get("payload")
        if not isinstance(payload, dict):
            payload = {}

        if event_type == "tree.generated":
            nodes = payload.get("nodes")
            if isinstance(nodes, list):
                for node in nodes:
                    if not isinstance(node, dict):
                        continue
                    node_id = node.get("id") or node.get("node_id")
                    node_name = node.get("name")
                    if node_id and node_name:
                        name_map[node_id] = node_name

            revision = payload.get("tree_revision")
            if isinstance(revision, int):
                if revision > max_revision:
                    max_revision = revision
                if revision > 0:
                    judge_events.append(
                        JudgeEvent(
                            wall=occurred_at,
                            kind="REPLAN",
                            name="replan",
                            status="SUCCESS",
                            info=f"tree revision {revision}",
                        )
                    )
            continue

        if event_type == "tree.node_states_changed":
            nodes = payload.get("nodes")
            if not isinstance(nodes, list):
                continue
            for node in nodes:
                if not isinstance(node, dict):
                    continue
                status = node.get("status")
                if status not in _TERMINAL_STATUSES:
                    continue
                node_id = node.get("id") or node.get("node_id")
                name = name_map.get(node_id) if node_id else None
                if not name:
                    continue
                feedback = node.get("feedback") or ""

                mkind = _classify_milestone(name)
                if mkind is not None:
                    milestones.append(
                        MilestoneEvent(
                            wall=occurred_at,
                            kind=mkind,
                            name=name,
                            status=status,
                            info=_milestone_info(mkind, feedback),
                        )
                    )
                    continue

                jkind = _classify_judge(name)
                if jkind is not None:
                    judge_events.append(
                        JudgeEvent(
                            wall=occurred_at,
                            kind=jkind,
                            name=name,
                            status=status,
                            info=feedback.strip(),
                        )
                    )
            continue

        if event_type == "run.finished":
            run_finished = payload
            continue

    meta = {
        "trajectory_id": trajectory_id,
        "tree_revisions": max_revision,
        "run_finished": run_finished,
    }
    return milestones, judge_events, meta
