"""Task 9 fix-round-1 integrated evidence-index tests (ROS-free, Python 3.12).

This module defines the production-shaped evidence-suite factory
(``write_canonical_evidence_tree`` / ``make_complete_evidence_suite``) that
mirrors the exact current Task 2-8 producer schemas, the capture-path selector
(``required_capture_paths``), and mutation-driven tests for
``validation.integrated_evidence_index``.

The factory writes real preserved artifact bytes (executor journals, capture
process keyframes under ``visual/source/``, Gate-B source-lock/static outputs,
gate verdicts, rosbag2 metadata, resource-cleanup evidence) and then derives
``evidence-index.json`` from those exact bytes via ``build_evidence_index``; it
never fabricates a verdict and never defines a parallel schema.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import re
import sys
from pathlib import Path

import pytest
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from validation.integrated_evidence_index import (  # noqa: E402
    INDEX_NAME,
    SUMMARY_NAME,
    REQUIRED_POSITIVE_EVENTS,
    CANCEL_EVENTS,
    SAFETY_EVENTS,
    canonical_sha256,
    build_evidence_index,
    build_qualification_summary,
    validate_gate_f,
)
from validation.integrated_contact_sheets import (  # noqa: E402
    AGENT_NAME,
    USER_NAME,
    build_contact_sheet,
    _read_sheet_metadata,
)

POSITIVE_EVENTS = tuple(REQUIRED_POSITIVE_EVENTS)
ALL_EVENTS = tuple(
    dict.fromkeys(POSITIVE_EVENTS + CANCEL_EVENTS + SAFETY_EVENTS)
)
IMAGE_SIZE = (960, 540)
CAMERAS = ("overview", "manipulation_closeup")
PHYSICS_HZ = 120.0
PHYSICS_FRAMES = 80
# F4.1: real capture latency in frames (raw = requested + CAPTURE_LATENCY).
CAPTURE_LATENCY = 2
SUITE_ATTEMPT = "suite-20260804T000000Z-1000-abcdef0123"
ATTEMPT_IDS = {
    "qualification-pick-place-positive": "attempt-positive",
    "qualification-pick-place-cancel-approach": "attempt-cancel",
    "qualification-pick-place-safety-transport": "attempt-safety",
}
SIM_COMMIT = "a" * 40
PROD_COMMIT = "b" * 40
DIGEST_RE = re.compile(r"^[0-9a-f]{64}$")
POSITIVE_ID = "qualification-pick-place-positive"
CANCEL_ID = "qualification-pick-place-cancel-approach"
CANCEL_TRANSPORT_ID = "qualification-pick-place-cancel-transport"
SAFETY_ID = "qualification-pick-place-safety-transport"
MOVEIT_SAFETY_ID = "qualification-moveit-safety"
MISSING_FP = "0" * 64

# Real RTX GPU inventory emitted by ``_gpu_processes`` (schema_version 2): the
# ``gpus`` field is the full nvidia-smi inventory, never a survivor list.
GPU_INVENTORY = [
    {"index": 0, "uuid": "gpu-6f1a-0000-4a2b-0000", "memory_used_mib": 2048},
    {"index": 1, "uuid": "gpu-6f1a-1111-4a2b-1111", "memory_used_mib": 1024},
]


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n"
    path.write_text(encoded, encoding="utf-8")


def _write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _write_bytes(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def _nonblank_image(path: Path, seed: int) -> None:
    """Deterministic valid nonblank RGB PNG."""
    path.parent.mkdir(parents=True, exist_ok=True)
    image = Image.new("RGB", IMAGE_SIZE, (25 + seed % 40, 45 + seed % 40, 75 + seed % 40))
    draw = ImageDraw.Draw(image)
    draw.rectangle((120, 80, 800, 430), fill=(180, 80 + seed % 100, 35))
    draw.line((0, seed % 10 + 10, 959, 500 - seed % 10), fill=(240, 240, 240), width=5)
    image.save(path, format="PNG", optimize=False, compress_level=9)


def _raw_frame(index: int, scenario_id: str) -> dict[str, object]:
    """Real raw physics truth frame schema (Task-8 fixture shape)."""
    return {
        "schema_version": 2,
        "frame_index": index,
        "timestamp": index / PHYSICS_HZ,
        "scenario": scenario_id,
        "task": scenario_id,
        "robot": {
            "tcp_pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
            "base_twist": {"linear": [0.0, 0.0, 0.0], "angular": [0.0, 0.0, 0.0]},
            "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
            "joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "joint_velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "joint_efforts": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "safety_stop": False,
        },
        "command_targets": {
            "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
            "joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "joint_velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "joint_efforts": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "snapshot_id": index,
            "gripper_effort_limit": 10.0,
        },
        "physics_device": "cpu",
        "seed": 7,
        "contacts": [],
        "contact_state": {},
        "contact_pairs": [],
        "expected_objects": {"qualification_cube": {"id": "qualification_cube"}},
        "objects": [{"id": "qualification_cube", "pose": None, "twist": None}],
        "object": {"id": "qualification_cube", "pose": None, "twist": None},
        "safety_stop": False,
        "actuator_limits": {"drive_joint": 10.0},
        "command_gateway": {
            "last_command_error": None,
            "command_stream_lost": False,
            "active_epoch": 1,
            "last_snapshot_id": None,
        },
    }


def _scenario_events(scenario_id: str) -> tuple[str, ...]:
    if scenario_id == POSITIVE_ID:
        return POSITIVE_EVENTS
    if scenario_id in (CANCEL_ID, CANCEL_TRANSPORT_ID):
        return CANCEL_EVENTS
    if scenario_id in (SAFETY_ID, MOVEIT_SAFETY_ID):
        return SAFETY_EVENTS
    return POSITIVE_EVENTS


def _scenario_integrated(scenario_id: str) -> dict[str, object]:
    if scenario_id == POSITIVE_ID:
        return {
            "acceptance": {"polarity": "positive"},
            "expected_negative": None,
            "authority": "physics_truth",
            "execution_profile": "sim_ompl",
        }
    if scenario_id in (CANCEL_ID, CANCEL_TRANSPORT_ID):
        return {
            "acceptance": {"polarity": "negative"},
            "expected_negative": {"required": ["cancel-quiescent"], "forbidden": ["retention"]},
            "authority": "physics_truth",
            "execution_profile": "sim_ompl",
        }
    if scenario_id in (SAFETY_ID, MOVEIT_SAFETY_ID):
        return {
            "acceptance": {"polarity": "negative"},
            "expected_negative": {"required": ["safety-quiescent"], "forbidden": ["transport"]},
            "authority": "physics_truth",
            "execution_profile": "sim_ompl",
        }
    return {"acceptance": {"polarity": "positive"}, "expected_negative": None}


def _write_attempt_dir(
    suite_dir: Path,
    scenario_id: str,
    *,
    attempt_id: str,
    attempt_subdir: str | None = None,
    include_rosbag: bool = True,
    include_planning_scene: bool = True,
) -> Path:
    """Write a real-shaped integrated attempt directory for one scenario.

    ``attempt_subdir`` overrides the on-disk attempt directory name so a second
    attempt bearing the same scenario id can be written to a distinct path.
    """
    attempt_dir = suite_dir / "E" / (attempt_subdir or scenario_id)
    attempt_dir.mkdir(parents=True, exist_ok=True)

    # manifest.json (real producer schema).
    _write_json(
        attempt_dir / "manifest.json",
        {
            "schema_version": 1,
            "attempt_id": attempt_id,
            "created_at": "2026-08-04T00:00:00+00:00",
            "root": "/repo/simulator",
            "scenario": {
                "id": scenario_id,
                "schema_version": 2,
                "seed": 7,
                "entity_ids": [],
                "object_ids": ["qualification_cube"],
                "path": f"simulation/scenarios/{scenario_id}.json",
            },
            "config": {"path": "simulation/qualification/integrated-ompl.json"},
            "artifact": [],
            "sources": {},
            "provenance": {"versions": {}, "executed_input_paths": []},
            "seed": 7,
            "gate": scenario_id,
            "selected_gates": [],
            "physics": {"hz": PHYSICS_HZ, "device": "cpu"},
            "thresholds": {},
            "rosbag_minimum_message_counts": {},
            "commands": {},
            "environment": {
                "ROS_DOMAIN_ID": "100",
                "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                "FASTRTPS_DEFAULT_PROFILES_FILE": "/repo/simulator/src/tk26_vision/config/fastdds_shm.xml",
                "process_policy": "single-process",
            },
            "topics": {"physics_truth": "/sim/internal/physics_truth"},
        },
    )
    # scenario-bundle.json (real orchestrator producer schema).
    _write_json(
        attempt_dir / "scenario-bundle.json",
        {
            "schema_version": 1,
            "scenario_id": scenario_id,
            "attempt_id": attempt_id,
            "attempt_dir": f"E/{scenario_id}",
            "scenario": {
                "id": scenario_id,
                "schema_version": 2,
                "seed": 7,
                "entity_ids": [],
                "object_ids": ["qualification_cube"],
                "path": f"simulation/scenarios/{scenario_id}.json",
            },
            "planning_scene": {"revision": "qualification-v1", "owner": "sim_fixture"},
            "planning_scene_declaration": {"revision": "qualification-v1"},
            "integrated": _scenario_integrated(scenario_id),
            "report_identities": {"report_revision": "2026-08-04"},
        },
    )
    # scenario-runner.json + physics-ready.json.
    _write_json(
        attempt_dir / "scenario-runner.json",
        {"schema_version": 1, "scenario_id": scenario_id, "attempt_id": attempt_id, "seed": 7},
    )
    _write_json(
        attempt_dir / "physics-ready.json",
        {"schema_version": 1, "state": "PHYSICS_READY", "seed": 7, "scenario_id": scenario_id},
    )

    # physics_truth.jsonl + evaluator.jsonl (exact raw/wrapper frames).
    raw_frames = [_raw_frame(i, scenario_id) for i in range(PHYSICS_FRAMES)]
    _write_text(
        attempt_dir / "physics_truth.jsonl",
        "\n".join(json.dumps(frame, sort_keys=True) for frame in raw_frames) + "\n",
    )
    evaluator_rows = [
        {"schema_version": 1, "frame": frame, "frame_index": frame["frame_index"]}
        for frame in raw_frames
    ]
    _write_text(
        attempt_dir / "evaluator.jsonl",
        "\n".join(json.dumps(row, sort_keys=True) for row in evaluator_rows) + "\n",
    )
    _write_json(
        attempt_dir / "gate-window.json",
        {
            "schema_version": 1,
            "gate": scenario_id,
            "attempt_id": attempt_id,
            "raw_start_index": 0,
            "evaluator_start_index": 0,
            "wall_timestamp": "2026-08-04T00:00:00+00:00",
        },
    )
    _write_json(
        attempt_dir / "truth-drain.json",
        {
            "status": "drained",
            "raw_truth_frames": PHYSICS_FRAMES,
            "evaluator_frames": PHYSICS_FRAMES,
            "counts_match_or_exceed": True,
            "exact_correlation": True,
            "raw_errors": [],
            "evaluator_errors": [],
            "mismatches": [],
            "captured_at": "2026-08-04T00:00:00+00:00",
        },
    )

    # planning-scene journal + final.
    if include_planning_scene:
        journal_events = ("fixture-ready", "before-pick", "scene-attach", "lift-complete",
                          "transport", "before-release", "scene-detach", "released-settled", "teardown")
        rows = [
            {
                "schema_version": 1,
                "sequence": index,
                "event": event,
                "frame_index": (index - 1) * 5 + 1,
                "timestamp": ((index - 1) * 5 + 1) / PHYSICS_HZ,
                "fixture_revision": "qualification-v1",
            }
            for index, event in enumerate(journal_events, 1)
        ]
        _write_text(
            attempt_dir / "planning-scene.jsonl",
            "\n".join(json.dumps(row, sort_keys=True) for row in rows) + "\n",
        )
        _write_json(
            attempt_dir / "planning-scene.json",
            {"schema_version": 1, "finalized": True, "revision": "qualification-v1", "owner": "sim_fixture"},
        )

    # moveit-plans.jsonl / controller-results.jsonl (real executor rows).
    _write_text(
        attempt_dir / "moveit-plans.jsonl",
        "\n".join(
            json.dumps(
                {
                    "schema_version": 1,
                    "report_revision": "2026-08-04",
                    "scenario_id": scenario_id,
                    "goal_kind": "pick-place",
                    "status": status,
                    "row_kind": row_kind,
                    "pick_goal_sent": True,
                    "place_goal_sent": True,
                    "diagnostic_only": True,
                },
                sort_keys=True,
            )
            for status, row_kind in (("diagnostic-pass", "lifecycle"), ("diagnostic-pass", "final"))
        )
        + "\n",
    )
    _write_text(
        attempt_dir / "controller-results.jsonl",
        "\n".join(
            json.dumps(
                {
                    "schema_version": 1,
                    "report_revision": "2026-08-04",
                    "scenario_id": scenario_id,
                    # Lifecycle rows carry no status; only the final corrective
                    # row carries the executor's evidence-status domain (F2.6).
                    **({"status": "diagnostic-pass"} if row_kind == "final" else {}),
                    "row_kind": row_kind,
                    "controller_goal_sent": True,
                    "controller_goal_uuid": f"goal-{scenario_id}",
                    "controller_endpoint": "/xarm/execute_trajectory",
                    "gripper_goal_sent": True,
                    "diagnostic_only": True,
                },
                sort_keys=True,
            )
            for row_kind in ("lifecycle", "final")
        )
        + "\n",
    )
    # integrated-execution.jsonl / .json.
    _write_text(
        attempt_dir / "integrated-execution.jsonl",
        json.dumps(
            {
                "schema_version": 1,
                "report_revision": "2026-08-04",
                "scenario_id": scenario_id,
                "event": "gate-e",
                "stage": "E",
                "status": "diagnostic-pass",
                "row_kind": "final",
                "diagnostic_only": True,
                "pick_goal_sent": True,
                "place_goal_sent": True,
                "controller_goal_sent": True,
                "isaac_joint_commands_published": False,
                "timestamp": 1234.5,
            },
            sort_keys=True,
        )
        + "\n",
    )
    _write_json(
        attempt_dir / "integrated-execution.json",
        {
            "schema_version": 1,
            "report_revision": "2026-08-04",
            "scenario_id": scenario_id,
            "stage": "E",
            "handler": "pick-place",
            "diagnostic_only": True,
            "status": "diagnostic-pass",
            "reason_code": None,
            "pick_goal_sent": True,
            "place_goal_sent": True,
            "place_goal_accepted": True,
            "controller_goal_sent": True,
            "controller_goal_uuid": f"goal-{scenario_id}",
            "isaac_joint_commands_published": False,
        },
    )
    _write_json(
        attempt_dir / "goals" / f"{scenario_id}.json",
        {"schema_version": 1, "scenario_id": scenario_id, "goal_kind": "pick-place", "diagnostic_only": True},
    )

    # gate-verdict.json (independent verifier producer schema).
    _write_json(
        attempt_dir / "gate-verdict.json",
        {
            "schema_version": 1,
            "attempt_id": attempt_id,
            "gate": scenario_id,
            "stage": "E",
            "polarity": "positive" if scenario_id == POSITIVE_ID else "negative",
            "status": "verified-pass",
            "pass": True,
            "verified": True,
            "authority": "physics_truth.jsonl",
            "action_results_diagnostic_only": True,
            "checks": [],
            "metrics": {},
            "errors": [],
            "execution_sources": ["integrated-execution.jsonl", "integrated-execution.json"],
        },
    )

    # resource-cleanup.json (real schema_version 2 producer shape): on an RTX
    # machine ``baseline.gpus``/``final.gpus`` are the FULL nvidia-smi inventory
    # (non-empty); ``attempt_owned_pids`` is the cumulative historical PID
    # observation set (non-empty after any child spawn), NOT a live survivor
    # list.  Live GPU survivors live in ``attempt_owned_gpu_survivors``.
    _write_json(
        attempt_dir / "resource-cleanup.json",
        {
            "schema_version": 2,
            "baseline": {
                "available": True,
                "gpus": [dict(gpu) for gpu in GPU_INVENTORY],
                "processes": [],
                "errors": [],
            },
            "final": {
                "available": True,
                "gpus": [dict(gpu) for gpu in GPU_INVENTORY],
                "processes": [],
                "errors": [],
            },
            "attempt_owned_pids": [1000, 1001],
            "attempt_owned_gpu_survivors": [],
            "unexplained_gpu_memory": [],
            "memory_tolerance_mib": 32,
            "settle_attempts": [{"attempt": 1, "available": True, "owned_gpu_survivors": [], "unexplained_gpu_memory": []}],
            "clean": True,
            "captured_at": "2026-08-04T00:00:00+00:00",
        },
    )

    # rosbag2 metadata + storage (real recorder metadata.yaml shape, valid YAML).
    if include_rosbag:
        import yaml

        bag = attempt_dir / "rosbag"
        bag.mkdir(parents=True, exist_ok=True)
        # F2.7: all 11 approved record topics with their exact message types.
        record_topic_types = {
            "/clock": "rosgraph_msgs/msg/Clock",
            "/isaac_joint_states": "sensor_msgs/msg/JointState",
            "/isaac_joint_commands": "sensor_msgs/msg/JointState",
            "/sim/truth/robot_state": "tinker_sim_interfaces/msg/RobotTruth",
            "/sim/truth/object_state": "tinker_sim_interfaces/msg/ObjectTruth",
            "/sim/truth/contacts": "tinker_sim_interfaces/msg/ContactTruth",
            "/sim/truth/task_state": "tinker_sim_interfaces/msg/TaskTruth",
            "/sim/safety/collision": "std_msgs/msg/Bool",
            "/sim/hardware/safety_stop": "std_msgs/msg/Bool",
            "/sim/status/contract": "std_msgs/msg/String",
            "/sim/status/command_gateway": "std_msgs/msg/String",
        }
        # F3.8/F4.2: realistic per-topic offered QoS.  The two
        # ROSBAG_QOS_OVERRIDE_PROFILES topics carry keep_last/depth1/
        # reliable/transient_local; the remaining approved publishers are
        # RELIABLE (with VOLATILE durability) per the overlay publisher contract.
        # Humble rosbag2 serializes the full nine-field rmw_qos_profile_t.
        override_qos = (
            "- history: 1\n"
            "  depth: 1\n"
            "  reliability: 1\n"
            "  durability: 1\n"
            "  deadline: 0\n"
            "  lifespan: 0\n"
            "  liveliness: 1\n"
            "  liveliness_lease_duration: 0\n"
            "  avoid_ros_namespace_conventions: false\n"
        )
        volatile_qos = (
            "- history: 1\n"
            "  depth: 10\n"
            "  reliability: 1\n"
            "  durability: 3\n"
            "  deadline: 0\n"
            "  lifespan: 0\n"
            "  liveliness: 1\n"
            "  liveliness_lease_duration: 0\n"
            "  avoid_ros_namespace_conventions: false\n"
        )
        topic_qos = {
            "/sim/hardware/safety_stop": override_qos,
            "/sim/status/contract": override_qos,
        }
        storage_name = f"{scenario_id}_0.db3"
        metadata_topics = []
        for topic, topic_type in record_topic_types.items():
            metadata_topics.append(
                {
                    "topic_metadata": {
                        "name": topic,
                        "type": topic_type,
                        "offered_qos_profiles": topic_qos.get(topic, volatile_qos),
                    },
                    "message_count": 100,
                }
            )
        metadata_document = {
            "rosbag2_bagfile_information": {
                "storage_identifier": "sqlite3",
                "duration": {"nanoseconds": 10000000000},
                "message_count": 900,
                "relative_file_paths": [storage_name],
                "topics_with_message_count": metadata_topics,
            }
        }
        (bag / "metadata.yaml").write_text(
            yaml.safe_dump(metadata_document, sort_keys=False, default_flow_style=False),
            encoding="utf-8",
        )
        _write_bytes(bag / storage_name, b"SQLite format 3\x00" + b"\x00" * 24)

    # ---- Visual two-journal transaction -------------------------------------
    events = _scenario_events(scenario_id)
    base_frame = {"qualification-pick-place-positive": 0, CANCEL_ID: 5, SAFETY_ID: 8}.get(scenario_id, 5)
    request_rows: list[dict[str, object]] = []
    keyframe_rows: list[dict[str, object]] = []
    for event_index, event in enumerate(events):
        for camera in CAMERAS:
            sequence = event_index * len(CAMERAS) + 1 + (0 if camera == CAMERAS[0] else 1)
            frame_index = base_frame + event_index * 10
            requested_time = frame_index / PHYSICS_HZ
            # F4.1: the real producer captures with a nonzero latency contract:
            # requested frame = round(requested_time / physics_dt), raw frame =
            # requested + CAPTURE_LATENCY, and latency = raw - requested.
            captured_frame_index = frame_index + CAPTURE_LATENCY
            captured_timestamp = captured_frame_index / PHYSICS_HZ
            request_rows.append(
                {
                    "schema_version": 1,
                    "sequence": sequence,
                    "gate": scenario_id,
                    "event": event,
                    "simulated_timestamp": requested_time,
                    "source_execution_event_sequence": sequence,
                }
            )
            relative = f"visual/source/{sequence:04d}-{event}-{camera}.png"
            keyframe_rows.append(
                {
                    "schema_version": 1,
                    "gate": scenario_id,
                    "event": event,
                    "request_sequence": sequence,
                    "execution_event_sequence": sequence,
                    "requested_simulated_timestamp": requested_time,
                    "requested_physics_frame_index": frame_index,
                    "capture_latency_frames": CAPTURE_LATENCY,
                    "max_capture_latency_frames": 4,
                    "simulated_timestamp": captured_timestamp,
                    "raw_frame_index": captured_frame_index,
                    "physics_dt": 1.0 / PHYSICS_HZ,
                    "camera": camera,
                    "camera_fixture": {"eye": [0.0, 0.0, 0.0], "target": [0.0, 0.0, 0.0]},
                    "path": relative,
                    "width": 960,
                    "height": 540,
                    "mode": "RGB",
                }
            )
            _nonblank_image(attempt_dir / relative, event_index * 10 + (0 if camera == CAMERAS[0] else 1))
    # Executor diagnostic request records (real `_append_visual_request` shape).
    for phase in ("before", "before-pick", "after", "terminal"):
        request_rows.append(
            {
                "schema_version": 1,
                "report_revision": "2026-08-04",
                "scenario_id": scenario_id,
                "phase": phase,
                "capture": {"kind": "gate-e-diagnostic", "target": None},
                "diagnostic_only": True,
            }
        )
    _write_text(
        attempt_dir / "visual-capture-requests.jsonl",
        "\n".join(json.dumps(row, sort_keys=True) for row in request_rows) + "\n",
    )
    _write_text(
        attempt_dir / "visual-keyframes.jsonl",
        "\n".join(json.dumps(row, sort_keys=True) for row in keyframe_rows) + "\n",
    )
    return attempt_dir


def write_canonical_evidence_tree(
    suite_dir: Path,
    *,
    simulator_commit: str | None = SIM_COMMIT,
    include_rosbag: bool = True,
    include_planning_scene: bool = True,
    include_gate_b: bool = True,
) -> Path:
    """Write the complete required artifact tree and valid nonblank captures.

    The factory writes exact producer-shaped bytes; the final
    ``evidence-index.json`` is derived from those bytes and is not a fabricated
    verdict.
    """
    suite_dir.mkdir(parents=True, exist_ok=True)
    # Config + overlay contract (recognized where present).
    _write_json(
        suite_dir / "config/integrated-ompl.json",
        {
            "schema_version": 3,
            "id": "integrated-ompl",
            "profile": "integrated-ompl",
            "execution_profile": "sim_ompl",
            "seed": 7,
            "checksum_algorithm": "sha256",
            "stages": {"F": {"cameras": list(CAMERAS)}},
        },
    )
    # Real nested overlay-contract shape (F2.5/F3.2): repositories carries the
    # scalar ``path_scope`` note next to the ``production``/``simulator``
    # mapping records; source_locks.status is the truthful real value
    # ``"excluded_in_task_8"`` (not a fabricated ``"pass"``).
    _write_json(
        suite_dir / "ompl-overlay-contract.json",
        {
            "schema_version": 1,
            "contract_id": "simulator-ompl-overlay-acceptance",
            "repositories": {
                "path_scope": (
                    "absolute checkout paths and build commands are environment "
                    "identities for this qualification workspace, not a claim that "
                    "the contract carries no machine-specific execution policy"
                ),
                "production": {
                    "dirty_policy": "read-only runtime input",
                    "implementation_identity": PROD_COMMIT,
                    "path": "/repo/production",
                },
                "simulator": {
                    "dirty_policy": "committed implementation identity is the clean tree",
                    "implementation_identity": SIM_COMMIT,
                    "path": "/repo/simulator",
                },
            },
            "source_locks": {
                "note": "Task 8 leaves source-lock creation to the lock-only phase",
                "production_lock_path": "/repo/production/integration/source-locks.json",
                "simulator_lock_path": "integration/source-locks.json",
                "status": "excluded_in_task_8",
            },
        },
    )
    _write_json(
        suite_dir / "integration" / "source-locks.json",
        {
            "schema_version": 1,
            "authorization": {"phase": "task-9b-simulator-repository-lock-only"},
            "repository": "simulator",
            "implementation_head": SIM_COMMIT,
            "mode": "clean",
            "status": "pass",
        },
    )
    # Attempt-start identity (real orchestrator producer schema).
    _write_json(
        suite_dir / f"attempt-start-{SUITE_ATTEMPT}.json",
        {
            "schema_version": 1,
            "attempt_id": SUITE_ATTEMPT,
            "started_at": "2026-08-04T00:00:00+00:00",
            "monotonic": 1.0,
            "seed": 7,
            "root": "/repo/simulator",
            "production_root": "/repo/production",
            "config": "simulation/qualification/integrated-ompl.json",
        },
    )
    # Gate-B source-lock / static / model artifacts.
    if include_gate_b:
        gate_b = suite_dir / f"gate-b-{SUITE_ATTEMPT}"
        gate_b.mkdir(parents=True, exist_ok=True)
        _write_json(
            gate_b / "source-lock-manifest.json",
            {
                "schema_version": 1,
                "status": "pass",
                "attempt_started_at": "2026-08-04T00:00:00+00:00",
                "repositories": ["production", "qualification_tooling", "simulator_overlay"],
                "simulator_overlay": {
                    "repository": "simulator_overlay",
                    "root": "/repo/simulator",
                    "policy_path": "integration/source-locks.json",
                    "head": SIM_COMMIT,
                    "implementation_head": "c" * 40,
                    "resolved_policy_commit": "d" * 40,
                    "mode": "clean",
                    "status": "pass",
                    "policy_file_missing": False,
                    "checks": {"evidence": True, "history": True},
                    "reasons": [],
                    "observed_clean": True,
                    "expected_status_sha256": "1" * 64,
                    "observed_status_sha256": "1" * 64,
                    "expected_diff_sha256": "2" * 64,
                    "observed_diff_sha256": "2" * 64,
                    "expected_untracked_manifest_sha256": "3" * 64,
                    "observed_untracked_manifest_sha256": "3" * 64,
                },
                "production": {
                    "repository": "production",
                    "root": "/repo/production",
                    "policy_path": "integration/source-locks.json",
                    "head": PROD_COMMIT,
                    "implementation_head": "e" * 40,
                    "resolved_policy_commit": "f" * 40,
                    "mode": "clean",
                    "status": "pass",
                    "policy_file_missing": False,
                    "checks": {"evidence": True, "history": True},
                    "reasons": [],
                    "observed_clean": True,
                    "expected_status_sha256": "4" * 64,
                    "observed_status_sha256": "4" * 64,
                    "expected_diff_sha256": "5" * 64,
                    "observed_diff_sha256": "5" * 64,
                    "expected_untracked_manifest_sha256": "6" * 64,
                    "observed_untracked_manifest_sha256": "6" * 64,
                },
                "qualification_tooling": {
                    "repository": "qualification_tooling",
                    "root": "/repo/simulator",
                    "policy_path": "integration/qualification-tooling-lock.json",
                    "head": SIM_COMMIT,
                    "implementation_head": "c" * 40,
                    "resolved_policy_commit": "d" * 40,
                    "mode": "clean",
                    "status": "pass",
                    "policy_file_missing": False,
                    "checks": {"evidence": True, "history": True},
                    "reasons": [],
                    "observed_clean": True,
                    "expected_status_sha256": "7" * 64,
                    "observed_status_sha256": "7" * 64,
                    "expected_diff_sha256": "8" * 64,
                    "observed_diff_sha256": "8" * 64,
                    "expected_untracked_manifest_sha256": "9" * 64,
                    "observed_untracked_manifest_sha256": "9" * 64,
                },
            },
        )
        _write_json(
            gate_b / "static-contract.json",
            {
                "schema_version": 1,
                "status": "pass",
                "model_fingerprint": "ab" * 32,
                "source_identities": {
                    "simulator": SIM_COMMIT,
                    "production": PROD_COMMIT,
                },
                "checks": [
                    {"name": "model-fingerprint", "passed": True, "details": {"fingerprint": "ab" * 32}, "reasons": []},
                    {"name": "source-identities", "passed": True, "details": {}, "reasons": []},
                ],
            },
        )
        _write_json(
            gate_b / "model-fingerprint.json",
            {"schema_version": 1, "model_fingerprint": "ab" * 32},
        )
        _write_json(
            gate_b / "source-identities.json",
            {
                "schema_version": 1,
                "source_identities": {"simulator": SIM_COMMIT, "production": PROD_COMMIT},
            },
        )

    # C/D/E attempt dirs for the three canonical visual scenarios.
    _write_attempt_dir(
        suite_dir, POSITIVE_ID,
        attempt_id=ATTEMPT_IDS[POSITIVE_ID],
        include_rosbag=include_rosbag,
        include_planning_scene=include_planning_scene,
    )
    _write_attempt_dir(
        suite_dir, CANCEL_ID,
        attempt_id=ATTEMPT_IDS[CANCEL_ID],
        include_rosbag=include_rosbag,
        include_planning_scene=include_planning_scene,
    )
    _write_attempt_dir(
        suite_dir, SAFETY_ID,
        attempt_id=ATTEMPT_IDS[SAFETY_ID],
        include_rosbag=include_rosbag,
        include_planning_scene=include_planning_scene,
    )

    build_evidence_index(suite_dir=suite_dir, output=suite_dir / INDEX_NAME)
    return suite_dir


def make_complete_evidence_suite(
    tmp_path: Path,
    *,
    simulator_commit: str | None = SIM_COMMIT,
    include_rosbag: bool = True,
    include_planning_scene: bool = True,
    include_gate_b: bool = True,
) -> Path:
    suite_dir = tmp_path / "suite"
    write_canonical_evidence_tree(
        suite_dir,
        simulator_commit=simulator_commit,
        include_rosbag=include_rosbag,
        include_planning_scene=include_planning_scene,
        include_gate_b=include_gate_b,
    )
    return suite_dir


def rebuild_index(suite_dir: Path) -> dict[str, object]:
    return build_evidence_index(suite_dir=suite_dir, output=suite_dir / INDEX_NAME)


def required_capture_paths(suite_dir: Path, *, events: set[str]) -> list[Path]:
    """Select bound live capture paths from the evidence index (never the
    old ``captures/{event}.png`` reconstruction)."""
    index = json.loads((suite_dir / INDEX_NAME).read_text(encoding="utf-8"))
    chosen: list[Path] = []
    for entry in index["files"]:
        if (
            entry.get("category") == "capture"
            and entry.get("bound")
            and entry.get("event") in events
        ):
            chosen.append(suite_dir / entry["path"])
    by_event: dict[str, Path] = {}
    for path in chosen:
        entry = next(
            e for e in index["files"]
            if e.get("path") == path.relative_to(suite_dir).as_posix()
        )
        event = entry["event"]
        if event not in by_event:
            by_event[event] = path
    missing = sorted(events - set(by_event))
    if missing:
        raise FileNotFoundError(f"required capture events missing: {missing}")
    return [by_event[event] for event in sorted(events, key=lambda e: list(POSITIVE_EVENTS + CANCEL_EVENTS + SAFETY_EVENTS).index(e))]


def render_sheets(suite_dir: Path, *, events: set[str] | None = None) -> None:
    events = set(POSITIVE_EVENTS + CANCEL_EVENTS + SAFETY_EVENTS) if events is None else events
    paths = required_capture_paths(suite_dir, events=events)
    build_contact_sheet(suite_dir, paths, output=suite_dir / AGENT_NAME)
    build_contact_sheet(suite_dir, paths, output=suite_dir / USER_NAME, user=True)


def _final_index(suite_dir: Path) -> dict[str, object]:
    return rebuild_index(suite_dir)


# --- Brief acceptance tests --------------------------------------------------


def test_index_is_deterministic_and_excludes_itself(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    first = build_evidence_index(suite_dir=suite_dir, output=suite_dir / INDEX_NAME)
    second = build_evidence_index(suite_dir=suite_dir, output=suite_dir / INDEX_NAME)
    assert first["files"] == second["files"]
    assert all(entry["path"] != INDEX_NAME for entry in first["files"])
    assert first["index_checksum"] == second["index_checksum"]


def test_missing_repository_commit_blocks_final_acceptance(tmp_path):
    # A source-lock manifest missing a repository's implementation commit
    # cannot pass Gate F (F2.5).
    suite_dir = make_complete_evidence_suite(tmp_path)
    lock = suite_dir / f"gate-b-{SUITE_ATTEMPT}" / "source-lock-manifest.json"
    value = json.loads(lock.read_text(encoding="utf-8"))
    del value["production"]["implementation_head"]
    lock.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    reasons = " ".join(verdict["reasons"])
    assert verdict["status"] == "verified-fail"
    assert "implementation_head" in reasons
    assert "production" in reasons


def test_missing_rosbag_metadata_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path, include_rosbag=False)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"
    assert "rosbag" in " ".join(verdict["reasons"]).lower()


def test_missing_planning_scene_journal_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path, include_planning_scene=False)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"
    assert "planning scene" in " ".join(verdict["reasons"]).lower()


# --- Production-shaped join (F1.1 / F1.2) ------------------------------------


def test_production_shaped_join_produces_bound_captures(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    index = _final_index(suite_dir)
    captures = [e for e in index["files"] if e.get("category") == "capture"]
    assert captures, "no capture entries bound from visual/source keyframes"
    assert all(
        e["path"].startswith("E/") and "visual/source/" in e["path"] for e in captures
    ), "capture paths must be the live visual/source producer paths"
    for entry in captures:
        assert entry.get("bound") is True
        assert entry.get("physics_bound") is True
        assert isinstance(entry.get("frame_index"), int)
        assert isinstance(entry.get("timestamp"), float)
        assert isinstance(entry.get("request_sequence"), int)
        assert isinstance(entry.get("execution_request"), str)
        assert entry.get("scenario") in (POSITIVE_ID, CANCEL_ID, SAFETY_ID)
        assert isinstance(entry.get("attempt"), str) and entry["attempt"]


def test_old_synthetic_capture_request_schema_rejected(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    request_path = suite_dir / "E" / POSITIVE_ID / "visual-capture-requests.jsonl"
    # Append a record in the old synthetic {path,event,frame_index,...} shape.
    with request_path.open("a", encoding="utf-8") as stream:
        stream.write(
            json.dumps(
                {
                    "path": "visual/source/9999-phantom-overview.png",
                    "event": "phantom",
                    "scenario": POSITIVE_ID,
                    "attempt": "x",
                    "frame_index": 1,
                    "timestamp": 0.1,
                },
                sort_keys=True,
            )
            + "\n"
        )
    index = _final_index(suite_dir)
    codes = {d["code"] for d in index.get("diagnostics", [])}
    assert "unrecognized-capture-request-shape" in codes


def test_keyframe_join_requires_exact_request_sequence(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    keyframes = suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl"
    rows = [json.loads(line) for line in keyframes.read_text(encoding="utf-8").splitlines()]
    rows[0]["request_sequence"] = 9999
    keyframes.write_text(
        "\n".join(json.dumps(row, sort_keys=True) for row in rows) + "\n",
        encoding="utf-8",
    )
    index = _final_index(suite_dir)
    codes = {d["code"] for d in index.get("diagnostics", [])}
    assert "keyframe-request-sequence-orphan" in codes
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"
    assert "unbound capture" in " ".join(verdict["reasons"])


def test_keyframe_invalid_frame_timestamp_fails_cleanly(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    keyframes = suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl"
    rows = [json.loads(line) for line in keyframes.read_text(encoding="utf-8").splitlines()]
    rows[0]["raw_frame_index"] = -1
    rows[0]["simulated_timestamp"] = float("nan")
    keyframes.write_text(
        "\n".join(json.dumps(row, sort_keys=True, allow_nan=True) for row in rows) + "\n",
        encoding="utf-8",
    )
    # Must fail cleanly (no uncaught exception) with a diagnostic.
    index = _final_index(suite_dir)
    codes = {d["code"] for d in index.get("diagnostics", [])}
    assert "keyframe-invalid-frame-index" in codes or "keyframe-invalid-timestamp" in codes
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


# --- Real scenario classification (F1.3) -------------------------------------


def test_real_cancel_safety_ids_require_their_event_groups(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    index = _final_index(suite_dir)
    assert "cancel" in index["scenario_kinds"]
    assert "safety" in index["scenario_kinds"]
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    required = verdict["required_events"]
    assert set(required["cancel"]) == set(CANCEL_EVENTS)
    assert set(required["safety"]) == set(SAFETY_EVENTS)


def test_cancel_event_missing_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    keyframes = suite_dir / "E" / CANCEL_ID / "visual-keyframes.jsonl"
    rows = [json.loads(line) for line in keyframes.read_text(encoding="utf-8").splitlines()]
    rows = [row for row in rows if row["event"] != "cancel-trigger"]
    keyframes.write_text(
        "\n".join(json.dumps(row, sort_keys=True) for row in rows) + "\n",
        encoding="utf-8",
    )
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    reasons = " ".join(verdict["reasons"]).lower()
    assert "missing required" in reasons
    assert CANCEL_ID in reasons
    assert "cancel-trigger" in reasons


def test_contradictory_scenario_declaration_fails_closed(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    bundle = suite_dir / "E" / CANCEL_ID / "scenario-bundle.json"
    value = json.loads(bundle.read_text(encoding="utf-8"))
    value["integrated"] = {"acceptance": {"polarity": "positive"}, "expected_negative": None}
    bundle.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
    index = _final_index(suite_dir)
    assert any(item.get("kind") == "invalid" for item in index.get("scenarios", [])), index.get("scenarios")
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert "invalid scenario declaration" in " ".join(verdict["reasons"])


# --- Semantic Gate F (F1.4) --------------------------------------------------


def test_semantic_verdict_fail_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    verdict_path = suite_dir / "E" / POSITIVE_ID / "gate-verdict.json"
    value = json.loads(verdict_path.read_text(encoding="utf-8"))
    value["status"] = "verified-fail"
    verdict_path.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "not verified-pass" in " ".join(verdict["reasons"])


def test_empty_raw_evaluator_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "E" / POSITIVE_ID / "physics_truth.jsonl").write_text("\n", encoding="utf-8")
    (suite_dir / "E" / POSITIVE_ID / "evaluator.jsonl").write_text("\n", encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "empty raw physics truth" in " ".join(verdict["reasons"])


def test_raw_evaluator_drain_mismatch_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    evaluator = suite_dir / "E" / POSITIVE_ID / "evaluator.jsonl"
    rows = [json.loads(line) for line in evaluator.read_text(encoding="utf-8").splitlines()]
    rows = rows[:-1]
    evaluator.write_text("\n".join(json.dumps(row, sort_keys=True) for row in rows) + "\n", encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "drain mismatch" in " ".join(verdict["reasons"])


def test_zero_message_rosbag_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    metadata = suite_dir / "E" / POSITIVE_ID / "rosbag" / "metadata.yaml"
    text = metadata.read_text(encoding="utf-8").replace("message_count: 100", "message_count: 0")
    metadata.write_text(text, encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "nonpositive message count" in " ".join(verdict["reasons"])


def test_missing_rosbag_storage_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    for db3 in (suite_dir / "E" / POSITIVE_ID / "rosbag").glob("*.db3"):
        db3.unlink()
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "no storage files" in " ".join(verdict["reasons"])


def test_invalid_source_lock_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    lock = suite_dir / f"gate-b-{SUITE_ATTEMPT}" / "source-lock-manifest.json"
    value = json.loads(lock.read_text(encoding="utf-8"))
    value["status"] = "fail"
    lock.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "source lock manifest status is not pass" in " ".join(verdict["reasons"])


def test_invalid_model_fingerprint_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    fp = suite_dir / f"gate-b-{SUITE_ATTEMPT}" / "model-fingerprint.json"
    fp.write_text(json.dumps({"schema_version": 1, "model_fingerprint": "not-a-digest"}, sort_keys=True) + "\n", encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "model fingerprint" in " ".join(verdict["reasons"])


def test_leaking_cleanup_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    cleanup = suite_dir / "E" / POSITIVE_ID / "resource-cleanup.json"
    value = json.loads(cleanup.read_text(encoding="utf-8"))
    value["clean"] = False
    value["attempt_owned_gpu_survivors"] = [{"pid": 999, "name": "isaac"}]
    cleanup.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "resource cleanup not clean" in " ".join(verdict["reasons"])


# --- Index integrity / current bytes (F1.5) ----------------------------------


def test_tampered_index_checksum_fails(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    index = _final_index(suite_dir)
    index["index_checksum"] = "0" * 64
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert "index_checksum does not match" in " ".join(verdict["reasons"])


def test_changed_ondisk_file_fails(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    index = _final_index(suite_dir)
    (suite_dir / "E" / POSITIVE_ID / "truth-drain.json").write_text(
        (suite_dir / "E" / POSITIVE_ID / "truth-drain.json").read_text(encoding="utf-8") + "\n",
        encoding="utf-8",
    )
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert "indexed digest mismatch" in " ".join(verdict["reasons"])


def test_extra_unindexed_file_fails(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    index = _final_index(suite_dir)
    (suite_dir / "unexpected-evidence.txt").write_text("x", encoding="utf-8")
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert "unindexed preserved file" in " ".join(verdict["reasons"])


def test_missing_indexed_file_fails(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    index = _final_index(suite_dir)
    (suite_dir / "E" / POSITIVE_ID / "truth-drain.json").unlink()
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert "stale index entry" in " ".join(verdict["reasons"]) or "missing indexed file" in " ".join(verdict["reasons"])


def test_index_rejects_symlink_escape(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    outside = tmp_path / "outside.txt"
    outside.write_text("secret", encoding="utf-8")
    (suite_dir / "escaped-link.txt").symlink_to(outside)
    with pytest.raises(ValueError, match="escape|outside|symlink"):
        rebuild_index(suite_dir)


def test_index_rejects_output_as_input(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    target = suite_dir / "ompl-overlay-contract.json"
    with pytest.raises(ValueError, match="output-as-input|output"):
        build_evidence_index(suite_dir=suite_dir, output=target)


def test_index_rejects_file_changing_during_hashing(tmp_path, monkeypatch):
    suite_dir = make_complete_evidence_suite(tmp_path)
    import validation.integrated_evidence_index as index_module

    target = suite_dir / "E" / POSITIVE_ID / "physics_truth.jsonl"
    original = target.read_bytes()

    def flaky_read(path: Path) -> bytes:
        if str(path).endswith("physics_truth.jsonl"):
            with open(path, "ab") as stream:
                stream.write(b"tampered")
        return path.read_bytes()

    monkeypatch.setattr(index_module, "_read_bytes", flaky_read)
    with pytest.raises(ValueError, match="changed during hashing"):
        build_evidence_index(suite_dir=suite_dir, output=suite_dir / INDEX_NAME)
    target.write_bytes(original)


def test_stale_temp_files_not_indexed(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / ".evidence-index.json.12345").write_text("stale", encoding="utf-8")
    index = _final_index(suite_dir)
    assert not any(e["path"].startswith(".evidence-index.json.") for e in index["files"])
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    # The stale temp is neither indexed nor flagged as an unexpected file.
    assert "unindexed preserved file" not in " ".join(verdict["reasons"])


# --- Index/contact-sheet cycle (F1.5 / F1.6) ---------------------------------


def test_full_cycle_gate_f_passes_with_sheets_and_summary(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    render_sheets(suite_dir)
    summary = build_qualification_summary(suite_dir)
    assert summary["status"] == "verified-pass", summary["reasons"]
    assert summary["reasons"] == []
    final = _final_index(suite_dir)
    assert "qualification-summary.json" in {e["path"] for e in final["files"]}
    verdict = validate_gate_f(final, suite_dir=suite_dir)
    assert verdict["status"] == "verified-pass", verdict["reasons"]


def test_summary_records_validated_pre_summary_checksum(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    render_sheets(suite_dir)
    build_qualification_summary(suite_dir)
    summary = json.loads((suite_dir / SUMMARY_NAME).read_text(encoding="utf-8"))
    assert DIGEST_RE.fullmatch(summary["validated_index_checksum"])


def test_summary_pre_summary_checksum_mismatch_fails(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    render_sheets(suite_dir)
    build_qualification_summary(suite_dir)
    summary_path = suite_dir / SUMMARY_NAME
    value = json.loads(summary_path.read_text(encoding="utf-8"))
    value["validated_index_checksum"] = "0" * 64
    summary_path.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "validated_index_checksum does not match" in " ".join(verdict["reasons"])


def test_sheets_are_indexed_and_deterministic_across_roots(tmp_path):
    suite_a = make_complete_evidence_suite(tmp_path)
    other = tmp_path / "other"
    suite_b = write_canonical_evidence_tree(other / "suite")
    render_sheets(suite_a)
    render_sheets(suite_b)
    build_qualification_summary(suite_a)
    build_qualification_summary(suite_b)
    index_a = _final_index(suite_a)
    index_b = _final_index(suite_b)
    assert index_a["index_checksum"] == index_b["index_checksum"]


def test_sheet_png_bytes_must_match_index(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    render_sheets(suite_dir)
    build_qualification_summary(suite_dir)
    sheet = suite_dir / AGENT_NAME
    sheet.write_bytes(b"NOTAPNG")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    reasons = " ".join(verdict["reasons"])
    assert "invalid PNG" in reasons or "missing embedded metadata" in reasons


# --- Determinism / taxonomy --------------------------------------------------


def test_index_files_sorted_and_checksums_lowercase_64hex(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    index = _final_index(suite_dir)
    paths = [entry["path"] for entry in index["files"]]
    assert paths == sorted(paths)
    for entry in index["files"]:
        assert DIGEST_RE.fullmatch(entry["sha256"]), entry["path"]
    assert DIGEST_RE.fullmatch(index["index_checksum"])


def test_validate_gate_f_required_taxonomy_present(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    index = _final_index(suite_dir)
    categories = {entry["category"] for entry in index["files"]}
    for required in (
        "attempt-start",
        "scenario-bundle",
        "source-lock-manifest",
        "static-contract",
        "model-fingerprint",
        "source-identities",
        "verdict",
        "moveit",
        "controller",
        "planning-scene-journal",
        "planning-scene-final",
        "physics",
        "evaluator",
        "drain",
        "cleanup",
        "capture-request-journal",
        "capture-keyframe-journal",
        "capture",
        "rosbag-metadata",
        "rosbag-storage",
        "config",
        "overlay-contract",
        "manifest",
    ):
        assert required in categories, required


def test_index_excludes_only_itself_and_indexes_every_other_file(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    index = _final_index(suite_dir)
    indexed = {entry["path"] for entry in index["files"]}
    on_disk = {
        str(path.relative_to(suite_dir))
        for path in suite_dir.rglob("*")
        if path.is_file() and not path.is_symlink()
    }
    on_disk = {rel for rel in on_disk if not rel.startswith(".evidence-index.json.")}
    assert INDEX_NAME not in indexed
    assert indexed == (on_disk - {INDEX_NAME})


def test_duplicate_keyframe_identity_rejected(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    keyframes = suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl"
    rows = [json.loads(line) for line in keyframes.read_text(encoding="utf-8").splitlines()]
    rows.append({**rows[0]})
    keyframes.write_text("\n".join(json.dumps(row, sort_keys=True) for row in rows) + "\n", encoding="utf-8")
    index = _final_index(suite_dir)
    codes = {d["code"] for d in index.get("diagnostics", [])}
    assert "duplicate-keyframe-identity" in codes


def test_canonical_sha256_is_stable_lowercase():
    value = {"b": [1, 2], "a": "x"}
    first = canonical_sha256(value)
    second = canonical_sha256({"a": "x", "b": [2, 1]})
    assert first != second
    assert DIGEST_RE.fullmatch(first)
    assert canonical_sha256({"a": "x", "b": [1, 2]}) == first


def test_simulator_commit_and_production_commit_present_in_manifest(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    index = _final_index(suite_dir)
    manifests = [e for e in index["files"] if e.get("category") == "manifest"]
    assert manifests


# --- Task 9 fix-round-2 mutation tests (F2.4-F2.9) ---------------------------


def _rewrite_jsonl(path: Path, rows: list[dict[str, object]]) -> None:
    path.write_text(
        "\n".join(json.dumps(row, sort_keys=True) for row in rows) + "\n",
        encoding="utf-8",
    )


def test_executor_diagnostic_only_journal_is_not_capture_driving(tmp_path):
    """F2.4: a journal with only executor diagnostic records must never drive a
    capture, never crash, and never emit a capture-request-without-image
    diagnostic for the diagnostic records themselves."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    request_path = suite_dir / "E" / POSITIVE_ID / "visual-capture-requests.jsonl"
    rows = [json.loads(line) for line in request_path.read_text(encoding="utf-8").splitlines()]
    sequence_rows = [row for row in rows if isinstance(row.get("sequence"), int)]
    executor_rows = [row for row in rows if row.get("diagnostic_only") is True]
    assert sequence_rows and executor_rows
    # Keep only the executor diagnostic records (no canonical sequence records).
    _rewrite_jsonl(request_path, executor_rows)
    index = _final_index(suite_dir)
    codes = {d["code"] for d in index.get("diagnostics", [])}
    # The executor diagnostic records never produce a request-without-image.
    assert "capture-request-without-image" not in codes
    # Keyframes can no longer join a canonical request -> orphan, fail-closed.
    assert "keyframe-request-sequence-orphan" in codes
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


def test_index_diagnostics_fail_closed(tmp_path):
    """F2.4: any index diagnostic fails Gate F closed."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    request_path = suite_dir / "E" / POSITIVE_ID / "visual-capture-requests.jsonl"
    rows = [json.loads(line) for line in request_path.read_text(encoding="utf-8").splitlines()]
    rows.append(dict(rows[0]))  # duplicate canonical sequence
    _rewrite_jsonl(request_path, rows)
    index = _final_index(suite_dir)
    assert any(d.get("code") == "duplicate-request-sequence" for d in index.get("diagnostics", []))
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    reasons = " ".join(verdict["reasons"])
    assert "any index diagnostic fails Gate F closed" in reasons


def test_duplicate_physics_key_fails_gate_f(tmp_path):
    """F2.4/F2.6: duplicate raw (scenario, frame_index) key is rejected."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    raw_path = suite_dir / "E" / POSITIVE_ID / "physics_truth.jsonl"
    rows = [json.loads(line) for line in raw_path.read_text(encoding="utf-8").splitlines()]
    rows.append(_raw_frame(0, POSITIVE_ID))
    _rewrite_jsonl(raw_path, rows)
    index = _final_index(suite_dir)
    assert any(d.get("code") == "duplicate-physics-key" for d in index.get("diagnostics", []))
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert "duplicate raw physics key" in " ".join(verdict["reasons"])


def test_duplicate_evaluator_key_fails_gate_f(tmp_path):
    """F2.4/F2.6: duplicate evaluator (scenario, frame_index) key is rejected."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    evaluator_path = suite_dir / "E" / POSITIVE_ID / "evaluator.jsonl"
    rows = [json.loads(line) for line in evaluator_path.read_text(encoding="utf-8").splitlines()]
    rows.append({"schema_version": 1, "frame": _raw_frame(0, POSITIVE_ID), "frame_index": 0})
    _rewrite_jsonl(evaluator_path, rows)
    index = _final_index(suite_dir)
    assert any(d.get("code") == "duplicate-evaluator-key" for d in index.get("diagnostics", []))
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert "duplicate evaluator key" in " ".join(verdict["reasons"])


def test_keyframe_binds_within_bounded_dt_window(tmp_path):
    """F2.4: a keyframe timestamp inside the 0.5*dt window still physics-binds."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    keyframes = suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl"
    rows = [json.loads(line) for line in keyframes.read_text(encoding="utf-8").splitlines()]
    rows[0]["simulated_timestamp"] = float(rows[0]["simulated_timestamp"]) + 0.002
    _rewrite_jsonl(keyframes, rows)
    index = _final_index(suite_dir)
    codes = {d["code"] for d in index.get("diagnostics", [])}
    assert "keyframe-physics-unbound" not in codes
    captures = [e for e in index["files"] if e.get("category") == "capture" and e.get("bound")]
    assert captures


def test_keyframe_outside_bounded_dt_window_fails(tmp_path):
    """F2.4: a keyframe timestamp beyond the 0.5*dt window is unbound."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    keyframes = suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl"
    rows = [json.loads(line) for line in keyframes.read_text(encoding="utf-8").splitlines()]
    rows[0]["simulated_timestamp"] = float(rows[0]["simulated_timestamp"]) + 0.1
    _rewrite_jsonl(keyframes, rows)
    index = _final_index(suite_dir)
    assert any(d.get("code") == "keyframe-physics-unbound" for d in index.get("diagnostics", []))
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


def test_source_lock_digest_mismatch_fails_gate_f(tmp_path):
    """F2.5: a source-lock repository with observed != expected digest fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    lock = suite_dir / f"gate-b-{SUITE_ATTEMPT}" / "source-lock-manifest.json"
    value = json.loads(lock.read_text(encoding="utf-8"))
    value["production"]["observed_status_sha256"] = "f" * 64
    lock.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    reasons = " ".join(verdict["reasons"])
    assert "digest mismatch" in reasons
    assert "production" in reasons


def test_source_lock_uppercase_commit_fails_gate_f(tmp_path):
    """F2.5: uppercase/malformed 40-hex commits are rejected."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    lock = suite_dir / f"gate-b-{SUITE_ATTEMPT}" / "source-lock-manifest.json"
    value = json.loads(lock.read_text(encoding="utf-8"))
    value["simulator_overlay"]["implementation_head"] = "A" * 40
    lock.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "lowercase 40-hex" in " ".join(verdict["reasons"])


def test_overlay_contract_missing_repositories_fails_gate_f(tmp_path):
    """F2.5: overlay-contract nested repositories identity is required."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    overlay = suite_dir / "ompl-overlay-contract.json"
    value = json.loads(overlay.read_text(encoding="utf-8"))
    del value["repositories"]
    overlay.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "overlay contract has no repositories map" in " ".join(verdict["reasons"])


def test_verdict_attempt_identity_mismatch_fails_gate_f(tmp_path):
    """F2.6: a gate verdict whose attempt_id does not match the manifest fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    verdict_path = suite_dir / "E" / POSITIVE_ID / "gate-verdict.json"
    value = json.loads(verdict_path.read_text(encoding="utf-8"))
    value["attempt_id"] = "wrong-attempt"
    verdict_path.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "does not match manifest" in " ".join(verdict["reasons"])


def test_moveit_out_of_domain_status_fails_gate_f(tmp_path):
    """F2.6: moveit row status must be in the evidence-status domain."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    moveit = suite_dir / "E" / POSITIVE_ID / "moveit-plans.jsonl"
    rows = [json.loads(line) for line in moveit.read_text(encoding="utf-8").splitlines()]
    rows[0]["status"] = "succeeded"
    _rewrite_jsonl(moveit, rows)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "out-of-domain status" in " ".join(verdict["reasons"])


def test_controller_out_of_domain_status_fails_gate_f(tmp_path):
    """F2.6: a controller row with an out-of-domain status fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    controller = suite_dir / "E" / POSITIVE_ID / "controller-results.jsonl"
    rows = [json.loads(line) for line in controller.read_text(encoding="utf-8").splitlines()]
    rows[0]["status"] = "succeeded"
    _rewrite_jsonl(controller, rows)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "out-of-domain status" in " ".join(verdict["reasons"])


def test_planning_scene_journal_without_final_fails_gate_f(tmp_path):
    """F2.6: PlanningScene evidence is per-attempt (journal requires sibling final)."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "E" / POSITIVE_ID / "planning-scene.json").unlink()
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "has a journal but no final artifact" in " ".join(verdict["reasons"])


def test_genuine_attempt_failure_status_not_schema_domain_error(tmp_path):
    """F3.10: a genuine attempt-failure status (``evidence-invalid``) in the
    executor's moveit rows is a valid in-domain status, never a schema-domain
    diagnostic -- the allowed status domain is not broadened, only kept truthful."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    moveit = suite_dir / "E" / POSITIVE_ID / "moveit-plans.jsonl"
    rows = [json.loads(line) for line in moveit.read_text(encoding="utf-8").splitlines()]
    rows[0]["status"] = "evidence-invalid"
    _rewrite_jsonl(moveit, rows)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    reasons = " ".join(verdict["reasons"])
    assert "out-of-domain status" not in reasons
    assert "unrecognized-capture-request-shape" not in reasons


def test_rosbag_missing_approved_topic_fails_gate_f(tmp_path):
    """F2.7: the full 11-topic approved record set is required."""
    import yaml

    suite_dir = make_complete_evidence_suite(tmp_path)
    metadata = suite_dir / "E" / POSITIVE_ID / "rosbag" / "metadata.yaml"
    document = yaml.safe_load(metadata.read_text(encoding="utf-8"))
    topics = document["rosbag2_bagfile_information"]["topics_with_message_count"]
    topics = [t for t in topics if t["topic_metadata"]["name"] != "/isaac_joint_states"]
    document["rosbag2_bagfile_information"]["topics_with_message_count"] = topics
    metadata.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "missing approved record topics" in " ".join(verdict["reasons"])


def test_rosbag_topic_type_mismatch_fails_gate_f(tmp_path):
    """F2.7: an approved topic with the wrong message type fails."""
    import yaml

    suite_dir = make_complete_evidence_suite(tmp_path)
    metadata = suite_dir / "E" / POSITIVE_ID / "rosbag" / "metadata.yaml"
    document = yaml.safe_load(metadata.read_text(encoding="utf-8"))
    topics = document["rosbag2_bagfile_information"]["topics_with_message_count"]
    for topic in topics:
        if topic["topic_metadata"]["name"] == "/clock":
            topic["topic_metadata"]["type"] = "std_msgs/msg/String"
    document["rosbag2_bagfile_information"]["topics_with_message_count"] = topics
    metadata.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "does not match expected" in " ".join(verdict["reasons"])


# --- Task 9 fix-round-3 F3.8 rosbag QoS/storage closure ----------------------


def _rosbag_document(suite_dir: Path) -> dict[str, object]:
    import yaml

    metadata = suite_dir / "E" / POSITIVE_ID / "rosbag" / "metadata.yaml"
    document = yaml.safe_load(metadata.read_text(encoding="utf-8"))
    assert isinstance(document, dict)
    return document


def _write_rosbag_document(suite_dir: Path, document: object) -> None:
    import yaml

    metadata = suite_dir / "E" / POSITIVE_ID / "rosbag" / "metadata.yaml"
    metadata.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")


def _topic_metadata(document: dict[str, object], topic: str) -> dict[str, object]:
    root = document["rosbag2_bagfile_information"]
    assert isinstance(root, dict)
    for record in root["topics_with_message_count"]:
        assert isinstance(record, dict)
        if record["topic_metadata"]["name"] == topic:
            return record
    raise AssertionError(topic)


def test_rosbag_wrong_qos_reliability_fails_gate_f(tmp_path):
    """F3.8: a best-effort profile on a RELIABLE publisher fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    document = _rosbag_document(suite_dir)
    record = _topic_metadata(document, "/clock")
    record["topic_metadata"]["offered_qos_profiles"] = (
        "- history: 1\n  depth: 10\n  reliability: 2\n  durability: 3\n"
    )
    _write_rosbag_document(suite_dir, document)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "no reliable QoS profile" in " ".join(verdict["reasons"])


def test_rosbag_malformed_qos_fails_gate_f(tmp_path):
    """F3.8: a non-YAML-list offered_qos_profiles fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    document = _rosbag_document(suite_dir)
    record = _topic_metadata(document, "/clock")
    record["topic_metadata"]["offered_qos_profiles"] = "not a yaml list"
    _write_rosbag_document(suite_dir, document)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "not a YAML profile list" in " ".join(verdict["reasons"])


def test_rosbag_malformed_qos_rmw_fields_fails_gate_f(tmp_path):
    """F3.8: an invalid RMW history enum value fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    document = _rosbag_document(suite_dir)
    record = _topic_metadata(document, "/clock")
    record["topic_metadata"]["offered_qos_profiles"] = (
        "- history: 99\n  depth: 10\n  reliability: 1\n  durability: 3\n"
    )
    _write_rosbag_document(suite_dir, document)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "malformed RMW QoS fields" in " ".join(verdict["reasons"])


def test_rosbag_override_qos_mismatch_fails_gate_f(tmp_path):
    """F3.8: /sim/hardware/safety_stop must match the recorder override QoS."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    document = _rosbag_document(suite_dir)
    record = _topic_metadata(document, "/sim/hardware/safety_stop")
    record["topic_metadata"]["offered_qos_profiles"] = (
        "- history: 1\n  depth: 5\n  reliability: 1\n  durability: 1\n"
    )
    _write_rosbag_document(suite_dir, document)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "recorder override contract" in " ".join(verdict["reasons"])


def test_rosbag_missing_storage_file_fails_gate_f(tmp_path):
    """F3.8: a metadata-listed storage file that does not exist fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    document = _rosbag_document(suite_dir)
    root = document["rosbag2_bagfile_information"]
    assert isinstance(root, dict)
    root["relative_file_paths"] = ["missing_0.db3"]
    _write_rosbag_document(suite_dir, document)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "missing storage file" in " ".join(verdict["reasons"])


def test_rosbag_empty_storage_file_fails_gate_f(tmp_path):
    """F3.8: an empty storage file fails (nonzero bytes required)."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "E" / POSITIVE_ID / "rosbag" / "qualification-pick-place-positive_0.db3").write_bytes(b"")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "storage file is empty" in " ".join(verdict["reasons"])


def test_rosbag_extra_conflicting_storage_fails_gate_f(tmp_path):
    """F3.8: a storage file not listed in metadata is a metadata/storage disagreement."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "E" / POSITIVE_ID / "rosbag" / "extra_0.db3").write_bytes(
        b"SQLite format 3\x00" + b"\x00" * 24
    )
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "not listed in metadata" in " ".join(verdict["reasons"])


def test_rosbag_missing_relative_file_paths_fails_gate_f(tmp_path):
    """F3.8: metadata without relative_file_paths is a storage-closure failure."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    document = _rosbag_document(suite_dir)
    root = document["rosbag2_bagfile_information"]
    assert isinstance(root, dict)
    del root["relative_file_paths"]
    _write_rosbag_document(suite_dir, document)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "no relative_file_paths" in " ".join(verdict["reasons"])


def test_cleanup_clean_flag_contradiction_fails_gate_f(tmp_path):
    """F3.1: a lying clean=True that contradicts a recorded owned GPU survivor fails.

    The real RTX producer shape (non-empty ``final.gpus`` inventory,
    non-empty historical ``attempt_owned_pids``) is NOT a contradiction; a
    ``clean:true`` claim that hides a live owned GPU survivor is.
    """
    suite_dir = make_complete_evidence_suite(tmp_path)
    cleanup = suite_dir / "E" / POSITIVE_ID / "resource-cleanup.json"
    value = json.loads(cleanup.read_text(encoding="utf-8"))
    value["attempt_owned_gpu_survivors"] = [{"pid": 999, "name": "isaac", "gpu_index": 0}]
    cleanup.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    reasons = " ".join(verdict["reasons"])
    assert "contradicts the recorded" in reasons


# --- Task 9 fix-round-3 tests (F3.1-F3.10) -----------------------------------


def _rewrite_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")


def _cleanup_value(suite_dir: Path, scenario_id: str = POSITIVE_ID) -> dict[str, object]:
    cleanup = suite_dir / "E" / scenario_id / "resource-cleanup.json"
    value = json.loads(cleanup.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def test_cleanup_real_gpu_inventory_passes_gate_f(tmp_path):
    """F3.1: the real non-empty GPU inventory with historical owned pids passes."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    reasons = " ".join(verdict["reasons"])
    assert "resource cleanup not clean" not in reasons
    assert "contradicts the recorded" not in reasons
    assert "owned pids surviving" not in reasons


def test_cleanup_gpu_topology_identity_change_fails_gate_f(tmp_path):
    """F3.1: a removed/added/mutated GPU identity fails (stable uuid keys)."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    value = _cleanup_value(suite_dir)
    value["final"]["gpus"] = [
        {"index": 0, "uuid": "gpu-other-0000-4a2b-0000", "memory_used_mib": 2048},
        {"index": 1, "uuid": "gpu-6f1a-1111-4a2b-1111", "memory_used_mib": 1024},
    ]
    _rewrite_json(suite_dir / "E" / POSITIVE_ID / "resource-cleanup.json", value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "GPU topology" in " ".join(verdict["reasons"])


def test_cleanup_gpu_topology_removed_gpu_fails_gate_f(tmp_path):
    """F3.1: a GPU present at baseline but absent at final is a topology change."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    value = _cleanup_value(suite_dir)
    value["final"]["gpus"] = [dict(value["baseline"]["gpus"][0])]
    _rewrite_json(suite_dir / "E" / POSITIVE_ID / "resource-cleanup.json", value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "GPU topology" in " ".join(verdict["reasons"])


def test_cleanup_owned_gpu_survivor_fails_gate_f(tmp_path):
    """F3.1: a live attempt-owned GPU survivor fails even when clean is false."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    value = _cleanup_value(suite_dir)
    value["clean"] = False
    value["attempt_owned_gpu_survivors"] = [{"pid": 1234, "name": "isaac", "gpu_index": 0}]
    _rewrite_json(suite_dir / "E" / POSITIVE_ID / "resource-cleanup.json", value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    reasons = " ".join(verdict["reasons"])
    assert "resource cleanup not clean" in reasons
    assert "owned gpu survivor" in reasons


def test_cleanup_unexplained_gpu_memory_fails_gate_f(tmp_path):
    """F3.1: unexplained GPU memory growth fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    value = _cleanup_value(suite_dir)
    value["unexplained_gpu_memory"] = [{"gpu": "gpu-6f1a-0000-4a2b-0000", "unexplained_growth_mib": 8192}]
    _rewrite_json(suite_dir / "E" / POSITIVE_ID / "resource-cleanup.json", value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "unexplained_gpu_memory" in " ".join(verdict["reasons"])


def test_cleanup_historical_owned_pids_do_not_fail_gate_f(tmp_path):
    """F3.1: cumulative historical owned pids are not live survivors."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    value = _cleanup_value(suite_dir)
    value["attempt_owned_pids"] = [1000, 1001, 1002, 1003]
    _rewrite_json(suite_dir / "E" / POSITIVE_ID / "resource-cleanup.json", value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "owned pids surviving" not in " ".join(verdict["reasons"])


def test_cleanup_availability_failure_fails_gate_f(tmp_path):
    """F3.1: a final unavailable snapshot fails even with clean flag true."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    value = _cleanup_value(suite_dir)
    value["final"]["available"] = False
    value["final"]["gpus"] = []
    value["clean"] = True
    _rewrite_json(suite_dir / "E" / POSITIVE_ID / "resource-cleanup.json", value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "contradicts the recorded" in " ".join(verdict["reasons"])


def test_overlay_path_scope_scalar_is_accepted(tmp_path):
    """F3.2: the real scalar ``repositories.path_scope`` is accepted."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "path_scope" not in " ".join(verdict["reasons"])
    assert verdict["status"] == "verified-fail" or "not an object" not in " ".join(verdict["reasons"])


def test_overlay_scalar_garbage_under_repository_key_fails(tmp_path):
    """F3.2: a scalar under a repository mapping key still fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    overlay = suite_dir / "ompl-overlay-contract.json"
    value = json.loads(overlay.read_text(encoding="utf-8"))
    value["repositories"]["production"] = "not-a-mapping"
    _rewrite_json(overlay, value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "production" in " ".join(verdict["reasons"])


def test_overlay_missing_production_fails(tmp_path):
    """F3.2: removing the production repository mapping fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    overlay = suite_dir / "ompl-overlay-contract.json"
    value = json.loads(overlay.read_text(encoding="utf-8"))
    del value["repositories"]["production"]
    _rewrite_json(overlay, value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "production" in " ".join(verdict["reasons"])


def test_overlay_malformed_identity_fails(tmp_path):
    """F3.2: a non-40-hex implementation identity fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    overlay = suite_dir / "ompl-overlay-contract.json"
    value = json.loads(overlay.read_text(encoding="utf-8"))
    value["repositories"]["simulator"]["implementation_identity"] = "A" * 40
    _rewrite_json(overlay, value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "40-hex" in " ".join(verdict["reasons"])


def test_overlay_malformed_path_scope_fails(tmp_path):
    """F3.2: a non-scalar ``path_scope`` is malformed."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    overlay = suite_dir / "ompl-overlay-contract.json"
    value = json.loads(overlay.read_text(encoding="utf-8"))
    value["repositories"]["path_scope"] = {"nested": True}
    _rewrite_json(overlay, value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "path_scope" in " ".join(verdict["reasons"])


def test_overlay_source_locks_missing_status_fails(tmp_path):
    """F3.2: source_locks must be a mapping with a truthful non-empty status."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    overlay = suite_dir / "ompl-overlay-contract.json"
    value = json.loads(overlay.read_text(encoding="utf-8"))
    del value["source_locks"]["status"]
    _rewrite_json(overlay, value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "source_locks" in " ".join(verdict["reasons"])


def test_overlay_source_locks_binding_missing_file_fails(tmp_path):
    """F3.2: simulator_lock_path must resolve to an existing source-lock artifact."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "integration" / "source-locks.json").unlink()
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "source-locks.json" in " ".join(verdict["reasons"])


# --- F3.3 per-attempt visual evidence closure --------------------------------


def _add_sibling_cancel(suite_dir: Path) -> Path:
    """Add a second cancel scenario (cancel-transport) as a full attempt dir."""
    _write_attempt_dir(
        suite_dir, CANCEL_TRANSPORT_ID,
        attempt_id="attempt-cancel-transport",
        include_rosbag=False,
        include_planning_scene=False,
    )
    return suite_dir


def _add_sibling_safety(suite_dir: Path) -> Path:
    """Add a second safety scenario (moveit-safety) as a full attempt dir."""
    _write_attempt_dir(
        suite_dir, MOVEIT_SAFETY_ID,
        attempt_id="attempt-moveit-safety",
        include_rosbag=False,
        include_planning_scene=False,
    )
    return suite_dir


def test_missing_sibling_attempt_fails_per_attempt(tmp_path):
    """F3.3: a sibling cancel attempt never satisfies another attempt's events."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    _add_sibling_cancel(suite_dir)
    # Remove every capture/keyframe from the transport sibling: the approach
    # sibling still has all cancel events, but transport must prove its own.
    for keyframe in (suite_dir / "E" / CANCEL_TRANSPORT_ID / "visual-keyframes.jsonl",):
        keyframe.unlink()
    for png in (suite_dir / "E" / CANCEL_TRANSPORT_ID / "visual/source").glob("*.png"):
        png.unlink()
    index = _final_index(suite_dir)
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    reasons = " ".join(verdict["reasons"])
    assert verdict["status"] == "verified-fail"
    assert CANCEL_TRANSPORT_ID in reasons
    assert "missing required" in reasons


def test_split_events_across_siblings_fails_per_attempt(tmp_path):
    """F3.3: cancel events split across two siblings still fail for both."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    _add_sibling_cancel(suite_dir)
    # Approach keeps cancel-execution-start + cancel-trigger; transport keeps
    # cancel-velocity-compliant + cancel-terminal.  Neither has the full set.
    approach = suite_dir / "E" / CANCEL_ID / "visual-keyframes.jsonl"
    approach_rows = [
        r for r in (json.loads(line) for line in approach.read_text(encoding="utf-8").splitlines())
        if r["event"] in ("cancel-execution-start", "cancel-trigger")
    ]
    _rewrite_jsonl(approach, approach_rows)
    transport = suite_dir / "E" / CANCEL_TRANSPORT_ID / "visual-keyframes.jsonl"
    transport_rows = [
        r for r in (json.loads(line) for line in transport.read_text(encoding="utf-8").splitlines())
        if r["event"] in ("cancel-velocity-compliant", "cancel-terminal")
    ]
    _rewrite_jsonl(transport, transport_rows)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    reasons = " ".join(verdict["reasons"])
    assert verdict["status"] == "verified-fail"
    assert CANCEL_ID in reasons and CANCEL_TRANSPORT_ID in reasons
    assert "missing required" in reasons


def test_duplicate_event_across_attempts_fails(tmp_path):
    """F3.3: the same (event, camera) identity in two attempts is a duplicate."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    _add_sibling_cancel(suite_dir)
    # Append the approach's first keyframe into the transport journal: the
    # (event, camera) identity is now owned by two attempts.
    approach = suite_dir / "E" / CANCEL_ID / "visual-keyframes.jsonl"
    first = json.loads(approach.read_text(encoding="utf-8").splitlines()[0])
    transport = suite_dir / "E" / CANCEL_TRANSPORT_ID / "visual-keyframes.jsonl"
    with transport.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(first, sort_keys=True) + "\n")
    index = _final_index(suite_dir)
    codes = {d["code"] for d in index.get("diagnostics", [])}
    assert "duplicate-keyframe-identity" in codes
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


def test_partial_sheet_metadata_with_complete_global_captures_fails(tmp_path):
    """F3.3: a sheet embedding only a subset of events fails even when the global
    capture set is complete."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    render_sheets(suite_dir)
    # Rewrite the agent sheet's embedded metadata to a partial event set while
    # leaving every capture on disk untouched.
    from PIL import Image, PngImagePlugin

    from validation.integrated_contact_sheets import AGENT_NAME as AGENT_FILE

    sheet_path = suite_dir / AGENT_FILE
    with Image.open(sheet_path) as image:
        image.load()
        metadata = json.loads(image.text["tinker.qualification.metadata"])
    metadata["events"] = list(POSITIVE_EVENTS)
    pnginfo = PngImagePlugin.PngInfo()
    pnginfo.add_text("tinker.qualification.metadata", json.dumps(metadata, sort_keys=True))
    image_out = Image.open(sheet_path)
    image_out.load()
    image_out.save(sheet_path, format="PNG", pnginfo=pnginfo)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "complete required suite event sequence" in " ".join(verdict["reasons"])


def test_contact_sheet_event_order_mismatch_fails(tmp_path):
    """F3.3: a sheet embedding the right events in the wrong order fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    render_sheets(suite_dir)
    from PIL import Image, PngImagePlugin

    from validation.integrated_contact_sheets import AGENT_NAME as AGENT_FILE

    sheet_path = suite_dir / AGENT_FILE
    with Image.open(sheet_path) as image:
        image.load()
        metadata = json.loads(image.text["tinker.qualification.metadata"])
    full = list(REQUIRED_POSITIVE_EVENTS + CANCEL_EVENTS + SAFETY_EVENTS)
    metadata["events"] = list(reversed(full))
    pnginfo = PngImagePlugin.PngInfo()
    pnginfo.add_text("tinker.qualification.metadata", json.dumps(metadata, sort_keys=True))
    image_out = Image.open(sheet_path)
    image_out.load()
    image_out.save(sheet_path, format="PNG", pnginfo=pnginfo)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "complete required suite event sequence" in " ".join(verdict["reasons"])


def test_wrong_scenario_id_binding_fails(tmp_path):
    """F3.3: a keyframe whose gate disagrees with its request/attempt fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    keyframes = suite_dir / "E" / CANCEL_ID / "visual-keyframes.jsonl"
    rows = [json.loads(line) for line in keyframes.read_text(encoding="utf-8").splitlines()]
    rows[0]["gate"] = SAFETY_ID
    _rewrite_jsonl(keyframes, rows)
    index = _final_index(suite_dir)
    codes = {d["code"] for d in index.get("diagnostics", [])}
    assert "keyframe-request-sequence-mismatch" in codes or "keyframe-scenario-mismatch" in codes
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


# --- F3.5 keyframe request-time / source-sequence binding --------------------


def _first_keyframe_rows(suite_dir: Path) -> list[dict[str, object]]:
    keyframes = suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl"
    return [json.loads(line) for line in keyframes.read_text(encoding="utf-8").splitlines()]


def test_keyframe_request_time_mismatch_fails(tmp_path):
    """F3.5: requested_simulated_timestamp disagreeing with the request fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    rows = _first_keyframe_rows(suite_dir)
    rows[0]["requested_simulated_timestamp"] = float(rows[0]["requested_simulated_timestamp"]) + 0.1
    _rewrite_jsonl(suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl", rows)
    index = _final_index(suite_dir)
    assert any(d.get("code") == "keyframe-request-time-mismatch" for d in index.get("diagnostics", []))
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


def test_keyframe_request_frame_mismatch_fails(tmp_path):
    """F3.5: requested_physics_frame_index disagreeing with the raw frame fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    rows = _first_keyframe_rows(suite_dir)
    rows[0]["requested_physics_frame_index"] = int(rows[0]["requested_physics_frame_index"]) + 1
    _rewrite_jsonl(suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl", rows)
    index = _final_index(suite_dir)
    assert any(d.get("code") == "keyframe-request-frame-mismatch" for d in index.get("diagnostics", []))
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


def test_keyframe_latency_out_of_range_fails(tmp_path):
    """F3.5: capture_latency_frames beyond [0, MAX_CAPTURE_LATENCY_FRAMES] fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    rows = _first_keyframe_rows(suite_dir)
    rows[0]["capture_latency_frames"] = 5
    _rewrite_jsonl(suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl", rows)
    index = _final_index(suite_dir)
    assert any(d.get("code") == "keyframe-latency-out-of-range" for d in index.get("diagnostics", []))
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


def test_keyframe_latency_negative_fails(tmp_path):
    """F3.5: a negative capture_latency_frames fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    rows = _first_keyframe_rows(suite_dir)
    rows[0]["capture_latency_frames"] = -1
    _rewrite_jsonl(suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl", rows)
    index = _final_index(suite_dir)
    assert any(d.get("code") == "keyframe-latency-out-of-range" for d in index.get("diagnostics", []))
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


def test_keyframe_source_sequence_mismatch_fails(tmp_path):
    """F3.5: execution_event_sequence disagreeing with source_execution_event_sequence fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    rows = _first_keyframe_rows(suite_dir)
    rows[0]["execution_event_sequence"] = int(rows[0]["execution_event_sequence"]) + 1
    _rewrite_jsonl(suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl", rows)
    index = _final_index(suite_dir)
    assert any(d.get("code") == "keyframe-source-sequence-mismatch" for d in index.get("diagnostics", []))
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


def test_duplicate_capture_path_fails(tmp_path):
    """F3.5: two keyframes writing the same capture path is a duplicate.

    rows[1] is readiness/manipulation_closeup; pointing it at the readiness/
    overview PNG gives a distinct (event, camera) identity but a duplicate
    canonical capture path.
    """
    suite_dir = make_complete_evidence_suite(tmp_path)
    rows = _first_keyframe_rows(suite_dir)
    rows[1]["path"] = rows[0]["path"]
    _rewrite_jsonl(suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl", rows)
    index = _final_index(suite_dir)
    codes = {d["code"] for d in index.get("diagnostics", [])}
    assert "duplicate-capture-path" in codes
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


def test_stray_source_png_without_keyframe_fails(tmp_path):
    """F3.5: a visual/source/*.png with no keyframe is an unbound capture."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "E" / POSITIVE_ID / "visual/source/0099-stray-extra.png").write_bytes(
        (suite_dir / "E" / POSITIVE_ID / "visual/source/0001-readiness-overview.png").read_bytes()
    )
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "unbound capture" in " ".join(verdict["reasons"])


# --- F3.6 identity / relocation binding closure ------------------------------


def test_verdict_schema_version_mismatch_fails(tmp_path):
    """F3.6: a verdict with the wrong schema_version fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    verdict_path = suite_dir / "E" / POSITIVE_ID / "gate-verdict.json"
    value = json.loads(verdict_path.read_text(encoding="utf-8"))
    value["schema_version"] = 2
    _rewrite_json(verdict_path, value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "schema_version" in " ".join(verdict["reasons"])


def test_verdict_not_pass_verified_fails(tmp_path):
    """F3.6: a verdict that is not pass+verified fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    verdict_path = suite_dir / "E" / POSITIVE_ID / "gate-verdict.json"
    value = json.loads(verdict_path.read_text(encoding="utf-8"))
    value["pass"] = False
    _rewrite_json(verdict_path, value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "not pass+verified" in " ".join(verdict["reasons"])


def test_verdict_missing_enclosing_manifest_fails(tmp_path):
    """F3.6: a missing enclosing manifest is a failure, never a skipped check."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "E" / POSITIVE_ID / "manifest.json").unlink()
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "no enclosing manifest" in " ".join(verdict["reasons"])


def test_bundle_scenario_relocation_fails(tmp_path):
    """F3.6: scenario-bundle scenario_id disagreeing with manifest scenario.id fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    bundle = suite_dir / "E" / POSITIVE_ID / "scenario-bundle.json"
    value = json.loads(bundle.read_text(encoding="utf-8"))
    value["scenario_id"] = CANCEL_ID
    _rewrite_json(bundle, value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "does not match manifest scenario.id" in " ".join(verdict["reasons"])


def test_config_seed_mismatch_fails(tmp_path):
    """F3.6: a config seed disagreeing with the manifest seed fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    config = suite_dir / "config" / "integrated-ompl.json"
    value = json.loads(config.read_text(encoding="utf-8"))
    value["seed"] = 99
    _rewrite_json(config, value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "config seed does not match manifest seed" in " ".join(verdict["reasons"])


def test_source_identities_overlay_mismatch_fails(tmp_path):
    """F3.6: source-identities disagreeing with the overlay contract fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    identities = suite_dir / f"gate-b-{SUITE_ATTEMPT}" / "source-identities.json"
    value = json.loads(identities.read_text(encoding="utf-8"))
    value["source_identities"]["production"] = "c" * 40
    _rewrite_json(identities, value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "does not match overlay" in " ".join(verdict["reasons"])


def test_model_fingerprint_static_mismatch_fails(tmp_path):
    """F3.6: model-fingerprint disagreeing with the static contract fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    fingerprint = suite_dir / f"gate-b-{SUITE_ATTEMPT}" / "model-fingerprint.json"
    value = json.loads(fingerprint.read_text(encoding="utf-8"))
    value["model_fingerprint"] = "cd" * 32
    _rewrite_json(fingerprint, value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "model fingerprint does not match static contract" in " ".join(verdict["reasons"])


# --------------------------------------------------------------------------- #
# F3.4 true integrated producer path
#
# The end-to-end transaction is driven by the real producers, never by
# hand-written canonical journals: the integrated executor's
# ``_append_visual_request`` / ``_append_visual_event`` write the request journal
# at durable checkpoints, the real ``QualificationVisualCapture`` (fake
# app/backend) consumes those requests into source PNGs + keyframes, and the
# validator's index/sheet/summary/Gate-F pipeline accepts the result only when
# every artifact is semantically valid.  A separate diagnostic-only journal test
# proves consumer skip + validator ignore + required-events fail-closed.
# ---------------------------------------------------------------------------


class _F34JournalStub:
    """Producer-facing journal double exposing only the durable checkpoint count."""

    def __init__(self, record_count: int = 0) -> None:
        self.record_count = int(record_count)


class _F34Backend:
    """Deterministic physics-truth double for the capture-consumer backend seam."""

    def __init__(self, dt: float) -> None:
        self.dt = dt
        self.physics_frame_index = 0
        self.simulation_time = 0.0

    def render_frame(self) -> None:
        return None


class _F34Sensor:
    def __init__(self, array: object) -> None:
        self.array = array

    def get_data(self, annotator: str):
        if annotator == "rgb":
            return self.array, {"width": 960, "height": 540, "frames": 1}
        return None, {}

    def close(self) -> None:
        return None


def _f34_rgb_array():
    import numpy as np

    return (
        np.arange(540 * 960 * 3, dtype=np.int32).reshape(540, 960, 3) % 256
    ).astype(np.uint8)


def _f34_producer(attempt_dir: Path, frames: list[int]):
    """Build a real ``IntegratedGateExecutor`` harness bound to one attempt dir.

    The harness reaches the real ``_join_key`` / ``_append_visual_request`` /
    ``_append_visual_event`` producer machinery with a strictly-advancing
    physics-truth join key and a durable journal checkpoint counter — no ROS, no
    Isaac, no camera.  The join key is the strict current physics-truth tail and
    the durable checkpoint count is the source execution sequence.
    """
    from validation.integrated_gate_executor import IntegratedGateExecutor

    executor = object.__new__(IntegratedGateExecutor)
    executor.attempt_dir = Path(attempt_dir)
    executor._emitted_visual_events = set()
    executor._visual_event_failures = []
    executor._visual_event_sequence = 0
    executor._last_join_key = None
    executor.journal = _F34JournalStub(record_count=0)
    cursor = {"n": 0}

    def _advancing_join():
        frame = frames[cursor["n"]]
        cursor["n"] += 1
        return (frame, frame / PHYSICS_HZ)

    executor.join_key_provider = _advancing_join
    return executor


def _f34_emit_positive(executor, scenario_id: str) -> list[str]:
    """Emit the real executor diagnostic + canonical requests for a positive flow."""
    spec = {"target_pose": None}
    executor._append_visual_request("before", scenario_id, spec, kind="gate-e-diagnostic")
    executor._append_visual_request("before-pick", scenario_id, spec, kind="gate-e-diagnostic")
    statuses: list[str] = []
    for index, event in enumerate(REQUIRED_POSITIVE_EVENTS):
        executor.journal.record_count = 10 + index
        key = executor._join_key()
        assert key is not None
        statuses.append(executor._append_visual_event(event, scenario_id))
    executor._append_visual_request("terminal", scenario_id, spec, kind="gate-e-diagnostic")
    return statuses


def _f34_capture(
    monkeypatch,
    attempt_dir: Path,
    gate: str,
    backend,
    frames: list[int],
    *,
    latency: int = CAPTURE_LATENCY,
):
    """Drive the real ``QualificationVisualCapture`` through its env factory.

    The backend renders ``latency`` physics frames past the requested frame, so
    the real producer emits nonzero ``capture_latency_frames`` (F4.1).
    """
    from simulation.tinker_sim_isaac.qualification_visual_capture import (
        QualificationVisualCapture,
    )

    sensors = {
        "overview": _F34Sensor(_f34_rgb_array()),
        "manipulation_closeup": _F34Sensor(_f34_rgb_array()),
    }

    def _fake_initialize(self):
        self._sensors = dict(sensors)

    monkeypatch.setattr(QualificationVisualCapture, "_initialize_cameras", _fake_initialize)
    monkeypatch.setenv("TINKER_SIM_VISUAL_EVIDENCE", "1")
    monkeypatch.setenv("TINKER_SIM_ATTEMPT_DIR", str(attempt_dir))
    monkeypatch.setenv("TINKER_SIM_QUALIFICATION_GATE", gate)
    capture = QualificationVisualCapture.from_environment(app=object(), backend=backend)
    assert capture is not None
    for frame in frames:
        backend.physics_frame_index = frame + latency
        backend.simulation_time = (frame + latency) / PHYSICS_HZ
        capture.poll()
    return capture


def _f34_request_rows(attempt_dir: Path) -> list[dict[str, object]]:
    return [
        json.loads(line)
        for line in (attempt_dir / "visual-capture-requests.jsonl")
        .read_text(encoding="utf-8")
        .strip()
        .splitlines()
    ]


def test_f34_integrated_producer_path_gate_f_verified_pass(tmp_path, monkeypatch):
    """F3.4: real executor producer -> real capture consumer -> index/sheets/
    summary -> Gate F ``verified-pass`` accepts genuine artifacts."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    attempt_dir = suite_dir / "E" / POSITIVE_ID

    # Drop the factory-written visual transaction; it is re-produced below by the
    # real executor producer and real capture consumer.
    (attempt_dir / "visual-capture-requests.jsonl").unlink()
    (attempt_dir / "visual-keyframes.jsonl").unlink()
    for png in (attempt_dir / "visual/source").glob("*.png"):
        png.unlink()

    # 1+2. Real executor canonical requests at durable checkpoints.
    frames = [10, 20, 30, 40, 50, 60, 70]
    executor = _f34_producer(attempt_dir, frames)
    statuses = _f34_emit_positive(executor, POSITIVE_ID)
    assert statuses == ["recorded"] * len(REQUIRED_POSITIVE_EVENTS)

    requests = _f34_request_rows(attempt_dir)
    sequence_rows = [
        r for r in requests
        if isinstance(r.get("sequence"), int) and not isinstance(r.get("sequence"), bool)
    ]
    diagnostic_rows = [r for r in requests if r.get("diagnostic_only") is True]
    assert len(sequence_rows) == len(REQUIRED_POSITIVE_EVENTS)
    assert len(diagnostic_rows) == 3
    for index, row in enumerate(sequence_rows):
        assert row["schema_version"] == 1
        assert row["sequence"] == index + 1
        assert row["gate"] == POSITIVE_ID
        assert row["event"] == REQUIRED_POSITIVE_EVENTS[index]
        assert row["simulated_timestamp"] == frames[index] / PHYSICS_HZ
        assert row["source_execution_event_sequence"] == 10 + index

    # 3+4. Real capture consumer produces source PNGs + keyframes (env-derived gate).
    backend = _F34Backend(dt=1.0 / PHYSICS_HZ)
    capture = _f34_capture(monkeypatch, attempt_dir, POSITIVE_ID, backend, frames)
    assert capture._errors == []
    capture.close()

    pngs = sorted((attempt_dir / "visual/source").glob("*.png"))
    assert len(pngs) == 2 * len(REQUIRED_POSITIVE_EVENTS)
    keyframes = [
        json.loads(line)
        for line in (attempt_dir / "visual-keyframes.jsonl")
        .read_text(encoding="utf-8")
        .strip()
        .splitlines()
    ]
    assert len(keyframes) == 2 * len(REQUIRED_POSITIVE_EVENTS)
    by_event: dict[str, int] = {}
    for record in keyframes:
        assert record["gate"] == POSITIVE_ID
        assert record["capture_latency_frames"] == CAPTURE_LATENCY
        assert record["max_capture_latency_frames"] == 4
        assert record["requested_physics_frame_index"] == record["raw_frame_index"] - CAPTURE_LATENCY
        assert (attempt_dir / record["path"]).is_file()
        by_event[record["event"]] = by_event.get(record["event"], 0) + 1
    assert set(by_event) == set(REQUIRED_POSITIVE_EVENTS)
    assert all(count == 2 for count in by_event.values())

    # 5. The remaining production-shaped artifacts are the factory's (raw/
    # evaluator/drain, scene, MoveIt/controller, verdict, cleanup, rosbag,
    # source/provenance).  6+7. Index + sheets + summary + Gate F verified-pass.
    rebuild_index(suite_dir)
    render_sheets(suite_dir)
    summary = build_qualification_summary(suite_dir)
    assert summary["status"] == "verified-pass", summary["reasons"]
    assert summary["reasons"] == []
    final = _final_index(suite_dir)
    verdict = validate_gate_f(final, suite_dir=suite_dir)
    assert verdict["status"] == "verified-pass", verdict["reasons"]


def test_f34_diagnostic_only_journal_fails_closed(tmp_path, monkeypatch):
    """F3.4: an attempt whose request journal holds only executor diagnostic
    records proves consumer skip + validator ignore + required-events fail-closed."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    attempt_dir = suite_dir / "E" / POSITIVE_ID

    # Replace the real visual transaction with ONLY executor diagnostic records.
    (attempt_dir / "visual-capture-requests.jsonl").unlink()
    (attempt_dir / "visual-keyframes.jsonl").unlink()
    for png in (attempt_dir / "visual/source").glob("*.png"):
        png.unlink()
    executor = _f34_producer(attempt_dir, [10])
    spec = {"target_pose": None}
    for phase in ("before", "before-pick", "after", "terminal"):
        executor._append_visual_request(phase, POSITIVE_ID, spec, kind="gate-e-diagnostic")

    # Consumer skip: polling the diagnostic-only journal never captures.
    backend = _F34Backend(dt=1.0 / PHYSICS_HZ)
    capture = _f34_capture(monkeypatch, attempt_dir, POSITIVE_ID, backend, [10])
    assert capture._errors == []
    assert capture._handled_sequences == set()
    assert not (attempt_dir / "visual-keyframes.jsonl").exists()
    assert not list((attempt_dir / "visual/source").glob("*.png"))

    # Validator ignore: the diagnostic records are recognized, never misparsed,
    # and never drive capture.
    rebuild_index(suite_dir)
    final = _final_index(suite_dir)
    assert final.get("diagnostics") == []
    # Required-events fail-closed: POSITIVE_ID has no canonical capture event.
    verdict = validate_gate_f(final, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"
    joined = " ".join(verdict["reasons"])
    assert "missing required positive visual events" in joined
    assert "unrecognized-capture-request-shape" not in joined
    assert "capture-request-without-image" not in joined


# --------------------------------------------------------------------------- #
# Task 9 fix round 4 (F4.1-F4.5) — real capture latency / nine-field QoS /
# canonical CLI order / verbatim overlay artifacts / attempt-key closure
# --------------------------------------------------------------------------- #


def _first_keyframe_path(suite_dir: Path) -> Path:
    return suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl"


def _nine_field_qos(
    history: int = 1,
    depth: int = 10,
    reliability: int = 1,
    durability: int = 3,
    deadline: int = 0,
    lifespan: int = 0,
    liveliness: int = 1,
    liveliness_lease_duration: int = 0,
    avoid_ros_namespace_conventions: bool = True,
) -> str:
    """Real Humble rosbag2 nine-field ``rmw_qos_profile_t`` YAML serialization."""
    return (
        f"- history: {history}\n"
        f"  depth: {depth}\n"
        f"  reliability: {reliability}\n"
        f"  durability: {durability}\n"
        f"  deadline: {deadline}\n"
        f"  lifespan: {lifespan}\n"
        f"  liveliness: {liveliness}\n"
        f"  liveliness_lease_duration: {liveliness_lease_duration}\n"
        f"  avoid_ros_namespace_conventions: {str(avoid_ros_namespace_conventions).lower()}\n"
    )


def _trim_attempt_visual(attempt_dir: Path, keep_events: set[str]) -> None:
    """Keep only the keyframes/requests/PNGs for the given events in an attempt."""
    kf_path = attempt_dir / "visual-keyframes.jsonl"
    req_path = attempt_dir / "visual-capture-requests.jsonl"
    kf_rows = [json.loads(line) for line in kf_path.read_text(encoding="utf-8").splitlines()]
    req_rows = [json.loads(line) for line in req_path.read_text(encoding="utf-8").splitlines()]
    kf_rows = [row for row in kf_rows if row["event"] in keep_events]
    kept_paths = {row["path"] for row in kf_rows}
    for png in (attempt_dir / "visual/source").glob("*.png"):
        if png.relative_to(attempt_dir).as_posix() not in kept_paths:
            png.unlink()
    # Keep canonical sequence requests only for kept events; executor diagnostic
    # records (no int sequence) stay co-tenanted.
    req_rows = [
        row
        for row in req_rows
        if (isinstance(row.get("sequence"), int) and row.get("event") in keep_events)
        or not isinstance(row.get("sequence"), int)
    ]
    _rewrite_jsonl(kf_path, kf_rows)
    _rewrite_jsonl(req_path, req_rows)


# --- F4.1 real capture-latency arithmetic -----------------------------------


def test_factory_latency_two_records_valid(tmp_path):
    """F4.1: the production-shaped factory emits latency-2 keyframes that satisfy
    the real arithmetic (raw - requested == latency) and pass Gate F."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    rows = [json.loads(line) for line in _first_keyframe_path(suite_dir).read_text(encoding="utf-8").splitlines()]
    assert rows
    for row in rows:
        assert row["capture_latency_frames"] == CAPTURE_LATENCY
        assert row["raw_frame_index"] - row["requested_physics_frame_index"] == CAPTURE_LATENCY
        expected_rounded = int(math.floor(row["requested_simulated_timestamp"] / row["physics_dt"] + 0.5))
        assert row["requested_physics_frame_index"] == expected_rounded
    render_sheets(suite_dir)
    summary = build_qualification_summary(suite_dir)
    assert summary["status"] == "verified-pass", summary["reasons"]


def test_keyframe_wrong_requested_rounding_fails(tmp_path):
    """F4.1: a requested frame that is not the producer's exact rounded-frame
    calculation from requested time / physics dt fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    rows = _first_keyframe_rows(suite_dir)
    rows[0]["requested_physics_frame_index"] = int(rows[0]["requested_physics_frame_index"]) - 1
    _rewrite_jsonl(suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl", rows)
    index = _final_index(suite_dir)
    codes = {d["code"] for d in index.get("diagnostics", [])}
    assert "keyframe-request-frame-mismatch" in codes
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


def test_keyframe_requested_frame_invalid_fails(tmp_path):
    """F4.1: a missing/non-integer requested_physics_frame_index is invalid."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    rows = _first_keyframe_rows(suite_dir)
    rows[0]["requested_physics_frame_index"] = None
    _rewrite_jsonl(suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl", rows)
    index = _final_index(suite_dir)
    codes = {d["code"] for d in index.get("diagnostics", [])}
    assert "keyframe-request-frame-invalid" in codes
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


def test_keyframe_latency_delta_mismatch_fails(tmp_path):
    """F4.1: a latency field inconsistent with the frame delta (raw - requested)
    fails even when it is within the [0, MAX] range."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    rows = _first_keyframe_rows(suite_dir)
    assert rows[0]["raw_frame_index"] - rows[0]["requested_physics_frame_index"] == CAPTURE_LATENCY
    rows[0]["capture_latency_frames"] = CAPTURE_LATENCY - 1
    _rewrite_jsonl(suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl", rows)
    index = _final_index(suite_dir)
    codes = {d["code"] for d in index.get("diagnostics", [])}
    assert "keyframe-latency-delta-mismatch" in codes
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


def test_keyframe_latency_noninteger_fails(tmp_path):
    """F4.1: a non-integer capture_latency_frames fails (out-of-range)."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    rows = _first_keyframe_rows(suite_dir)
    rows[0]["capture_latency_frames"] = 1.5
    _rewrite_jsonl(suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl", rows)
    index = _final_index(suite_dir)
    codes = {d["code"] for d in index.get("diagnostics", [])}
    assert "keyframe-latency-out-of-range" in codes
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


# --- F5.2 required latency/execution-sequence fields cannot be absent --------


def test_keyframe_latency_missing_fails(tmp_path):
    """F5.2: capture_latency_frames is mandatory; a missing value is a critical
    diagnostic (never a silently skipped check)."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    rows = _first_keyframe_rows(suite_dir)
    del rows[0]["capture_latency_frames"]
    _rewrite_jsonl(suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl", rows)
    index = _final_index(suite_dir)
    codes = {d["code"] for d in index.get("diagnostics", [])}
    assert "keyframe-latency-missing" in codes
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


def test_keyframe_execution_sequence_missing_fails(tmp_path):
    """F5.2: execution_event_sequence is mandatory on the keyframe side; a
    missing value is a critical diagnostic."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    rows = _first_keyframe_rows(suite_dir)
    del rows[0]["execution_event_sequence"]
    _rewrite_jsonl(suite_dir / "E" / POSITIVE_ID / "visual-keyframes.jsonl", rows)
    index = _final_index(suite_dir)
    codes = {d["code"] for d in index.get("diagnostics", [])}
    assert "keyframe-execution-sequence-missing" in codes
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


def test_request_source_sequence_missing_fails(tmp_path):
    """F5.2: source_execution_event_sequence is mandatory on the canonical
    request side; a missing value is a critical diagnostic even when the
    keyframe side is present."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    request_path = suite_dir / "E" / POSITIVE_ID / "visual-capture-requests.jsonl"
    rows = [json.loads(line) for line in request_path.read_text(encoding="utf-8").splitlines()]
    for row in rows:
        if isinstance(row.get("sequence"), int) and not isinstance(row.get("sequence"), bool):
            del row["source_execution_event_sequence"]
    _rewrite_jsonl(request_path, rows)
    index = _final_index(suite_dir)
    codes = {d["code"] for d in index.get("diagnostics", [])}
    assert "keyframe-source-sequence-missing" in codes
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    assert verdict["status"] == "verified-fail"


def test_keyframe_latency_two_still_passes(tmp_path):
    """F5.2: the real latency-2 relation (raw = requested + latency) remains a
    valid pass; requested is never restored to equal raw."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    rows = [json.loads(line) for line in _first_keyframe_path(suite_dir).read_text(encoding="utf-8").splitlines()]
    assert rows
    for row in rows:
        assert row["capture_latency_frames"] == CAPTURE_LATENCY
        assert row["raw_frame_index"] - row["requested_physics_frame_index"] == CAPTURE_LATENCY
    render_sheets(suite_dir)
    summary = build_qualification_summary(suite_dir)
    assert summary["status"] == "verified-pass", summary["reasons"]


# --- F4.2 real nine-field Humble rosbag2 QoS profiles ------------------------


def test_rosbag_nine_field_qos_passes_gate_f(tmp_path):
    """F4.2: the real nine-field rmw_qos_profile_t passes for all approved topics,
    including the recorder override subset on the two override topics."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    render_sheets(suite_dir)
    summary = build_qualification_summary(suite_dir)
    assert summary["status"] == "verified-pass", summary["reasons"]
    assert "recorder override contract" not in " ".join(summary["reasons"])


def test_rosbag_override_wrong_required_field_fails(tmp_path):
    """F4.2: an override topic with the wrong depth (still nine-field) fails the
    recorder override subset contract."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    document = _rosbag_document(suite_dir)
    record = _topic_metadata(document, "/sim/hardware/safety_stop")
    record["topic_metadata"]["offered_qos_profiles"] = _nine_field_qos(depth=5, durability=1)
    _write_rosbag_document(suite_dir, document)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "recorder override contract" in " ".join(verdict["reasons"])


def test_rosbag_malformed_extra_field_fails(tmp_path):
    """F4.2: a present-but-malformed extra rmw field (negative deadline) fails
    the real schema."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    document = _rosbag_document(suite_dir)
    record = _topic_metadata(document, "/clock")
    record["topic_metadata"]["offered_qos_profiles"] = _nine_field_qos(deadline=-5)
    _write_rosbag_document(suite_dir, document)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "malformed RMW QoS fields" in " ".join(verdict["reasons"])


def test_rosbag_malformed_liveliness_extra_fails(tmp_path):
    """F4.2: an out-of-range liveliness enum value in a real profile fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    document = _rosbag_document(suite_dir)
    record = _topic_metadata(document, "/clock")
    record["topic_metadata"]["offered_qos_profiles"] = _nine_field_qos(liveliness=99)
    _write_rosbag_document(suite_dir, document)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "malformed RMW QoS fields" in " ".join(verdict["reasons"])


def test_rosbag_missing_required_field_fails(tmp_path):
    """F4.2: a profile missing a required field (depth) fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    document = _rosbag_document(suite_dir)
    record = _topic_metadata(document, "/clock")
    text = _nine_field_qos()
    record["topic_metadata"]["offered_qos_profiles"] = text.replace("  depth: 10\n", "")
    _write_rosbag_document(suite_dir, document)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "malformed RMW QoS fields" in " ".join(verdict["reasons"])


def test_rosbag_arbitrary_string_qos_fails(tmp_path):
    """F4.2: an arbitrary non-YAML-list string for offered_qos_profiles fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    document = _rosbag_document(suite_dir)
    record = _topic_metadata(document, "/sim/status/contract")
    record["topic_metadata"]["offered_qos_profiles"] = "reliable"
    _write_rosbag_document(suite_dir, document)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "not a YAML profile list" in " ".join(verdict["reasons"])


# --- F4.3 canonical production CLI sheet event order -------------------------


def test_cli_path_order_would_be_cancel_positive_safety(tmp_path):
    """F4.3: the index's raw path-sorted capture order is cancel-first; the
    production helper corrects it to positive -> cancel -> safety."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    index = json.loads((suite_dir / INDEX_NAME).read_text(encoding="utf-8"))
    path_sorted = list(
        dict.fromkeys(
            entry["event"]
            for entry in index["files"]
            if entry.get("category") == "capture" and entry.get("bound")
        )
    )
    assert path_sorted[0] in CANCEL_EVENTS  # raw path order is cancel-first
    from validation.integrated_contact_sheets import _all_bound_capture_entries

    entries = _all_bound_capture_entries(suite_dir)
    assert [entry["event"] for entry in entries] == list(
        POSITIVE_EVENTS + CANCEL_EVENTS + SAFETY_EVENTS
    )


def test_cli_multi_scenario_sheets_canonical_order_gate_f_pass(tmp_path):
    """F4.3: the real CLI main() over a positive+cancel+safety suite embeds the
    canonical ordered events in both sheets and reaches Gate F verified-pass."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    from validation.integrated_contact_sheets import main

    assert main(["--suite-dir", str(suite_dir)]) == 0
    agent_meta = _read_sheet_metadata(suite_dir / AGENT_NAME)
    user_meta = _read_sheet_metadata(suite_dir / USER_NAME)
    assert agent_meta is not None and user_meta is not None
    expected = list(POSITIVE_EVENTS + CANCEL_EVENTS + SAFETY_EVENTS)
    assert agent_meta["events"] == expected
    assert user_meta["events"] == expected
    assert agent_meta["role"] == "agent" and user_meta["role"] == "user"
    assert agent_meta["captures"] == user_meta["captures"]
    rebuild_index(suite_dir)
    summary = build_qualification_summary(suite_dir)
    assert summary["status"] == "verified-pass", summary["reasons"]


def test_cli_rejects_unknown_event_identity(tmp_path):
    """F4.3: an unknown visual event identity in the index is rejected, never
    silently placed."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    index = json.loads((suite_dir / INDEX_NAME).read_text(encoding="utf-8"))
    index["files"].append(
        {
            "path": "E/qualification-pick-place-positive/visual/source/9999-bogus-overview.png",
            "sha256": "ab" * 32,
            "size": 1,
            "mode": "0644",
            "category": "capture",
            "bound": True,
            "event": "bogus-event",
            "scenario": POSITIVE_ID,
            "attempt": "attempt-positive",
        }
    )
    (suite_dir / INDEX_NAME).write_text(json.dumps(index, sort_keys=True), encoding="utf-8")
    from validation.integrated_contact_sheets import _all_bound_capture_entries

    with pytest.raises(ValueError, match="unknown visual event identity"):
        _all_bound_capture_entries(suite_dir)


# --- F4.4 verbatim overlay artifacts and root-relative lock paths ------------


def test_overlay_ompl_filename_and_root_relative_lock_pass(tmp_path):
    """F4.4: the real ompl-overlay-contract.json filename and the verbatim
    root-relative integration/source-locks.json reference bind through Gate F."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    render_sheets(suite_dir)
    summary = build_qualification_summary(suite_dir)
    assert summary["status"] == "verified-pass", summary["reasons"]
    index = _final_index(suite_dir)
    assert any(e.get("category") == "overlay-contract" and e["path"].endswith("ompl-overlay-contract.json") for e in index["files"])


def test_overlay_renamed_overlay_contract_recognized(tmp_path):
    """F4.4: a legitimate ``*-overlay-contract.json`` is categorized as
    overlay-contract, not ``other``."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "legacy-overlay-contract.json").write_text(
        (suite_dir / "ompl-overlay-contract.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    index = _final_index(suite_dir)
    overlay = [e for e in index["files"] if e.get("category") == "overlay-contract"]
    assert any(e["path"] == "ompl-overlay-contract.json" for e in overlay)
    assert any(e["path"] == "legacy-overlay-contract.json" for e in overlay)


def test_overlay_duplicate_identical_duplicates_fail(tmp_path):
    """F5.3: the Gate-F suite must contain exactly one overlay-contract artifact.
    Identical duplicates (identical production/simulator implementation
    identities) fail with the exactly-one reason, even though they are not a
    contradiction."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "overlay-contract.json").write_text(
        (suite_dir / "ompl-overlay-contract.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    index = _final_index(suite_dir)
    overlay = [e for e in index["files"] if e.get("category") == "overlay-contract"]
    assert len(overlay) == 2
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    reasons = " ".join(verdict["reasons"])
    assert verdict["status"] == "verified-fail"
    assert "exactly one overlay contract artifact" in reasons
    assert "contradict across overlay-contract artifacts" not in reasons


def test_overlay_duplicate_contradictory_identities_fail(tmp_path):
    """F4.4: two overlay artifacts with contradictory production identities
    fail closed."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    value = json.loads((suite_dir / "ompl-overlay-contract.json").read_text(encoding="utf-8"))
    value["repositories"]["production"]["implementation_identity"] = "c" * 40
    _write_json(suite_dir / "overlay-contract.json", value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "contradict across overlay-contract artifacts" in " ".join(verdict["reasons"])


def test_overlay_missing_overlay_contract_fails(tmp_path):
    """F4.4: a suite with no recognized overlay contract fails closed."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "ompl-overlay-contract.json").unlink()
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "missing overlay contract" in " ".join(verdict["reasons"])


def test_overlay_lock_path_escape_fails(tmp_path):
    """F4.4: an escaping (..) simulator_lock_path never binds."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    overlay = suite_dir / "ompl-overlay-contract.json"
    value = json.loads(overlay.read_text(encoding="utf-8"))
    value["source_locks"]["simulator_lock_path"] = "../outside.json"
    _rewrite_json(overlay, value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "not suite-relative" in " ".join(verdict["reasons"])


def test_overlay_lock_path_absolute_fails(tmp_path):
    """F4.4: an absolute simulator_lock_path never reads outside the suite."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    overlay = suite_dir / "ompl-overlay-contract.json"
    value = json.loads(overlay.read_text(encoding="utf-8"))
    value["source_locks"]["simulator_lock_path"] = "/etc/passwd"
    _rewrite_json(overlay, value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "not suite-relative" in " ".join(verdict["reasons"])


# --- F4.5 fail-closed lows ----------------------------------------------------


def test_split_same_scenario_across_two_attempts_fails(tmp_path):
    """F4.5: two attempts bearing the same scenario id must never merge their
    event subsets; each attempt is checked independently."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    # Add a second cancel attempt in its own directory, manifest-only (no
    # scenario-bundle) so it does not trigger a duplicate-scenario-declaration.
    _write_attempt_dir(
        suite_dir,
        CANCEL_ID,
        attempt_id="attempt-cancel-2",
        attempt_subdir="cancel-2",
        include_rosbag=False,
        include_planning_scene=True,
    )
    (suite_dir / "E" / "cancel-2" / "scenario-bundle.json").unlink()
    # Split the four cancel events across the two attempts.
    _trim_attempt_visual(
        suite_dir / "E" / CANCEL_ID,
        {"cancel-execution-start", "cancel-trigger"},
    )
    _trim_attempt_visual(
        suite_dir / "E" / "cancel-2",
        {"cancel-velocity-compliant", "cancel-terminal"},
    )
    index = _final_index(suite_dir)
    assert not index.get("diagnostics"), index.get("diagnostics")
    verdict = validate_gate_f(index, suite_dir=suite_dir)
    reasons = " ".join(verdict["reasons"])
    assert verdict["status"] == "verified-fail"
    assert CANCEL_ID in reasons
    assert "attempt-cancel-2" in reasons
    assert "missing required" in reasons


def test_cleanup_empty_gpu_inventory_with_available_true_fails(tmp_path):
    """F4.5: empty/empty GPU inventories never pass vacuously when a snapshot
    reports available=true."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    value = _cleanup_value(suite_dir)
    value["baseline"]["gpus"] = []
    value["final"]["gpus"] = []
    _rewrite_json(suite_dir / "E" / POSITIVE_ID / "resource-cleanup.json", value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "empty GPU inventory despite available=true" in " ".join(verdict["reasons"])


def test_cleanup_malformed_gpu_record_fails(tmp_path):
    """F4.5: a GPU inventory record missing the physical uuid identity fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    value = _cleanup_value(suite_dir)
    value["final"]["gpus"] = [{"index": 0, "memory_used_mib": 2048}]
    _rewrite_json(suite_dir / "E" / POSITIVE_ID / "resource-cleanup.json", value)
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "malformed GPU inventory record" in " ".join(verdict["reasons"])
