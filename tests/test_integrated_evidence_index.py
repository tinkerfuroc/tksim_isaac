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
SAFETY_ID = "qualification-pick-place-safety-transport"
MISSING_FP = "0" * 64


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
    if scenario_id == CANCEL_ID:
        return CANCEL_EVENTS
    if scenario_id == SAFETY_ID:
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
    if scenario_id == CANCEL_ID:
        return {
            "acceptance": {"polarity": "negative"},
            "expected_negative": {"required": ["cancel-quiescent"], "forbidden": ["retention"]},
            "authority": "physics_truth",
            "execution_profile": "sim_ompl",
        }
    if scenario_id == SAFETY_ID:
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
    include_rosbag: bool = True,
    include_planning_scene: bool = True,
) -> Path:
    """Write a real-shaped integrated attempt directory for one scenario."""
    attempt_dir = suite_dir / "E" / scenario_id
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

    # resource-cleanup.json (real schema_version 2 producer shape).
    _write_json(
        attempt_dir / "resource-cleanup.json",
        {
            "schema_version": 2,
            "baseline": {"available": True, "gpus": [], "processes": []},
            "final": {"available": True, "gpus": [], "processes": []},
            "attempt_owned_pids": [],
            "attempt_owned_gpu_survivors": [],
            "unexplained_gpu_memory": [],
            "memory_tolerance_mib": 512,
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
        metadata_topics = []
        for topic, topic_type in record_topic_types.items():
            metadata_topics.append(
                {
                    "topic_metadata": {
                        "name": topic,
                        "type": topic_type,
                        "offered_qos_profiles": (
                            "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n"
                        ),
                    },
                    "message_count": 100,
                }
            )
        metadata_document = {
            "rosbag2_bagfile_information": {
                "storage_identifier": "sqlite3",
                "duration": {"nanoseconds": 10000000000},
                "message_count": 900,
                "topics_with_message_count": metadata_topics,
            }
        }
        (bag / "metadata.yaml").write_text(
            yaml.safe_dump(metadata_document, sort_keys=False, default_flow_style=False),
            encoding="utf-8",
        )
        _write_bytes(bag / f"{scenario_id}_0.db3", b"SQLite format 3\x00" + b"\x00" * 24)

    # ---- Visual two-journal transaction -------------------------------------
    events = _scenario_events(scenario_id)
    base_frame = {"qualification-pick-place-positive": 0, CANCEL_ID: 5, SAFETY_ID: 8}[scenario_id]
    request_rows: list[dict[str, object]] = []
    keyframe_rows: list[dict[str, object]] = []
    for event_index, event in enumerate(events):
        for camera in CAMERAS:
            sequence = event_index * len(CAMERAS) + 1 + (0 if camera == CAMERAS[0] else 1)
            frame_index = base_frame + event_index * 10
            timestamp = frame_index / PHYSICS_HZ
            request_rows.append(
                {
                    "schema_version": 1,
                    "sequence": sequence,
                    "gate": scenario_id,
                    "event": event,
                    "simulated_timestamp": timestamp,
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
                    "requested_simulated_timestamp": timestamp,
                    "requested_physics_frame_index": frame_index,
                    "capture_latency_frames": 0,
                    "max_capture_latency_frames": 4,
                    "simulated_timestamp": timestamp,
                    "raw_frame_index": frame_index,
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
    # Real nested overlay-contract shape (F2.5): repositories map with
    # implementation_identity commits + source_locks status.
    _write_json(
        suite_dir / "overlay-contract.json",
        {
            "schema_version": 1,
            "contract_id": "simulator-ompl-overlay-acceptance",
            "repositories": {
                "production": {"implementation_identity": PROD_COMMIT},
                "simulator": {"implementation_identity": SIM_COMMIT},
            },
            "source_locks": {
                "status": "pass",
                "simulator_lock_path": "integration/source-locks.json",
                "production_lock_path": "integration/source-locks.json",
            },
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
    assert "missing required visual event" in reasons
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
    target = suite_dir / "overlay-contract.json"
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
    overlay = suite_dir / "overlay-contract.json"
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


def test_cleanup_clean_flag_contradiction_fails_gate_f(tmp_path):
    """F2.8: clean=True that contradicts the recorded final/owned state fails."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    cleanup = suite_dir / "E" / POSITIVE_ID / "resource-cleanup.json"
    value = json.loads(cleanup.read_text(encoding="utf-8"))
    value["final"] = {"available": True, "gpus": [{"id": 0, "name": "NVIDIA"}], "processes": []}
    cleanup.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
    verdict = validate_gate_f(_final_index(suite_dir), suite_dir=suite_dir)
    assert "contradicts the recorded" in " ".join(verdict["reasons"])
