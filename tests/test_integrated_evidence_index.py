"""Task 9 integrated evidence-index tests (ROS-free, Python 3.12).

This module defines the canonical evidence-suite factory
(``write_canonical_evidence_tree`` / ``make_complete_evidence_suite``) and the
capture-path selector (``required_capture_paths``) consumed by the contact-sheet
suite, the six acceptance tests from the Task 9 brief, and adversarial
determinism/self-exclusion/taxonomy/identity/rosbag/journal/verdict/cleanup/
process/GPU/capture/security tests for
``validation.integrated_evidence_index``.

The factory writes real preserved artifact bytes and then derives
``evidence-index.json`` from those exact bytes via ``build_evidence_index``; it
never fabricates a verdict.  Captures are written to ``captures/<event>.png``
and every visual event is bound to exact scenario/attempt/execution-request plus
``(frame_index, timestamp)`` metadata in ``visual-capture-requests.jsonl``.
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

from validation.integrated_evidence_index import (  # noqa: E402
    INDEX_NAME,
    REQUIRED_POSITIVE_EVENTS,
    canonical_sha256,
    build_evidence_index,
    build_qualification_summary,
    validate_gate_f,
)
from validation.integrated_contact_sheets import build_contact_sheet  # noqa: E402

POSITIVE_EVENTS = tuple(REQUIRED_POSITIVE_EVENTS)
CANCEL_EVENTS = (
    "cancel-execution-start",
    "cancel-trigger",
    "cancel-velocity-compliant",
    "cancel-terminal",
)
SAFETY_EVENTS = (
    "safety-execution-start",
    "safety-trigger",
    "safety-velocity-compliant",
    "safety-post-clear",
)
ALL_CAPTURE_EVENTS = POSITIVE_EVENTS + CANCEL_EVENTS + SAFETY_EVENTS
IMAGE_SIZE = (960, 540)
SIM_COMMIT = "sim-head"
PROD_COMMIT = "39d96a176904c0b7966b11333c5517b3b54b6ae3"
DIGEST_RE = re.compile(r"^[0-9a-f]{64}$")
POSITIVE_SCENARIO_ID = "qualification-pick-place-positive"


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
    image = Image.new("RGB", IMAGE_SIZE, (25 + seed % 40, 45 + seed % 40, 75 + seed % 40))
    draw = ImageDraw.Draw(image)
    draw.rectangle((120, 80, 800, 430), fill=(180, 80 + seed % 100, 35))
    draw.line((0, seed % 10 + 10, 959, 500 - seed % 10), fill=(240, 240, 240), width=5)
    image.save(path, format="PNG", optimize=False, compress_level=9)


def _scenario_file(suite_dir: Path, scenario_id: str, kind: str) -> Path:
    path = suite_dir / "scenario" / f"{scenario_id}.json"
    _write_json(
        path,
        {
            "schema_version": 2,
            "scenario": {"id": scenario_id, "seed": 7},
            "planning_scene": {"revision": "qualification-v1", "owner": "sim_fixture"},
            "integrated": {
                "execution_profile": "sim_ompl",
                "authority": "physics_truth",
                "kind": kind,
            },
        },
    )
    return path


def write_canonical_evidence_tree(
    suite_dir: Path,
    *,
    simulator_commit: str | None = "sim-head",
    include_rosbag_metadata: bool = True,
    include_planning_scene: bool = True,
) -> Path:
    """Write the complete required artifact tree and valid nonblank captures.

    ``simulator_commit``, ``include_rosbag_metadata``, and
    ``include_planning_scene`` mutate only the named missing-artifact condition.
    Every artifact ``build_evidence_index``/``validate_gate_f`` reads is written
    from real bytes; the final ``evidence-index.json`` is derived from those
    exact bytes and is not a fabricated verdict.
    """
    suite_dir.mkdir(parents=True, exist_ok=True)
    _write_json(
        suite_dir / "config/integrated-ompl.json",
        {
            "schema_version": 3,
            "id": "integrated-ompl",
            "profile": "integrated-ompl",
            "execution_profile": "sim_ompl",
            "seed": 7,
            "checksum_algorithm": "sha256",
            "stages": {"F": {"cameras": ["overview", "manipulation_closeup"]}},
        },
    )
    _scenario_file(suite_dir, POSITIVE_SCENARIO_ID, "positive")
    _write_json(
        suite_dir / "overlay-contract.json",
        {"repository": "simulator", "implementation_head": "490f907831d9f6f06242e0d151ac014547973d6e"},
    )
    _write_json(
        suite_dir / "model-fingerprint.json",
        {"robot": "tinker2", "sha256": "2" * 64},
    )
    _write_json(
        suite_dir / "source/production-commit.json",
        {"repository": "production", "commit": PROD_COMMIT, "status_sha256": "a" * 64, "diff_sha256": "b" * 64, "untracked_sha256": "c" * 64},
    )
    if simulator_commit is not None:
        _write_json(
            suite_dir / "source/simulator-commit.json",
            {"repository": "simulator", "commit": simulator_commit, "status_sha256": "d" * 64, "diff_sha256": "e" * 64, "untracked_sha256": "f" * 64},
        )
    _write_json(
        suite_dir / "source/source-locks.json",
        {"schema_version": 1, "repository": "simulator", "implementation_head": "490f907831d9f6f06242e0d151ac014547973d6e", "policy_commit": "ab8cf7e9645b1e019aba81e2c7923177ba13d1ac", "policy_path": "integration/source-locks.json", "mode": "clean"},
    )
    _write_json(
        suite_dir / "source/dependency-locks.json",
        {"dependencies": {"isaac-sim": "6.0.1.0", "isaac-lab": "v3.0.0-beta2.patch1"}},
    )
    _write_json(
        suite_dir / "runtime/command.json",
        {"argv": [".venv/bin/python", "validation/integrated_qualification.py", "--stage", "all", "--seed", "7"], "allowlist": ["--stage", "--seed", "--attempt-root"]},
    )
    _write_json(
        suite_dir / "runtime/environment.json",
        {"env_allowlist": {"ROS_DOMAIN_ID": "25", "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", "TINKER_SIM_ATTEMPT_DIR": "attempt-1"}},
    )
    _write_json(
        suite_dir / "runtime/ros.json",
        {"domain_id": 25, "rmw_implementation": "rmw_fastrtps_cpp", "dds_profile": "fastdds_shm.xml"},
    )
    if include_planning_scene:
        journal_lines = [
            json.dumps({"journal_sequence": 1, "event": "fixture-ready", "frame_index": 0, "timestamp": 0.0}),
            json.dumps({"journal_sequence": 2, "event": "scene-attach", "frame_index": 20, "timestamp": 20.0 / 120.0}),
            json.dumps({"journal_sequence": 3, "event": "scene-detach", "frame_index": 50, "timestamp": 50.0 / 120.0}),
        ]
        _write_text(suite_dir / "planning-scene/planning-scene.jsonl", "\n".join(journal_lines) + "\n")
        _write_json(
            suite_dir / "planning-scene/planning-scene.json",
            {"schema_version": 1, "finalized": True, "revision": "qualification-v1", "owner": "sim_fixture"},
        )
    _write_text(
        suite_dir / "moveit/moveit-plans.jsonl",
        json.dumps({"schema_version": 1, "row_kind": "plan-result", "scenario_id": POSITIVE_SCENARIO_ID, "success": True}) + "\n",
    )
    _write_text(
        suite_dir / "moveit/controller-results.jsonl",
        json.dumps({"schema_version": 1, "row_kind": "controller-result", "scenario_id": POSITIVE_SCENARIO_ID, "status": "succeeded"}) + "\n",
    )
    _write_text(
        suite_dir / "physics/physics_truth.jsonl",
        "\n".join(
            json.dumps({"schema_version": 1, "frame_index": index, "timestamp": index / 120.0, "joint": [0.0] * 7})
            for index in range(10)
        )
        + "\n",
    )
    _write_text(
        suite_dir / "physics/evaluator.jsonl",
        "\n".join(
            json.dumps({"schema_version": 1, "frame_index": index, "timestamp": index / 120.0, "metric": "bilateral-contact", "value": 0.0})
            for index in range(10)
        )
        + "\n",
    )
    _write_text(
        suite_dir / "physics/drain.jsonl",
        json.dumps({"schema_version": 1, "raw_records": 10, "evaluator_records": 10, "exact": True}) + "\n",
    )
    _write_json(
        suite_dir / "verdict/gate-verdict.json",
        {"schema_version": 1, "gate": "integrated", "status": "verified-pass", "scenario_id": POSITIVE_SCENARIO_ID},
    )
    _write_json(
        suite_dir / "verdict/cleanup-report.json",
        {"schema_version": 1, "clean": True, "orphans": [], "processes": []},
    )
    _write_json(
        suite_dir / "verdict/gpu-report.json",
        {"schema_version": 1, "gpus": [], "memory_used_bytes": 0},
    )
    _write_json(
        suite_dir / "verdict/process-report.json",
        {"schema_version": 1, "processes": [{"pid": 100, "name": "isaac", "exit_status": 0}], "accounted": True},
    )
    if include_rosbag_metadata:
        _write_json(
            suite_dir / "rosbag/rosbag-metadata.json",
            {
                "schema_version": 1,
                "message_count": 1200,
                "duration_s": 10.0,
                "topics": {
                    "/joint_states": {"count": 1000, "qos": {"reliability": "reliable", "depth": 10}},
                    "/xarm/joint_states": {"count": 200, "qos": {"reliability": "reliable", "depth": 10}},
                },
            },
        )
        _write_bytes(suite_dir / "rosbag/rosbag.db3", b"SQLite format 3\x00" + b"\x00" * 24)

    captures = suite_dir / "captures"
    captures.mkdir(parents=True, exist_ok=True)
    request_records = []
    for index, event in enumerate(ALL_CAPTURE_EVENTS):
        _nonblank_image(captures / f"{event}.png", index)
        request_records.append(
            {
                "schema_version": 1,
                "scenario": POSITIVE_SCENARIO_ID,
                "attempt": "attempt-1",
                "event": event,
                "camera": "overview",
                "execution_request": index + 1,
                "frame_index": index * 10,
                "timestamp": float(index * 10) / 120.0,
                "path": f"captures/{event}.png",
            }
        )
    _write_text(
        suite_dir / "visual-capture-requests.jsonl",
        "\n".join(json.dumps(record) for record in request_records) + "\n",
    )
    build_evidence_index(suite_dir=suite_dir, output=suite_dir / INDEX_NAME)
    return suite_dir


def make_complete_evidence_suite(
    tmp_path: Path,
    *,
    simulator_commit: str | None = "sim-head",
    include_rosbag_metadata: bool = True,
    include_planning_scene: bool = True,
) -> Path:
    suite_dir = tmp_path / "suite"
    write_canonical_evidence_tree(
        suite_dir,
        simulator_commit=simulator_commit,
        include_rosbag_metadata=include_rosbag_metadata,
        include_planning_scene=include_planning_scene,
    )
    return suite_dir


def required_capture_paths(suite_dir: Path, *, events: set[str]) -> list[Path]:
    paths = [suite_dir / "captures" / f"{event}.png" for event in sorted(events)]
    if any(not path.is_file() for path in paths):
        raise FileNotFoundError("required capture event is missing")
    return paths


def rebuild_index(suite_dir: Path) -> dict[str, object]:
    return build_evidence_index(suite_dir=suite_dir, output=suite_dir / INDEX_NAME)


def add_cancel_safety_scenarios(suite_dir: Path) -> None:
    _scenario_file(suite_dir, "qualification-pick-place-cancel-approach", "cancel")
    _scenario_file(suite_dir, "qualification-pick-place-safety-transport", "safety")


# --- Brief acceptance tests -------------------------------------------------


def test_index_is_deterministic_and_excludes_itself(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    first = build_evidence_index(suite_dir=suite_dir, output=suite_dir / INDEX_NAME)
    second = build_evidence_index(suite_dir=suite_dir, output=suite_dir / INDEX_NAME)
    assert first["files"] == second["files"]
    assert all(entry["path"] != INDEX_NAME for entry in first["files"])
    assert first["index_checksum"] == second["index_checksum"]


def test_missing_repository_commit_blocks_final_acceptance(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path, simulator_commit=None)
    index = build_evidence_index(suite_dir=suite_dir, output=suite_dir / INDEX_NAME)
    verdict = validate_gate_f(index)
    assert verdict["status"] == "verified-fail"
    assert "simulator commit" in " ".join(verdict["reasons"]).lower()


def test_missing_rosbag_metadata_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path, include_rosbag_metadata=False)
    index = build_evidence_index(suite_dir=suite_dir, output=suite_dir / INDEX_NAME)
    verdict = validate_gate_f(index)
    assert verdict["status"] == "verified-fail"
    assert "rosbag" in " ".join(verdict["reasons"]).lower()


def test_missing_planning_scene_journal_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path, include_planning_scene=False)
    index = build_evidence_index(suite_dir=suite_dir, output=suite_dir / INDEX_NAME)
    verdict = validate_gate_f(index)
    assert verdict["status"] == "verified-fail"
    assert "planning scene" in " ".join(verdict["reasons"]).lower()


# --- Deterministic ordering / checksum -------------------------------------


def test_index_files_sorted_and_checksums_lowercase_64hex(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    index = rebuild_index(suite_dir)
    paths = [entry["path"] for entry in index["files"]]
    assert paths == sorted(paths)
    for entry in index["files"]:
        assert isinstance(entry["sha256"], str)
        assert DIGEST_RE.fullmatch(entry["sha256"]), entry["path"]
        assert entry["size"] > 0
    assert isinstance(index["index_checksum"], str)
    assert DIGEST_RE.fullmatch(index["index_checksum"])


def test_index_excludes_only_itself_and_indexes_every_other_file(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    index = rebuild_index(suite_dir)
    indexed = {entry["path"] for entry in index["files"]}
    on_disk = {
        str(path.relative_to(suite_dir))
        for path in suite_dir.rglob("*")
        if path.is_file() and not path.is_symlink()
    }
    assert INDEX_NAME not in indexed
    assert indexed == (on_disk - {INDEX_NAME})


# --- Index/contact-sheet cycle ---------------------------------------------


def test_index_cycle_includes_generated_sheets_and_remains_deterministic(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    required = set(POSITIVE_EVENTS)
    agent = build_contact_sheet(
        suite_dir=suite_dir,
        image_paths=required_capture_paths(suite_dir, events=required),
        output=suite_dir / "contact-sheet-integrated-agent.png",
    )
    build_contact_sheet(
        suite_dir=suite_dir,
        image_paths=required_capture_paths(suite_dir, events=required),
        output=suite_dir / "contact-sheet-integrated-user.png",
    )
    assert agent["role"] == "agent"
    rebuilt = rebuild_index(suite_dir)
    sheet_paths = {entry["path"] for entry in rebuilt["files"] if entry["category"] == "contact-sheet"}
    assert sheet_paths == {"contact-sheet-integrated-agent.png", "contact-sheet-integrated-user.png"}
    again = rebuild_index(suite_dir)
    assert rebuilt["files"] == again["files"]
    assert rebuilt["index_checksum"] == again["index_checksum"]


def test_full_cycle_gate_f_passes_with_sheets_and_summary(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    required = set(POSITIVE_EVENTS)
    build_contact_sheet(
        suite_dir=suite_dir,
        image_paths=required_capture_paths(suite_dir, events=required),
        output=suite_dir / "contact-sheet-integrated-agent.png",
    )
    build_contact_sheet(
        suite_dir=suite_dir,
        image_paths=required_capture_paths(suite_dir, events=required),
        output=suite_dir / "contact-sheet-integrated-user.png",
    )
    rebuild_index(suite_dir)
    verdict = validate_gate_f(
        rebuild_index(suite_dir),
        output=suite_dir / "qualification-summary.json",
    )
    assert verdict["status"] == "verified-pass"
    assert verdict["reasons"] == []
    assert (suite_dir / "qualification-summary.json").is_file()
    summary = json.loads((suite_dir / "qualification-summary.json").read_text(encoding="utf-8"))
    assert summary["status"] == "verified-pass"
    final = rebuild_index(suite_dir)
    assert "qualification-summary.json" in {entry["path"] for entry in final["files"]}
    assert "qualification-summary.json" in {entry["path"] for entry in rebuild_index(suite_dir)["files"]}


def test_build_qualification_summary_writes_both_artifacts(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    summary = build_qualification_summary(
        suite_dir,
        index_output=suite_dir / INDEX_NAME,
        summary_output=suite_dir / "qualification-summary.json",
    )
    assert summary["status"] == "verified-fail"  # sheets absent -> fail closed
    assert (suite_dir / INDEX_NAME).is_file()
    assert (suite_dir / "qualification-summary.json").is_file()


# --- Gate F taxonomy / identities -------------------------------------------


def test_validate_gate_f_required_taxonomy_present(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    index = rebuild_index(suite_dir)
    categories = {entry["category"] for entry in index["files"]}
    for required in (
        "config",
        "scenario",
        "overlay-contract",
        "model-fingerprint",
        "source-identity",
        "source-lock",
        "dependency-lock",
        "runtime",
        "planning-scene-journal",
        "moveit",
        "physics",
        "verdict",
        "rosbag",
        "capture",
    ):
        assert required in categories, required


def test_missing_production_commit_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "source/production-commit.json").unlink()
    index = rebuild_index(suite_dir)
    verdict = validate_gate_f(index)
    assert verdict["status"] == "verified-fail"
    assert "production commit" in " ".join(verdict["reasons"]).lower()


def test_missing_source_lock_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "source/source-locks.json").unlink()
    index = rebuild_index(suite_dir)
    verdict = validate_gate_f(index)
    assert verdict["status"] == "verified-fail"
    assert "source lock" in " ".join(verdict["reasons"]).lower()


def test_missing_dependency_lock_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "source/dependency-locks.json").unlink()
    verdict = validate_gate_f(rebuild_index(suite_dir))
    assert "dependency lock" in " ".join(verdict["reasons"]).lower()


def test_missing_runtime_ros_evidence_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "runtime/ros.json").unlink()
    verdict = validate_gate_f(rebuild_index(suite_dir))
    assert "ros domain" in " ".join(verdict["reasons"]).lower()


def test_missing_verdict_cleanup_gpu_process_fail_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    for name in ("gate-verdict.json", "cleanup-report.json", "gpu-report.json", "process-report.json"):
        (suite_dir / "verdict" / name).unlink()
    verdict = validate_gate_f(rebuild_index(suite_dir))
    reasons = " ".join(verdict["reasons"]).lower()
    assert "gate verdict" in reasons
    assert "cleanup report" in reasons
    assert "gpu report" in reasons
    assert "process report" in reasons


def test_missing_moveit_controller_physics_evidence_fail_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "moveit/moveit-plans.jsonl").unlink()
    (suite_dir / "moveit/controller-results.jsonl").unlink()
    (suite_dir / "physics/physics_truth.jsonl").unlink()
    (suite_dir / "physics/evaluator.jsonl").unlink()
    (suite_dir / "physics/drain.jsonl").unlink()
    verdict = validate_gate_f(rebuild_index(suite_dir))
    reasons = " ".join(verdict["reasons"]).lower()
    assert "moveit plans" in reasons
    assert "controller results" in reasons
    assert "raw physics" in reasons
    assert "evaluator" in reasons
    assert "drain" in reasons


def test_missing_rosbag_qos_counts_fail_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "rosbag/rosbag-metadata.json").unlink()
    verdict = validate_gate_f(rebuild_index(suite_dir))
    assert "rosbag" in " ".join(verdict["reasons"]).lower()


def test_missing_contact_sheet_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    verdict = validate_gate_f(rebuild_index(suite_dir))
    reasons = " ".join(verdict["reasons"]).lower()
    assert "contact-sheet-integrated-agent.png" in reasons
    assert "contact-sheet-integrated-user.png" in reasons


def test_validate_gate_f_never_fabricates_a_verdict(tmp_path):
    # A suite with the verdict artifact removed can never report verified-pass.
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "verdict/gate-verdict.json").unlink()
    verdict = validate_gate_f(rebuild_index(suite_dir))
    assert verdict["status"] == "verified-fail"
    assert "gate verdict" in " ".join(verdict["reasons"]).lower()


# --- Capture event coverage -------------------------------------------------


def test_missing_required_capture_event_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "captures/terminal.png").unlink()
    # Remove the corresponding binding record so the journal is consistent.
    path = suite_dir / "visual-capture-requests.jsonl"
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    records = [record for record in records if record["event"] != "terminal"]
    path.write_text("\n".join(json.dumps(record) for record in records) + "\n", encoding="utf-8")
    verdict = validate_gate_f(rebuild_index(suite_dir))
    reasons = " ".join(verdict["reasons"]).lower()
    assert "missing required visual event" in reasons
    assert "terminal" in reasons


def test_unbound_capture_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    extra = suite_dir / "captures/orphan.png"
    _nonblank_image(extra, 200)
    verdict = validate_gate_f(rebuild_index(suite_dir))
    assert "unbound capture" in " ".join(verdict["reasons"]).lower()


def test_cancel_and_safety_event_coverage_required_when_scenarios_present(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    add_cancel_safety_scenarios(suite_dir)
    index = rebuild_index(suite_dir)
    assert "cancel" in index["scenario_kinds"]
    assert "safety" in index["scenario_kinds"]
    verdict = validate_gate_f(index)
    required = verdict["required_events"]
    assert set(required["cancel"]) == set(CANCEL_EVENTS)
    assert set(required["safety"]) == set(SAFETY_EVENTS)
    reasons = " ".join(verdict["reasons"])
    # Cancel/safety captures are all present; no missing cancel/safety event.
    assert "cancel-velocity-compliant" not in reasons
    assert "safety-post-clear" not in reasons
    # Only the absent contact sheets block acceptance at this stage.
    assert "contact-sheet-integrated-agent.png" in reasons


def test_cancel_event_missing_fails_gate_f(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    add_cancel_safety_scenarios(suite_dir)
    (suite_dir / "captures/cancel-trigger.png").unlink()
    verdict = validate_gate_f(rebuild_index(suite_dir))
    reasons = " ".join(verdict["reasons"]).lower()
    assert "missing required visual event" in reasons
    assert "cancel-trigger" in reasons


# --- Security: traversal / symlink / output-as-input / duplicates / change --


def test_index_rejects_symlink_escape(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    outside = tmp_path / "outside.txt"
    outside.write_text("secret", encoding="utf-8")
    (suite_dir / "escaped-link.txt").symlink_to(outside)
    with pytest.raises(ValueError, match="escape|outside|symlink"):
        rebuild_index(suite_dir)


def test_index_rejects_output_as_input(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    target = suite_dir / "model-fingerprint.json"
    with pytest.raises(ValueError, match="output-as-input|output"):
        build_evidence_index(suite_dir=suite_dir, output=target)


def test_index_rejects_file_changing_during_hashing(tmp_path, monkeypatch):
    suite_dir = make_complete_evidence_suite(tmp_path)
    import validation.integrated_evidence_index as index_module

    target = suite_dir / "physics/physics_truth.jsonl"
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


def test_index_duplicate_capture_event_binding_rejected(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    # Duplicate the readiness binding record.
    path = suite_dir / "visual-capture-requests.jsonl"
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    records.append({**records[0], "execution_request": 999})
    path.write_text("\n".join(json.dumps(record) for record in records) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="duplicate event|duplicate.*readiness"):
        rebuild_index(suite_dir)


def test_canonical_sha256_is_stable_lowercase():
    value = {"b": [1, 2], "a": "x"}
    first = canonical_sha256(value)
    second = canonical_sha256({"a": "x", "b": [2, 1]})
    assert first != second
    assert DIGEST_RE.fullmatch(first)
    assert canonical_sha256({"a": "x", "b": [1, 2]}) == first
