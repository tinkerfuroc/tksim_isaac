"""Task 2: integrated Gate B static contract checks.

The tests copy the canonical static-contract fixture tree, rewrite only the
named structured field or marker, write an already-produced three-entry
source-lock manifest, and call :func:`validate_static_contracts`.  Overrides
operate on structured JSON records (model/controllers/providers/runtime
markers/result contract/prerequisites) or the selected launch source text;
they are never source-text or comment-match shortcuts.
"""
from __future__ import annotations

import json
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

from integrated_static_contracts import StaticReport, validate_static_contracts  # noqa: E402

FIXTURE_TREE = ROOT / "tests/fixtures/integrated_static_contract"
CONFIG_REL = "simulation/qualification/integrated-ompl.json"
MANIFEST_NAME = "source-lock-manifest.json"


def _failed_reasons(report: StaticReport) -> list[str]:
    return [reason for check in report.checks for reason in check.reasons]


@dataclass(frozen=True)
class StaticContractFixture:
    simulator_root: Path
    production_root: Path
    source_lock_manifest: Path
    config: dict[str, object]


def _write_json_canonical(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, sort_keys=True, separators=(",", ":")), encoding="utf-8"
    )


def _read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def copy_fixture_tree(tmp_path: Path) -> StaticContractFixture:
    fixture_root = tmp_path / "fixture"
    if fixture_root.exists():
        shutil.rmtree(fixture_root)
    shutil.copytree(FIXTURE_TREE, fixture_root)
    simulator_root = fixture_root / "simulator"
    production_root = fixture_root / "production"
    config = _read_json(simulator_root / CONFIG_REL)
    source_lock_manifest = fixture_root / MANIFEST_NAME
    return StaticContractFixture(
        simulator_root=simulator_root,
        production_root=production_root,
        source_lock_manifest=source_lock_manifest,
        config=config,
    )


def apply_structured_overrides(fixture: StaticContractFixture, overrides: Mapping[str, object]) -> None:
    """Rewrite only the named structured field or marker."""
    records = fixture.production_root / "qualification/records"
    for key, value in overrides.items():
        if key == "model":
            path = records / "model.json"
            base = _read_json(path)
            merged = {**base, **value}
            _write_json_canonical(path, merged)
        elif key == "controller_mapping":
            _write_json_canonical(records / "controllers.json", {"controller_mapping": value})
        elif key == "provider_counts":
            _write_json_canonical(records / "providers.json", {"provider_counts": value})
        elif key == "runtime_markers":
            path = records / "runtime_markers.json"
            base = _read_json(path)
            merged = {**base, **value}
            _write_json_canonical(path, merged)
        elif key == "result_fields":
            _write_json_canonical(records / "result_contract.json", value)
        elif key == "prerequisites":
            _write_json_canonical(records / "prerequisites.json", value)
        elif key == "selected_launch_text":
            launch = fixture.production_root / "src/mobile_bringup/launch/manipulation_planning_task_only.launch.py"
            launch.write_text(str(value), encoding="utf-8")
        else:
            raise AssertionError("unknown static-contract override key: {}".format(key))


def write_source_lock_manifest_for_fixture(fixture: StaticContractFixture) -> None:
    """Write a valid three-entry source-lock manifest consumed by the checker."""
    policies = fixture.config["source_lock_policies"]
    assert set(policies) == {"simulator_overlay", "production", "qualification_tooling"}
    records: dict[str, object] = {}
    for role in ("simulator_overlay", "production", "qualification_tooling"):
        records[role] = {
            "repository": role,
            "status": "verified-pass",
            "policy_path": str(policies[role]),
            "implementation_head": "6aff6106acdbc27f0b4020becd663f8bcd030220",
            "resolved_policy_commit": "fa79ef40999d5251d75e71672db325f4874c5243",
        }
    manifest = {
        "schema_version": 1,
        "status": "verified-pass",
        "repositories": ["simulator_overlay", "production", "qualification_tooling"],
        **records,
    }
    _write_json_canonical(fixture.source_lock_manifest, manifest)


def make_static_contract_fixture(tmp_path: Path, *, overrides: Mapping[str, object]) -> StaticContractFixture:
    fixture = copy_fixture_tree(tmp_path)
    apply_structured_overrides(fixture, overrides)
    write_source_lock_manifest_for_fixture(fixture)
    return fixture


def _run_static_fixture(tmp_path: Path, **overrides: object) -> StaticReport:
    fixture = make_static_contract_fixture(tmp_path, overrides=overrides)
    return validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )


def test_all_static_contracts_pass(tmp_path):
    report = _run_static_fixture(tmp_path)
    assert report.status == "verified-pass"
    assert all(check.passed for check in report.checks)
    assert report.model_fingerprint == "8b1185628f7474a6e44468eaf7fafd3755f417e1dcb7645916d7e24468b8a41b"
    assert set(report.source_identities) == {"simulator_overlay", "production", "qualification_tooling"}
    names = [check.name for check in report.checks]
    assert names == [
        "model-fingerprint",
        "controller-mapping",
        "selected-launch",
        "provider-cardinality",
        "fixture-ownership",
        "action-lifecycle",
        "scene-and-collision-safety",
        "source-identities",
        "transport-contract",
    ]


def test_missing_link_tcp_fails_model_contract(tmp_path):
    report = _run_static_fixture(tmp_path, model={"tcp_link": None})
    assert report.status == "verified-fail"
    assert any("tcp" in reason.lower() for reason in _failed_reasons(report))


def test_controller_endpoint_mismatch_fails(tmp_path):
    report = _run_static_fixture(
        tmp_path,
        controller_mapping={"xarm7": "/wrong/follow_joint_trajectory"},
    )
    assert report.status == "verified-fail"
    assert any("controller" in reason.lower() for reason in _failed_reasons(report))


def test_cumotion_token_in_selected_launch_fails(tmp_path):
    report = _run_static_fixture(
        tmp_path,
        selected_launch_text="ros2 launch cumotion.launch.py",
    )
    assert report.status == "verified-fail"
    assert any("cumotion" in reason.lower() for reason in _failed_reasons(report))


def test_duplicate_controller_manager_fails(tmp_path):
    report = _run_static_fixture(tmp_path, provider_counts={"controller_manager": 2})
    assert report.status == "verified-fail"
    assert any("controller manager" in reason.lower() for reason in _failed_reasons(report))


def test_detached_motion_thread_marker_fails(tmp_path):
    report = _run_static_fixture(tmp_path, runtime_markers={"detached_motion_thread": True})
    assert report.status == "verified-fail"
    assert any("thread" in reason.lower() for reason in _failed_reasons(report))


def test_global_scene_cleanup_marker_fails(tmp_path):
    report = _run_static_fixture(tmp_path, runtime_markers={"global_scene_cleanup": True})
    assert report.status == "verified-fail"
    assert any("scene" in reason.lower() for reason in _failed_reasons(report))


def test_collision_disabled_lift_marker_fails(tmp_path):
    report = _run_static_fixture(tmp_path, runtime_markers={"lift_collision_checking": False})
    assert report.status == "verified-fail"
    assert any("collision" in reason.lower() for reason in _failed_reasons(report))


def test_action_result_contract_mismatch_fails(tmp_path):
    report = _run_static_fixture(
        tmp_path,
        result_fields={"required": ["stage", "status", "error_msg"], "present": ["status"]},
    )
    assert report.status == "verified-fail"
    assert any("result" in reason.lower() for reason in _failed_reasons(report))


def test_unpinned_prerequisite_is_reported(tmp_path):
    report = _run_static_fixture(tmp_path, prerequisites={"pinned": False})
    assert report.status == "verified-fail"
    assert any(
        "prerequisite" in reason.lower() or "commit" in reason.lower()
        for reason in _failed_reasons(report)
    )


def test_manifest_not_passed_fails_source_identities(tmp_path):
    fixture = copy_fixture_tree(tmp_path)
    write_source_lock_manifest_for_fixture(fixture)
    manifest = _read_json(fixture.source_lock_manifest)
    manifest["status"] = "evidence-invalid"
    _write_json_canonical(fixture.source_lock_manifest, manifest)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert report.status == "verified-fail"
    assert any("source-lock manifest status" in reason for reason in _failed_reasons(report))
