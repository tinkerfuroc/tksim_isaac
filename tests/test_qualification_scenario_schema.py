"""Task 6: qualification ``_scenario_readiness`` schema-tolerance tests.

The integrated overlay produces the canonical ``scenario-runner.json`` report
(``scenario`` as ``{id, seed, declaration}``); the legacy non-overlay path
produces the previous top-level string + ``seed`` shape.  ``_scenario_readiness``
must accept both and must not weaken identity/digest validation.  This module
imports neither ROS nor Isaac Sim, so it runs under the simulator CPython 3.12
venv.
"""
from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from manipulation_qualification import QualificationRunner  # noqa: E402
from tinker_sim_bridge.integrated_readiness import (  # noqa: E402
    build_canonical_report,
    public_integrated_mapping,
)

SCENARIO_ID = "qualification-moveit-plan-joint"
SEED = 7
SCENARIO_FILE = ROOT / "simulation/scenarios" / f"{SCENARIO_ID}.json"


def _runner() -> QualificationRunner:
    return QualificationRunner(
        root=ROOT,
        scenario_path=SCENARIO_FILE,
        seed=SEED,
        gate="all",
        isaac_command=[],
        humble_command=[],
    )


def _manifest(attempt_dir: Path):
    return SimpleNamespace(attempt_dir=attempt_dir)


def _legacy_report() -> dict[str, object]:
    raw = json.loads(SCENARIO_FILE.read_text(encoding="utf-8"))
    return {
        "schema_version": 1,
        "scenario": SCENARIO_ID,
        "seed": SEED,
        "control_api": "simulation_interfaces",
        "custom_control_services": False,
        "operations": [
            {"operation": "reset_spawned", "accepted": True},
            {
                "operation": "set_simulation_state",
                "accepted": True,
                "state": 1,
                "boundary": "PHYSICS_READY",
            },
        ],
    }


def _canonical_report() -> dict[str, object]:
    raw = json.loads(SCENARIO_FILE.read_text(encoding="utf-8"))
    declaration = {k: v for k, v in raw.items() if k not in {"id", "seed"}}
    return dict(
        build_canonical_report(
            scenario_id=SCENARIO_ID,
            seed=SEED,
            declaration=declaration,
            planning_scene=raw["planning_scene"],
            integrated=public_integrated_mapping(),
            operations=[
                {"operation": "reset_spawned", "accepted": True},
                {
                    "operation": "set_simulation_state",
                    "accepted": True,
                    "state": 1,
                    "boundary": "PHYSICS_READY",
                },
            ],
            model_fingerprint="2" * 64,
            provider_manifest_sha256="3" * 64,
        )
    )


def _write(tmp_path: Path, report: dict[str, object]) -> Path:
    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir(parents=True, exist_ok=True)
    path = attempt_dir / "scenario-runner.json"
    path.write_text(json.dumps(report), encoding="utf-8")
    return attempt_dir


def test_legacy_report_passes(tmp_path) -> None:
    attempt_dir = _write(tmp_path, _legacy_report())
    ok, evidence, reason = _runner()._scenario_readiness(_manifest(attempt_dir))
    assert ok is True, reason
    assert evidence["identity"]["observed_scenario"] == SCENARIO_ID
    assert evidence["identity"]["observed_seed"] == SEED


def test_canonical_report_passes(tmp_path) -> None:
    attempt_dir = _write(tmp_path, _canonical_report())
    ok, evidence, reason = _runner()._scenario_readiness(_manifest(attempt_dir))
    assert ok is True, reason
    assert evidence["identity"]["observed_scenario"] == SCENARIO_ID
    assert evidence["identity"]["observed_seed"] == SEED


def test_canonical_report_wrong_scenario_rejected(tmp_path) -> None:
    report = _canonical_report()
    report["scenario"] = {
        **report["scenario"],
        "id": "qualification-other",
    }
    attempt_dir = _write(tmp_path, report)
    ok, _evidence, reason = _runner()._scenario_readiness(_manifest(attempt_dir))
    assert ok is False
    assert "identity does not match" in reason


def test_canonical_report_wrong_seed_rejected(tmp_path) -> None:
    report = _canonical_report()
    report["scenario"] = {
        **report["scenario"],
        "seed": 99,
    }
    attempt_dir = _write(tmp_path, report)
    ok, _evidence, reason = _runner()._scenario_readiness(_manifest(attempt_dir))
    assert ok is False
    assert "identity does not match" in reason


def test_missing_report_rejected(tmp_path) -> None:
    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir(parents=True, exist_ok=True)
    ok, _evidence, reason = _runner()._scenario_readiness(_manifest(attempt_dir))
    assert ok is False
    assert "scenario-runner.json is missing" in reason
