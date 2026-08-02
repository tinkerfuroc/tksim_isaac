"""Task 1 shared test scenario loader (ROS-free).

This module defines the canonical ``load_test_scenario`` helper consumed by the
Task 1 config tests and by later tasks (Task 4/7 ``scenario_report_contract``
style consumers).  It exposes the complete immutable scenario mapping, the
four-key public planning-scene report mapping, the full planning-scene
declaration, the full per-scenario ``integrated`` mapping, and the report
identity inputs.

The public ``scenario-runner.json`` report carries only the one-key public
``integrated`` mapping (``{"execution_profile": "sim_ompl"}``); the full
scenario ``integrated`` mapping is bound by the scenario declaration SHA-256 and
preserved in separate readiness/executor evidence.  ``expected_physics_ready_report``
builds exactly that canonical public report shape.
"""
from __future__ import annotations

import copy
import json
import sys
from pathlib import Path
from typing import Any, Mapping

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.integrated_readiness import (  # noqa: E402
    build_canonical_report,
    planning_scene_mapping,
    public_integrated_mapping,
    report_identities,
    sha256_json,
)

MODEL_FINGERPRINT = "2" * 64
PROVIDER_MANIFEST_SHA256 = "3" * 64


def load_test_scenario(scenario_name: str) -> dict[str, Any]:
    """Load a scenario file into its complete immutable Task 1 mappings.

    Returns:
      ``scenario``: canonical report scenario mapping ``{id, seed, declaration}``
      ``planning_scene``: the four-key public report planning-scene mapping
      ``planning_scene_declaration``: the full planning-scene declaration
      ``integrated``: the full per-scenario integrated mapping
      ``report_identities``: the complete seven-key identity mapping produced by
        ``tinker_sim_bridge.integrated_readiness.report_identities`` over the
        full planning-scene declaration and the public one-key integrated
        mapping (so ``integrated_sha256`` is the digest of
        ``{"execution_profile": "sim_ompl"}``, never the full scenario mapping)
    """
    if not scenario_name or "/" in scenario_name or scenario_name in {".", ".."}:
        raise ValueError(f"unsafe scenario name: {scenario_name!r}")
    path = ROOT / "simulation" / "scenarios" / f"{scenario_name}.json"
    if not path.is_file():
        raise FileNotFoundError(f"scenario not found: {path}")
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"{path}: scenario declaration must be a JSON object")
    if raw.get("schema_version") != 2:
        raise ValueError(f"{path}: scenario schema_version must be 2")
    seed = int(raw["seed"])
    if str(raw.get("id")) != scenario_name:
        raise ValueError(f"{path}: id does not match filename")
    declaration = {
        str(key): value for key, value in raw.items() if key not in {"id", "seed"}
    }
    scenario_mapping = {"id": scenario_name, "seed": seed, "declaration": declaration}
    planning_scene_declaration = raw.get("planning_scene")
    if not isinstance(planning_scene_declaration, dict):
        raise ValueError(f"{path}: scenario has no planning_scene object")
    planning_scene = planning_scene_mapping(planning_scene_declaration)
    integrated = raw.get("integrated")
    if not isinstance(integrated, dict):
        raise ValueError(f"{path}: scenario has no integrated object")
    identities = report_identities(
        scenario_id=scenario_name,
        seed=seed,
        declaration=declaration,
        planning_scene=planning_scene_declaration,
        integrated=public_integrated_mapping(),
        model_fingerprint=MODEL_FINGERPRINT,
        provider_manifest_sha256=PROVIDER_MANIFEST_SHA256,
    )
    return {
        "scenario": scenario_mapping,
        "planning_scene": planning_scene,
        "planning_scene_declaration": planning_scene_declaration,
        "integrated": integrated,
        "report_identities": dict(identities),
    }


def expected_physics_ready_report(
    *,
    scenario_mapping: Mapping[str, Any],
    planning_scene: Mapping[str, Any],
    integrated: Mapping[str, Any],
    expected_identities: Mapping[str, Any],
) -> dict[str, Any]:
    """Build the canonical public ``scenario-runner.json`` report mapping.

    The report carries the one-key public ``integrated`` mapping exactly as the
    shipped production canonical parser requires; the full scenario ``integrated``
    mapping passed in is asserted to carry ``execution_profile == "sim_ompl"`` and
    is bound by the scenario declaration SHA-256 (checked here).
    """
    assert integrated.get("execution_profile") == "sim_ompl"
    assert str(scenario_mapping["id"]) == str(expected_identities["scenario_id"])
    assert int(scenario_mapping["seed"]) == int(expected_identities["seed"])
    public_integrated = public_integrated_mapping()
    report = build_canonical_report(
        scenario_id=str(scenario_mapping["id"]),
        seed=int(scenario_mapping["seed"]),
        declaration=dict(scenario_mapping["declaration"]),
        planning_scene=planning_scene,
        integrated=public_integrated,
        operations=[
            {"operation": "reset_spawned", "accepted": True},
            {
                "operation": "set_simulation_state",
                "accepted": True,
                "state": 1,
                "boundary": "PHYSICS_READY",
            },
        ],
        model_fingerprint=MODEL_FINGERPRINT,
        provider_manifest_sha256=PROVIDER_MANIFEST_SHA256,
    )
    report = copy.deepcopy(dict(report))
    # The full integrated mapping is bound by the scenario declaration SHA-256;
    # the public report carries only the one-key mapping and its digest.
    assert report["identities"]["scenario_declaration_sha256"] == sha256_json(
        scenario_mapping
    )
    assert report["integrated"] == public_integrated
    assert report["identities"]["integrated_sha256"] == sha256_json(public_integrated)
    return report
