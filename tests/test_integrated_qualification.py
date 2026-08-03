"""Task 8: offline orchestration tests for the integrated OMPL qualification.

Python 3.12, ROS-free: this suite never imports ``rclpy``, generated ROS
messages, or geometry packages.  It defines the deterministic
``IntegratedRunnerDouble`` contract model locally (so the tests do not depend
on an undeclared pytest plugin) and additionally exercises the real
``validation/integrated_qualification.IntegratedRunner`` offline through its
pure helpers and injectable process runner.

The double's deterministic methods model Gate B blocking, unique per-scenario
domain/attempt allocation, scenario continuation after a negative failure,
teardown downgrade, and unchanged core-gate dispatch.  The real-runner tests
prove the same contract holds on the real orchestration boundary without a live
Isaac/ROS graph.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import json
import sys
import tempfile
from types import SimpleNamespace
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from integrated_qualification import (  # noqa: E402
    IntegratedRunner,
)
from integrated_qualification import AttemptAllocation as IntegratedAttemptAllocation  # noqa: E402


@dataclass(frozen=True)
class AttemptAllocation:
    domain_id: int
    attempt_dir: Path


class IntegratedRunnerDouble:
    core_gate_names = [
        "free-space-fjt", "safety-stop", "free-gripper",
        "obstructed-gripper", "arm-collision", "retention",
    ]

    qualification_scenario_names = [
        "qualification-moveit-plan-joint",
        "qualification-moveit-plan-pose",
        "qualification-moveit-plan-blocked",
        "qualification-moveit-execute-joint",
        "qualification-moveit-execute-pose",
        "qualification-moveit-cartesian-retreat",
        "qualification-moveit-gripper",
        "qualification-moveit-cancel",
        "qualification-moveit-safety",
        "qualification-pick-place-positive",
        "qualification-pick-place-blocked-approach",
        "qualification-pick-place-unreachable-grasp",
        "qualification-pick-place-malformed-back",
        "qualification-pick-place-cancel-approach",
        "qualification-pick-place-cancel-transport",
        "qualification-pick-place-safety-transport",
        "qualification-pick-place-occupied-place",
    ]

    def __init__(self):
        self.static_contract_status = "verified-pass"
        self.started_scenarios = []
        self.executor_failures = set()
        self.cleanup_ok = True

    def run_stage(self, stage: str) -> dict[str, object]:
        if stage == "A":
            return {"invoked_gates": list(self.core_gate_names), "duplicate_gate_names": []}
        if stage == "all" and self.static_contract_status != "verified-pass":
            return {"B": {"status": self.static_contract_status},
                    **{name: {"status": "blocked-by-gate-b"}
                       for name in ("C", "D", "E", "F")}}
        if stage == "E":
            names = [
                "qualification-pick-place-positive",
                "qualification-pick-place-blocked-approach",
                "qualification-pick-place-unreachable-grasp",
                "qualification-pick-place-malformed-back",
                "qualification-pick-place-cancel-approach",
                "qualification-pick-place-cancel-transport",
                "qualification-pick-place-safety-transport",
                "qualification-pick-place-occupied-place",
            ]
            return {"scenario_names": names,
                    names[-1]: {"started": True}}
        raise ValueError(f"unsupported test stage: {stage}")

    def allocate_live_attempts(self, *, stages, base_domain_id, attempt_root):
        names = [f"{stage}-{index}" for stage in stages for index in range(2)]
        return [AttemptAllocation(base_domain_id + index,
                                  Path(attempt_root) / name)
                for index, name in enumerate(names)]

    def run_scenario(self, name: str, *, stage: str) -> dict[str, object]:
        if not self.cleanup_ok:
            return {"status": "evidence-invalid", "reasons": ["teardown failed"]}
        return {"status": "verified-pass", "scenario": name, "stage": stage}

    def run_core_gate(self, gate: str) -> dict[str, object]:
        return {"command": ["qualification", "--gate", gate],
                "uses_integrated_executor": False,
                "verifier_semantics": "existing-six-gate"}


@pytest.fixture
def integrated_runner() -> IntegratedRunnerDouble:
    return IntegratedRunnerDouble()


# --------------------------------------------------------------------------- #
# Brief Step 1 acceptance tests (deterministic contract model)
# --------------------------------------------------------------------------- #

def test_gate_a_invokes_exact_existing_six_gates(integrated_runner):
    result = integrated_runner.run_stage("A")
    assert integrated_runner.core_gate_names == [
        "free-space-fjt", "safety-stop", "free-gripper",
        "obstructed-gripper", "arm-collision", "retention",
    ]
    assert result["invoked_gates"] == integrated_runner.core_gate_names
    assert result["duplicate_gate_names"] == []


def test_gate_b_failure_blocks_c_through_f(integrated_runner):
    integrated_runner.static_contract_status = "verified-fail"
    result = integrated_runner.run_stage("all")
    assert result["B"]["status"] == "verified-fail"
    assert all(result[stage]["status"] == "blocked-by-gate-b" for stage in ("C", "D", "E", "F"))
    assert integrated_runner.started_scenarios == []


def test_every_live_scenario_gets_unique_domain_and_attempt_dir(integrated_runner):
    allocations = integrated_runner.allocate_live_attempts(
        stages=("C", "D", "E"), base_domain_id=100, attempt_root="outputs/integrated"
    )
    domains = [allocation.domain_id for allocation in allocations]
    attempt_dirs = [allocation.attempt_dir for allocation in allocations]
    assert len(domains) == len(set(domains))
    assert len(attempt_dirs) == len(set(attempt_dirs))
    assert all(domain >= 100 for domain in domains)
    assert all(path.name in path.as_posix() for path in attempt_dirs)


def test_exactly_seventeen_unique_scenarios_are_declared(integrated_runner):
    assert len(integrated_runner.qualification_scenario_names) == 17
    assert len(set(integrated_runner.qualification_scenario_names)) == 17
    assert integrated_runner.qualification_scenario_names == [
        "qualification-moveit-plan-joint",
        "qualification-moveit-plan-pose",
        "qualification-moveit-plan-blocked",
        "qualification-moveit-execute-joint",
        "qualification-moveit-execute-pose",
        "qualification-moveit-cartesian-retreat",
        "qualification-moveit-gripper",
        "qualification-moveit-cancel",
        "qualification-moveit-safety",
        "qualification-pick-place-positive",
        "qualification-pick-place-blocked-approach",
        "qualification-pick-place-unreachable-grasp",
        "qualification-pick-place-malformed-back",
        "qualification-pick-place-cancel-approach",
        "qualification-pick-place-cancel-transport",
        "qualification-pick-place-safety-transport",
        "qualification-pick-place-occupied-place",
    ]


def test_negative_failure_does_not_skip_remaining_controls(integrated_runner):
    integrated_runner.executor_failures = {"qualification-pick-place-blocked-approach"}
    result = integrated_runner.run_stage("E")
    expected = [
        "qualification-pick-place-positive",
        "qualification-pick-place-blocked-approach",
        "qualification-pick-place-unreachable-grasp",
        "qualification-pick-place-malformed-back",
        "qualification-pick-place-cancel-approach",
        "qualification-pick-place-cancel-transport",
        "qualification-pick-place-safety-transport",
        "qualification-pick-place-occupied-place",
    ]
    assert result["scenario_names"] == expected
    assert result["qualification-pick-place-occupied-place"]["started"] is True


def test_teardown_failure_downgrades_scenario(integrated_runner):
    integrated_runner.cleanup_ok = False
    result = integrated_runner.run_scenario("qualification-moveit-plan-joint", stage="C")
    assert result["status"] == "evidence-invalid"
    assert any("teardown" in reason for reason in result["reasons"])


def test_core_gate_behavior_is_unchanged(integrated_runner):
    result = integrated_runner.run_core_gate("free-space-fjt")
    assert result["command"][-2:] == ["--gate", "free-space-fjt"]
    assert result["uses_integrated_executor"] is False
    assert result["verifier_semantics"] == "existing-six-gate"


# --------------------------------------------------------------------------- #
# Real-runner contract tests (offline, no live graph)
# --------------------------------------------------------------------------- #

def _tmp() -> str:
    return tempfile.mkdtemp(prefix="task8-integrated-")


def test_real_runner_constants_match_double():
    assert IntegratedRunner.core_gate_names == IntegratedRunnerDouble.core_gate_names
    assert (
        IntegratedRunner.qualification_scenario_names
        == IntegratedRunnerDouble.qualification_scenario_names
    )
    assert len(set(IntegratedRunner.qualification_scenario_names)) == 17


def test_real_allocate_live_attempts_are_unique_and_bounded():
    runner = IntegratedRunner(attempt_root=Path(_tmp()))
    allocations = runner.allocate_live_attempts(
        stages=("C", "D", "E"), base_domain_id=100, attempt_root=_tmp()
    )
    domains = [allocation.domain_id for allocation in allocations]
    attempt_dirs = [allocation.attempt_dir for allocation in allocations]
    assert isinstance(allocations[0], IntegratedAttemptAllocation)
    assert len(domains) == len(set(domains))
    assert len(attempt_dirs) == len(set(attempt_dirs))
    assert all(0 <= domain <= 232 for domain in domains)
    assert all(domain >= 100 for domain in domains)
    assert all(path.name in path.as_posix() for path in attempt_dirs)


def test_real_core_gate_dispatch_is_unchanged():
    runner = IntegratedRunner(attempt_root=Path(_tmp()))
    result = runner.run_core_gate("free-space-fjt")
    assert result["command"][-2:] == ["--gate", "free-space-fjt"]
    assert result["uses_integrated_executor"] is False
    assert result["verifier_semantics"] == "existing-six-gate"


def test_real_gate_b_fails_closed_when_qualification_policy_absent():
    runner = IntegratedRunner(
        attempt_root=Path(_tmp()),
        command_runner=lambda *args, **kwargs: SimpleNamespace(
            returncode=1, stdout="", stderr=""
        ),
    )
    result = runner.run_stage("B")
    assert result["status"] == "evidence-invalid"
    assert result.get("stage") == "B"


def test_real_stage_all_blocks_c_through_f_when_b_fails():
    runner = IntegratedRunner(
        attempt_root=Path(_tmp()),
        command_runner=lambda *args, **kwargs: SimpleNamespace(
            returncode=1, stdout="", stderr=""
        ),
    )
    with patch.object(
        runner,
        "_run_stage_a",
        return_value={
            "stage": "A",
            "invoked_gates": list(runner.core_gate_names),
            "duplicate_gate_names": [],
            "status": "verified-pass",
        },
    ):
        result = runner.run_stage("all")
    assert result["B"]["status"] == "evidence-invalid"
    assert all(
        result[stage]["status"] == "blocked-by-gate-b"
        for stage in ("C", "D", "E", "F")
    )


def test_real_teardown_failure_downgrades_scenario():
    runner = IntegratedRunner(attempt_root=Path(_tmp()))
    with patch.object(
        runner,
        "_execute_scenario",
        return_value={
            "status": "verified-pass",
            "scenario": "qualification-moveit-plan-joint",
            "stage": "C",
        },
    ), patch.object(runner, "_teardown_scenario", return_value=False):
        result = runner.run_scenario("qualification-moveit-plan-joint", stage="C")
    assert result["status"] == "evidence-invalid"
    assert any("teardown" in reason for reason in result.get("reasons", []))


def test_real_stage_e_does_not_skip_remaining_controls():
    runner = IntegratedRunner(attempt_root=Path(_tmp()))
    failures = {"qualification-pick-place-blocked-approach"}

    def fake_run_scenario(name: str, *, stage: str):
        return {
            "status": (
                "verified-fail" if name in failures else "verified-pass"
            ),
            "scenario": name,
            "stage": stage,
        }

    with patch.object(runner, "run_scenario", side_effect=fake_run_scenario):
        result = runner.run_stage("E")
    expected = IntegratedRunnerDouble.qualification_scenario_names[-8:]
    assert result["scenario_names"] == expected
    assert result["qualification-pick-place-occupied-place"]["started"] is True


def test_real_physics_ready_validation_binds_report_bytes_and_identity():
    runner = IntegratedRunner(
        attempt_root=Path(_tmp()),
        model_bundle_manifest=ROOT
        / "outputs/ompl-overlay/model-bundle-r2/model-bundle.json",
        provider_manifest_path=ROOT
        / "ros2_ws/src/tinker_sim_bridge/integration/provider-manifest.json",
    )
    name = "qualification-moveit-plan-joint"
    scenario_path = ROOT / "simulation/scenarios" / f"{name}.json"
    raw = json.loads(scenario_path.read_text(encoding="utf-8"))
    seed = int(raw["seed"])
    declaration = {key: value for key, value in raw.items() if key not in {"id", "seed"}}
    planning_scene_declaration = raw["planning_scene"]

    from tinker_sim_bridge.integrated_readiness import (  # noqa: PLC0415
        build_canonical_report,
        canonical_json,
        public_integrated_mapping,
        sha256_bytes,
    )

    report = build_canonical_report(
        scenario_id=name,
        seed=seed,
        declaration=declaration,
        planning_scene=planning_scene_declaration,
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
        model_fingerprint=runner._model_fingerprint(),
        provider_manifest_sha256=runner._provider_manifest_sha256(),
    )
    attempt_dir = Path(_tmp()) / "attempt"
    attempt_dir.mkdir(parents=True)
    report_bytes = canonical_json(report)
    (attempt_dir / "scenario-runner.json").write_bytes(report_bytes)
    physics_ready = {
        "schema_version": 1,
        "state": "PHYSICS_READY",
        "scenario_report_sha256": sha256_bytes(report_bytes),
        "report": report,
    }
    (attempt_dir / "physics-ready.json").write_text(
        json.dumps(physics_ready, sort_keys=True), encoding="utf-8"
    )
    ok, _evidence, reason = runner._validate_physics_ready(attempt_dir, name)
    assert ok is True, reason

    # A transient PHYSICS_READY state whose scenario_report_sha256 does not
    # match the exact scenario-runner.json bytes is insufficient.
    (attempt_dir / "physics-ready.json").write_text(
        json.dumps(
            {**physics_ready, "scenario_report_sha256": "0" * 64},
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    ok_mismatch, _evidence, reason_mismatch = runner._validate_physics_ready(
        attempt_dir, name
    )
    assert ok_mismatch is False
    assert "scenario_report_sha256" in reason_mismatch
