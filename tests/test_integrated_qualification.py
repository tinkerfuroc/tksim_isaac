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

from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path
import json
import os
import sys
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

import integrated_qualification as iq  # noqa: E402
from integrated_qualification import (  # noqa: E402
    IntegratedRunner,
)
from integrated_qualification import AttemptAllocation as IntegratedAttemptAllocation  # noqa: E402
from integrated_qualification import (  # noqa: E402
    STATUS_BLOCKED,
    STATUS_EVIDENCE_INVALID,
    STATUS_NOT_IMPLEMENTED,
    STATUS_VERIFIED_FAIL,
    STATUS_VERIFIED_PASS,
    _exit_code_for_status,
    _overall_status,
    main,
)
from integrated_qualification import qualification_jsonl_records  # noqa: E402


class FakeProcess:
    """A process double that reports an already-completed child (poll()=0).

    ``poll()`` returning a non-None value keeps ``QualificationRunner._stop`` on
    the planned-termination False branch, so no real ``killpg``/signal is ever
    sent to a fake pid during offline lifecycle tests.
    """

    def __init__(self, pid: int = 424242):
        self.pid = pid
        self.returncode = 0

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        return self.returncode

    def send_signal(self, sig):
        return None

    def terminate(self):
        return None

    def kill(self):
        return None


class FakePopen:
    """A popen double that records every child invocation and returns FakeProcess."""

    def __init__(self, on_start=None):
        self.calls: list[dict[str, object]] = []
        self._on_start = on_start or (lambda name, command, env: None)

    @staticmethod
    def _infer_name(command: list[str]) -> str:
        joined = " ".join(str(part) for part in command)
        if "launch-isaac" in joined:
            return "isaac"
        if "launch-humble" in joined:
            return "humble"
        if "integrated_gate_executor_driver.py" in joined:
            return "executor"
        if "ros2" in joined and "bag" in joined:
            return "rosbag"
        return "unknown"

    def __call__(self, command, **kwargs):
        name = self._infer_name(list(command))
        env = dict(kwargs.get("env", {}))
        self.calls.append({"name": name, "command": list(command), "env": env})
        self._on_start(name, list(command), env)
        return FakeProcess(pid=424242 + len(self.calls))


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

    def fake_run_scenario(name: str, *, stage: str, allocation=None):
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


# --------------------------------------------------------------------------- #
# Fix round 1 — real manifest/launch/lifecycle path exercised with doubles
# --------------------------------------------------------------------------- #

SCENARIO_C = "qualification-moveit-plan-joint"


def _scenario_evidence_files(attempt_dir: Path, runner: IntegratedRunner, name: str):
    """Write a valid scenario-runner.json + physics-ready.json pair in a dir."""
    from tinker_sim_bridge.integrated_readiness import (  # noqa: PLC0415
        build_canonical_report,
        canonical_json,
        public_integrated_mapping,
        sha256_bytes,
    )

    scenario_path = ROOT / "simulation/scenarios" / f"{name}.json"
    raw = json.loads(scenario_path.read_text(encoding="utf-8"))
    seed = int(raw["seed"])
    declaration = {key: value for key, value in raw.items() if key not in {"id", "seed"}}
    report = build_canonical_report(
        scenario_id=name,
        seed=seed,
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
        model_fingerprint=runner._model_fingerprint(),
        provider_manifest_sha256=runner._provider_manifest_sha256(),
    )
    report_bytes = canonical_json(report)
    (attempt_dir / "scenario-runner.json").write_bytes(report_bytes)
    (attempt_dir / "physics-ready.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "state": "PHYSICS_READY",
                "scenario_report_sha256": sha256_bytes(report_bytes),
                "report": report,
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )


def _evidence_runner(**kwargs):
    """A runner wired with the real model/provider manifests and a fake popen."""
    return IntegratedRunner(
        attempt_root=Path(_tmp()),
        model_bundle_manifest=ROOT / "outputs/ompl-overlay/model-bundle-r2/model-bundle.json",
        provider_manifest_path=ROOT
        / "ros2_ws/src/tinker_sim_bridge/integration/provider-manifest.json",
        **kwargs,
    )


def test_real_scenario_runner_uses_integrated_gate_not_core_selection():
    """A real scenario id never reaches the core six-gate ``_selected_gates``."""
    runner = _evidence_runner()
    allocation = runner._allocate_one(SCENARIO_C, "C")
    scenario_runner = runner._new_scenario_runner(allocation, SCENARIO_C)
    assert scenario_runner.gate == "integrated"
    # prepare_manifest_at with a real scenario id must not raise "unknown gate".
    manifest = scenario_runner.prepare_manifest_at(
        allocation.attempt_dir.name, allocation.attempt_dir, scenario_id=SCENARIO_C
    )
    assert manifest.data["gate"] == "integrated"
    assert manifest.data["selected_gates"] == []


def test_real_launch_path_starts_two_children_with_exact_wiring():
    """The real launch path calls both children with env/domain/attempt wiring."""
    popen = FakePopen()
    runner = _evidence_runner(popen=popen)
    allocation = runner._allocate_one(SCENARIO_C, "C")
    scenario_runner, manifest = runner._launch_scenario(allocation, SCENARIO_C, "C")

    assert len(popen.calls) == 2
    names = [call["name"] for call in popen.calls]
    assert names == ["isaac", "humble"]
    for call in popen.calls:
        env = call["env"]
        assert env["ROS_DOMAIN_ID"] == str(allocation.domain_id)
        assert env["TINKER_SIM_ATTEMPT_DIR"] == str(allocation.attempt_dir)
    humble_command = " ".join(str(part) for part in popen.calls[1]["command"])
    assert f"scenario:={SCENARIO_C}" in humble_command
    assert f"seed:={runner.seed}" in humble_command
    assert f"attempt_dir:={allocation.attempt_dir}" in humble_command
    assert "qualification:=true" in humble_command
    # One authoritative immutable directory: allocation == manifest == runner.
    assert manifest.attempt_dir == allocation.attempt_dir
    assert scenario_runner._attempt_dir == allocation.attempt_dir


def test_real_zero_child_launch_cannot_pass_readiness():
    """Launching children that produce no evidence can never satisfy readiness."""
    popen = FakePopen()
    runner = _evidence_runner(popen=popen, readiness_timeout_s=0.6)
    allocation = runner._allocate_one(SCENARIO_C, "C")
    _scenario_runner, manifest = runner._launch_scenario(allocation, SCENARIO_C, "C")
    ok, _evidence, reason = runner._wait_for_physics_ready(
        allocation.attempt_dir, SCENARIO_C, manifest=manifest
    )
    assert ok is False
    assert reason is not None
    assert "scenario-runner.json" in reason or "missing" in reason


def test_real_stale_prepopulated_attempt_evidence_cannot_pass():
    """A prior attempt's evidence cannot satisfy readiness for a fresh run."""
    runner = _evidence_runner(readiness_timeout_s=0.4)
    name = SCENARIO_C
    attempt_dir = Path(_tmp()) / "attempt"
    attempt_dir.mkdir(parents=True)
    _scenario_evidence_files(attempt_dir, runner, name)
    # No manifest.json exists: the runner never launched this attempt, so the
    # pre-existing evidence must be rejected (not trusted as a false pass).
    manifest = SimpleNamespace(
        attempt_id="fresh-attempt",
        attempt_dir=attempt_dir,
        data={"environment": {"ROS_DOMAIN_ID": "105"}},
    )
    ok, _evidence, reason = runner._wait_for_physics_ready(attempt_dir, name, manifest=manifest)
    assert ok is False
    assert "manifest.json is missing" in reason


def test_real_launch_manifest_binds_attempt_id_to_allocation():
    """A stale manifest (different attempt_id) cannot satisfy readiness."""
    runner = _evidence_runner(readiness_timeout_s=0.4)
    name = SCENARIO_C
    attempt_dir = Path(_tmp()) / "attempt"
    attempt_dir.mkdir(parents=True)
    _scenario_evidence_files(attempt_dir, runner, name)
    (attempt_dir / "manifest.json").write_text(
        json.dumps(
            {
                "attempt_id": "other-attempt",
                "scenario": {"id": name},
                "environment": {"ROS_DOMAIN_ID": "105"},
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    manifest = SimpleNamespace(
        attempt_id="fresh-attempt",
        attempt_dir=attempt_dir,
        data={"environment": {"ROS_DOMAIN_ID": "105"}},
    )
    ok, _evidence, reason = runner._wait_for_physics_ready(attempt_dir, name, manifest=manifest)
    assert ok is False
    assert "attempt_id" in reason


def test_real_repeated_allocation_creates_distinct_preserved_dirs():
    """Repeated allocation for one scenario yields distinct preserved dirs."""
    runner = _evidence_runner()
    first = runner._allocate_one(SCENARIO_C, "C")
    second = runner._allocate_one(SCENARIO_C, "C")
    assert first.attempt_dir != second.attempt_dir
    assert first.attempt_dir.is_dir() and second.attempt_dir.is_dir()
    # Prior evidence is preserved: the second allocation never reuses/overwrites.
    (first.attempt_dir / "stale-evidence.json").write_text(
        json.dumps({"stale": True}), encoding="utf-8"
    )
    assert (second.attempt_dir / "stale-evidence.json").exists() is False


def test_real_shutdown_appended_frames_are_verified_after_final_frame():
    """Verification runs only after shutdown-appended frames enter the drain."""
    popen = FakePopen()
    runner = _evidence_runner(popen=popen)
    allocation = runner._allocate_one(SCENARIO_C, "C")
    observed_raw_counts: list[int] = []

    def fake_verify(attempt_dir, name, stage):
        raw_records, _raw_errors = qualification_jsonl_records(
            Path(attempt_dir) / "physics_truth.jsonl"
        )
        observed_raw_counts.append(len(raw_records))
        return {"status": "verified-pass", "observed_raw": len(raw_records)}

    original_stop = iq.qualification_stop_process

    def appending_stop(runner, name):
        if name == "isaac":
            attempt_dir = Path(runner._attempt_dir)
            raw = {"frame_id": 7, "seq": 7}
            evaluator = {"frame": raw, "seq": 7}
            with (attempt_dir / "physics_truth.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(raw, sort_keys=True) + "\n")
            with (attempt_dir / "evaluator.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(evaluator, sort_keys=True) + "\n")
        return original_stop(runner, name)

    with patch.object(
        runner, "_wait_for_physics_ready", return_value=(True, {}, None)
    ), patch.object(
        runner, "_wait_for_scenario_terminal", return_value={"ok": True, "terminal": {}}
    ), patch.object(
        runner, "_verify_attempt", side_effect=fake_verify
    ), patch.object(
        iq, "qualification_stop_process", side_effect=appending_stop
    ), patch.object(
        iq.QualificationRunner, "_start_rosbag", return_value=True
    ):
        result = runner._execute_scenario(allocation, SCENARIO_C, "C")

    assert result["status"] == "verified-pass"
    assert observed_raw_counts == [1]
    assert result["verdict"]["observed_raw"] == 1
    assert result["finalize"]["drained"] is True


def test_real_cleanup_runs_after_injected_lifecycle_exception():
    """A lifecycle exception still stops children and clears the owner."""
    popen = FakePopen()
    runner = _evidence_runner(popen=popen)
    allocation = runner._allocate_one(SCENARIO_C, "C")
    with patch.object(
        runner, "_wait_for_physics_ready", side_effect=RuntimeError("injected boom")
    ):
        result = runner._execute_scenario(allocation, SCENARIO_C, "C")
    assert result["status"] == "evidence-invalid"
    assert "injected boom" in result["error"]
    assert allocation.attempt_dir not in runner._scenario_manifest_store
    finalize = result.get("finalize", {})
    # Both children were stopped and accounted despite the injected exception.
    assert finalize.get("exit_codes", {}).get("isaac") == 0
    assert finalize.get("exit_codes", {}).get("humble") == 0


def test_real_cleanup_runs_after_launch_partial_failure():
    """A humble launch failure still stops the already-started Isaac child."""
    popen = FakePopen()
    runner = _evidence_runner(popen=popen)
    allocation = runner._allocate_one(SCENARIO_C, "C")
    original_start = iq.qualification_start_process

    def failing_start(runner, name, command, manifest):
        if name == "humble":
            raise RuntimeError("humble start failed")
        return original_start(runner, name, command, manifest)

    with patch.object(iq, "qualification_start_process", side_effect=failing_start):
        result = runner._execute_scenario(allocation, SCENARIO_C, "C")
    assert result["status"] == "evidence-invalid"
    assert "humble start failed" in result["error"]
    assert allocation.attempt_dir not in runner._scenario_manifest_store
    finalize = result.get("finalize", {})
    assert finalize.get("exit_codes", {}).get("isaac") == 0


def test_real_source_lock_fail_is_evidence_invalid():
    """A producer 'fail' status is authorization evidence invalid, not verified-fail."""

    def fake_command_runner(command, **kwargs):
        command = list(command)
        if "--output" in command:
            output = Path(command[command.index("--output") + 1])
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "status": "fail",
                        "output_predates_attempt": False,
                        "reasons": ["mismatched lock"],
                    },
                    sort_keys=True,
                ),
                encoding="utf-8",
            )
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    runner = IntegratedRunner(
        attempt_root=Path(_tmp()), command_runner=fake_command_runner
    )
    result = runner.run_stage("B")
    assert result["status"] == STATUS_EVIDENCE_INVALID


def test_real_static_contract_producer_exit_zero_without_output_is_invalid():
    """Producer exit 0 without a newly written static contract is invalid."""

    def fake_command_runner(command, **kwargs):
        command = list(command)
        joined = " ".join(str(part) for part in command)
        if "source_lock_manifest.py" in joined and "--output" in command:
            output = Path(command[command.index("--output") + 1])
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "status": "pass",
                        "output_predates_attempt": False,
                    },
                    sort_keys=True,
                ),
                encoding="utf-8",
            )
        # integrated_static_contracts.py exits 0 but never writes its output.
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    runner = IntegratedRunner(
        attempt_root=Path(_tmp()), command_runner=fake_command_runner
    )
    result = runner.run_stage("B")
    assert result["status"] == STATUS_EVIDENCE_INVALID
    assert any("static-contract.json" in reason for reason in result.get("reasons", []))


def test_real_static_contract_fingerprint_mismatch_is_rejected():
    """Gate B's model fingerprint must match the runtime model bundle."""

    def fake_command_runner(command, **kwargs):
        command = list(command)
        joined = " ".join(str(part) for part in command)
        if "source_lock_manifest.py" in joined and "--output" in command:
            output = Path(command[command.index("--output") + 1])
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "status": "pass",
                        "output_predates_attempt": False,
                    },
                    sort_keys=True,
                ),
                encoding="utf-8",
            )
        if "integrated_static_contracts.py" in joined and "--output" in command:
            output = Path(command[command.index("--output") + 1])
            output.mkdir(parents=True, exist_ok=True)
            (output / "static-contract.json").write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "status": "verified-pass",
                        "model_fingerprint": "0" * 64,
                    },
                    sort_keys=True,
                ),
                encoding="utf-8",
            )
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    runner = IntegratedRunner(
        attempt_root=Path(_tmp()),
        model_bundle_manifest=ROOT / "outputs/ompl-overlay/model-bundle-r2/model-bundle.json",
        command_runner=fake_command_runner,
    )
    result = runner.run_stage("B")
    assert result["status"] == STATUS_EVIDENCE_INVALID
    assert any(
        "model_fingerprint" in reason for reason in result.get("reasons", [])
    )


def test_real_malformed_scenario_does_not_skip_later_controls():
    """One malformed scenario fails closed without aborting the rest of stage E."""
    runner = _evidence_runner()
    malformed = "qualification-pick-place-malformed-back"
    last = "qualification-pick-place-occupied-place"

    def fake_scenario_bundle(name):
        if name == malformed:
            raise ValueError("malformed scenario declaration")
        return {"scenario": {"id": name, "seed": 7}, "planning_scene": {}, "report_identities": {}}

    def fake_execute(allocation, name, stage):
        return {"scenario": name, "stage": stage, "status": "verified-pass", "started": True}

    with patch.object(runner, "_scenario_bundle", side_effect=fake_scenario_bundle), patch.object(
        runner, "_execute_scenario", side_effect=fake_execute
    ):
        result = runner.run_stage("E")
    assert result["scenario_names"][-1] == last
    assert result[malformed]["status"] == STATUS_EVIDENCE_INVALID
    assert result[malformed]["started"] is False
    assert result[last]["started"] is True
    assert result["status"] == STATUS_EVIDENCE_INVALID


def test_real_standalone_stage_c_pass_exits_zero():
    with patch.object(
        IntegratedRunner, "run_stage", return_value={"stage": "C", "status": STATUS_VERIFIED_PASS}
    ):
        assert main(["--stage", "C"]) == 0


def test_real_standalone_stage_c_fail_exits_one():
    with patch.object(
        IntegratedRunner, "run_stage", return_value={"stage": "C", "status": STATUS_VERIFIED_FAIL}
    ):
        assert main(["--stage", "C"]) == 1


def test_real_standalone_stage_c_invalid_exits_two():
    with patch.object(
        IntegratedRunner,
        "run_stage",
        return_value={"stage": "C", "status": STATUS_EVIDENCE_INVALID},
    ):
        assert main(["--stage", "C"]) == 2


def test_real_all_blocks_c_through_f_exits_nonzero_and_retains_a():
    blocked_shape = {
        "A": {"status": STATUS_VERIFIED_PASS, "invoked_gates": list(IntegratedRunner.core_gate_names)},
        "B": {"status": STATUS_EVIDENCE_INVALID, "stage": "B"},
        **{stage: {"status": STATUS_BLOCKED} for stage in ("C", "D", "E", "F")},
    }
    with patch.object(IntegratedRunner, "run_stage", return_value=blocked_shape):
        assert main(["--stage", "all"]) != 0
    assert "A" in blocked_shape
    overall = _overall_status(blocked_shape)
    assert overall == STATUS_EVIDENCE_INVALID
    assert _exit_code_for_status(overall) != 0


def test_real_all_retains_stage_a_when_gate_b_fails():
    runner = IntegratedRunner(attempt_root=Path(_tmp()))
    with patch.object(
        runner, "_run_stage_a", return_value={"stage": "A", "status": STATUS_VERIFIED_PASS}
    ), patch.object(
        runner, "_run_stage_b", return_value={"stage": "B", "status": STATUS_EVIDENCE_INVALID}
    ):
        result = runner.run_stage("all")
    assert "A" in result
    assert result["A"]["status"] == STATUS_VERIFIED_PASS
    assert result["B"]["status"] == STATUS_EVIDENCE_INVALID
    assert all(result[stage]["status"] == STATUS_BLOCKED for stage in ("C", "D", "E", "F"))


def test_real_a_e_pass_f_not_implemented_exits_nonzero():
    pass_shape = {
        "A": {"status": STATUS_VERIFIED_PASS},
        "B": {"status": STATUS_VERIFIED_PASS},
        "C": {"status": STATUS_VERIFIED_PASS},
        "D": {"status": STATUS_VERIFIED_PASS},
        "E": {"status": STATUS_VERIFIED_PASS},
        "F": {"status": STATUS_NOT_IMPLEMENTED},
    }
    overall = _overall_status(pass_shape)
    assert overall == STATUS_NOT_IMPLEMENTED
    assert _exit_code_for_status(overall) != 0
    with patch.object(IntegratedRunner, "run_stage", return_value=pass_shape):
        assert main(["--stage", "all"]) != 0


def test_real_stage_a_required_executed_gate_drift_is_rejected():
    runner = IntegratedRunner(attempt_root=Path(_tmp()))
    with patch.object(
        runner, "_core_config_gates", return_value=["free-space-fjt"]
    ), patch.object(runner, "_run_core_suite", side_effect=AssertionError("must not run")):
        result = runner.run_stage("A")
    assert result["status"] == STATUS_EVIDENCE_INVALID
    assert result["executed_gates"] == ["free-space-fjt"]
    assert any("does not equal" in reason for reason in result.get("reasons", []))


# --------------------------------------------------------------------------- #
# Task 8 fix round 2 — executor evidence producer + sealed finalization
# --------------------------------------------------------------------------- #

def test_real_derived_terminal_timeout_is_exactly_305_and_separate_from_readiness():
    """F2.5: terminal budget derives to 305.0 and is not the readiness budget."""
    runner = _evidence_runner()
    assert runner.terminal_timeout_s == 305.0
    # Separate budget: a marker arriving after the 30 s readiness budget (but
    # before the 305 s derived deadline) is still eligible.
    assert runner.readiness_timeout_s == 30.0
    assert runner.terminal_timeout_s != runner.readiness_timeout_s
    assert runner.terminal_timeout_s > 30.0
    # Constructor override is accepted for deterministic offline tests.
    assert _evidence_runner(terminal_timeout_s=0.4).terminal_timeout_s == 0.4
    # Malformed config fails closed at construction.
    with patch.object(
        runner, "_config", return_value={"thresholds": {"plan_timeout_s": 15.0}}
    ):
        with pytest.raises(ValueError):
            runner._derived_terminal_timeout()


def test_real_terminal_marker_after_readiness_budget_is_eligible():
    """F2.5: the terminal wait uses the derived deadline, not the 30 s readiness
    budget, so a marker arriving after 30 s but before 305 s is eligible."""
    runner = _evidence_runner(terminal_timeout_s=305.0, readiness_timeout_s=30.0)
    attempt_dir = Path(_tmp()) / "attempt"
    attempt_dir.mkdir(parents=True)
    (attempt_dir / "execution-terminal.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "scenario_id": SCENARIO_C,
                "attempt_id": "attempt-x",
                "attempt_dir": str(attempt_dir.resolve()),
                "status": "verified-pass",
                "marker": "executor-driver",
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    start = 1000.0
    # First call sets the deadline (start + 305); the loop then observes 35 s
    # elapsed — inside the terminal budget, but past the readiness budget.
    with patch.object(iq.time, "monotonic", side_effect=[start, start + 35.0]):
        result = runner._wait_for_scenario_terminal(
            attempt_dir, SCENARIO_C, attempt_id="attempt-x"
        )
    assert result["ok"] is True
    assert result["source"] == "executor-driver"


def test_real_terminal_cross_binds_reject_wrong_identity():
    """F2.3: a terminal marker must bind the current scenario/attempt/path."""
    runner = _evidence_runner()
    attempt_dir = Path(_tmp()) / "attempt"
    attempt_dir.mkdir(parents=True)
    good = {
        "schema_version": 1,
        "scenario_id": SCENARIO_C,
        "attempt_id": "attempt-x",
        "attempt_dir": str(attempt_dir.resolve()),
        "status": "verified-pass",
        "marker": "executor-driver",
    }
    assert runner._terminal_cross_binds(good, attempt_dir, SCENARIO_C, "attempt-x") is True
    assert (
        runner._terminal_cross_binds(dict(good, scenario_id="qualification-moveit-plan-pose"),
                                     attempt_dir, SCENARIO_C, "attempt-x")
        is False
    )
    assert (
        runner._terminal_cross_binds(dict(good, attempt_id="attempt-other"),
                                     attempt_dir, SCENARIO_C, "attempt-x")
        is False
    )
    assert (
        runner._terminal_cross_binds(dict(good, attempt_dir=str(Path(_tmp()) / "elsewhere")),
                                     attempt_dir, SCENARIO_C, "attempt-x")
        is False
    )
    # attempt_id=None binds scenario + path only.
    assert (
        runner._terminal_cross_binds(dict(good, attempt_id="anything"),
                                     attempt_dir, SCENARIO_C, None)
        is True
    )


def test_real_executor_launches_after_physics_ready_with_exact_wiring():
    """F2.3: the executor driver launches after the rosbag recorder only after
    canonical PHYSICS_READY, with the exact source-run command and ros-tooling env."""
    order: list[str] = []

    def on_start(name, command, env):
        order.append(f"start:{name}")

    popen = FakePopen(on_start=on_start)
    runner = _evidence_runner(popen=popen)
    allocation = runner._allocate_one(SCENARIO_C, "C")

    def fake_ready(attempt_dir, name, *, manifest=None):
        order.append("physics-ready")
        return True, {"ready": True}, None

    def fake_terminal(attempt_dir, name, *, runner=None, attempt_id=None):
        return {"ok": True, "source": "executor-driver"}

    def fake_verify(attempt_dir, name, stage):
        return {"status": "verified-pass", "scenario": name}

    def fake_start_rosbag(scenario_runner, manifest):
        iq.qualification_start_process(
            scenario_runner, "rosbag", ["ros2", "bag", "record"], manifest
        )
        return True

    with patch.object(runner, "_wait_for_physics_ready", side_effect=fake_ready), patch.object(
        runner, "_wait_for_scenario_terminal", side_effect=fake_terminal
    ), patch.object(runner, "_verify_attempt", side_effect=fake_verify), patch.object(
        iq.QualificationRunner, "_start_rosbag", new=fake_start_rosbag
    ):
        result = runner._execute_scenario(allocation, SCENARIO_C, "C")

    # Launch-after-readiness ordering: isaac + humble first, PHYSICS_READY
    # validated, the rosbag recorder next, and only then the executor.
    assert order == ["start:isaac", "start:humble", "physics-ready", "start:rosbag", "start:executor"]
    assert len(popen.calls) == 4
    assert [call["name"] for call in popen.calls] == ["isaac", "humble", "rosbag", "executor"]

    executor_call = popen.calls[3]
    command = executor_call["command"]
    assert command[0] == "/usr/bin/python3"
    assert command[1].endswith("validation/integrated_gate_executor_driver.py")
    joined = " ".join(str(part) for part in command)
    assert f"--scenario-bundle {allocation.attempt_dir / 'scenario-bundle.json'}" in joined
    assert f"--attempt-dir {allocation.attempt_dir}" in joined
    assert f"--config {runner.config_path}" in joined
    assert f"--domain {allocation.domain_id}" in joined
    assert f"--seed {runner.seed}" in joined

    env = executor_call["env"]
    assert env["RMW_IMPLEMENTATION"] == "rmw_fastrtps_cpp"
    assert env["ROS_DOMAIN_ID"] == str(allocation.domain_id)
    assert env["TINKER_SIM_ATTEMPT_DIR"] == str(allocation.attempt_dir)
    assert "AMENT_PREFIX_PATH" in env
    assert "PYTHONPATH" in env
    assert "LD_LIBRARY_PATH" in env

    # The already-validated bundle was atomically written and cross-bound.
    bundle_path = allocation.attempt_dir / "scenario-bundle.json"
    assert bundle_path.is_file()
    bundle_value = json.loads(bundle_path.read_text(encoding="utf-8"))
    assert bundle_value["schema_version"] == 1
    assert bundle_value["scenario_id"] == SCENARIO_C
    assert bool(bundle_value["attempt_id"])
    assert Path(bundle_value["attempt_dir"]).resolve() == allocation.attempt_dir.resolve()

    # The executor process was stopped and accounted in finalize.
    finalize = result.get("finalize", {})
    assert finalize.get("exit_codes", {}).get("executor") == 0
    # The scenario lifecycle owner is cleared after _execute_scenario.
    assert allocation.attempt_dir not in runner._scenario_manifest_store


def test_real_executor_exit_without_terminal_fails_immediately():
    """F2.3: executor exit without a current-attempt marker is immediate invalid."""
    popen = FakePopen()
    runner = _evidence_runner(popen=popen)
    allocation = runner._allocate_one(SCENARIO_C, "C")
    scenario_runner, manifest = runner._launch_scenario(allocation, SCENARIO_C, "C")
    scenario_runner._processes["executor"] = FakeProcess()
    result = runner._wait_for_scenario_terminal(
        allocation.attempt_dir, SCENARIO_C, runner=scenario_runner, attempt_id=manifest.attempt_id
    )
    assert result["ok"] is False
    assert "exited without" in result["reason"]


def test_real_start_role_map_adds_executor_and_keeps_six_gate_names():
    """F2.3: the additive 'executor' role maps to ros-tooling; six-gate roles and
    unknown-name rejection are unchanged."""
    popen = FakePopen()
    runner = _evidence_runner(popen=popen)
    allocation = runner._allocate_one(SCENARIO_C, "C")
    scenario_runner, manifest = runner._launch_scenario(allocation, SCENARIO_C, "C")

    # Unknown names still fail closed.
    with pytest.raises(ValueError, match="unknown qualification process name"):
        scenario_runner._start("nonsense", ["echo", "x"], manifest)

    # Executor is an additive valid role under the ros-tooling environment.
    executor_command = [
        "/usr/bin/python3",
        str(ROOT / "validation/integrated_gate_executor_driver.py"),
        "--scenario-bundle",
        str(allocation.attempt_dir / "scenario-bundle.json"),
    ]
    scenario_runner._start("executor", executor_command, manifest)
    env = popen.calls[-1]["env"]
    assert env["RMW_IMPLEMENTATION"] == "rmw_fastrtps_cpp"
    assert env["ROS_DOMAIN_ID"] == str(allocation.domain_id)
    assert env["TINKER_SIM_ATTEMPT_DIR"] == str(allocation.attempt_dir)
    assert "AMENT_PREFIX_PATH" in env
    assert "PYTHONPATH" in env

    # The six-gate roles still resolve (isaac/humble/rosbag) unchanged.
    scenario_runner._start("isaac", ["launch-isaac"], manifest)
    scenario_runner._start("humble", ["launch-humble"], manifest)
    scenario_runner._start("rosbag", ["ros2", "bag", "record", "-a"], manifest)
    assert [call["name"] for call in popen.calls] == [
        "isaac", "humble", "executor", "isaac", "humble", "rosbag",
    ]


FINALIZE_PHASES = ("executor", "isaac", "drain", "humble", "orphan", "resource", "settle")
EXPECTED_FINALIZE_ORDER = [
    "stop:executor", "stop:isaac", "drain", "stop:humble", "orphan", "resource", "settle",
]


@pytest.mark.parametrize("inject_at", FINALIZE_PHASES)
def test_real_finalize_phase_exception_runs_all_later_phases(inject_at):
    """F2.6: each finalize phase exception is isolated; all later phases still run."""
    popen = FakePopen()
    runner = _evidence_runner(popen=popen)
    allocation = runner._allocate_one(SCENARIO_C, "C")
    scenario_runner, manifest = runner._launch_scenario(allocation, SCENARIO_C, "C")
    scenario_runner._processes["executor"] = FakeProcess(pid=55555)
    baseline = runner._gpu_baselines[allocation.attempt_dir]

    phase_log: list[str] = []
    real_stop = iq.qualification_stop_process
    real_drain = iq.qualification_wait_for_evaluator_drain
    real_orphans = iq.qualification_attempt_processes
    real_resource = iq.qualification_write_resource_evidence
    real_settle = iq.qualification_settle_evidence_files

    def stop(rnr, name):
        phase_log.append(f"stop:{name}")
        if inject_at == name:
            raise RuntimeError(f"{inject_at} stop boom")
        return real_stop(rnr, name)

    def drain(rnr, mnf):
        phase_log.append("drain")
        if inject_at == "drain":
            raise RuntimeError("drain boom")
        return real_drain(rnr, mnf)

    def orphans(rnr):
        phase_log.append("orphan")
        if inject_at == "orphan":
            raise RuntimeError("orphan boom")
        return real_orphans(rnr)

    def resource(rnr, mnf, base):
        phase_log.append("resource")
        if inject_at == "resource":
            raise RuntimeError("resource boom")
        return real_resource(rnr, mnf, base)

    def settle(rnr, mnf):
        phase_log.append("settle")
        if inject_at == "settle":
            raise RuntimeError("settle boom")

    with patch.object(iq, "qualification_stop_process", side_effect=stop), patch.object(
        iq, "qualification_wait_for_evaluator_drain", side_effect=drain
    ), patch.object(iq, "qualification_attempt_processes", side_effect=orphans), patch.object(
        iq, "qualification_write_resource_evidence", side_effect=resource
    ), patch.object(iq, "qualification_settle_evidence_files", side_effect=settle):
        finalize = runner._finalize_attempt(scenario_runner, manifest, baseline)

    # Every phase — including the injected one — was attempted in order.
    assert phase_log == EXPECTED_FINALIZE_ORDER
    assert any(f"{inject_at}" in reason for reason in finalize["failures"])
    # The non-injected producer stops were still attempted and accounted.  The
    # injected stop phase raises before returning, so that phase's exit code is
    # intentionally absent (the exception is the recorded failure).
    expected_exit_producers = [name for name in ("executor", "isaac", "humble") if name != inject_at]
    for name in expected_exit_producers:
        assert finalize["exit_codes"].get(name) == 0
    # The evaluator drain ran before the settle phases (partial artifact), except
    # when the drain phase itself was the injected failure.
    if inject_at != "drain":
        assert (manifest.attempt_dir / "truth-drain.json").is_file()


@pytest.mark.parametrize("inject_at", FINALIZE_PHASES)
def test_real_finalize_phase_exception_does_not_skip_next_scenario(inject_at):
    """F2.6: a first-scenario finalize failure yields evidence-invalid but the
    stage continues to every later scenario and the process store is cleared."""
    popen = FakePopen()
    runner = _evidence_runner(popen=popen, readiness_timeout_s=0.4)
    first_dir: dict[str, Path | None] = {"dir": None}
    real_start = iq.qualification_start_process
    real_stop = iq.qualification_stop_process
    real_drain = iq.qualification_wait_for_evaluator_drain
    real_orphans = iq.qualification_attempt_processes
    real_resource = iq.qualification_write_resource_evidence
    real_settle = iq.qualification_settle_evidence_files

    def recording_start(rnr, name, command, mnf):
        result = real_start(rnr, name, command, mnf)
        if name == "isaac" and first_dir["dir"] is None:
            first_dir["dir"] = rnr._attempt_dir
        return result

    def stop(rnr, name):
        if inject_at in ("executor", "isaac", "humble") and rnr._attempt_dir == first_dir["dir"] and name == inject_at:
            raise RuntimeError(f"{inject_at} stop boom")
        return real_stop(rnr, name)

    def drain(rnr, mnf):
        if inject_at == "drain" and rnr._attempt_dir == first_dir["dir"]:
            raise RuntimeError("drain boom")
        return real_drain(rnr, mnf)

    def orphans(rnr):
        if inject_at == "orphan" and rnr._attempt_dir == first_dir["dir"]:
            raise RuntimeError("orphan boom")
        return real_orphans(rnr)

    def resource(rnr, mnf, base):
        if inject_at == "resource" and rnr._attempt_dir == first_dir["dir"]:
            raise RuntimeError("resource boom")
        return real_resource(rnr, mnf, base)

    def settle(rnr, mnf):
        if inject_at == "settle" and rnr._attempt_dir == first_dir["dir"]:
            raise RuntimeError("settle boom")

    def fake_ready(attempt_dir, name, *, manifest=None):
        # The first scenario reaches canonical PHYSICS_READY so its rosbag
        # recorder is registered and the executor driver then launches as the
        # fourth owned child (this is what makes the executor-stop injection
        # reachable and exercises the full F2.3 path); later scenarios are left
        # to fail readiness and never launch.
        if Path(attempt_dir) == first_dir["dir"]:
            return True, {"ready": True}, None
        return False, {}, "physics-ready timeout"

    def fake_start_rosbag(scenario_runner, manifest):
        # Register a real rosbag-owned child so the executor (which launches
        # only after a successful recorder start) is genuinely started and the
        # executor-stop injection below is reachable.
        iq.qualification_start_process(
            scenario_runner, "rosbag", ["ros2", "bag", "record"], manifest
        )
        return True

    with patch.object(iq, "qualification_start_process", side_effect=recording_start), patch.object(
        iq, "qualification_stop_process", side_effect=stop
    ), patch.object(iq, "qualification_wait_for_evaluator_drain", side_effect=drain), patch.object(
        iq, "qualification_attempt_processes", side_effect=orphans
    ), patch.object(iq, "qualification_write_resource_evidence", side_effect=resource), patch.object(
        iq, "qualification_settle_evidence_files", side_effect=settle
    ), patch.object(iq.QualificationRunner, "_start_rosbag", new=fake_start_rosbag), patch.object(
        runner, "_wait_for_physics_ready", side_effect=fake_ready
    ):
        result = runner._run_scenario_stage("C")

    names = runner._stage_scenarios("C")
    assert result["scenario_names"] == names
    # Every scenario was attempted; none was skipped.
    assert runner.started_scenarios == names
    assert len(result["scenario_names"]) == 3
    # The first scenario is evidence-invalid and records the injected phase.
    first = result[names[0]]
    assert first["status"] == STATUS_EVIDENCE_INVALID
    assert any(inject_at in reason for reason in first.get("reasons", []))
    # Later scenarios still ran to completion of their own lifecycle.
    for name in names[1:]:
        assert result[name]["started"] is True
    # Process store and GPU baselines are fully cleared.
    assert runner._scenario_manifest_store == {}
    assert runner._gpu_baselines == {}


# --------------------------------------------------------------------------- #
# Task 10 — Stage records and Gate F
# --------------------------------------------------------------------------- #

import hashlib  # noqa: E402

sys.path.insert(0, str(ROOT / "tests"))  # noqa: E402
from test_integrated_evidence_index import write_canonical_evidence_tree  # noqa: E402

T10_STAGE_RECORD = iq.STAGE_RECORD_FILENAMES
T10_DERIVED = (iq.INDEX_NAME, iq.SUMMARY_NAME, iq.AGENT_SHEET_NAME, iq.USER_SHEET_NAME)


def _write_valid_suite(tmp_path: Path) -> tuple[IntegratedRunner, Path]:
    """Write a valid immutable integrated suite with sibling core + A-E records.

    Reuses the production-shaped Task-9 ``write_canonical_evidence_tree`` for the
    evidence bytes, persists write-once stage records through the real
    ``_persist_stage_record``, and allocates a unique existing attempt directory
    under the suite for every configured C/D/E scenario.  The sibling
    ``<suite>-core/suite-result.json`` carries the lowercase SHA-256 bound into
    the stage-A record.
    """
    suite_dir = tmp_path / "suite"
    write_canonical_evidence_tree(suite_dir)
    runner = IntegratedRunner(attempt_root=suite_dir)
    configured_gates = runner._core_gates()
    core_root = suite_dir.parent / f"{suite_dir.name}{iq.CORE_SUITE_DIRNAME_SUFFIX}"
    core_root.mkdir(parents=True, exist_ok=True)
    core_payload = {
        "status": STATUS_VERIFIED_PASS,
        "gates": {gate: {"status": STATUS_VERIFIED_PASS} for gate in configured_gates},
    }
    core_bytes = json.dumps(core_payload, sort_keys=True).encode("utf-8")
    (core_root / "suite-result.json").write_bytes(core_bytes)
    runner._persist_stage_record("A", {
        "stage": "A",
        "status": STATUS_VERIFIED_PASS,
        "invoked_gates": list(configured_gates),
        "executed_gates": list(configured_gates),
        "duplicate_gate_names": [],
        "core_suite": {
            "status": STATUS_VERIFIED_PASS,
            "suite_dir": str(core_root),
            "suite_result_sha256": hashlib.sha256(core_bytes).hexdigest(),
            "gate_results": {gate: {"status": STATUS_VERIFIED_PASS} for gate in configured_gates},
        },
    })
    runner._persist_stage_record("B", {"stage": "B", "status": STATUS_VERIFIED_PASS})
    for stage in ("C", "D", "E"):
        names = runner._stage_scenarios(stage)
        record = {"stage": stage, "status": STATUS_VERIFIED_PASS, "scenario_names": list(names)}
        for name in names:
            attempt_dir = suite_dir / stage / name
            attempt_dir.mkdir(parents=True, exist_ok=True)
            record[name] = {"status": STATUS_VERIFIED_PASS, "attempt_dir": str(attempt_dir)}
        runner._persist_stage_record(stage, record)
    return runner, suite_dir


def _read_stage_record(suite_dir: Path, stage: str) -> dict[str, object]:
    return json.loads((suite_dir / T10_STAGE_RECORD[stage]).read_text(encoding="utf-8"))


def _write_stage_record(suite_dir: Path, stage: str, record: dict[str, object]) -> None:
    (suite_dir / T10_STAGE_RECORD[stage]).write_text(
        json.dumps(record, sort_keys=True), encoding="utf-8"
    )


def _core_result_path(suite_dir: Path) -> Path:
    return suite_dir.parent / f"{suite_dir.name}{iq.CORE_SUITE_DIRNAME_SUFFIX}" / "suite-result.json"


def test_real_valid_predecessors_standalone_f_verified_pass(tmp_path):
    """Valid persisted A-E predecessors validate; standalone F is verified-pass
    (never not-implemented) and writes the four derived outputs."""
    runner, suite_dir = _write_valid_suite(tmp_path)
    validation = runner._validate_f_predecessors(suite_dir)
    assert validation["status"] == STATUS_VERIFIED_PASS, validation["reasons"]
    result = runner.run_stage("F")
    assert result["status"] == STATUS_VERIFIED_PASS, result["reasons"]
    assert result["status"] != STATUS_NOT_IMPLEMENTED
    for name in T10_DERIVED:
        assert (suite_dir / name).is_file(), name
    index = json.loads((suite_dir / iq.INDEX_NAME).read_text(encoding="utf-8"))
    assert index["kind"] == "integrated-evidence-index"
    assert index["checksum_algorithm"] == "sha256"
    indexed = {entry["path"] for entry in index["files"]}
    assert iq.INDEX_NAME not in indexed  # the index excludes only itself
    assert iq.SUMMARY_NAME in indexed
    assert iq.AGENT_SHEET_NAME in indexed
    assert iq.USER_SHEET_NAME in indexed


def test_real_repeated_f_regenerates_derived_outputs(tmp_path):
    """Repeated F regenerates the derived summary/index deterministically."""
    runner, suite_dir = _write_valid_suite(tmp_path)
    first = runner.run_stage("F")
    assert first["status"] == STATUS_VERIFIED_PASS, first["reasons"]
    first_index = json.loads((suite_dir / iq.INDEX_NAME).read_text(encoding="utf-8"))
    (suite_dir / iq.SUMMARY_NAME).unlink()
    second = runner.run_stage("F")
    assert second["status"] == STATUS_VERIFIED_PASS, second["reasons"]
    assert (suite_dir / iq.SUMMARY_NAME).is_file()
    second_index = json.loads((suite_dir / iq.INDEX_NAME).read_text(encoding="utf-8"))
    assert second_index["files"] == first_index["files"]
    assert second_index["index_checksum"] == first_index["index_checksum"]


def test_real_semantic_tamper_to_predecessor_fails_closed(tmp_path):
    """A semantic tamper to a persisted predecessor fails F closed."""
    runner, suite_dir = _write_valid_suite(tmp_path)
    record = _read_stage_record(suite_dir, "C")
    name = record["scenario_names"][0]
    record[name]["status"] = STATUS_VERIFIED_FAIL
    _write_stage_record(suite_dir, "C", record)
    result = runner.run_stage("F")
    assert result["status"] == STATUS_EVIDENCE_INVALID
    assert "is not verified-pass" in " ".join(result["reasons"])


@pytest.mark.parametrize("stage,kind,expect", [
    ("A", "missing", "stage-a-result.json is missing"),
    ("A", "malformed", "not finite JSON"),
    ("A", "failed", "stage A record status is verified-fail"),
    ("C", "missing", "stage-c-result.json is missing"),
    ("B", "malformed", "not finite JSON"),
    ("D", "failed", "record status is not verified-pass"),
])
def test_real_bad_predecessor_blocks_before_derived_writes(tmp_path, stage, kind, expect):
    """A missing/malformed/failed predecessor blocks F before any derived write."""
    runner, suite_dir = _write_valid_suite(tmp_path)
    record_path = suite_dir / T10_STAGE_RECORD[stage]
    if kind == "missing":
        record_path.unlink()
    elif kind == "malformed":
        record_path.write_text("{not-json", encoding="utf-8")
    else:
        record = _read_stage_record(suite_dir, stage)
        record["status"] = STATUS_VERIFIED_FAIL
        _write_stage_record(suite_dir, stage, record)
    (suite_dir / iq.INDEX_NAME).unlink(missing_ok=True)
    result = runner.run_stage("F")
    assert result["status"] == STATUS_EVIDENCE_INVALID
    assert expect in " ".join(result["reasons"])
    for name in T10_DERIVED:
        assert not (suite_dir / name).exists(), name


@pytest.mark.parametrize("kind,expect", [
    ("hash", "SHA-256 no longer matches the record"),
    ("status", "status no longer matches the record"),
    ("gate-removed", "gates keys do not equal the configured gates exactly"),
    ("gate-extra", "gates keys do not equal the configured gates exactly"),
])
def test_real_core_suite_mutation_fails_closed(tmp_path, kind, expect):
    """Core suite current-byte/hash/status/gate-key mutation fails F closed."""
    runner, suite_dir = _write_valid_suite(tmp_path)
    core_path = _core_result_path(suite_dir)
    value = json.loads(core_path.read_text(encoding="utf-8"))
    if kind == "hash":
        core_path.write_bytes(core_path.read_bytes() + b"\n")
    elif kind == "status":
        value["status"] = STATUS_VERIFIED_FAIL
        core_path.write_text(json.dumps(value, sort_keys=True), encoding="utf-8")
    elif kind == "gate-removed":
        del value["gates"]["retention"]
        core_path.write_text(json.dumps(value, sort_keys=True), encoding="utf-8")
    else:
        value["gates"]["ghost"] = {"status": STATUS_VERIFIED_PASS}
        core_path.write_text(json.dumps(value, sort_keys=True), encoding="utf-8")
    result = runner.run_stage("F")
    assert result["status"] == STATUS_EVIDENCE_INVALID
    assert expect in " ".join(result["reasons"]), result["reasons"]


def test_real_stage_a_persists_bound_sibling_core_record(tmp_path):
    """A real Stage-A run persists a record binding the external sibling core
    suite: resolved path, lowercase SHA-256 of the current suite-result bytes,
    and exactly the configured six gate keys/statuses — with no core files
    inside the integrated suite."""
    suite_dir = tmp_path / "suite"
    runner = IntegratedRunner(attempt_root=suite_dir)
    gates = runner._core_gates()
    core_root = suite_dir.parent / f"{suite_dir.name}{iq.CORE_SUITE_DIRNAME_SUFFIX}"

    def fake_core_suite():
        core_root.mkdir(parents=True, exist_ok=True)
        core_payload = {
            "status": STATUS_VERIFIED_PASS,
            "gates": {gate: {"status": STATUS_VERIFIED_PASS} for gate in gates},
        }
        core_bytes = json.dumps(core_payload, sort_keys=True).encode("utf-8")
        (core_root / "suite-result.json").write_bytes(core_bytes)
        return {
            "status": STATUS_VERIFIED_PASS,
            "attempt_dir": str(core_root),
            "suite_dir": str(core_root),
            "suite_result_sha256": hashlib.sha256(core_bytes).hexdigest(),
            "gate_results": {gate: {"status": STATUS_VERIFIED_PASS} for gate in gates},
        }

    with patch.object(runner, "_run_core_suite", side_effect=fake_core_suite):
        result = runner.run_stage("A")

    assert result["status"] == STATUS_VERIFIED_PASS, result.get("reasons")
    record = _read_stage_record(suite_dir, "A")
    core = record["core_suite"]
    assert Path(core["suite_dir"]).resolve() == core_root.resolve()
    recorded_sha = core["suite_result_sha256"]
    current_sha = hashlib.sha256(
        (core_root / "suite-result.json").read_bytes()
    ).hexdigest()
    assert recorded_sha == current_sha == recorded_sha.lower()
    assert len(recorded_sha) == 64
    assert sorted(core["gate_results"]) == sorted(gates)
    assert all(
        entry["status"] == STATUS_VERIFIED_PASS
        for entry in core["gate_results"].values()
    )
    # The sibling core root is external: no core files lie under the suite.
    assert core_root.is_relative_to(suite_dir) is False
    assert {path.name for path in suite_dir.iterdir()} == {T10_STAGE_RECORD["A"]}


def _pass_write_once_seam(suite_dir, runner, stage, seam, calls):
    """Return a seam double that makes a real first stage invocation persist a
    verified-pass record (writing the external sibling core for stage A)."""
    gates = runner._core_gates()

    def pass_seam(*args, **kwargs):
        calls.append(seam)
        if stage == "A":
            core_root = suite_dir.parent / f"{suite_dir.name}{iq.CORE_SUITE_DIRNAME_SUFFIX}"
            core_root.mkdir(parents=True, exist_ok=True)
            core_payload = {
                "status": STATUS_VERIFIED_PASS,
                "gates": {gate: {"status": STATUS_VERIFIED_PASS} for gate in gates},
            }
            core_bytes = json.dumps(core_payload, sort_keys=True).encode("utf-8")
            (core_root / "suite-result.json").write_bytes(core_bytes)
            return {
                "status": STATUS_VERIFIED_PASS,
                "attempt_dir": str(core_root),
                "suite_dir": str(core_root),
                "suite_result_sha256": hashlib.sha256(core_bytes).hexdigest(),
                "gate_results": {gate: {"status": STATUS_VERIFIED_PASS} for gate in gates},
            }
        if stage == "B":
            return (Path(args[1]), {"status": "pass", "manifest": {"status": "pass"}})
        names = runner._stage_scenarios(stage)
        return {
            "stage": stage,
            "status": STATUS_VERIFIED_PASS,
            "scenario_names": list(names),
            **{name: {"status": STATUS_VERIFIED_PASS} for name in names},
        }

    return pass_seam


@pytest.mark.parametrize("stage,seam", [
    ("A", "_run_core_suite"),
    ("B", "_invoke_source_lock_manifest"),
    ("C", "_run_scenario_stage"),
    ("D", "_run_scenario_stage"),
    ("E", "_run_scenario_stage"),
])
def test_real_stage_records_write_once_and_refuse_duplicate_invocation(
    tmp_path, stage, seam
):
    """A/B/C/D/E persist write-once: the first invocation writes the record, a
    second invocation returns evidence-invalid without re-invoking the stage's
    implementation/launch seam, and the persisted bytes are unchanged."""
    suite_dir = tmp_path / "suite"
    runner = IntegratedRunner(attempt_root=suite_dir)
    calls: list[str] = []
    pass_seam = _pass_write_once_seam(suite_dir, runner, stage, seam, calls)
    patches = [patch.object(runner, seam, side_effect=pass_seam)]
    if stage == "B":
        patches.append(patch.object(
            runner,
            "_invoke_static_contracts",
            return_value={
                "status": STATUS_VERIFIED_PASS,
                "static_contract": {"status": STATUS_VERIFIED_PASS},
            },
        ))
    with ExitStack() as stack:
        for item in patches:
            stack.enter_context(item)
        first = runner.run_stage(stage)
    assert first["status"] == STATUS_VERIFIED_PASS, first.get("reasons")
    record_path = suite_dir / T10_STAGE_RECORD[stage]
    assert record_path.is_file()
    record_bytes = record_path.read_bytes()

    with patch.object(runner, seam, side_effect=pass_seam):
        second = runner.run_stage(stage)
    assert second["status"] == STATUS_EVIDENCE_INVALID
    assert "already exists" in " ".join(second["reasons"])
    assert record_path.read_bytes() == record_bytes
    assert calls == [seam]


def test_real_run_all_retains_a_and_b_blocks_live_stages_when_a_fails():
    """A Gate-A failure retains the A and B results, blocks C/D/E/F before any
    live stage runs, and preserves the fail-dominant overall status."""
    runner = IntegratedRunner(attempt_root=Path(_tmp()))
    invoked: list[str] = []

    def fake_stage_a():
        invoked.append("_run_stage_a")
        return {"stage": "A", "status": STATUS_EVIDENCE_INVALID, "reasons": ["a boom"]}

    def fake_stage_b():
        invoked.append("_run_stage_b")
        return {"stage": "B", "status": STATUS_VERIFIED_PASS}

    def blocked(name):
        def fake():
            invoked.append(name)
            return None

        return fake

    with patch.object(runner, "_run_stage_a", side_effect=fake_stage_a), patch.object(
        runner, "_run_stage_b", side_effect=fake_stage_b
    ), patch.object(runner, "_run_stage_c", side_effect=blocked("_run_stage_c")), patch.object(
        runner, "_run_stage_d", side_effect=blocked("_run_stage_d")
    ), patch.object(runner, "_run_stage_e", side_effect=blocked("_run_stage_e")), patch.object(
        runner, "_run_stage_f", side_effect=blocked("_run_stage_f")
    ):
        result = runner.run_stage("all")

    assert invoked == ["_run_stage_a", "_run_stage_b"]
    assert result["A"]["status"] == STATUS_EVIDENCE_INVALID
    assert result["B"]["status"] == STATUS_VERIFIED_PASS
    assert all(result[s]["status"] == STATUS_BLOCKED for s in ("C", "D", "E", "F"))
    assert _overall_status(result) == STATUS_EVIDENCE_INVALID


@pytest.mark.parametrize("kind", ["predecessor-invalid", "producer-exception"])
def test_real_f_failure_modes_keep_stable_derived_paths(tmp_path, kind):
    """Both a predecessor-invalid F and a producer-exception F fail closed with
    stable derived-output path strings; a predecessor-invalid F performs no
    derived generation writes."""
    runner, suite_dir = _write_valid_suite(tmp_path)
    for name in T10_DERIVED:
        (suite_dir / name).unlink(missing_ok=True)
    if kind == "predecessor-invalid":
        (suite_dir / T10_STAGE_RECORD["A"]).unlink()
        result = runner.run_stage("F")
    else:
        with patch.object(
            runner, "_regenerate_contact_sheets", side_effect=RuntimeError("sheet boom")
        ):
            result = runner.run_stage("F")

    expected_paths = {
        "index": suite_dir / iq.INDEX_NAME,
        "summary": suite_dir / iq.SUMMARY_NAME,
        "agent_sheet": suite_dir / iq.AGENT_SHEET_NAME,
        "user_sheet": suite_dir / iq.USER_SHEET_NAME,
    }
    assert result["status"] == STATUS_EVIDENCE_INVALID
    for key, path in expected_paths.items():
        assert result[key] == str(path)
    evidence = result["evidence"]
    assert evidence["status"] == STATUS_EVIDENCE_INVALID
    if kind == "predecessor-invalid":
        for name in T10_DERIVED:
            assert not (suite_dir / name).exists(), name
    else:
        assert evidence["producer_exception"] is True
        assert evidence["reasons"] == result["reasons"]


@pytest.mark.parametrize("kind,expect", [
    ("scenario-mismatch", "scenario set does not match"),
    ("scenario-duplicate", "scenario_names are not unique"),
    ("extra-key", "record keys do not equal"),
    ("attempt-escape", "escapes the integrated suite"),
    ("attempt-missing", "attempt directory does not exist"),
    ("attempt-shared", "is shared across scenarios"),
])
def test_real_cde_record_mutation_fails_closed(tmp_path, kind, expect):
    """C/D/E scenario_names mismatch/duplicate, extra key, and attempt
    escape/missing/shared directory failures all fail F closed."""
    runner, suite_dir = _write_valid_suite(tmp_path)
    record = _read_stage_record(suite_dir, "C")
    names = list(record["scenario_names"])
    if kind == "scenario-mismatch":
        record["scenario_names"] = [names[0], names[1], "extra-scenario"]
    elif kind == "scenario-duplicate":
        record["scenario_names"] = names + [names[0]]
    elif kind == "extra-key":
        record["ghost"] = 1
    elif kind == "attempt-escape":
        record[names[0]]["attempt_dir"] = str(tmp_path / "outside")
    elif kind == "attempt-missing":
        record[names[0]]["attempt_dir"] = str(suite_dir / "C" / "no-such-attempt")
    else:
        record[names[1]]["attempt_dir"] = record[names[0]]["attempt_dir"]
    _write_stage_record(suite_dir, "C", record)
    result = runner.run_stage("F")
    assert result["status"] == STATUS_EVIDENCE_INVALID
    assert expect in " ".join(result["reasons"]), result["reasons"]


def test_real_run_all_registers_cde_in_stage_results_before_f():
    """_run_all stores C/D/E in _stage_results before invoking F."""
    runner = IntegratedRunner(attempt_root=Path(_tmp()))
    snapshot: dict[str, object] = {}

    def fake_stage_f():
        snapshot.update(dict(runner._stage_results))
        return {"stage": "F", "status": STATUS_VERIFIED_PASS}

    pass_record = {"status": STATUS_VERIFIED_PASS, "scenario_names": []}
    with patch.object(runner, "_run_stage_a", return_value={"stage": "A", "status": STATUS_VERIFIED_PASS}), patch.object(
        runner, "_run_stage_b", return_value={"stage": "B", "status": STATUS_VERIFIED_PASS}
    ), patch.object(runner, "_run_stage_c", return_value=dict(pass_record, stage="C")), patch.object(
        runner, "_run_stage_d", return_value=dict(pass_record, stage="D")
    ), patch.object(runner, "_run_stage_e", return_value=dict(pass_record, stage="E")), patch.object(
        runner, "_run_stage_f", side_effect=fake_stage_f
    ):
        result = runner.run_stage("all")
    assert set(snapshot) == {"A", "B", "C", "D", "E"}
    assert result["F"]["status"] == STATUS_VERIFIED_PASS
    assert runner._stage_results["F"]["status"] == STATUS_VERIFIED_PASS


def test_real_standalone_f_launches_no_children(tmp_path):
    """Standalone F spawns no child processes and never invokes the command runner."""
    runner, suite_dir = _write_valid_suite(tmp_path)
    popen = FakePopen()
    runner._popen = popen

    def _no_subprocess(*args, **kwargs):
        raise AssertionError("standalone F must not spawn a child process")

    runner._command_runner = _no_subprocess
    result = runner.run_stage("F")
    assert result["status"] == STATUS_VERIFIED_PASS, result["reasons"]
    assert popen.calls == []


@pytest.mark.parametrize("status,expected", [
    (STATUS_VERIFIED_PASS, 0),
    (STATUS_VERIFIED_FAIL, 1),
    (STATUS_EVIDENCE_INVALID, 2),
])
def test_real_cli_stage_f_statuses_map_to_exit_codes(status, expected):
    with patch.object(IntegratedRunner, "run_stage", return_value={"stage": "F", "status": status}):
        assert main(["--stage", "F"]) == expected


@pytest.mark.parametrize("stage", ["A", "C", "D", "E", "F", "all"])
def test_real_offline_rejected_for_non_b_stages_before_run_stage(stage):
    """--offline is rejected for A/C/D/E/F/all before run_stage is invoked."""
    calls: list[str] = []

    def fake_run_stage(self, requested):
        calls.append(str(requested))
        raise AssertionError("run_stage must not be invoked for rejected --offline")

    with patch.object(IntegratedRunner, "run_stage", fake_run_stage):
        assert main(["--stage", stage, "--offline"]) == 2
    assert calls == []


def test_real_offline_accepted_only_for_stage_b():
    with patch.object(IntegratedRunner, "run_stage", return_value={"stage": "B", "status": STATUS_VERIFIED_PASS}):
        assert main(["--stage", "B", "--offline"]) == 0


# --------------------------------------------------------------------------- #
# Task 10 — load-bearing integrated rosbag lifecycle
# --------------------------------------------------------------------------- #

import sqlite3  # noqa: E402
import yaml  # noqa: E402

_T10_RECORD_TOPIC_TYPES = {
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

# Humble rosbag2 serializes the full nine-field rmw_qos_profile_t; the two
# ROSBAG_QOS_OVERRIDE topics carry keep_last/depth1/reliable/transient_local and
# the remaining approved publishers are RELIABLE with VOLATILE durability.
_T10_OVERRIDE_QOS = (
    "- history: 1\n  depth: 1\n  reliability: 1\n  durability: 1\n"
    "  deadline: 0\n  lifespan: 0\n  liveliness: 1\n"
    "  liveliness_lease_duration: 0\n  avoid_ros_namespace_conventions: false\n"
)
_T10_VOLATILE_QOS = (
    "- history: 1\n  depth: 10\n  reliability: 1\n  durability: 3\n"
    "  deadline: 0\n  lifespan: 0\n  liveliness: 1\n"
    "  liveliness_lease_duration: 0\n  avoid_ros_namespace_conventions: false\n"
)
_T10_TOPIC_QOS = {
    "/sim/hardware/safety_stop": _T10_OVERRIDE_QOS,
    "/sim/status/contract": _T10_OVERRIDE_QOS,
}


class StoppableFakeProcess:
    """A process double that is alive until stopped, then reports returncode 0.

    ``QualificationRunner._stop`` classifies a live child as
    ``planned-termination`` (SIGINT-then-SIGTERM) with ``forced=False``, which
    is exactly the load-bearing recorder termination contract.  ``poll()``
    returns None until ``wait()`` is called, so the double never trips the
    unexpected-exit or forced branches.
    """

    def __init__(self, pid: int = 424242):
        self.pid = pid
        self._stopped = False
        self.returncode = 0

    def poll(self):
        return None if not self._stopped else self.returncode

    def wait(self, timeout=None):
        self._stopped = True
        return self.returncode

    def send_signal(self, sig):
        return None

    def terminate(self):
        self._stopped = True

    def kill(self):
        self._stopped = True


class LifecycleFakePopen(FakePopen):
    """A FakePopen whose rosbag child is a StoppableFakeProcess so the recorder
    stop records a planned, non-forced termination."""

    def __call__(self, command, **kwargs):
        name = self._infer_name(list(command))
        env = dict(kwargs.get("env", {}))
        self.calls.append({"name": name, "command": list(command), "env": env})
        self._on_start(name, list(command), env)
        if name == "rosbag":
            return StoppableFakeProcess(pid=424242 + len(self.calls))
        return FakeProcess(pid=424242 + len(self.calls))


def _rosbag_document(bag: Path) -> dict[str, object]:
    return yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8"))


def _write_rosbag_document(bag: Path, document: object) -> None:
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump(document, sort_keys=False, default_flow_style=False),
        encoding="utf-8",
    )


def _write_valid_integrated_rosbag(attempt_dir: Path) -> Path:
    """Write a real, semantically valid rosbag (metadata.yaml + openable db3)."""
    bag = attempt_dir / "rosbag"
    bag.mkdir(parents=True, exist_ok=True)
    metadata_topics = [
        {
            "topic_metadata": {
                "name": topic,
                "type": topic_type,
                "offered_qos_profiles": _T10_TOPIC_QOS.get(topic, _T10_VOLATILE_QOS),
            },
            "message_count": 100,
        }
        for topic, topic_type in _T10_RECORD_TOPIC_TYPES.items()
    ]
    _write_rosbag_document(bag, {
        "rosbag2_bagfile_information": {
            "storage_identifier": "sqlite3",
            "duration": {"nanoseconds": 10_000_000_000},
            "message_count": 1100,
            "relative_file_paths": ["rosbag_0.db3"],
            "topics_with_message_count": metadata_topics,
        }
    })
    connection = sqlite3.connect(bag / "rosbag_0.db3")
    connection.execute("CREATE TABLE IF NOT EXISTS t (id INTEGER PRIMARY KEY, x TEXT)")
    connection.commit()
    connection.close()
    return bag


def _mutate_rosbag(bag: Path, kind: str) -> None:
    """Mutate a valid bag to exercise one load-bearing semantic failure."""
    if kind == "corrupt-metadata":
        (bag / "metadata.yaml").write_text(
            "rosbag2_bagfile_information: [unclosed\n", encoding="utf-8"
        )
        return
    document = _rosbag_document(bag)
    root = document["rosbag2_bagfile_information"]
    topics = root["topics_with_message_count"]
    if kind == "missing-topic":
        root["topics_with_message_count"] = topics[1:]
    elif kind == "malformed-qos":
        topics[0]["topic_metadata"]["offered_qos_profiles"] = "- history: invalid\n"
    elif kind == "wrong-type":
        topics[0]["topic_metadata"]["type"] = "std_msgs/msg/Float32"
    elif kind == "bad-count":
        topics[0]["message_count"] = 0
    elif kind == "missing-storage":
        for db3 in bag.glob("*.db3"):
            db3.unlink()
    elif kind == "unopenable-storage":
        for db3 in bag.glob("*.db3"):
            db3.write_bytes(b"not-a-real-sqlite-database")
    _write_rosbag_document(bag, document)


def test_t10_recorder_starts_after_physics_ready_and_bundle_before_executor():
    """Recorder starts only after PHYSICS_READY + scenario-bundle write and
    before the executor, with the exact real QoS override path and exactly the
    approved record-topic set (no extras/omissions)."""
    order: list[str] = []
    popen = FakePopen(on_start=lambda name, command, env: order.append(f"start:{name}"))
    runner = _evidence_runner(popen=popen)
    allocation = runner._allocate_one(SCENARIO_C, "C")
    observed: dict[str, object] = {}

    def fake_ready(attempt_dir, name, *, manifest=None):
        order.append("physics-ready")
        return True, {"ready": True}, None

    def fake_start_rosbag(scenario_runner, manifest):
        observed["scenario_runner"] = scenario_runner
        observed["manifest"] = manifest
        observed["bundle_present_at_start"] = (
            scenario_runner._attempt_dir / "scenario-bundle.json"
        ).is_file()
        iq.qualification_start_process(
            scenario_runner, "rosbag", ["ros2", "bag", "record"], manifest
        )
        return True

    def fake_terminal(attempt_dir, name, *, runner=None, attempt_id=None):
        return {"ok": True, "source": "executor-driver"}

    def fake_verify(attempt_dir, name, stage):
        return {"status": "verified-pass", "scenario": name}

    with patch.object(runner, "_wait_for_physics_ready", side_effect=fake_ready), patch.object(
        runner, "_wait_for_scenario_terminal", side_effect=fake_terminal
    ), patch.object(runner, "_verify_attempt", side_effect=fake_verify), patch.object(
        iq.QualificationRunner, "_start_rosbag", new=fake_start_rosbag
    ):
        result = runner._execute_scenario(allocation, SCENARIO_C, "C")

    assert order.index("physics-ready") < order.index("start:rosbag") < order.index("start:executor")
    assert observed["bundle_present_at_start"] is True
    manifest = observed["manifest"]
    assert manifest.attempt_dir == allocation.attempt_dir
    recorded = json.loads((allocation.attempt_dir / "manifest.json").read_text(encoding="utf-8"))
    assert recorded["attempt_id"] == manifest.attempt_id
    command = observed["scenario_runner"]._default_rosbag_command(manifest)
    assert command[:3] == ["ros2", "bag", "record"]
    qos_index = command.index("--qos-profile-overrides-path")
    assert Path(command[qos_index + 1]).resolve() == (
        allocation.attempt_dir / "rosbag-qos-overrides.yaml"
    ).resolve()
    assert command[qos_index + 2:] == list(iq.APPROVED_RECORD_TOPICS)


def test_t10_rosbag_start_failure_blocks_executor_and_finalizes_isaac_humble():
    popen = FakePopen()
    runner = _evidence_runner(popen=popen)
    allocation = runner._allocate_one(SCENARIO_C, "C")

    def fake_ready(attempt_dir, name, *, manifest=None):
        return True, {}, None

    def fake_start_rosbag(scenario_runner, manifest):
        return False

    with patch.object(runner, "_wait_for_physics_ready", side_effect=fake_ready), patch.object(
        iq.QualificationRunner, "_start_rosbag", new=fake_start_rosbag
    ):
        result = runner._execute_scenario(allocation, SCENARIO_C, "C")

    assert result["status"] == STATUS_EVIDENCE_INVALID
    assert any(
        "rosbag recorder failed to start" in reason for reason in result.get("reasons", [])
    )
    assert [call["name"] for call in popen.calls] == ["isaac", "humble"]
    finalize = result.get("finalize", {})
    assert finalize.get("exit_codes", {}).get("isaac") == 0
    assert finalize.get("exit_codes", {}).get("humble") == 0
    assert "executor" not in finalize.get("exit_codes", {})


def test_t10_recorder_stop_precedes_bag_evidence_and_valid_contract_is_load_bearing():
    popen = LifecycleFakePopen()
    runner = _evidence_runner(popen=popen)
    allocation = runner._allocate_one(SCENARIO_C, "C")

    def fake_ready(attempt_dir, name, *, manifest=None):
        return True, {}, None

    def fake_start_rosbag(scenario_runner, manifest):
        iq.qualification_start_process(
            scenario_runner, "rosbag", ["ros2", "bag", "record"], manifest
        )
        return True

    def fake_terminal(attempt_dir, name, *, runner=None, attempt_id=None):
        return {"ok": True, "source": "executor-driver"}

    def fake_verify(attempt_dir, name, stage):
        return {"status": "verified-pass", "scenario": name}

    events: list[str] = []
    real_stop = iq.qualification_stop_process
    real_bag = runner._integrated_rosbag_evidence

    def recording_stop(rnr, name):
        if name == "rosbag":
            events.append("stop:rosbag")
        return real_stop(rnr, name)

    def recording_bag(attempt_dir):
        events.append("bag-evidence")
        return real_bag(attempt_dir)

    with patch.object(runner, "_wait_for_physics_ready", side_effect=fake_ready), patch.object(
        runner, "_wait_for_scenario_terminal", side_effect=fake_terminal
    ), patch.object(runner, "_verify_attempt", side_effect=fake_verify), patch.object(
        iq.QualificationRunner, "_start_rosbag", new=fake_start_rosbag
    ), patch.object(iq, "qualification_stop_process", side_effect=recording_stop), patch.object(
        runner, "_integrated_rosbag_evidence", side_effect=recording_bag
    ), patch.object(
        # Narrow the non-rosbag finalize seams so the verdict depends only on the
        # rosbag lifecycle, not live GPU/resource state.
        iq, "qualification_wait_for_evaluator_drain", return_value=True
    ), patch.object(
        iq, "qualification_attempt_processes", return_value=[]
    ), patch.object(
        iq, "qualification_write_resource_evidence", return_value=True
    ), patch.object(
        iq, "qualification_settle_evidence_files", return_value=None
    ):
        result = runner._execute_scenario(allocation, SCENARIO_C, "C")

    assert events.index("stop:rosbag") < events.index("bag-evidence")
    assert result["status"] == STATUS_VERIFIED_PASS, result.get("reasons")
    finalize = result.get("finalize", {})
    assert finalize.get("exit_codes", {}).get("rosbag") == 0
    termination = finalize.get("rosbag_termination", {})
    assert termination.get("classification") == "planned-termination"
    assert termination.get("returncode") == 0
    assert termination.get("forced") is False


@pytest.mark.parametrize("termination,expect", [
    ({"classification": "unexpected-exit", "returncode": 0, "forced": False},
     "termination classification is unexpected-exit, expected planned-termination"),
    ({"classification": "planned-termination", "returncode": 1, "forced": False},
     "termination returncode is 1, expected 0"),
    ({"classification": "planned-termination", "returncode": 0, "forced": True},
     "termination forced is True, expected false"),
    (None, "rosbag recorder has no termination record after stop"),
])
def test_t10_bad_recorder_termination_makes_final_rosbag_ok_false(termination, expect):
    popen = FakePopen()
    runner = _evidence_runner(popen=popen)
    allocation = runner._allocate_one(SCENARIO_C, "C")
    scenario_runner, manifest = runner._launch_scenario(allocation, SCENARIO_C, "C")
    iq.qualification_start_process(scenario_runner, "rosbag", ["ros2", "bag", "record"], manifest)
    baseline = runner._gpu_baselines[allocation.attempt_dir]
    real_stop = iq.qualification_stop_process

    def fake_stop(rnr, name):
        if name == "rosbag":
            if termination is None:
                rnr._termination.pop("rosbag", None)
            else:
                rnr._termination["rosbag"] = dict(termination)
            return 0
        return real_stop(rnr, name)

    with patch.object(iq, "qualification_stop_process", side_effect=fake_stop):
        finalize = runner._finalize_attempt(scenario_runner, manifest, baseline)
    assert finalize["rosbag_ok"] is False
    assert any(expect in reason for reason in finalize["failures"]), finalize["failures"]


@pytest.mark.parametrize("mutate,expect", [
    ("corrupt-metadata", "not structured rosbag2 metadata"),
    ("missing-topic", "is missing approved topic"),
    ("malformed-qos", "has malformed RMW QoS fields"),
    ("wrong-type", "does not match"),
    ("bad-count", "has an invalid message count"),
    ("missing-storage", "output database is missing or not openable"),
    ("unopenable-storage", "output database is missing or not openable"),
])
def test_t10_present_bag_semantic_failures_are_load_bearing(mutate, expect):
    runner = _evidence_runner()
    attempt_dir = Path(_tmp()) / "attempt"
    attempt_dir.mkdir(parents=True)
    bag = _write_valid_integrated_rosbag(attempt_dir)
    _mutate_rosbag(bag, mutate)
    ok, _evidence, failures = runner._integrated_rosbag_evidence(attempt_dir)
    assert ok is False
    assert any(expect in reason for reason in failures), failures


def test_t10_valid_present_bag_passes_integrated_evidence():
    runner = _evidence_runner()
    attempt_dir = Path(_tmp()) / "attempt"
    attempt_dir.mkdir(parents=True)
    _write_valid_integrated_rosbag(attempt_dir)
    ok, evidence, failures = runner._integrated_rosbag_evidence(attempt_dir)
    assert ok is True, failures
    assert evidence["status"] == "valid"


def test_t10_integrated_finalization_uses_integrated_bag_evidence_only():
    popen = FakePopen()
    runner = _evidence_runner(popen=popen)
    allocation = runner._allocate_one(SCENARIO_C, "C")
    scenario_runner, manifest = runner._launch_scenario(allocation, SCENARIO_C, "C")
    baseline = runner._gpu_baselines[allocation.attempt_dir]
    calls: list[str] = []

    def fake_integrated_evidence(attempt_dir):
        calls.append("integrated")
        return True, {"status": "not-recorded", "load_bearing": False}, []

    def fake_rosbag_final_evidence(*args, **kwargs):
        calls.append("rosbag_final_evidence")
        return False, {}, ["must never be used"]

    with patch.object(
        runner, "_integrated_rosbag_evidence", side_effect=fake_integrated_evidence
    ), patch.object(
        iq, "qualification_rosbag_final_evidence", side_effect=fake_rosbag_final_evidence
    ), patch.object(
        iq.QualificationRunner, "_rosbag_final_evidence", side_effect=fake_rosbag_final_evidence
    ):
        finalize = runner._finalize_attempt(scenario_runner, manifest, baseline)
    assert calls == ["integrated"]
    assert finalize["rosbag_ok"] is True
