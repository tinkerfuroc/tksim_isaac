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
