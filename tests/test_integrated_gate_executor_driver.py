"""Task 8 fix round 2: ROS-free tests for the Humble executor driver.

Python 3.12, ROS-free: importing ``validation.integrated_gate_executor_driver``
never imports ``rclpy`` or any generated message type.  This suite covers the
driver's pure dispatch/serialization/bundle/terminal/lift layer and exercises
:func:`~integrated_gate_executor_driver.run_driver` with ROS-free executor and
parameter-client doubles.

Provider liveness (the real readiness snapshot, TF TCP pose, PointCloud2
environment cloud, native gripper goal count, long-motion UUIDs, and the
``/pick_and_place.post_grasp_lift_m`` live set/read-back) is a live obligation
and is deliberately NOT claimed by any double here.
"""
from __future__ import annotations

import json
import sys
import time
import types
from pathlib import Path
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))
sys.path.insert(0, str(ROOT / "simulation"))

import integrated_gate_executor_driver as d  # noqa: E402
from integrated_gate_executor import (  # noqa: E402
    REQUIRED_ACTIONS,
    REQUIRED_SERVICES,
    STAGE_C_SCENARIOS,
    STAGE_D_KIND,
    STAGE_D_SCENARIOS,
    STAGE_E_SCENARIOS,
    _REQUIRED_ENDPOINT_SOURCES,
)


def _tmp() -> str:
    import tempfile

    return tempfile.mkdtemp(prefix="task8-driver-")


def _config() -> dict[str, object]:
    return {"execution_profile": "sim_ompl", "thresholds": {"tf_fresh_s": 0.25}}


def _bundle(scenario_id: str = "qualification-moveit-plan-joint", attempt_id: str = "attempt-1"):
    return {
        "schema_version": 1,
        "scenario_id": scenario_id,
        "attempt_id": attempt_id,
        "attempt_dir": "",
        "scenario": {"id": scenario_id, "seed": 7, "declaration": {}},
        "planning_scene": {"revision": "r1", "owned_ids": [], "target_source_id": "", "target_handoff": ""},
        "planning_scene_declaration": {
            "revision": "r1",
            "owned_ids": [],
            "target_source_id": "",
            "target_handoff": "",
            "revision_digest": "a" * 64,
            "fixture_descriptor_sha256": "b" * 64,
        },
        "integrated": {"stage": "C", "execution_profile": "sim_ompl"},
        "report_identities": {
            "scenario_id": scenario_id,
            "seed": 7,
            "scenario_declaration_sha256": "c" * 64,
            "planning_scene_sha256": "d" * 64,
            "integrated_sha256": "e" * 64,
            "model_fingerprint": "f" * 64,
            "provider_manifest_sha256": "g" * 64,
        },
    }


class FakeExecutor:
    """ROS-free executor double recording run-method dispatch."""

    def __init__(self, *, run_status="verified-pass", run_error=None, write_artifacts=False):
        self.calls: list[tuple[str, str, dict]] = []
        self.shutdown_calls = 0
        self._run_status = run_status
        self._run_error = run_error
        self._write_artifacts = write_artifacts
        self.attempt_dir: Path | None = None

    def _readiness(self):
        return {"ready": True, "reasons": []}

    def shutdown(self):
        self.shutdown_calls += 1

    def _record(self, method: str, scenario_id: str, kwargs: dict):
        self.calls.append((method, scenario_id, dict(kwargs)))
        if self._write_artifacts and self.attempt_dir is not None:
            for name in d.EXECUTOR_ARTIFACT_FILENAMES:
                (self.attempt_dir / name).write_text(
                    json.dumps({"schema_version": 1, "name": name}, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
        if self._run_error is not None:
            raise self._run_error
        return {"status": self._run_status, "scenario_id": scenario_id}

    def run_gate_c_plan_only(self, scenario_id, **kwargs):
        return self._record("run_gate_c_plan_only", scenario_id, kwargs)

    def run_execute_sequence(self, scenario_id, **kwargs):
        return self._record("run_execute_sequence", scenario_id, kwargs)

    def run_cartesian_retreat(self, scenario_id, **kwargs):
        return self._record("run_cartesian_retreat", scenario_id, kwargs)

    def run_gripper_sequence(self, scenario_id, **kwargs):
        return self._record("run_gripper_sequence", scenario_id, kwargs)

    def run_cancel_sequence(self, scenario_id, **kwargs):
        return self._record("run_cancel_sequence", scenario_id, kwargs)

    def run_safety_sequence(self, scenario_id, **kwargs):
        return self._record("run_safety_sequence", scenario_id, kwargs)

    def run_pick_place_sequence(self, scenario_id, **kwargs):
        return self._record("run_pick_place_sequence", scenario_id, kwargs)


def _fake_executor_factory(executor: FakeExecutor):
    def factory(*, bundle, attempt_dir, config, domain_id, seed):
        executor.attempt_dir = Path(attempt_dir)
        return executor

    return factory


# --------------------------------------------------------------------------- #
# F2.8 — Python 3.12 import stays ROS-lazy
# --------------------------------------------------------------------------- #

def test_driver_import_is_ros_lazy():
    assert "rclpy" not in sys.modules
    assert "moveit_msgs" not in sys.modules
    assert "sensor_msgs" not in sys.modules


# --------------------------------------------------------------------------- #
# F2.2 — exact 17-scenario dispatch mapping
# --------------------------------------------------------------------------- #

def test_dispatch_table_is_exactly_the_16_canonical_scenarios():
    table = d.canonical_dispatch()
    assert len(table) == 16
    assert len(set(table)) == 16
    expected = set(STAGE_C_SCENARIOS) | set(STAGE_D_SCENARIOS) | set(STAGE_E_SCENARIOS)
    assert set(table) == expected
    assert len(expected) == 16
    # Every scenario id maps to exactly one executor run method.
    for name in STAGE_C_SCENARIOS:
        assert table[name] == "run_gate_c_plan_only"
    for name in STAGE_D_SCENARIOS:
        assert table[name] == d.D_METHOD_BY_KIND[STAGE_D_KIND[name]]
    for name in STAGE_E_SCENARIOS:
        assert table[name] == "run_pick_place_sequence"
    assert set(table.values()) == {
        "run_gate_c_plan_only",
        "run_execute_sequence",
        "run_cartesian_retreat",
        "run_gripper_sequence",
        "run_cancel_sequence",
        "run_safety_sequence",
        "run_pick_place_sequence",
    }


def test_unknown_scenario_fails_closed_before_ros_traffic():
    with pytest.raises(d.DriverError, match="unknown scenario id"):
        d.run_method_for("not-a-canonical-scenario")
    executor = FakeExecutor()
    bundle = _bundle("not-a-canonical-scenario")
    attempt_dir = Path(_tmp())
    bundle["attempt_dir"] = str(attempt_dir)
    with pytest.raises(d.DriverError, match="unknown scenario id"):
        d.run_driver(
            bundle=bundle,
            attempt_dir=attempt_dir,
            config=_config(),
            domain_id=100,
            seed=7,
            executor_factory=_fake_executor_factory(executor),
            runtime_provider_factory=lambda **kwargs: {},
        )
    assert executor.calls == []


def test_e_transport_kind_detection_is_exact():
    assert d.is_e_scenario("qualification-pick-place-positive")
    assert d.is_e_scenario("qualification-pick-place-occupied-place")
    assert not d.is_e_scenario("qualification-moveit-plan-joint")
    assert d.is_e_transport_scenario("qualification-pick-place-positive")
    assert d.is_e_transport_scenario("qualification-pick-place-occupied-place")
    assert d.is_e_transport_scenario("qualification-pick-place-cancel-transport")
    assert d.is_e_transport_scenario("qualification-pick-place-safety-transport")
    assert not d.is_e_transport_scenario("qualification-pick-place-cancel-approach")
    assert not d.is_e_transport_scenario("qualification-pick-place-unreachable-grasp")
    assert not d.is_e_transport_scenario("qualification-pick-place-malformed-back")
    assert not d.is_e_transport_scenario("qualification-moveit-execute-joint")


# --------------------------------------------------------------------------- #
# F2.1/F2.3 — bundle identity binding
# --------------------------------------------------------------------------- #

def test_bundle_attempt_dir_mismatch_fails_closed():
    bundle = _bundle()
    attempt_dir = Path(_tmp())
    bundle["attempt_dir"] = str(Path(_tmp()) / "other")
    executor = FakeExecutor()
    with pytest.raises(d.DriverError, match="attempt_dir"):
        d.run_driver(
            bundle=bundle,
            attempt_dir=attempt_dir,
            config=_config(),
            domain_id=100,
            seed=7,
            executor_factory=_fake_executor_factory(executor),
            runtime_provider_factory=lambda **kwargs: {},
        )
    assert executor.calls == []


def test_bundle_seed_mismatch_fails_closed():
    bundle = _bundle()
    attempt_dir = Path(_tmp())
    bundle["attempt_dir"] = str(attempt_dir)
    executor = FakeExecutor()
    with pytest.raises(d.DriverError, match="seed"):
        d.run_driver(
            bundle=bundle,
            attempt_dir=attempt_dir,
            config=_config(),
            domain_id=100,
            seed=999,
            executor_factory=_fake_executor_factory(executor),
            runtime_provider_factory=lambda **kwargs: {},
        )
    assert executor.calls == []


def test_domain_out_of_bounds_fails_closed():
    bundle = _bundle()
    attempt_dir = Path(_tmp())
    bundle["attempt_dir"] = str(attempt_dir)
    executor = FakeExecutor()
    with pytest.raises(d.DriverError, match="ROS_DOMAIN_ID"):
        d.run_driver(
            bundle=bundle,
            attempt_dir=attempt_dir,
            config=_config(),
            domain_id=9999,
            seed=7,
            executor_factory=_fake_executor_factory(executor),
            runtime_provider_factory=lambda **kwargs: {},
        )
    assert executor.calls == []


def test_build_executor_scenario_carries_committed_identities():
    bundle = _bundle()
    scenario = d.build_executor_scenario(bundle)
    assert scenario["id"] == "qualification-moveit-plan-joint"
    assert scenario["seed"] == 7
    assert scenario["scenario_mapping"]["declaration"] == {}
    assert scenario["integrated"]["execution_profile"] == "sim_ompl"
    assert scenario["identities"]["model_fingerprint"] == "f" * 64
    assert scenario["model_fingerprint"] == "f" * 64
    assert scenario["provider_manifest_sha256"] == "g" * 64


# --------------------------------------------------------------------------- #
# F2.2/F2.8 — terminal marker ordering, dispatch, fail-closed
# --------------------------------------------------------------------------- #

def test_run_driver_dispatches_and_writes_terminal_after_artifacts():
    executor = FakeExecutor(write_artifacts=True)
    attempt_dir = Path(_tmp())
    bundle = _bundle("qualification-moveit-plan-joint")
    bundle["attempt_dir"] = str(attempt_dir)
    result = d.run_driver(
        bundle=bundle,
        attempt_dir=attempt_dir,
        config=_config(),
        domain_id=100,
        seed=7,
        executor_factory=_fake_executor_factory(executor),
        runtime_provider_factory=lambda **kwargs: {},
    )
    assert executor.calls == [
        ("run_gate_c_plan_only", "qualification-moveit-plan-joint", {})
    ]
    assert executor.shutdown_calls == 1
    assert result["status"] == "verified-pass"
    marker = attempt_dir / "execution-terminal.json"
    assert marker.is_file()
    value = json.loads(marker.read_text(encoding="utf-8"))
    assert value["schema_version"] == 1
    assert value["scenario_id"] == "qualification-moveit-plan-joint"
    assert value["attempt_id"] == "attempt-1"
    assert value["marker"] == "executor-driver"
    assert value["status"] == "verified-pass"
    # All verifier-required executor artifacts existed before the terminal marker.
    for name in d.EXECUTOR_ARTIFACT_FILENAMES:
        assert (attempt_dir / name).is_file()
        assert (attempt_dir / name).stat().st_mtime_ns <= marker.stat().st_mtime_ns


def test_run_driver_writes_gate_window_before_executor_run():
    """RED: run_driver must write gate-window.json before the executor runs.

    The integrated verifier's ``select_integrated_gate_window`` requires
    ``gate-window.json`` whenever the attempt carries a manifest (always true in
    the integrated flow), but the driver never writes it.  After readiness, the
    driver must record the evidence boundary (raw/evaluator start indices) before
    invoking the executor method, and ``gate-window.json`` must be part of
    ``EXECUTOR_ARTIFACT_FILENAMES`` so the terminal marker is only written after
    it is final.
    """
    attempt_dir = Path(_tmp())
    records = "".join(
        json.dumps(
            {"frame_index": i, "scenario": "qualification-moveit-plan-joint", "seed": 7}
        ) + "\n"
        for i in range(3)
    )
    (attempt_dir / "physics_truth.jsonl").write_text(records, encoding="utf-8")
    (attempt_dir / "evaluator.jsonl").write_text(records, encoding="utf-8")

    class GateWindowRecordingExecutor(FakeExecutor):
        """Records whether the driver wrote gate-window.json before the run
        method and writes the executor artifact set WITHOUT clobbering the
        driver-owned gate window."""

        def __init__(self, *, write_artifacts=False):
            super().__init__(write_artifacts=write_artifacts)
            self.gate_window_at_run: bool | None = None

        def run_gate_c_plan_only(self, scenario_id, **kwargs):
            self.gate_window_at_run = (
                self.attempt_dir / "gate-window.json"
            ).is_file()
            if self._write_artifacts and self.attempt_dir is not None:
                for name in d.EXECUTOR_ARTIFACT_FILENAMES:
                    if name == "gate-window.json":
                        # The driver owns the gate window; the executor stub
                        # must not clobber it with a generic artifact stub.
                        continue
                    (self.attempt_dir / name).write_text(
                        json.dumps({"schema_version": 1, "name": name}, sort_keys=True) + "\n",
                        encoding="utf-8",
                    )
            self.calls.append(("run_gate_c_plan_only", scenario_id, dict(kwargs)))
            if self._run_error is not None:
                raise self._run_error
            return {"status": self._run_status, "scenario_id": scenario_id}

    executor = GateWindowRecordingExecutor(write_artifacts=True)
    bundle = _bundle("qualification-moveit-plan-joint")
    bundle["attempt_dir"] = str(attempt_dir)
    d.run_driver(
        bundle=bundle,
        attempt_dir=attempt_dir,
        config=_config(),
        domain_id=100,
        seed=7,
        executor_factory=_fake_executor_factory(executor),
        runtime_provider_factory=lambda **kwargs: {},
    )

    # The window must already exist by the time the executor's run method fires.
    assert executor.gate_window_at_run is True

    gate_window_path = attempt_dir / "gate-window.json"
    assert gate_window_path.is_file()
    window = json.loads(gate_window_path.read_text(encoding="utf-8"))
    assert window["schema_version"] == 1
    assert window["gate"] == "qualification-moveit-plan-joint"
    assert window["attempt_id"] == "attempt-1"
    assert window["raw_start_index"] == 3
    assert window["evaluator_start_index"] == 3

    # RED: the driver must declare the gate window as an executor artifact so it
    # is final before the terminal marker.
    assert "gate-window.json" in d.EXECUTOR_ARTIFACT_FILENAMES


class _GateWindowStallExecutor(FakeExecutor):
    """Executor double reproducing the live gate-window spinner-starvation shape.

    A fresh DDS joint_states/collision message lands WHILE the synchronous
    ``_write_gate_window`` parse runs (``queue_pending_callback``, invoked from
    the ``_count_valid_jsonl_records`` side effect).  ``_spin_once`` simulates
    the shared-spinner dispatch that would deliver that queued callback.  The
    gate method re-checks ``_readiness()`` exactly like the real executor
    (``integrated_gate_executor.py:4199``): with the callback still undelivered
    every readiness stream reads stale and the gate fail-closes with
    ``evidence-invalid``.
    """

    def __init__(self, *, write_artifacts=False):
        super().__init__(write_artifacts=write_artifacts)
        self._pending_callback = False
        self._callback_dispatched = False
        self._callback_dispatched_before_run: bool | None = None
        self.events: list[str] = []

    def queue_pending_callback(self) -> None:
        # A fresh joint_states + collision message is queued on the observer's
        # subscription while the gate-window parse blocks the main thread.
        self._pending_callback = True
        self.events.append("queue")

    def _spin_once(self) -> None:
        # One shared-spinner dispatch delivers the queued DDS callbacks.
        self.events.append("spin")
        if self._pending_callback:
            self._pending_callback = False
            self._callback_dispatched = True

    def _readiness(self) -> dict[str, object]:
        # The gate's own readiness snapshot: if the queued callback has not been
        # dispatched yet, every stream it feeds (joint_states/collision/etc.) is
        # stale and readiness fails closed on all checks.
        if self._pending_callback:
            return {
                "ready": False,
                "reasons": ["stale: queued DDS callback undelivered"],
            }
        return {"ready": True, "reasons": []}

    def run_gate_c_plan_only(self, scenario_id, **kwargs):
        # Mirrors the real gate method: record whether the driver's bounded
        # re-drain already dispatched the queued callback, then run the gate's
        # OWN readiness re-check before doing any work.
        self._callback_dispatched_before_run = self._callback_dispatched
        readiness = self._readiness()
        self.calls.append(("run_gate_c_plan_only", scenario_id, dict(kwargs)))
        self.events.append("run")
        if self.attempt_dir is not None:
            for name in d.EXECUTOR_ARTIFACT_FILENAMES:
                if name == "gate-window.json":
                    # The driver owns the gate window; never clobber it.
                    continue
                (self.attempt_dir / name).write_text(
                    json.dumps({"schema_version": 1, "name": name}, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
        if not readiness.get("ready"):
            return {
                "status": "evidence-invalid",
                "scenario_id": scenario_id,
                "reasons": list(readiness.get("reasons") or ["readiness re-check failed"]),
            }
        return {"status": self._run_status, "scenario_id": scenario_id}


def test_run_driver_redrains_spinner_after_gate_window_write_before_gate_run():
    """RED: run_driver must re-drain the shared spinner after the synchronous
    gate-window write and before the gate method's own readiness re-check.

    Live Stage C failure (``task66-ompl-stage-c-20260807T213555``,
    ``C-qualification-moveit-plan-blocked-088e8210-2``): ``_write_gate_window``
    parses EVERY line of ``physics_truth.jsonl`` (4.5MB) and ``evaluator.jsonl``
    (5.3MB) on the SingleThreadedExecutor's main thread — 1.19s for the blocked
    scenario — starving the spinner so no DDS callbacks are dispatched.  The
    gate's own ``_readiness()`` re-check moments later snapshots every stream
    stale and fail-closes with ``reason_code: readiness-failed`` on all six
    checks, even though ``_wait_for_readiness`` had returned ``ready: true``.

    Here a fresh joint_states + collision callback is queued while the gate
    window's synchronous parse runs.  With no bounded drain between the write
    and the gate run, the gate's own readiness re-check sees the undelivered
    callback and returns ``evidence-invalid``.  The driver must run the
    established bounded ``_spin_readiness_callbacks`` drain after the write so
    the queued callback is dispatched before ``run_gate_c_plan_only``'s
    ``_readiness()`` re-check.  This test fails against current code (no drain).
    """
    executor = _GateWindowStallExecutor(write_artifacts=False)
    attempt_dir = Path(_tmp())
    bundle = _bundle("qualification-moveit-plan-joint")
    bundle["attempt_dir"] = str(attempt_dir)

    def _count_with_callback_queued(path: Path) -> int:
        # A fresh DDS joint_states/collision message lands while the gate-window
        # parse blocks the spinner on the main thread.
        executor.queue_pending_callback()
        return 3

    with patch.object(
        d, "_count_valid_jsonl_records", side_effect=_count_with_callback_queued
    ):
        result = d.run_driver(
            bundle=bundle,
            attempt_dir=attempt_dir,
            config=_config(),
            domain_id=100,
            seed=7,
            executor_factory=_fake_executor_factory(executor),
            runtime_provider_factory=lambda **kwargs: {},
        )

    # The queued DDS callback must be dispatched by a bounded re-drain between
    # the gate-window write and the gate run.
    assert executor._callback_dispatched_before_run is True, (
        "a bounded spinner re-drain must dispatch the DDS callbacks queued "
        f"during the synchronous gate-window parse before the gate run "
        f"(events={executor.events})"
    )
    # Ordering: queue (during the gate-window write) → spin (the re-drain) → run.
    queue_index = executor.events.index("queue")
    spins_after_queue = [
        i for i, event in enumerate(executor.events) if event == "spin" and i > queue_index
    ]
    run_index = executor.events.index("run")
    assert spins_after_queue and spins_after_queue[0] < run_index, (
        "the re-drain spin must run after the gate-window write (queue) and "
        f"before the gate run (events={executor.events})"
    )
    assert result["status"] == "verified-pass", (
        "the gate must not fail-closed on its own readiness re-check after the "
        f"gate-window write (got {result['status']})"
    )


def test_write_gate_window_writes_consistent_indices_under_race():
    """RED: gate-window indices must be consistent even under a frame race.

    The live failure shape: the two sequential ``_count_valid_jsonl_records``
    reads race a frame boundary so the second (evaluator) count sees one more
    than the first (raw) — raw_start_index=927, evaluator_start_index=928 —
    while the FINAL files are exactly correlated (957/957).  The verifier's
    exact correlation (``_raw_evaluator_correlation``) slices both tails from
    these indices, so the one-index skew misaligns the tails and fails with
    ``raw/evaluator drain mismatch``.  The writer must emit the same earlier
    (min) boundary for both so the 1:1 correlation holds.
    """
    attempt_dir = Path(_tmp())
    records = "".join(
        json.dumps(
            {"frame_index": index, "scenario": "qualification-moveit-plan-joint", "seed": 7}
        ) + "\n"
        for index in range(5)
    )
    # Final files are exactly correlated (both 5 records) — the drain is 1:1.
    (attempt_dir / "physics_truth.jsonl").write_text(records, encoding="utf-8")
    (attempt_dir / "evaluator.jsonl").write_text(records, encoding="utf-8")

    real_count = d._count_valid_jsonl_records

    def _racy_count(path: Path) -> int:
        # The evaluator read races a transient extra record ahead of raw
        # (the live 927/928 shape), while the final files stay equal.
        if path.name == "evaluator.jsonl":
            return real_count(path) + 1
        return real_count(path)

    with patch.object(d, "_count_valid_jsonl_records", side_effect=_racy_count):
        d._write_gate_window(attempt_dir, "qualification-moveit-plan-joint", "attempt-1")

    window = json.loads((attempt_dir / "gate-window.json").read_text(encoding="utf-8"))
    assert window["raw_start_index"] == window["evaluator_start_index"], (
        "raw/evaluator start indices must be consistent under a count race "
        f"(got raw={window['raw_start_index']} evaluator={window['evaluator_start_index']})"
    )
    assert window["raw_start_index"] == 5, (
        "the consistent boundary must be the earlier index (min), so both "
        "tails start from a frame present in both files at the first read"
    )


def test_run_driver_dispatch_passes_provider_kwargs_for_each_method():
    cases = {
        "qualification-moveit-execute-joint": "run_execute_sequence",
        "qualification-moveit-execute-pose": "run_execute_sequence",
        "qualification-moveit-cartesian-retreat": "run_cartesian_retreat",
        "qualification-moveit-gripper": "run_gripper_sequence",
        "qualification-moveit-cancel": "run_cancel_sequence",
        "qualification-moveit-safety": "run_safety_sequence",
        "qualification-pick-place-positive": "run_pick_place_sequence",
    }
    for scenario_id, method in cases.items():
        executor = FakeExecutor(write_artifacts=True)
        attempt_dir = Path(_tmp())
        bundle = _bundle(scenario_id)
        bundle["attempt_dir"] = str(attempt_dir)
        seen_kwargs: dict = {}

        def provider_factory(**kwargs):
            seen_kwargs.update(kwargs)
            return {}

        d.run_driver(
            bundle=bundle,
            attempt_dir=attempt_dir,
            config=_config(),
            domain_id=100,
            seed=7,
            executor_factory=_fake_executor_factory(executor),
            runtime_provider_factory=provider_factory,
        )
        assert executor.calls == [(method, scenario_id, {})]
        assert (attempt_dir / "execution-terminal.json").is_file()


def test_run_driver_readiness_failure_writes_no_terminal_and_raises():
    class NotReadyExecutor(FakeExecutor):
        def _readiness(self):
            return {"ready": False, "reasons": ["robot starts in collision"]}

    executor = NotReadyExecutor()
    attempt_dir = Path(_tmp())
    bundle = _bundle()
    bundle["attempt_dir"] = str(attempt_dir)
    with pytest.raises(d.DriverError, match="did not become ready"):
        d.run_driver(
            bundle=bundle,
            attempt_dir=attempt_dir,
            config=_config(),
            domain_id=100,
            seed=7,
            executor_factory=_fake_executor_factory(executor),
            runtime_provider_factory=lambda **kwargs: {},
            readiness_timeout_s=0.2,
        )
    assert executor.calls == []
    assert not (attempt_dir / "execution-terminal.json").exists()


def test_run_driver_executor_run_exception_propagates_and_writes_no_terminal():
    executor = FakeExecutor(run_error=RuntimeError("executor boom"))
    attempt_dir = Path(_tmp())
    bundle = _bundle()
    bundle["attempt_dir"] = str(attempt_dir)
    with pytest.raises(RuntimeError, match="executor boom"):
        d.run_driver(
            bundle=bundle,
            attempt_dir=attempt_dir,
            config=_config(),
            domain_id=100,
            seed=7,
            executor_factory=_fake_executor_factory(executor),
            runtime_provider_factory=lambda **kwargs: {},
        )
    assert not (attempt_dir / "execution-terminal.json").exists()


def test_run_driver_missing_integrated_execution_refuses_terminal():
    class NoSummaryExecutor(FakeExecutor):
        def _record(self, method, scenario_id, kwargs):
            # run method returns without writing the executor's own summary.
            self.calls.append((method, scenario_id, dict(kwargs)))
            return {"status": "evidence-invalid", "scenario_id": scenario_id}

    executor = NoSummaryExecutor()
    attempt_dir = Path(_tmp())
    bundle = _bundle()
    bundle["attempt_dir"] = str(attempt_dir)
    with pytest.raises(d.DriverError, match="integrated-execution.json"):
        d.run_driver(
            bundle=bundle,
            attempt_dir=attempt_dir,
            config=_config(),
            domain_id=100,
            seed=7,
            executor_factory=_fake_executor_factory(executor),
            runtime_provider_factory=lambda **kwargs: {},
        )
    assert not (attempt_dir / "execution-terminal.json").exists()


def test_write_terminal_rejects_preexisting_stale_marker():
    attempt_dir = Path(_tmp())
    (attempt_dir / "execution-terminal.json").write_text(
        json.dumps({"schema_version": 1, "scenario_id": "stale"}), encoding="utf-8"
    )
    with pytest.raises(d.DriverError, match="already exists"):
        d.write_terminal(attempt_dir, "qualification-moveit-plan-joint", "attempt-1", "verified-pass")
    # The stale marker is preserved byte-for-byte.
    assert json.loads((attempt_dir / "execution-terminal.json").read_text())["scenario_id"] == "stale"


def test_reject_preexisting_terminal_fails_closed():
    attempt_dir = Path(_tmp())
    (attempt_dir / "execution-terminal.json").write_text("{}", encoding="utf-8")
    with pytest.raises(d.DriverError, match="already contains"):
        d.reject_preexisting_terminal(attempt_dir)


def test_main_writes_fail_closed_terminal_and_exits_nonzero():
    attempt_dir = Path(_tmp())
    bundle = _bundle()
    bundle["attempt_dir"] = str(attempt_dir)
    bundle_path = attempt_dir / "scenario-bundle.json"
    bundle_path.write_text(json.dumps(bundle, sort_keys=True), encoding="utf-8")
    config_path = attempt_dir / "config.json"
    config_path.write_text(json.dumps(_config(), sort_keys=True), encoding="utf-8")

    def exploding_factory(**kwargs):
        raise RuntimeError("driver construction boom")

    with patch.object(d, "_construct_executor", side_effect=exploding_factory):
        rc = d.main(
            [
                "--scenario-bundle", str(bundle_path),
                "--attempt-dir", str(attempt_dir),
                "--config", str(config_path),
                "--domain", "100",
                "--seed", "7",
            ]
        )
    assert rc == 1
    marker = attempt_dir / "execution-terminal.json"
    assert marker.is_file()
    value = json.loads(marker.read_text(encoding="utf-8"))
    assert value["status"] == "evidence-invalid"
    assert value["marker"] == "executor-driver"
    assert value["scenario_id"] == "qualification-moveit-plan-joint"
    assert value["attempt_id"] == "attempt-1"


# --------------------------------------------------------------------------- #
# F2.4 — post_grasp_lift_m set/read-back parameter-client doubles
# --------------------------------------------------------------------------- #

class _FakeSetResult:
    def __init__(self, ok=True):
        self.successful = ok


class _FakeGetResult:
    def __init__(self, value):
        self.value = value


class FakeParameterClient:
    def __init__(self, *, set_ok=True, readback=0.10, wait_ok=True):
        self._set_ok = set_ok
        self._readback = readback
        self._wait_ok = wait_ok
        self.set_calls: list[list[object]] = []
        self.get_calls: list[list[str]] = []

    def wait_for_service(self, timeout_sec):
        return self._wait_ok

    def set_parameters(self, params):
        self.set_calls.append(list(params))
        if not self._set_ok:
            return _FakeSetResult(ok=False)
        return _FakeSetResult(ok=True)

    def get_parameters(self, names):
        self.get_calls.append(list(names))
        return _FakeGetResult(self._readback)


def test_set_post_grasp_lift_m_sets_then_reads_back_exact():
    client = FakeParameterClient()
    observed = d.set_post_grasp_lift_m(client, value_m=0.10)
    assert observed["value_m"] == 0.10
    assert observed["requested_value_m"] == 0.10
    assert len(client.set_calls) == 1
    param = client.set_calls[0][0]
    assert param["name"] == "post_grasp_lift_m"
    assert param["value"] == 0.10
    assert client.get_calls == [["post_grasp_lift_m"]]
    # The returned provider is fresh typed evidence of the observed read-back.
    provider = d._post_grasp_lift_m_provider(observed)
    sample = provider()
    assert sample["value_m"] == 0.10
    assert sample["age_s"] == 0.0
    assert sample["identity"]


def test_set_post_grasp_lift_m_set_rejected_fails_closed():
    client = FakeParameterClient(set_ok=False)
    with pytest.raises(d.LiftParameterError, match="set was rejected"):
        d.set_post_grasp_lift_m(client, value_m=0.10)


def test_set_post_grasp_lift_m_low_readback_fails_closed():
    client = FakeParameterClient(readback=0.08)
    with pytest.raises(d.LiftParameterError, match="below required"):
        d.set_post_grasp_lift_m(client, value_m=0.10)


def test_set_post_grasp_lift_m_service_unavailable_fails_closed():
    client = FakeParameterClient(wait_ok=False)
    with pytest.raises(d.LiftParameterError, match="unavailable"):
        d.set_post_grasp_lift_m(client, value_m=0.10)


def test_set_post_grasp_lift_m_malformed_value_fails_closed():
    client = FakeParameterClient()
    with pytest.raises(d.LiftParameterError, match="finite"):
        d.set_post_grasp_lift_m(client, value_m=float("nan"))
    with pytest.raises(d.LiftParameterError, match="finite"):
        d.set_post_grasp_lift_m(client, value_m=True)
    assert client.set_calls == []


def test_lift_requirement_failure_prevents_e_dispatch_and_terminal():
    executor = FakeExecutor()
    attempt_dir = Path(_tmp())
    bundle = _bundle("qualification-pick-place-positive")
    bundle["attempt_dir"] = str(attempt_dir)

    def failing_provider(**kwargs):
        raise d.LiftParameterError("pick_and_place parameter service is unavailable")

    with pytest.raises(d.LiftParameterError):
        d.run_driver(
            bundle=bundle,
            attempt_dir=attempt_dir,
            config=_config(),
            domain_id=100,
            seed=7,
            executor_factory=_fake_executor_factory(executor),
            runtime_provider_factory=failing_provider,
        )
    assert executor.calls == []
    assert not (attempt_dir / "execution-terminal.json").exists()


# --------------------------------------------------------------------------- #
# F2.5 — config-derived terminal budget = 305.0, separate from readiness
# --------------------------------------------------------------------------- #

def test_derived_terminal_timeout_is_exactly_305_for_current_config():
    config = json.loads(
        (ROOT / "simulation/qualification/integrated-ompl.json").read_text(encoding="utf-8")
    )
    assert d.derive_terminal_timeout(config) == 305.0


def test_derived_terminal_timeout_is_not_shorter_than_sequential_run_budgets():
    # Worst E transport sequential budget ~275 s and D gripper path 240 s are
    # both under the derived 305.0 s terminal budget.
    config = json.loads(
        (ROOT / "simulation/qualification/integrated-ompl.json").read_text(encoding="utf-8")
    )
    budget = d.derive_terminal_timeout(config)
    thresholds = config["thresholds"]
    assert budget >= (
        thresholds["plan_timeout_s"]
        + 2.0 * thresholds["execute_timeout_s"]
        + thresholds["cancel_timeout_s"]
        + thresholds["scene_timeout_s"]
        + 30.0
    )
    assert budget >= 275.0
    assert budget >= 240.0


def test_derived_terminal_timeout_malformed_fails_closed():
    with pytest.raises(ValueError):
        d.derive_terminal_timeout({})
    with pytest.raises(ValueError):
        d.derive_terminal_timeout(
            {
                "thresholds": {
                    "plan_timeout_s": -1.0,
                    "execute_timeout_s": 120.0,
                    "cancel_timeout_s": 10.0,
                    "scene_timeout_s": 10.0,
                }
            }
        )
    with pytest.raises(ValueError):
        d.derive_terminal_timeout(
            {
                "thresholds": {
                    "plan_timeout_s": 15.0,
                    "execute_timeout_s": 120.0,
                    "cancel_timeout_s": 10.0,
                }
            }
        )


# --------------------------------------------------------------------------- #
# F3.7 — pure layer stays pure: double-parameter records and duck-typed results
# --------------------------------------------------------------------------- #

def test_double_parameter_is_a_plain_pure_dict():
    record = d._double_parameter("post_grasp_lift_m", 0.10)
    assert type(record) is dict
    assert record == {"name": "post_grasp_lift_m", "value": 0.10}
    # The pure layer never fabricates an rclpy object and stays import-safe.
    assert "rclpy" not in sys.modules


def test_double_parameter_coerces_name_and_value():
    record = d._double_parameter(123, "0.5")
    assert record == {"name": "123", "value": 0.5}


def test_set_result_ok_duck_typing_is_exact():
    class Ok:
        successful = True

    class Bad:
        successful = False

    assert d._set_result_ok(Ok()) is True
    assert d._set_result_ok(Bad()) is False
    assert d._set_result_ok([Ok()]) is True
    assert d._set_result_ok([Bad()]) is False
    assert d._set_result_ok([]) is False
    assert d._set_result_ok({"successful": True}) is True
    assert d._set_result_ok({"successful": False}) is False
    assert d._set_result_ok(None) is False
    assert d._set_result_ok(True) is True
    assert d._set_result_ok(False) is False


def test_extract_double_accepts_response_values_and_plain_records():
    class ParamValue:
        type = 3  # rcl_interfaces PARAMETER_DOUBLE
        double_value = 0.10

    class GetResponse:
        values = [ParamValue()]

    assert d._extract_double(GetResponse(), "post_grasp_lift_m") == 0.10
    assert d._extract_double({"value": 0.3}, "x") == 0.3

    class AttrValue:
        value = 0.5

    assert d._extract_double(AttrValue(), "x") == 0.5


def _get_response(values):
    class GetResponse:
        pass

    response = GetResponse()
    response.values = values
    return response


class _ParamValue:
    def __init__(self, type_, double_value=None):
        self.type = type_
        self.double_value = double_value


def test_extract_double_fails_closed_on_non_double():
    assert d._extract_double(None, "x") is None
    assert d._extract_double(float("nan"), "x") is None
    assert d._extract_double(float("inf"), "x") is None
    assert d._extract_double(True, "x") is None
    assert d._extract_double(_get_response([]), "x") is None
    # Wrong parameter type (not PARAMETER_DOUBLE) fails closed.
    assert d._extract_double(_get_response([_ParamValue(1, 0.1)]), "x") is None
    # Non-finite / boolean double values fail closed.
    assert d._extract_double(_get_response([_ParamValue(3, float("nan"))]), "x") is None
    assert d._extract_double(_get_response([_ParamValue(3, True)]), "x") is None
    # Missing values entirely fails closed.
    assert d._extract_double(_get_response(None), "x") is None


# --------------------------------------------------------------------------- #
# F3.1/F3.2 — bundle committed-identity binding stays fail-closed
# --------------------------------------------------------------------------- #

def test_build_executor_scenario_missing_committed_identity_fails_closed():
    bundle = _bundle()
    for missing in (
        "report_identities",
        "integrated",
        "planning_scene_declaration",
        "planning_scene",
        "scenario",
    ):
        variant = dict(bundle)
        del variant[missing]
        with pytest.raises(d.DriverError):
            d.build_executor_scenario(variant)


# --------------------------------------------------------------------------- #
# Option A+ — qualification-only occupancy from committed scenario geometry
# --------------------------------------------------------------------------- #

import run_sim as rs  # noqa: E402


def test_build_occupancy_dimensions_origin_and_footprint():
    import math

    obj = {
        "id": "sim_fixture/pedestal",
        "primitive": {"type": "box", "dimensions": [0.12, 0.12, 0.6]},
        "pose": {"xyz": [0.65, 0.0, 0.3], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
    }
    m = rs.build_occupancy_from_planning_scene([obj], resolution=0.05, half_extent=60.0)
    assert m.width == 2400
    assert m.height == 2400
    assert m.resolution == 0.05
    assert m.origin_x == -60.0
    assert m.origin_y == -60.0
    assert m.occupied_at_world(0.65, 0.0) is True
    assert m.occupied_at_world(-4.0, -4.0) is False
    rects = m.rectangles()
    assert len(rects) == 1
    cx, cy, sx, sy = rects[0]
    assert abs(cx - 0.65) < 0.1
    assert abs(cy) < 0.1
    assert 0.10 <= sx <= 0.20
    assert 0.10 <= sy <= 0.20
    # A 40 m ray from the origin toward +x (where the pedestal sits) is
    # blocked well before the lidar range; the boundary is never a fake
    # obstacle within range.
    hit = m.raycast(0.0, 0.0, 0.0, minimum=0.3, maximum=40.0)
    assert math.isfinite(hit)
    assert 0.3 <= hit <= 1.0
    # A ray in free space reaches the map limit without a fake obstacle.
    free = m.raycast(0.0, 0.0, math.pi / 2.0, minimum=0.3, maximum=40.0)
    assert free == float("inf")


def test_build_occupancy_rejects_malformed_fixture_geometry():
    base = {
        "id": "f",
        "primitive": {"type": "box", "dimensions": [0.1, 0.1, 0.1]},
        "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
    }
    # Non-box primitive is rejected, never silently invented.
    bad = {**base, "primitive": {"type": "cylinder", "dimensions": [0.1, 0.1, 0.1]}}
    with pytest.raises(ValueError, match="unsupported qualification fixture"):
        rs.build_occupancy_from_planning_scene([bad])
    # F4.6: a yaw-only rotation (90 deg about z) is a valid oriented footprint.
    yaw = {
        **base,
        "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.7071068, 0.7071068]},
    }
    assert rs.build_occupancy_from_planning_scene([yaw]) is not None
    # A roll rotation (rotation about x) is not representable by the 2-D
    # development lidar and is rejected.
    bad = {
        **base,
        "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.7071068, 0.0, 0.0, 0.7071068]},
    }
    with pytest.raises(ValueError, match="yaw-only"):
        rs.build_occupancy_from_planning_scene([bad])
    # A non-normalized quaternion is rejected.
    bad = {
        **base,
        "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.5]},
    }
    with pytest.raises(ValueError, match="normalized"):
        rs.build_occupancy_from_planning_scene([bad])
    # Non-positive dimensions are rejected.
    bad = {**base, "primitive": {"type": "box", "dimensions": [0.0, 0.1, 0.1]}}
    with pytest.raises(ValueError, match="positive"):
        rs.build_occupancy_from_planning_scene([bad])
    # Non-finite values are rejected.
    bad = {
        **base,
        "pose": {"xyz": [float("nan"), 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
    }
    with pytest.raises(ValueError, match="finite"):
        rs.build_occupancy_from_planning_scene([bad])
    # Missing pose is rejected.
    bad = {"id": "f", "primitive": {"type": "box", "dimensions": [0.1, 0.1, 0.1]}}
    with pytest.raises(ValueError, match="no pose"):
        rs.build_occupancy_from_planning_scene([bad])


def test_build_occupancy_rejects_bad_grid_parameters():
    obj = {
        "id": "f",
        "primitive": {"type": "box", "dimensions": [0.1, 0.1, 0.1]},
        "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
    }
    with pytest.raises(ValueError, match="resolution"):
        rs.build_occupancy_from_planning_scene([obj], resolution=0.0)
    with pytest.raises(ValueError, match="resolution"):
        rs.build_occupancy_from_planning_scene([obj], resolution=True)
    # The half-extent must exceed the 40 m lidar range.
    with pytest.raises(ValueError, match="lidar range"):
        rs.build_occupancy_from_planning_scene([obj], half_extent=40.0)
    with pytest.raises(ValueError, match="lidar range"):
        rs.build_occupancy_from_planning_scene([obj], half_extent=41.0)


def test_qualification_occupancy_resolves_committed_scenario():
    # The free-space scenario has no PlanningScene boxes → no occupancy map.
    assert rs.qualification_occupancy(ROOT, "qualification-free-space") is None
    # The positive E scenario has box fixtures → a deterministic occupancy map.
    m = rs.qualification_occupancy(ROOT, "qualification-pick-place-positive")
    assert m is not None
    assert m.resolution == 0.05
    assert m.occupied_at_world(0.65, 0.0) is True
    assert m.occupied_at_world(0.85, 0.0) is True
    assert m.occupied_at_world(-4.0, -4.0) is False


def test_qualification_occupancy_resolves_all_17_canonical_scenarios():
    # F4.6: every canonical scenario must resolve qualification occupancy
    # without raising.  The two rotated-target pose scenarios (plan-pose /
    # execute-pose, 45 deg about z) are exactly the B4.3 regression closed.
    from integrated_qualification import QUALIFICATION_SCENARIO_NAMES

    import math

    resolved = {}
    for name in QUALIFICATION_SCENARIO_NAMES:
        try:
            resolved[name] = rs.qualification_occupancy(ROOT, name)
        except Exception as exc:  # noqa: BLE001 - fail-closed enumeration
            raise AssertionError(
                f"qualification_occupancy raised for canonical {name}: {exc}"
            ) from exc
    # The two rotated-target pose scenarios must produce usable occupancy.
    for rotated in ("qualification-moveit-plan-pose", "qualification-moveit-execute-pose"):
        m = resolved[rotated]
        assert m is not None, f"{rotated} must resolve to an occupancy map"
        assert m.resolution == 0.05
    # The D-retreat pedestal still produces a finite +x raycast.
    retreat = resolved["qualification-moveit-cartesian-retreat"]
    assert retreat is not None
    hit = retreat.raycast(0.0, 0.0, 0.0, minimum=0.3, maximum=40.0)
    assert math.isfinite(hit)
    assert 0.3 <= hit <= 1.5


def test_gateway_lidar_enabled_resolution_is_exact():
    # navigation-parity always enables the development lidar (unchanged).
    assert rs.gateway_lidar_enabled("navigation-parity", False) is True
    assert rs.gateway_lidar_enabled("navigation-parity", True) is True
    # manipulation-core enables it only under --qualification.
    assert rs.gateway_lidar_enabled("manipulation-core", True) is True
    assert rs.gateway_lidar_enabled("manipulation-core", False) is False
    # Any other profile stays disabled.
    assert rs.gateway_lidar_enabled("streaming", True) is False
    assert rs.gateway_lidar_enabled("manipulation-cumotion", True) is False


# --------------------------------------------------------------------------- #
# F3.7 — duplicate-FQN graph collectors: private MoveIt helpers dedup + source
# --------------------------------------------------------------------------- #

class _FakeGraphNode:
    """ROS-free graph-cache double that reports duplicate identical FQNs and a
    named service/client surface for the ``move_group_private_*`` label."""

    def __init__(self, *, pairs, service_name="/get_planning_scene", serve=True, client=False):
        self._pairs = pairs
        self._service_name = service_name
        self._serve = serve
        self._client = client

    def get_node_names_and_namespaces(self):
        return list(self._pairs)

    def get_service_names_and_types_by_node(self, node_name, node_namespace):
        del node_namespace
        if self._serve and node_name == "move_group_private_123":
            return [(self._service_name, ["moveit_msgs/srv/GetPlanningScene"])]
        return []

    def get_client_names_and_types_by_node(self, node_name, node_namespace):
        del node_namespace
        if self._client and node_name == "move_group_private_123":
            return [(self._service_name, ["moveit_msgs/srv/GetPlanningScene"])]
        return []


def test_service_servers_and_clients_deduplicates_duplicate_fqns():
    """A launch global-remap can surface duplicate identical FQNs in the graph;
    each server and client label must be reported exactly once per canonical
    node."""
    node = _FakeGraphNode(
        pairs=[("move_group_private_123", ""), ("move_group_private_123", "")],
        client=True,
    )
    servers, clients = d._service_servers_and_clients(node, "/get_planning_scene")
    assert servers == ["/move_group_private_123"]
    assert clients == ["/move_group_private_123"]


def test_service_servers_and_source_private_moveit_label_maps_to_move_group():
    """MoveIt private-helper nodes (``move_group_private_*``) are owned by the
    ``/move_group`` planner; ``/get_planning_scene`` resolves to exactly one
    server sourced ``/move_group``."""
    node = _FakeGraphNode(pairs=[("move_group_private_123", "")])
    assert d._service_servers_and_source(node, "/get_planning_scene") == (1, "/move_group")


# --------------------------------------------------------------------------- #
# Operator continuous-freshness RED regression (publish once per snapshot)
# --------------------------------------------------------------------------- #

class _FakeClock:
    """ROS-free monotonic wall clock advanced continuously in wall time.

    ``advance`` sleeps the caller so snapshot work elapses continuously in
    wall/fake time.  A concurrent heartbeat publishing during a snapshot
    therefore records intermediate timestamps (the previous atomic time jump
    hid them).
    """

    def __init__(self) -> None:
        self.t = time.monotonic()

    def advance(self, seconds: float) -> None:
        time.sleep(float(seconds))
        self.t = time.monotonic()


class _SlowReadinessOperatorPublisher:
    """Faithful ROS-free model of the driver's readiness operator publishing.

    Mirrors ``_build_readiness_snapshot``'s operator block (F3.2): each readiness
    evaluation first performs snapshot work — the controller-manager
    ``ListControllers`` query (bounded at 1.0 s by ``_CONTROLLER_QUERY_TIMEOUT_S``)
    plus the graph-introspection warm-up spins (10 x 20 ms) — which elapses
    continuously for ``snapshot_work_s`` and takes well over the 0.25 s operator
    max-age window, and then publishes ``/sim/safety/operator`` exactly once at
    the very end.  The observer records each receipt at the wall-clock instant it
    occurs, so a concurrent heartbeat publishing during the snapshot records
    intermediate timestamps.  Readiness stays not-ready for a few snapshots so
    ``_wait_for_readiness`` drives multiple publish cycles.
    """

    def __init__(self, clock: _FakeClock, *, snapshot_work_s: float) -> None:
        self.clock = clock
        self.snapshot_work_s = float(snapshot_work_s)
        self.operator_publish_times: list[float] = []
        self.operator_received_mono: float | None = None
        self.operator_samples = 0
        self.iterations_left = 3

    def _spin_once(self) -> None:
        pass

    def publish_operator(self, value: bool) -> None:
        del value
        # Record the wall-clock receipt; monotonic is the shared time base for
        # the continuous snapshot work and the concurrent heartbeat.
        self.operator_publish_times.append(time.monotonic())
        self.operator_received_mono = time.monotonic()
        self.operator_samples += 1

    def _readiness(self) -> dict[str, object]:
        # Snapshot work (controller query + graph introspection) elapses
        # continuously in small wall-clock steps so a concurrent heartbeat can
        # publish intermediate operator samples.
        end = self.clock.t + self.snapshot_work_s
        while self.clock.t < end:
            self.clock.advance(0.02)
        self.publish_operator(False)  # exactly once per snapshot
        self.iterations_left -= 1
        return {"ready": self.iterations_left <= 0, "reasons": []}


def test_operator_is_refreshed_continuously_below_max_age_during_slow_readiness():
    """GREEN: ``/sim/safety/operator`` stays fresh during slow readiness.

    The committed operator freshness contract is the config ``operator_fresh_s``
    (fallback ``fixture_fresh_s`` = 0.25 s).  Readiness snapshot work
    (controller-manager query + graph introspection) exceeds 0.25 s, so a driver
    that publishes the operator baseline only once per snapshot leaves the topic
    on the wire stale for most of the snapshot.  This test drives the real
    ``_wait_for_readiness`` seam, which runs an independent wall-clock operator
    heartbeat at a cadence safely below the max age, and asserts the publish
    cadence never exceeds the limit.
    """
    config = json.loads(
        (ROOT / "simulation/qualification/integrated-ompl.json").read_text(encoding="utf-8")
    )
    thresholds = config["thresholds"]
    operator_fresh_s = float(thresholds.get("operator_fresh_s", thresholds["fixture_fresh_s"]))
    assert operator_fresh_s == 0.25
    snapshot_work_s = 0.5  # controller query + graph introspection > max age

    clock = _FakeClock()
    publisher = _SlowReadinessOperatorPublisher(clock, snapshot_work_s=snapshot_work_s)
    d._wait_for_readiness(publisher, timeout_s=5.0)

    publish_times = publisher.operator_publish_times
    assert publish_times, "operator baseline must be published during readiness"
    gaps = [later - earlier for earlier, later in zip(publish_times, publish_times[1:])]
    assert max(gaps) <= operator_fresh_s, (
        "/sim/safety/operator went stale between readiness snapshots: publish "
        f"gaps {gaps} exceed the {operator_fresh_s}s max age; the operator is "
        "refreshed only once per snapshot, not continuously"
    )


def test_operator_once_per_snapshot_goes_stale_without_heartbeat():
    """RED: once-per-snapshot operator publishing exceeds the max-age window.

    With the independent operator heartbeat disabled
    (``operator_heartbeat_period_s=None``), the only operator publishes come from
    the readiness snapshot provider exactly once per snapshot.  Snapshot work
    exceeds the 0.25 s max age, so the publish gaps exceed the limit.  This is
    the regression guard for the continuous-freshness contract: a driver that
    reverts to once-per-snapshot publishing must fail here.
    """
    config = json.loads(
        (ROOT / "simulation/qualification/integrated-ompl.json").read_text(encoding="utf-8")
    )
    thresholds = config["thresholds"]
    operator_fresh_s = float(thresholds.get("operator_fresh_s", thresholds["fixture_fresh_s"]))
    assert operator_fresh_s == 0.25
    snapshot_work_s = 0.5  # controller query + graph introspection > max age

    clock = _FakeClock()
    publisher = _SlowReadinessOperatorPublisher(clock, snapshot_work_s=snapshot_work_s)
    d._wait_for_readiness(publisher, timeout_s=5.0, operator_heartbeat_period_s=None)

    publish_times = publisher.operator_publish_times
    assert publish_times, "operator baseline must be published during readiness"
    gaps = [later - earlier for earlier, later in zip(publish_times, publish_times[1:])]
    assert max(gaps) > operator_fresh_s, (
        "once-per-snapshot operator publishing must exceed the max age: publish "
        f"gaps {gaps} did not exceed the {operator_fresh_s}s limit"
    )


# --------------------------------------------------------------------------- #
# RED regression — one server/client inventory query per unique graph node
# --------------------------------------------------------------------------- #

class _ControllerRecord:
    """Minimal ``controller_manager_msgs`` controller record double."""

    def __init__(self, name: str, state: str) -> None:
        self.name = name
        self.state = state


class _ReadyClient:
    """Action/service client double that reports its server ready."""

    def server_is_ready(self) -> bool:
        return True

    def service_is_ready(self) -> bool:
        return True


class _LookupException(Exception):
    """Stand-in for ``tf2_ros.LookupException`` under the fake ``tf2_ros``."""


class _GraphCountingNode:
    """Fake rmw graph node recording server/client inventory query counts.

    ``get_service_names_and_types_by_node`` and
    ``get_client_names_and_types_by_node`` are the expensive per-node graph
    queries.  Endpoint validation needs only *server* names from the relevant
    provider nodes, so a corrected snapshot must issue exactly one server query
    per unique relevant provider node, zero client queries, and zero queries on
    irrelevant (noise) nodes.  Every queried node name is logged so the test can
    prove noise nodes are never touched.
    """

    def __init__(self, node_services: dict[str, list[str]]) -> None:
        self._node_services = dict(node_services)
        self.service_query_calls = 0
        self.client_query_calls = 0
        self.queried_service_nodes: list[str] = []
        self.queried_client_nodes: list[str] = []

    def get_node_names_and_namespaces(self):
        return [(name, "/") for name in self._node_services]

    def get_service_names_and_types_by_node(self, node_name, node_namespace):
        del node_namespace
        self.service_query_calls += 1
        self.queried_service_nodes.append(node_name)
        return [(service, ["x"]) for service in self._node_services.get(node_name, ())]

    def get_client_names_and_types_by_node(self, node_name, node_namespace):
        del node_namespace
        self.client_query_calls += 1
        self.queried_client_nodes.append(node_name)
        return []

    def get_publishers_info_by_topic(self, topic):
        raise RuntimeError("no publishers in the fake readiness graph")


class _SnapshotObserver:
    """Readiness observer double with a TF buffer that reports a lookup miss."""

    def __init__(self, node: _GraphCountingNode) -> None:
        self.node = node
        self.last_tf_received_mono: float | None = None
        self.joint_received_mono: float | None = 0.0
        self.safety_received_mono: float | None = 0.0
        self.safety_received_timestamp_ns: int | None = 1
        self.safety_samples = 2
        self.fixture_received_mono: float | None = 0.0
        self.fixture_samples = 1
        self.operator_samples = 1
        self.operator_received_mono: float | None = 0.0
        self.operator_received_timestamp_ns: int | None = 1
        self.operator_value = False
        self.collision_samples = 1
        self.collision_value = False
        self.collision_received_mono: float | None = 0.0

        class _Buffer:
            def lookup_transform(self, target, source, time):
                raise _LookupException()

        self.tf_buffer = _Buffer()

    def list_controllers(self):
        return [
            _ControllerRecord("joint_state_broadcaster", "active"),
            _ControllerRecord("xarm7_traj_controller", "active"),
        ]


class _SnapshotExecutor:
    """Executor double carrying the full per-attempt surface the snapshot reads."""

    def __init__(self, node: _GraphCountingNode, observer: _SnapshotObserver) -> None:
        self.node = node
        self._driver_observer = observer
        self._latest_joint_state = types.SimpleNamespace(
            name=d._REQUIRED_JOINT_NAMES,
            position=[0.0] * len(d._REQUIRED_JOINT_NAMES),
            velocity=[0.0] * len(d._REQUIRED_JOINT_NAMES),
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=1, nanosec=0)),
        )
        self._latest_safety_stop = types.SimpleNamespace(data=False)
        self._fixture_payload = ""
        self._latest_planning_scene: dict[str, object] = {"owned_ids": [], "attached_ids": []}
        self._action_clients = {name: _ReadyClient() for name in REQUIRED_ACTIONS}
        self._service_clients = {name: _ReadyClient() for name in REQUIRED_SERVICES}

    def publish_operator(self, value: bool) -> None:
        del value

    def _spin_once(self) -> None:
        pass


#: Unique relevant provider nodes the endpoint validation must query.  The
#: ``move_group_private_*`` helper is part of the ``/move_group`` provider and
#: hosts the canonical ``/get_planning_scene`` server; its label canonicalizes
#: to ``/move_group``.
RELEVANT_PROVIDER_NODES: frozenset[str] = frozenset(
    {
        "controller_manager",
        "move_group",
        "move_group_private_123",
        "tinker_sim_gripper_facade",
        "pick_and_place",
        "tinker_sim_physics_ready_gate",
        "fixture_planning_scene",
    }
)

#: Irrelevant main-launch noise nodes that host none of the required endpoints;
#: a corrected snapshot must never query them.
NOISE_NODES: tuple[str, ...] = (
    "isaac_ros_ov_engine",
    "livox_lidar",
    "navigation_processor",
    "simulation_runner",
    "record_provider",
)


def _readiness_fake_graph() -> dict[str, list[str]]:
    """Realistic per-node service/action surface for every required endpoint.

    The six canonical ``_REQUIRED_ENDPOINT_SOURCES`` providers plus the
    ``move_group_private_*`` canonical helper host all 20 required endpoints;
    several irrelevant noise nodes host only unrelated services.
    """
    graph = {
        "controller_manager": [
            "/controller_manager/list_controllers",
            "/controller_manager/load_controller",
            "/controller_manager/configure_controller",
            "/controller_manager/switch_controller",
            "/xarm7_traj_controller/follow_joint_trajectory/_action/send_goal",
        ],
        "move_group": [
            "/move_action/_action/send_goal",
            "/execute_trajectory/_action/send_goal",
            "/apply_planning_scene",
            "/check_state_validity",
            "/compute_cartesian_path",
        ],
        "move_group_private_123": [
            "/get_planning_scene",
        ],
        "pick_and_place": [
            "/pickup_action/_action/send_goal",
            "/place_action/_action/send_goal",
            "/cartesian_move_action/_action/send_goal",
            "/joint_move_action/_action/send_goal",
            "/fold_action/_action/send_goal",
            "/arm_joint_service",
        ],
        "tinker_sim_gripper_facade": [
            "/xarm_gripper/gripper_action/_action/send_goal",
        ],
        "tinker_sim_physics_ready_gate": [
            "/sim/ready/physics",
        ],
        "fixture_planning_scene": [
            "/sim/ready/fixture",
        ],
    }
    for index, name in enumerate(NOISE_NODES):
        graph[name] = [f"/noise/{name}/service{index}"]
    return graph


def test_readiness_snapshot_queries_only_relevant_provider_servers_once():
    """RED: one readiness snapshot must issue exactly one *server* inventory query
    per unique relevant provider node, zero client queries, and zero queries on
    irrelevant noise nodes, preserving all 20 endpoint counts/sources.

    Live evidence: ``_build_readiness_snapshot`` calls
    ``_action_servers_and_source`` / ``_service_servers_and_source`` once per
    REQUIRED_ACTIONS/REQUIRED_SERVICES, and each ``_service_servers_and_clients``
    rescans *every* node via ``get_service_names_and_types_by_node`` **and**
    ``get_client_names_and_types_by_node``.  Endpoint validation needs only
    server names from the six unique ``_REQUIRED_ENDPOINT_SOURCES`` providers
    (``/move_group`` incl. its canonical ``move_group_private_*`` helper,
    ``/controller_manager``, ``/tinker_sim_gripper_facade``, ``/pick_and_place``,
    ``/tinker_sim_physics_ready_gate``, ``/fixture_planning_scene``).  Even with
    the endpoint rescan cut, inventorying servers *and* clients on every graph
    node still blocked the live snapshot ~29.8 s (exhausting the 30 s readiness
    deadline).  This test drives the real ``_build_readiness_snapshot`` against a
    fake graph hosting every required endpoint on the relevant provider nodes
    plus several irrelevant noise nodes, and asserts the corrected query shape:
    exactly one server query per relevant provider node, zero client queries,
    zero queries on noise nodes, with every endpoint count/source preserved.
    """
    node_services = _readiness_fake_graph()
    node = _GraphCountingNode(node_services)
    observer = _SnapshotObserver(node)
    executor = _SnapshotExecutor(node, observer)
    attempt_dir = Path(_tmp())
    (attempt_dir / "scenario-runner.json").write_text("{}", encoding="utf-8")
    bundle = _bundle()
    bundle["attempt_dir"] = str(attempt_dir)

    fake_tf2 = types.ModuleType("tf2_ros")
    fake_tf2.LookupException = _LookupException
    fake_tf2.ConnectivityException = _LookupException
    fake_tf2.ExtrapolationException = _LookupException

    with patch.dict(sys.modules, {"tf2_ros": fake_tf2}), patch.object(
        d, "_tf_zero_time", return_value=object()
    ):
        snapshot = d._build_readiness_snapshot(executor, bundle, _config(), attempt_dir)

    assert node.service_query_calls == len(RELEVANT_PROVIDER_NODES), (
        "readiness snapshot issued "
        f"{node.service_query_calls} server queries (nodes queried: "
        f"{sorted(set(node.queried_service_nodes))}); it must issue exactly one "
        "server query per unique relevant provider node, "
        f"i.e. {len(RELEVANT_PROVIDER_NODES)} — the per-endpoint full-graph rescan "
        "is the ~29.8 s block that exhausts the 30 s readiness deadline"
    )
    assert node.client_query_calls == 0, (
        f"readiness snapshot issued {node.client_query_calls} client inventory "
        "queries; endpoint validation needs only server names, so client queries "
        "must be zero"
    )
    assert set(node.queried_service_nodes) == set(RELEVANT_PROVIDER_NODES), (
        f"server inventory queried {sorted(set(node.queried_service_nodes))}, but "
        f"must query exactly the relevant provider nodes "
        f"{sorted(RELEVANT_PROVIDER_NODES)}"
    )
    assert not (set(node.queried_service_nodes) & set(NOISE_NODES)), (
        f"readiness snapshot queried irrelevant noise nodes "
        f"{sorted(set(node.queried_service_nodes) & set(NOISE_NODES))}; it must "
        "issue zero queries on nodes that host no required endpoint"
    )
    for name, expected_source in _REQUIRED_ENDPOINT_SOURCES.items():
        entry = snapshot["actions"].get(name) or snapshot["services"].get(name)
        assert entry is not None, f"required endpoint {name} missing from snapshot"
        assert entry["server_count"] == 1, (name, entry)
        assert entry["source_node"] == expected_source, (name, entry)


class _JournalGraphCountingNode:
    """Graph double counting journal server/client inventory queries per node."""

    def __init__(self) -> None:
        self.servers = {
            "move_group": ["/apply_planning_scene"],
            "move_group_private_123": ["/get_planning_scene"],
            "tinker_integrated_gate_executor": [],
            "noise": ["/unrelated"],
        }
        self.clients = {
            "move_group": [],
            "move_group_private_123": [],
            "tinker_integrated_gate_executor": [
                "/apply_planning_scene",
                "/get_planning_scene",
            ],
            "noise": ["/unrelated_client"],
        }
        self.server_queries: list[str] = []
        self.client_queries: list[str] = []

    def get_node_names_and_namespaces(self):
        return [(name, "/") for name in self.servers]

    def get_service_names_and_types_by_node(self, node_name, node_namespace):
        del node_namespace
        self.server_queries.append(node_name)
        return [(name, ["x"]) for name in self.servers[node_name]]

    def get_client_names_and_types_by_node(self, node_name, node_namespace):
        del node_namespace
        self.client_queries.append(node_name)
        return [(name, ["x"]) for name in self.clients[node_name]]


def test_observe_journal_graph_collects_service_inventory_once_per_node():
    """RED: the two journal services must share one full graph inventory pass.

    Live defect: ``_observe_journal_graph`` calls the per-service scanner twice;
    each call queries every node for servers and clients, repeating the slow full
    graph walk that previously exhausted readiness.  One shared inventory must
    query each unique node exactly once for servers and once for clients.
    """
    node = _JournalGraphCountingNode()
    executor = types.SimpleNamespace(node=node, _spin_once=lambda: None)

    with patch.object(d, "_publishers_for", return_value=([], [])), patch.object(
        d, "_subscribers_for", return_value=([], [])
    ):
        observed = d._observe_journal_graph(executor)

    expected_nodes = set(node.servers)
    assert len(node.server_queries) == len(expected_nodes), node.server_queries
    assert len(node.client_queries) == len(expected_nodes), node.client_queries
    assert set(node.server_queries) == expected_nodes
    assert set(node.client_queries) == expected_nodes
    assert observed["services"]["/get_planning_scene"]["servers"] == [
        {"node": "/move_group_private_123", "node_namespace": ""}
    ]
    assert observed["services"]["/get_planning_scene"]["clients"] == [
        {"node": "/tinker_integrated_gate_executor", "node_namespace": ""}
    ]


def test_build_journal_graph_projection_returns_raw_observation_without_prevalidation():
    """RED: the driver provider returns raw graph evidence for one executor validation.

    Pre-validating in the driver converts a concrete graph mismatch into ``None``
    via ``IntegratedGateExecutor._graph_observation`` and the terminal loses the
    real cause as ``observed graph evidence is unavailable``.
    """
    raw = {"raw_observed_graph": True}
    executor = object()
    with patch.object(d, "_observe_journal_graph", return_value=raw):
        assert d._build_journal_graph_projection(executor) is raw


# --------------------------------------------------------------------------- #
# RED regression (task #54) — attempt-local readiness timing trace
# --------------------------------------------------------------------------- #

#: Load-bearing synchronous readiness stages that must carry structured
#: start/end wall-clock timing records in the attempt-local
#: ``executor-readiness-trace.jsonl``.  Live evidence (2026-08-05 Stage C):
#: ``integrated_readiness`` warns every 0.2 s during readiness, then goes silent
#: for ~30 s exactly for the executor lifetime and resumes ~0.15 s after
#: ``execution-terminal.json`` — a single synchronous readiness stage consumes
#: the whole 30 s deadline with nothing today reporting which stage or the wall
#: time it took.  Each of these stages is a candidate for that block.
REQUIRED_READINESS_TRACE_STAGES: tuple[str, ...] = (
    "executor_spin",
    "controller_list_controllers",
    "service_inventory",
    "operator_publish_spin",
    "snapshot_build",
    "readiness_evaluate",
)

#: Recommended exact name for the persistent attempt-local readiness trace.
READINESS_TRACE_FILENAME = "executor-readiness-trace.jsonl"


class _TraceReadinessExecutor(_SnapshotExecutor):
    """Production-shaped readiness provider for the trace regression tests.

    Mirrors the live ``_construct_executor`` ``_readiness_snapshot_provider``:
    each evaluation calls the real ``_build_readiness_snapshot`` and stores the
    exact snapshot it produced (as ``_last_readiness_snapshot``) plus the exact
    readiness mapping it returned (as ``last_returned_mapping``) so a timeout
    trace can be verified against the true last rejected values.
    """

    def __init__(
        self,
        node,
        observer,
        *,
        bundle,
        config,
        attempt_dir,
        ready_after,
    ) -> None:
        super().__init__(node, observer)
        self._bundle = bundle
        self._config = config
        self._attempt_dir = attempt_dir
        self.ready_after = int(ready_after)
        self.iterations = 0
        self.last_returned_mapping: dict[str, object] | None = None
        self._last_readiness_snapshot: dict[str, object] | None = None
        # Distinct planning-scene ids + cache sequence so the timeout raw
        # summary is diagnosable (owned/attached ids and the cache sequence).
        # ``scene_sequence`` mirrors the real executor's normalized planning-
        # scene cache (``IntegratedGateExecutor._normalize_planning_scene``
        # emits ``scene_sequence``, never ``sequence``).
        self._latest_planning_scene = {
            "owned_ids": ["fixture"],
            "attached_ids": ["cup"],
            "scene_sequence": 3,
        }

    def _readiness(self) -> dict[str, object]:
        self.iterations += 1
        snapshot = d._build_readiness_snapshot(
            self, self._bundle, self._config, self._attempt_dir
        )
        self._last_readiness_snapshot = snapshot
        ready = self.iterations >= self.ready_after
        reasons: list[str] = (
            [] if ready else [f"iteration {self.iterations}: not ready yet"]
        )
        mapping: dict[str, object] = {"ready": ready, "reasons": reasons}
        self.last_returned_mapping = mapping
        return mapping


def _read_attempt_trace(attempt_dir: Path) -> list[dict[str, object]]:
    path = attempt_dir / READINESS_TRACE_FILENAME
    assert path.is_file(), f"readiness trace missing at {path}"
    records: list[dict[str, object]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        records.append(json.loads(line))
    assert records, "readiness trace is empty"
    return records


def _trace_stage_records(
    records: list[dict[str, object]], stage: str
) -> list[dict[str, object]]:
    return [
        record
        for record in records
        if record.get("event") == "stage" and record.get("stage") == stage
    ]


def _assert_stage_timing_record(record: dict[str, object], stage: str) -> None:
    assert record["event"] == "stage"
    assert record["stage"] == stage
    start = record["start_mono_s"]
    end = record["end_mono_s"]
    elapsed = record["elapsed_s"]
    assert isinstance(start, float) and isinstance(end, float)
    assert isinstance(elapsed, float)
    assert start <= end, f"{stage} start {start} must precede end {end}"
    assert elapsed >= 0.0, f"{stage} elapsed {elapsed} must be non-negative"


def test_readiness_trace_records_stage_timing_to_attempt_local_jsonl(tmp_path):
    """RED: a readiness run must emit structured start/end wall-clock timing
    records for every load-bearing synchronous stage to a persistent
    attempt-local JSONL.

    Live evidence: the Stage-C executor's ``integrated_readiness`` warns every
    0.2 s, then is silent for ~30 s exactly for the executor lifetime and resumes
    ~0.15 s after ``execution-terminal.json`` — the synchronous executor
    readiness consumes the entire 30 s deadline inside one stage, but nothing
    today records which stage or the wall time it took.  This test drives the
    real ``_wait_for_readiness`` (with the desired optional ``diagnostics_path``
    API) against a production-shaped readiness provider and requires the trace
    file to carry start/end/elapsed records for the executor spin, controller
    ListControllers, provider-only service inventory, operator publish/spin, and
    snapshot build/evaluation stages.  The record must be a persistent
    attempt-local JSONL, not only logger/capsys output.
    """
    attempt_dir = tmp_path / "attempt-1"
    attempt_dir.mkdir()
    (attempt_dir / "scenario-runner.json").write_text("{}", encoding="utf-8")
    trace_path = attempt_dir / READINESS_TRACE_FILENAME

    node = _GraphCountingNode(_readiness_fake_graph())
    observer = _SnapshotObserver(node)
    bundle = _bundle()
    bundle["attempt_dir"] = str(attempt_dir)
    config = _config()
    executor = _TraceReadinessExecutor(
        node,
        observer,
        bundle=bundle,
        config=config,
        attempt_dir=attempt_dir,
        ready_after=2,
    )

    fake_tf2 = types.ModuleType("tf2_ros")
    fake_tf2.LookupException = _LookupException
    fake_tf2.ConnectivityException = _LookupException
    fake_tf2.ExtrapolationException = _LookupException
    with patch.dict(sys.modules, {"tf2_ros": fake_tf2}), patch.object(
        d, "_tf_zero_time", return_value=object()
    ):
        result = d._wait_for_readiness(
            executor, timeout_s=5.0, diagnostics_path=trace_path
        )

    assert result["ready"] is True  # return semantics unchanged
    records = _read_attempt_trace(attempt_dir)
    for stage in REQUIRED_READINESS_TRACE_STAGES:
        matches = _trace_stage_records(records, stage)
        assert matches, f"readiness trace has no {stage!r} timing record"
        for record in matches:
            _assert_stage_timing_record(record, stage)


def test_readiness_timeout_trace_records_terminal_event_with_last_snapshot(tmp_path):
    """RED: on readiness timeout the attempt-local JSONL must record a terminal
    timeout event carrying the exact last readiness mapping/reasons and a compact
    JSON-safe raw-value summary, without changing the driver's raise semantics.

    Live evidence: the executor silently burns the full 30 s readiness deadline
    and then reports only the readiness reasons — the exact rejected joint
    names/positions/velocities/stamp/age/source, controller records, FJT action
    count/source, and PlanningScene owned/attached ids + cache sequence are lost.
    This test drives the real ``run_driver`` with a never-ready production-shaped
    executor, asserts the unchanged ``DriverError`` raise, and requires the
    terminal timeout event in the attempt-local trace: the exact last readiness
    mapping/reasons plus a compact raw summary of the last rejected snapshot, all
    JSON-safe primitives.
    """
    attempt_dir = tmp_path / "attempt-1"
    attempt_dir.mkdir()
    (attempt_dir / "scenario-runner.json").write_text("{}", encoding="utf-8")

    node = _GraphCountingNode(_readiness_fake_graph())
    observer = _SnapshotObserver(node)
    bundle = _bundle()
    bundle["attempt_dir"] = str(attempt_dir)
    config = _config()

    holder: dict[str, _TraceReadinessExecutor] = {}

    def factory(*, bundle, attempt_dir, config, domain_id, seed):
        del domain_id, seed
        executor = _TraceReadinessExecutor(
            node,
            observer,
            bundle=bundle,
            config=config,
            attempt_dir=Path(attempt_dir),
            ready_after=10**9,  # never ready -> timeout
        )
        holder["executor"] = executor
        return executor

    fake_tf2 = types.ModuleType("tf2_ros")
    fake_tf2.LookupException = _LookupException
    fake_tf2.ConnectivityException = _LookupException
    fake_tf2.ExtrapolationException = _LookupException
    with patch.dict(sys.modules, {"tf2_ros": fake_tf2}), patch.object(
        d, "_tf_zero_time", return_value=object()
    ):
        with pytest.raises(d.DriverError, match="did not become ready"):
            d.run_driver(
                bundle=bundle,
                attempt_dir=attempt_dir,
                config=config,
                domain_id=100,
                seed=7,
                executor_factory=factory,
                runtime_provider_factory=lambda **kwargs: {},
                readiness_timeout_s=0.2,
            )

    executor = holder["executor"]
    assert executor.iterations >= 1, "readiness must have been polled before timeout"
    last = executor.last_returned_mapping
    assert last is not None and last["ready"] is False

    records = _read_attempt_trace(attempt_dir)
    timeout_events = [
        record for record in records if record.get("event") == "readiness_timeout"
    ]
    assert timeout_events, "readiness trace has no terminal timeout event"
    event = timeout_events[-1]
    assert event["last_ready"] == last["ready"]
    assert event["last_reasons"] == last["reasons"]

    summary = event["raw_summary"]
    assert summary["joint_names"] == list(d._REQUIRED_JOINT_NAMES)
    assert summary["joint_positions"] == [0.0] * len(d._REQUIRED_JOINT_NAMES)
    assert summary["joint_velocities"] == [0.0] * len(d._REQUIRED_JOINT_NAMES)
    assert summary["joint_header_stamp_ns"] == 1_000_000_000
    assert isinstance(summary["joint_age_s"], float) and summary["joint_age_s"] >= 0.0
    assert summary["joint_source"] == ""
    assert summary["controller_manager_healthy"] is True
    logical = summary["logical_controllers"]
    assert logical["joint_state_broadcaster"]["state"] == "active"
    assert logical["xarm7_traj_controller"]["state"] == "active"
    assert summary["fjt_action_count"] == 1
    assert summary["fjt_source"] == "/controller_manager"
    assert summary["planning_scene_owned_ids"] == ["fixture"]
    assert summary["planning_scene_attached_ids"] == ["cup"]
    assert summary["planning_scene_sequence"] == 3


class _ExoticRawSnapshotExecutor:
    """Tiny not-ready executor whose last raw snapshot is uncoercible.

    The raw-value summary is diagnostic-only; best-effort timeout diagnostics
    must never change ``_wait_for_readiness`` return/raise semantics.  When the
    last rejected snapshot contains a value that cannot be coerced to a
    JSON-safe primitive (here an ``object()`` in the joint positions, which
    ``float()`` cannot convert), the terminal ``readiness_timeout`` event must
    degrade to ``raw_summary={}`` and the wait must still return the same
    not-ready mapping instead of raising ``TypeError``/``ValueError``.
    """

    def __init__(self) -> None:
        self._last_readiness_snapshot: dict[str, object] = {
            "joint_state": {
                "names": ["joint1"],
                "positions": [object()],  # uncoercible -> float() raises
                "velocities": [0.0],
                "header_stamp_ns": 0,
                "age_s": 0.0,
                "source_node": "",
            },
            "controllers": {},
            "actions": {},
            "planning_scene": {},
        }

    def _spin_once(self) -> None:
        pass

    def _readiness(self) -> dict[str, object]:
        return {"ready": False, "reasons": ["exotic raw snapshot never ready"]}


def test_readiness_timeout_exotic_raw_summary_never_raises_and_degrades(tmp_path):
    """RED: best-effort timeout diagnostics cannot change ``_wait_for_readiness``
    return semantics when ``_readiness_raw_summary`` hits an uncoercible raw
    snapshot value.

    The raw summary is built only for the terminal ``readiness_timeout`` trace
    event; an exotic/uncoercible value in the last rejected snapshot (an
    ``object()`` in the joint positions, which ``float()`` cannot convert) must
    never raise out of the wait.  Desired behavior: return the same not-ready
    mapping and write a terminal event with ``raw_summary={}`` (safe
    degradation).  Current production evaluates ``_readiness_raw_summary`` while
    constructing the record argument, *before* the best-effort write boundary,
    so it raises ``TypeError``/``ValueError`` during raw-summary construction —
    this test is RED.
    """
    attempt_dir = tmp_path / "attempt-1"
    attempt_dir.mkdir()
    trace_path = attempt_dir / READINESS_TRACE_FILENAME

    executor = _ExoticRawSnapshotExecutor()
    result = d._wait_for_readiness(
        executor, timeout_s=0.2, diagnostics_path=trace_path
    )
    assert result == {
        "ready": False,
        "reasons": ["exotic raw snapshot never ready"],
    }

    records = _read_attempt_trace(attempt_dir)
    timeout_events = [
        record for record in records if record.get("event") == "readiness_timeout"
    ]
    assert timeout_events, "readiness trace has no terminal timeout event"
    event = timeout_events[-1]
    assert event["last_ready"] is False
    assert event["last_reasons"] == ["exotic raw snapshot never ready"]
    assert event["raw_summary"] == {}, (
        "uncoercible raw snapshot values must degrade the summary to {} rather "
        "than raise out of the wait"
    )


# --------------------------------------------------------------------------- #
# RED regressions (task #58) — late-observer persistent clients
# --------------------------------------------------------------------------- #

class _FakeListControllersFuture:
    """Controllable async future for the ListControllers client double."""

    def __init__(self) -> None:
        self._done = False
        self._result = None

    def done(self) -> bool:
        return self._done

    def result(self) -> Any:
        return self._result

    def complete(self, controller_records: list[object]) -> None:
        self._result = types.SimpleNamespace(controller=list(controller_records))
        self._done = True


class _FakeListControllersClient:
    """ServiceClient double recording every ``call_async``."""

    def __init__(self) -> None:
        self.srv_type = types.SimpleNamespace(Request=lambda: types.SimpleNamespace())
        self.call_async_count = 0
        self.future = _FakeListControllersFuture()
        self.requests: list[object] = []

    def service_is_ready(self) -> bool:
        return True

    def call_async(self, request: object) -> _FakeListControllersFuture:
        self.call_async_count += 1
        self.requests.append(request)
        return self.future


class _MinimalSpinExecutor:
    """Executor double exposing only the spinner the ListControllers path drives."""

    def __init__(self) -> None:
        self.spins = 0

    def _spin_once(self) -> None:
        self.spins += 1


def test_list_controllers_is_persistent_and_nonblocking():
    """RED: ``_LiveProviderObserver.list_controllers`` must be persistent and
    nonblocking.

    Live defect: readiness calls ``list_controllers`` once per snapshot; the
    current implementation abandons/restarts the async request on every call and
    bounds a synchronous spinner wait (``_call_service_with_spinner``) per call.
    Desired: one persistent async ``call_async`` starts the request and returns
    ``None`` without a bounded spin wait while pending; a concurrent call reuses
    the in-flight future; when that same future completes with two active
    records the next call returns them and later calls reuse the cache.  The
    controller ``call_async`` count must remain exactly 1.
    """
    observer = object.__new__(d._LiveProviderObserver)
    client = _FakeListControllersClient()
    observer._controllers_client = client
    observer._executor = _MinimalSpinExecutor()

    with patch.object(d, "_CONTROLLER_QUERY_TIMEOUT_S", 0.01):  # deterministic, no real 1 s wait
        # First call starts one request and returns None while the future is pending.
        assert observer.list_controllers() is None
        assert client.call_async_count == 1
        # A second call while pending must not start a second request.
        assert observer.list_controllers() is None
        assert client.call_async_count == 1, (
            "list_controllers restarted the pending ListControllers request: "
            f"call_async count {client.call_async_count}, expected 1"
        )
        # Completing the same future with two active records is served by the
        # next call, and later calls reuse the cached records.
        client.future.complete(
            [
                types.SimpleNamespace(name="joint_state_broadcaster", state="active"),
                types.SimpleNamespace(name="xarm7_traj_controller", state="active"),
            ]
        )
        records = observer.list_controllers()
        assert [record.name for record in records] == [
            "joint_state_broadcaster",
            "xarm7_traj_controller",
        ]
        assert [record.name for record in observer.list_controllers()] == [
            "joint_state_broadcaster",
            "xarm7_traj_controller",
        ]
        assert client.call_async_count == 1


def test_list_controllers_times_out_stale_pending_future_and_retries():
    """RED: a pending ``/controller_manager/list_controllers`` future that never
    completes (ros2_control_node's cold start DROPS the response server-side,
    RMW ``failed to send response ... timeout``) must be discarded after a
    bounded TTL and re-issued on the same call, instead of stalling the
    ``integrated_readiness`` gate with ``controller_manager_healthy=False`` and
    ``logical_controllers={}`` for the whole run.

    Live defect: ``list_controllers`` keeps a single in-flight future with no
    timeout and no retry.  When the cold-start response is dropped the future
    never completes, the cache stays ``None`` forever, and readiness times out.
    Desired: the stale pending future is discarded after
    ``_CONTROLLERS_LIST_TIMEOUT_S`` and the same call re-issues a fresh request
    on the same client; completing that fresh request seeds the cache with the
    two active logical controllers, preserving the single-in-flight invariant.
    """
    observer = object.__new__(d._LiveProviderObserver)
    client = _FakeListControllersClient()
    observer._controllers_client = client
    observer._executor = _MinimalSpinExecutor()

    # Step 1: request R1 issued and left pending (cold-start drop simulated).
    assert observer.list_controllers() is None
    assert client.call_async_count == 1

    # Step 2: advance past the controllers TTL (mirrors the production
    # ``_CONTROLLERS_LIST_TIMEOUT_S = 5.0``; a 6.0 s advance is safely past
    # it).  The stale pending future must be discarded and a fresh request
    # re-issued in the same call (never awaited forever, never left to stall
    # the readiness deadline).
    with patch.object(
        d.time,
        "monotonic",
        return_value=d.time.monotonic() + 6.0,
    ):
        assert observer.list_controllers() is None
    assert client.call_async_count == 2, (
        "stale pending ListControllers future must be re-issued after the "
        f"controllers TTL; call_async count {client.call_async_count}, expected 2"
    )
    assert getattr(observer, "_controllers_future", None) is not None
    assert getattr(observer, "_controllers_future_started_mono", None) is not None

    # Step 3: complete R2 with two active records; the next call serves them
    # and later calls reuse the cache (single in-flight invariant preserved).
    client.future.complete(
        [
            types.SimpleNamespace(name="joint_state_broadcaster", state="active"),
            types.SimpleNamespace(name="xarm7_traj_controller", state="active"),
        ]
    )
    records = observer.list_controllers()
    assert [record.name for record in records] == [
        "joint_state_broadcaster",
        "xarm7_traj_controller",
    ]
    assert client.call_async_count == 2
    assert [record.name for record in observer.list_controllers()] == [
        "joint_state_broadcaster",
        "xarm7_traj_controller",
    ]


def test_controller_list_client_reuses_executor_owned_service_client():
    """RED: the controller query client must come from the executor-owned
    service-client map, never a newly created client on the late observer node.

    Live defect: the readiness ListControllers query drives a client created on
    the driver's late observer node rather than reusing the service client the
    IntegratedGateExecutor already owns at
    ``executor._service_clients['/controller_manager/list_controllers']``.
    Desired seam: ``d._controller_list_client(executor)`` returns exactly that
    executor-owned client by identity.  The seam does not exist yet, so this
    test is RED (AttributeError on the missing ``_controller_list_client``).
    """
    sentinel = object()
    executor = types.SimpleNamespace(
        _service_clients={"/controller_manager/list_controllers": sentinel}
    )
    assert d._controller_list_client(executor) is sentinel


class _FakeReadinessDrainSpinner:
    """Spinner double recording every ``spin_once`` and its timeout."""

    def __init__(self) -> None:
        self.spin_once_count = 0
        self.drain_timeouts: list[float] = []

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        self.spin_once_count += 1
        self.drain_timeouts.append(timeout_sec)


class _FakeReadinessDrainExecutor:
    """Executor double for the readiness callback drain seam.

    Records one initial ``_spin_once`` (the executor's own event loop getting a
    chance to dispatch already-queued readiness callbacks) plus the bounded,
    nonblocking ``_spinner.spin_once`` drain batch that follows.
    """

    def __init__(self) -> None:
        self.spinner = _FakeReadinessDrainSpinner()
        self.executor_spin_count = 0
        self._spinner = self.spinner

    def _spin_once(self) -> None:
        self.executor_spin_count += 1


def test_spin_readiness_callbacks_drains_bounded_nonblocking_batch():
    """RED: readiness callback draining is one initial spin plus a bounded
    nonblocking batch.

    Desired seam ``d._spin_readiness_callbacks(executor)``: exactly one
    ``executor._spin_once()`` lets the executor dispatch its already-queued
    readiness callbacks, then ``d._READINESS_DRAIN_SPINS`` additional
    ``executor._spinner.spin_once(timeout_sec=0.0)`` calls drain a continuous
    callback backlog without blocking.  Every drain spin must be nonblocking,
    and the batch must be at least 32 spins — meaningfully larger than the
    observed continuous callback backlog.  The seam and the constant do not
    exist yet, so this test is RED (AttributeError).
    """
    executor = _FakeReadinessDrainExecutor()

    d._spin_readiness_callbacks(executor)

    assert executor.executor_spin_count == 1, (
        "initial executor._spin_once must run exactly once: "
        f"executor_spin_count {executor.executor_spin_count}"
    )
    assert executor.spinner.spin_once_count == d._READINESS_DRAIN_SPINS, (
        "spinner drain batch must equal _READINESS_DRAIN_SPINS: "
        f"drained {executor.spinner.spin_once_count}, "
        f"constant {d._READINESS_DRAIN_SPINS}"
    )
    assert d._READINESS_DRAIN_SPINS >= 32, (
        "drain batch must be meaningfully larger than the observed continuous "
        f"callback backlog: _READINESS_DRAIN_SPINS {d._READINESS_DRAIN_SPINS}"
    )
    assert all(
        timeout == 0.0 for timeout in executor.spinner.drain_timeouts
    ), (
        "every additional drain spin must be nonblocking (timeout_sec=0.0): "
        f"drain_timeouts {executor.spinner.drain_timeouts}"
    )


class _FakeGetPlanningSceneFuture:
    """Controllable async future for the ``/get_planning_scene`` client double."""

    def __init__(self) -> None:
        self._done = False
        self._response = None

    def done(self) -> bool:
        return self._done

    def result(self) -> Any:
        return self._response

    def complete(self, response: Any) -> None:
        self._response = response
        self._done = True


class _FakePlanningSceneClient:
    """ServiceClient double with the MoveIt ``GetPlanningScene`` request shape.

    Humble ``GetPlanningScene.Request().components`` is a
    ``PlanningSceneComponents`` submessage with an integer field ``.components``;
    assigning ``request.components = 1`` raises ``AssertionError``.  The fake
    mirrors that shape: ``request.components.components`` is the integer mask.
    """

    def __init__(self) -> None:
        self.srv_type = types.SimpleNamespace(
            Request=lambda: types.SimpleNamespace(
                components=types.SimpleNamespace(components=0)
            )
        )
        self.call_async_count = 0
        self.requests: list[object] = []
        self.future = _FakeGetPlanningSceneFuture()

    def service_is_ready(self) -> bool:
        return True

    def call_async(self, request: object) -> _FakeGetPlanningSceneFuture:
        self.call_async_count += 1
        self.requests.append(request)
        return self.future


class _LatePlanningSceneExecutor:
    """Executor double for the planning-scene readback seam."""

    def __init__(self, client: _FakePlanningSceneClient) -> None:
        self._planning_scene_client = client
        self._latest_planning_scene: dict[str, object] | None = None

    def _normalize_planning_scene(self, message: Any, *, source: str) -> dict[str, object]:
        assert source == "/get_planning_scene"
        owned = [str(obj.id) for obj in message.world.collision_objects]
        attached = [
            str(obj.object.id) for obj in message.robot_state.attached_collision_objects
        ]
        return {
            "owned_ids": owned,
            "attached_ids": attached,
            "scene_sequence": 1,
        }

    def _spin_once(self) -> None:
        pass


def test_step_planning_scene_readback_seeds_late_planning_scene(tmp_path):
    """RED: the driver must expose a ``_step_planning_scene_readback(executor)``
    seam that seeds the executor's planning-scene cache from a late
    ``/get_planning_scene`` response.

    Live defect: a PlanningScene that arrives late (after readiness begins) is
    never seeded, so ``_build_readiness_snapshot`` reads an empty/None cache.
    Desired: the first step starts exactly one async request with a nonzero
    ``components`` mask; while the single persistent future is pending no second
    request starts; once it completes with ``response.scene`` the next step
    normalizes via ``executor._normalize_planning_scene(source='/get_planning_scene')``
    and stores the full owned ids in ``_latest_planning_scene``.  The real
    ``_build_readiness_snapshot`` invokes the step before reading the cache.
    """
    client = _FakePlanningSceneClient()
    executor = _LatePlanningSceneExecutor(client)

    # The seam does not exist yet in production (RED: AttributeError).
    d._step_planning_scene_readback(executor)
    assert client.call_async_count == 1
    assert client.requests[0].components.components != 0, (
        "readback must request full scene components via the "
        "PlanningSceneComponents submessage mask"
    )
    assert executor._latest_planning_scene is None, "future still pending, nothing seeded"

    scene = types.SimpleNamespace(
        world=types.SimpleNamespace(
            collision_objects=[
                types.SimpleNamespace(id="obj_a"),
                types.SimpleNamespace(id="obj_b"),
            ]
        ),
        robot_state=types.SimpleNamespace(attached_collision_objects=[]),
    )
    client.future.complete(types.SimpleNamespace(scene=scene))
    d._step_planning_scene_readback(executor)
    assert client.call_async_count == 1, "second step must reuse the same future"
    assert executor._latest_planning_scene["owned_ids"] == ["obj_a", "obj_b"]

    # The real snapshot build must invoke the step before reading the cache.
    attempt_dir = tmp_path / "attempt-1"
    attempt_dir.mkdir()
    (attempt_dir / "scenario-runner.json").write_text("{}", encoding="utf-8")
    node = _GraphCountingNode(_readiness_fake_graph())
    observer = _SnapshotObserver(node)
    snapshot_executor = _SnapshotExecutor(node, observer)
    bundle = _bundle()
    bundle["attempt_dir"] = str(attempt_dir)
    fake_tf2 = types.ModuleType("tf2_ros")
    fake_tf2.LookupException = _LookupException
    fake_tf2.ConnectivityException = _LookupException
    fake_tf2.ExtrapolationException = _LookupException
    with patch.dict(sys.modules, {"tf2_ros": fake_tf2}), patch.object(
        d, "_tf_zero_time", return_value=object()
    ), patch.object(d, "_step_planning_scene_readback", create=True) as step_mock:
        d._build_readiness_snapshot(snapshot_executor, bundle, _config(), attempt_dir)
    step_mock.assert_called_with(snapshot_executor)


def test_step_planning_scene_readback_rearms_after_late_topic_clobber():
    """RED: after a first successful full response latches
    ``_ps_readback_done=True``, a later topic clobber that empties the cache
    world must rearm and start exactly one new ``/get_planning_scene`` async
    request instead of returning permanently due the done latch.

    Live defect: the fixture subscription can emit a full scene once, then a
    later diff/empty scene clobbers ``_latest_planning_scene`` (empty owned ids).
    The readback must notice the world fields went empty and re-query, not stay
    latched forever.  Current code returns immediately when
    ``_ps_readback_done`` is True, so ``call_async`` count stays 1.
    """
    client = _FakePlanningSceneClient()
    executor = _LatePlanningSceneExecutor(client)

    # First full response seeds the cache and latches done.
    d._step_planning_scene_readback(executor)
    assert client.call_async_count == 1
    scene = types.SimpleNamespace(
        world=types.SimpleNamespace(
            collision_objects=[types.SimpleNamespace(id="obj_a")]
        ),
        robot_state=types.SimpleNamespace(attached_collision_objects=[]),
    )
    client.future.complete(types.SimpleNamespace(scene=scene))
    d._step_planning_scene_readback(executor)
    assert executor._latest_planning_scene["owned_ids"] == ["obj_a"]
    assert client.call_async_count == 1
    assert getattr(executor, "_ps_readback_done", False) is True

    # A later topic clobber empties the world fields of the cache.
    executor._latest_planning_scene = {
        "owned_ids": [],
        "attached_ids": [],
        "scene_sequence": 2,
        "source": "/sim/status/planning_scene_fixture",
        "fixture_geometry_digest": "",
        "fixture_geometry": [],
    }

    # The next step must rearm: start a new request, not return due the latch.
    client.future = _FakeGetPlanningSceneFuture()  # fresh pending future
    d._step_planning_scene_readback(executor)
    assert client.call_async_count == 2, (
        "readback returned permanently due the done latch after a topic "
        f"clobber; call_async count {client.call_async_count}, expected 2"
    )


def test_step_planning_scene_readback_times_out_stale_pending_future_and_retries():
    """RED: a pending ``/get_planning_scene`` future that never completes (the
    move_group cold-start RMW drops the response) must be discarded after a
    bounded TTL and re-issued on the next snapshot tick, instead of stalling the
    readiness readback with ``planning_scene_owned_ids=[]`` forever.

    Live defect: ``_step_planning_scene_readback`` starts one async request and,
    while the single future is pending, returns without any timeout.  When
    move_group's cold start drops the response the future never completes and
    the readback is stalled for the whole 30s readiness deadline.  Desired: the
    stale future is discarded after ``_PLANNING_SCENE_READBACK_TIMEOUT_S``, the
    next tick issues a fresh request on the same client, and completing that
    response seeds ``_latest_planning_scene["owned_ids"]``.
    """
    client = _FakePlanningSceneClient()
    executor = _LatePlanningSceneExecutor(client)

    # Step 1: request R1 issued and left pending.
    d._step_planning_scene_readback(executor)
    assert client.call_async_count == 1
    assert getattr(executor, "_ps_readback_started_mono", None) is not None

    # Step 2: advance past the readback TTL.  The stale pending future must be
    # discarded (not awaited forever); the discard tick does not re-issue.
    with patch.object(
        d.time,
        "monotonic",
        return_value=d.time.monotonic() + d._PLANNING_SCENE_READBACK_TIMEOUT_S + 1.0,
    ):
        d._step_planning_scene_readback(executor)
    assert getattr(executor, "_ps_readback_future", None) is None, (
        "stale pending future must be discarded after the readback TTL"
    )
    assert client.call_async_count == 1, "discard tick must not re-issue in the same tick"

    # Step 3: the next snapshot tick re-issues a fresh request R2.
    d._step_planning_scene_readback(executor)
    assert client.call_async_count == 2, (
        "readback must re-issue a fresh request after discarding the stale "
        f"future; call_async count {client.call_async_count}, expected 2"
    )

    # Complete R2 with a scene; the following step seeds the cache.
    scene = types.SimpleNamespace(
        world=types.SimpleNamespace(
            collision_objects=[
                types.SimpleNamespace(id="obj_a"),
                types.SimpleNamespace(id="obj_b"),
            ]
        ),
        robot_state=types.SimpleNamespace(attached_collision_objects=[]),
    )
    client.future.complete(types.SimpleNamespace(scene=scene))
    d._step_planning_scene_readback(executor)
    assert executor._latest_planning_scene["owned_ids"] == ["obj_a", "obj_b"]


def _fjt_raw_controller_graph() -> dict[str, list[str]]:
    """Readiness graph hosting the FJT send_goal on the raw controller-resource
    node ``xarm7_traj_controller`` instead of ``controller_manager``."""
    graph = _readiness_fake_graph()
    graph["controller_manager"] = [
        service
        for service in graph["controller_manager"]
        if service != "/xarm7_traj_controller/follow_joint_trajectory/_action/send_goal"
    ]
    graph["xarm7_traj_controller"] = [
        "/xarm7_traj_controller/follow_joint_trajectory/_action/send_goal"
    ]
    return graph


def test_fjt_raw_controller_resource_host_counts_as_controller_manager():
    """RED: FJT served by the raw controller-resource node must count as one
    server with logical source ``/controller_manager``.

    Live defect: the controller-manager hosts the FJT action through the raw
    ``xarm7_traj_controller`` resource node; the provider-only inventory never
    queries that node, so the readiness snapshot reports ``server_count=0`` and
    ``source_node=''`` even though ``ListControllers`` reports the controller
    active.  Desired: the snapshot reports ``server_count=1`` and logical
    ``source_node='/controller_manager'``; provider-only inventory may query this
    one additional controller-resource node but still issues zero client queries
    and never touches unrelated noise nodes.
    """
    node = _GraphCountingNode(_fjt_raw_controller_graph())
    observer = _SnapshotObserver(node)
    executor = _SnapshotExecutor(node, observer)
    attempt_dir = Path(_tmp())
    (attempt_dir / "scenario-runner.json").write_text("{}", encoding="utf-8")
    bundle = _bundle()
    bundle["attempt_dir"] = str(attempt_dir)

    fake_tf2 = types.ModuleType("tf2_ros")
    fake_tf2.LookupException = _LookupException
    fake_tf2.ConnectivityException = _LookupException
    fake_tf2.ExtrapolationException = _LookupException
    with patch.dict(sys.modules, {"tf2_ros": fake_tf2}), patch.object(
        d, "_tf_zero_time", return_value=object()
    ):
        snapshot = d._build_readiness_snapshot(executor, bundle, _config(), attempt_dir)

    fjt = snapshot["actions"]["/xarm7_traj_controller/follow_joint_trajectory"]
    assert fjt["server_count"] == 1, (
        "FJT hosted by the raw controller-resource node xarm7_traj_controller "
        f"reports server_count={fjt['server_count']}; it must be 1"
    )
    assert fjt["source_node"] == "/controller_manager", (
        "FJT hosted by the raw controller-resource node must resolve to the "
        f"logical /controller_manager source, got {fjt['source_node']!r}"
    )
    assert node.client_query_calls == 0, (
        "provider-only inventory issued client queries for the controller-resource graph"
    )
    assert not (set(node.queried_service_nodes) & set(NOISE_NODES)), (
        "provider-only inventory queried an unrelated noise node"
    )
    assert "xarm7_traj_controller" in set(node.queried_service_nodes), (
        "provider-only inventory never queried the raw controller-resource node; "
        f"queried {sorted(set(node.queried_service_nodes))}"
    )


# --------------------------------------------------------------------------- #
# F6 — pre-send long-motion plan goal must pin the current joint state.
#
# The Stage-D cancel/safety driver's ``_presend_long_motion`` built the plan-only
# joint MoveGroup goal WITHOUT ``start_state``, so move_group observed an empty
# JointState ("Found empty JointState message") and could not sample the goal
# tree — the plan failed and the executor raised "ExecuteTrajectory goal requires
# a non-empty planned trajectory".  The Stage-C empty-JointState fix passed
# ``start_state`` in ``_build_d_goal``; the presend path must do the same.
# --------------------------------------------------------------------------- #

def test_f6_presend_long_motion_pins_current_joint_state():
    """RED (F6): ``_presend_long_motion`` must build the long-motion plan-only
    goal with ``start_state=executor._latest_joint_state`` (never an empty
    JointState) so move_group can sample the goal tree."""
    import integrated_gate_executor as ie

    captured: dict[str, object] = {}

    def _spy_joint_goal(target, *, plan_only=True, start_state=None, **kwargs):
        captured["target"] = tuple(target)
        captured["plan_only"] = plan_only
        captured["start_state"] = start_state
        return object()  # fake MoveGroup goal

    def _spy_execute_goal(planned_trajectory):
        return object()  # fake ExecuteTrajectory goal

    scenario = {
        "id": "qualification-moveit-safety",
        "seed": 7,
        "integrated": {
            "stage": "D",
            "execution_profile": "sim_ompl",
            "acceptance": {"polarity": "safety"},
            "expected_physical": [
                "safety_effective_stop", "target_frozen", "no_auto_resume",
            ],
            "forbidden_endpoints": ["/isaac_joint_commands"],
        },
    }

    class _F6Executor:
        def __init__(self):
            self.scenario = scenario
            self._latest_joint_state = {
                "joint_positions": [0.0] * 7,
                "header": {"stamp": {"sec": 1}},
            }
            self._presend_execute_goal_uuid = None

        def _send_plan_only_retaining_handle(self, scenario_id, goal, spec):
            return {
                "status": "diagnostic-pass",
                "planning_goal_id": "a" * 32,
                "planned_trajectory": object(),
                "goal_handle": object(),
            }

        def _d_baseline(self):
            return {}

        def _spin_once(self):
            return None

        def _threshold_timeout(self, key, default):
            return 0.1

        def _wait_for_fjt_status(self, goal_id, statuses, timeout_s, baseline=None):
            return {"goal_uuid": "c" * 32}

    executor = _F6Executor()
    with patch.object(ie, "build_joint_move_group_goal", _spy_joint_goal), patch.object(
        ie, "build_execute_trajectory_goal", _spy_execute_goal
    ), patch.object(
        d, "_send_execute_retaining_handle", return_value=("b" * 32, object())
    ):
        d._presend_long_motion(executor, "qualification-moveit-safety")

    assert captured["plan_only"] is True
    assert tuple(captured["target"]) == tuple(d._LONG_MOTION_JOINT_TARGET)
    assert captured["start_state"] == executor._latest_joint_state, (
        "presend long-motion plan goal must carry the executor's current joint "
        f"state (got {captured['start_state']!r})"
    )


def test_c_presend_long_motion_slows_the_execute_trajectory():
    """RED (C/C2): the cancel/safety pre-send long motion must apply the
    ``apply_execution_slowdown`` factor to the planned trajectory BEFORE building
    the ExecuteTrajectory goal, exactly like ``run_execute_sequence``.  C2
    (rerun-5): at k=2.0 the presend joint motion completed in ~2.3 s — exactly
    the time the cancel arbitration (FJT-executing join + motion trigger) took
    to confirm the in-flight motion — so ``run_cancel_sequence`` issued
    ``cancel_goal_async`` against an already-SUCCEEDED goal and got
    ``ERROR_GOAL_ALREADY_TERMINATED`` (return_code 3, cancel_response
    "rejected"), so the cancel evidence never reaches ``quiescent``.  The
    presend is a synthetic test fixture (not production motion), so it is slowed
    harder (k>=4.0 -> ~4.6 s) to keep the goal EXECUTING for the entire
    cancel-setup + FJT-discovery + arbitration window with margin."""
    import integrated_gate_executor as ie

    slowed = []

    def _fake_slowdown(trajectory, k: float = 2.0):
        slowed.append((trajectory, k))
        return trajectory

    def _spy_joint_goal(target, *, plan_only=True, start_state=None, **kwargs):
        return object()

    captured_execute = {}

    def _spy_execute_goal(planned_trajectory):
        captured_execute["trajectory"] = planned_trajectory
        return object()

    scenario = {
        "id": "qualification-moveit-cancel",
        "seed": 7,
        "integrated": {
            "stage": "D",
            "execution_profile": "sim_ompl",
            "acceptance": {"polarity": "cancel"},
            "expected_physical": [
                "execute_goal_canceled", "quiescent_after_cancel", "no_later_stage",
            ],
            "forbidden_endpoints": ["/isaac_joint_commands"],
        },
    }

    class _CExecutor:
        def __init__(self):
            self.scenario = scenario
            self._latest_joint_state = {
                "joint_positions": [0.0] * 7,
                "header": {"stamp": {"sec": 1}},
            }
            self._presend_execute_goal_uuid = None

        def _send_plan_only_retaining_handle(self, scenario_id, goal, spec):
            planned = types.SimpleNamespace()
            planned._is_the_presend_plan = True
            return {
                "status": "diagnostic-pass",
                "planning_goal_id": "a" * 32,
                "planned_trajectory": planned,
                "goal_handle": object(),
            }

        def _d_baseline(self):
            return {}

        def _spin_once(self):
            return None

        def _threshold_timeout(self, key, default):
            return 0.1

        def _wait_for_fjt_status(self, goal_id, statuses, timeout_s, baseline=None):
            return {"goal_uuid": "c" * 32}

    executor = _CExecutor()
    with patch.object(ie, "build_joint_move_group_goal", _spy_joint_goal), patch.object(
        ie, "build_execute_trajectory_goal", _spy_execute_goal
    ), patch.object(ie, "apply_execution_slowdown", _fake_slowdown), patch.object(
        d, "_send_execute_retaining_handle", return_value=("b" * 32, object())
    ):
        d._presend_long_motion(executor, "qualification-moveit-cancel")

    assert slowed, "apply_execution_slowdown was never called on the presend plan"
    trajectory, k = slowed[0]
    assert getattr(trajectory, "_is_the_presend_plan", False) is True
    # C2 (rerun-5): cancelled a SUCCEEDED goal — the long motion completed in
    # ~2.3 s at k=2.0, so by the time run_cancel_sequence issued
    # cancel_goal_async the ExecuteTrajectory goal was already terminal
    # (ERROR_GOAL_ALREADY_TERMINATED, cancel_response "rejected").  C2
    # (rerun-6): k=4.0 (~4.6 s) overcorrected the OTHER way — the FJT
    # controller's ``goal_time: 0.5`` tolerance aborted the presend goal
    # (GOAL_TOLERANCE_VIOLATED) while the arm lagged the long synthetic motion,
    # BEFORE the ~5.5 s cancel arbitration landed, so the cancel hit an
    # already-ABORTED goal (return_code 3).  The presend slowdown stays at
    # k=4.0 (a higher k would push the motion-trigger joint velocity below the
    # 0.005 rad/s trigger threshold and the cancel arbitration would never
    # fire); the goal-time race is resolved in controllers.yaml by raising the
    # FJT ``goal_time`` tolerance 0.5 -> 2.0 (see
    # test_manipulation_integration_contract.py), so the presend goal remains
    # EXECUTING through the ~5.5 s arbitration instead of aborting on goal-time.
    assert float(k) >= 4.0, f"presend slowdown factor must be >= 4.0 (got {k})"
    assert captured_execute["trajectory"] is trajectory, (
        "the ExecuteTrajectory goal must carry the SLOWED planned trajectory"
    )


# --------------------------------------------------------------------------- #
# S2-REGRESSION (round-6) — the live runtime provider factory passes
# ``planning_goal_id``/``execute_goal_id`` to BOTH ``run_cancel_sequence`` AND
# ``run_safety_sequence``.  ``run_cancel_sequence`` accepts those ids; the
# ``run_safety_sequence`` signature does NOT (only ``execute_goal_handle``,
# ``fjt_goal_id``, ``transaction_baseline``, ``long_motion_provider``,
# ``fjt_transaction_provider``), so the live safety run crashed instantly with
# ``TypeError: run_safety_sequence() got an unexpected keyword argument
# 'planning_goal_id'`` (rerun-6 ``gate-verdict.json``: evidence-invalid,
# ``missing integrated-execution.jsonl`` — the executor never ran).  The
# provider must branch the kwargs: cancel carries the two id strings + handle;
# safety carries the handle only.
# --------------------------------------------------------------------------- #

def test_s2_live_runtime_provider_factory_omits_ids_for_safety():
    """RED (S2): ``_live_runtime_provider_factory`` must NOT pass
    ``planning_goal_id``/``execute_goal_id`` to ``run_safety_sequence`` (the
    signature rejects them); it must pass ``execute_goal_handle`` +
    ``fjt_goal_id`` + ``transaction_baseline``."""
    presend = {
        "planning_goal_id": "a" * 32,
        "execute_goal_id": "b" * 32,
        "execute_goal_handle": object(),
        "fjt_goal_id": "c" * 32,
        "fjt_entry": {},
        "transaction_baseline": {},
    }
    executor = object()
    with patch.object(d, "_presend_long_motion", return_value=presend), patch.object(
        d, "_long_motion_provider_from_presend", return_value=lambda: {}
    ), patch.object(d, "_fjt_transaction_provider", return_value=lambda: {}):
        kwargs = d._live_runtime_provider_factory(
            executor=executor,
            scenario_id="qualification-moveit-safety",
            bundle={},
            config={},
            attempt_dir=Path(_tmp()),
        )
    assert "planning_goal_id" not in kwargs, (
        "run_safety_sequence does not accept planning_goal_id; the live driver "
        f"passed it and crashed (got {sorted(kwargs)})"
    )
    assert "execute_goal_id" not in kwargs, (
        "run_safety_sequence does not accept execute_goal_id; the live driver "
        f"passed it and crashed (got {sorted(kwargs)})"
    )
    assert kwargs["execute_goal_handle"] is presend["execute_goal_handle"]
    assert kwargs["fjt_goal_id"] == "c" * 32
    assert kwargs["transaction_baseline"] == {}


def test_s2_live_runtime_provider_factory_keeps_ids_for_cancel():
    """RED (S2 companion): ``run_cancel_sequence`` DOES accept the plan/execute
    id strings plus the handle — the provider must keep passing them for cancel."""
    presend = {
        "planning_goal_id": "a" * 32,
        "execute_goal_id": "b" * 32,
        "execute_goal_handle": object(),
        "fjt_goal_id": "c" * 32,
        "fjt_entry": {},
        "transaction_baseline": {},
    }
    executor = object()
    with patch.object(d, "_presend_long_motion", return_value=presend), patch.object(
        d, "_long_motion_provider_from_presend", return_value=lambda: {}
    ), patch.object(d, "_fjt_transaction_provider", return_value=lambda: {}):
        kwargs = d._live_runtime_provider_factory(
            executor=executor,
            scenario_id="qualification-moveit-cancel",
            bundle={},
            config={},
            attempt_dir=Path(_tmp()),
        )
    assert kwargs["planning_goal_id"] == "a" * 32
    assert kwargs["execute_goal_id"] == "b" * 32
    assert kwargs["execute_goal_handle"] is presend["execute_goal_handle"]
    assert kwargs["fjt_goal_id"] == "c" * 32


# --------------------------------------------------------------------------- #
# G4 — the long-motion joint target pre-sent for cancel/safety must be a
# collision-free, reachable configuration with meaningful motion.
#
# The original ``_LONG_MOTION_JOINT_TARGET`` ``(0.0, -0.3, 0.0, 0.6, 0.0, 0.3,
# 0.0)`` placed the TCP at (0.461, 0.0, 0.764) — inside the Stage-D pedestal
# fixture (box [0.7,0.7,0.85] centered at [0.8,0,0.425], so x in 0.45..1.15).
# OMPL therefore reported "Unable to sample any valid states for goal tree" and
# the driver raised "ExecuteTrajectory goal requires a non-empty planned
# trajectory".  The target must keep every arm/finger frame clear of the
# pedestal while still producing real joint motion so the safety stop fires
# mid-motion.
# --------------------------------------------------------------------------- #

def _safety_pedestal_box():
    """Stage-D pedestal: box [0.7,0.7,0.85] at [0.8,0,0.425] in base_link."""
    dims = [0.7, 0.7, 0.85]
    xyz = [0.8, 0.0, 0.425]
    half = [dim * 0.5 for dim in dims]
    return {
        "min": [xyz[i] - half[i] for i in range(3)],
        "max": [xyz[i] + half[i] for i in range(3)],
    }


def _simulator_urdf_path():
    return ROOT / "integration" / "model-bundle-r2" / "simulator_full_urdf" / "source-tinker-full.urdf"


def _urdf_fk_links():
    """Return {link_name: (parent, xyz, rpy, axis, joint_type, joint_name)}."""
    import math
    import xml.etree.ElementTree as ET

    tree = ET.parse(str(_simulator_urdf_path()))
    root = tree.getroot()
    transforms = {}
    for joint in root.findall("joint"):
        pe = joint.find("parent")
        ce = joint.find("child")
        if pe is None or ce is None:
            continue
        origin = joint.find("origin")
        oxyz = [float(v) for v in origin.get("xyz").split()] if origin is not None else [0.0, 0.0, 0.0]
        orpy = [float(v) for v in origin.get("rpy").split()] if origin is not None else [0.0, 0.0, 0.0]
        axis_el = joint.find("axis")
        axis = [float(v) for v in axis_el.get("xyz").split()] if axis_el is not None else [0.0, 0.0, 1.0]
        transforms[ce.get("link")] = (pe.get("link"), oxyz, orpy, axis, joint.get("type"), joint.get("name"))
    return transforms


def _fk_pose(transforms, target_link, joint_positions):
    """Return (rotation_3x3, position_3) of *target_link* in base_link."""
    import math

    I = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    def mmul(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    def mvec(A, v):
        return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]

    def vecadd(a, b):
        return [a[i] + b[i] for i in range(3)]

    def rpy_to_mat(r, p, y):
        c, s = math.cos, math.sin
        Rz = [[c(y), -s(y), 0.0], [s(y), c(y), 0.0], [0.0, 0.0, 1.0]]
        Ry = [[c(p), 0.0, s(p)], [0.0, 1.0, 0.0], [-s(p), 0.0, c(p)]]
        Rx = [[1.0, 0.0, 0.0], [0.0, c(r), -s(r)], [0.0, s(r), c(r)]]
        return mmul(Rz, mmul(Ry, Rx))

    def rot_about(axis, ang):
        k = axis
        K = [[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]]
        out = [[I[i][j] for j in range(3)] for i in range(3)]
        for i in range(3):
            for j in range(3):
                out[i][j] += math.sin(ang) * K[i][j]
                out[i][j] += (1.0 - math.cos(ang)) * sum(K[i][k] * K[k][j] for k in range(3))
        return out

    chain = []
    node = target_link
    while node != "base_link":
        if node not in transforms:
            return None
        chain.append(node)
        node = transforms[node][0]
    chain.reverse()
    T = I
    p = [0.0, 0.0, 0.0]
    for child in chain:
        parent, oxyz, orpy, axis, jtype, jname = transforms[child]
        Rf = rpy_to_mat(*orpy)
        if jtype in ("revolute", "continuous"):
            Rc = mmul(Rf, rot_about(axis, joint_positions.get(jname, 0.0)))
        else:
            Rc = Rf
        p = vecadd(p, mvec(T, oxyz))
        T = mmul(T, Rc)
    return T, p


def _long_motion_fk():
    """Compute every arm/gripper frame origin for _LONG_MOTION_JOINT_TARGET."""
    transforms = _urdf_fk_links()
    target = dict(zip(
        ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
        d._LONG_MOTION_JOINT_TARGET,
    ))
    frames = [
        "link_tcp", "xarm_gripper_base_link", "link_eef",
        "left_finger", "right_finger", "left_outer_knuckle", "right_outer_knuckle",
        "link7", "link6", "link5", "link4", "link3", "link2", "link1",
    ]
    poses = {}
    for frame in frames:
        res = _fk_pose(transforms, frame, target)
        if res is not None:
            _, p = res
            poses[frame] = p
    return poses, target


def test_g4_long_motion_target_is_collision_free_and_meaningful():
    """RED (G4): the long-motion joint target pre-sent for cancel/safety must be
    a reachable, collision-free configuration (no frame inside the pedestal
    fixture) that moves the arm meaningfully from home."""
    box = _safety_pedestal_box()
    lo = [box["min"][i] + 1e-6 for i in range(3)]
    hi = [box["max"][i] - 1e-6 for i in range(3)]

    def in_box(x, y, z):
        return lo[0] <= x <= hi[0] and lo[1] <= y <= hi[1] and lo[2] <= z <= hi[2]

    poses, target = _long_motion_fk()
    assert len(poses) >= 10, f"FK failed to resolve frames: {sorted(poses)}"

    inside = {frame: (round(p[0], 3), round(p[1], 3), round(p[2], 3))
              for frame, p in poses.items() if in_box(*p)}
    assert not inside, (
        "long-motion target places frames inside the pedestal fixture: "
        f"{inside} for target {d._LONG_MOTION_JOINT_TARGET}"
    )

    # Reachability: every joint inside the xarm7 declared limits.
    limits = {
        "joint1": (-6.283185307179586, 6.283185307179586),
        "joint2": (-2.059, 2.0944),
        "joint3": (-6.283185307179586, 6.283185307179586),
        "joint4": (-0.192, 3.927),
        "joint5": (-6.283185307179586, 6.283185307179586),
        "joint6": (-1.693, 3.142),
        "joint7": (-6.283185307179586, 6.283185307179586),
    }
    for name, value in target.items():
        lo_lim, hi_lim = limits[name]
        assert lo_lim <= value <= hi_lim, f"{name}={value} out of range {limits[name]}"

    # Meaningful motion: at least one joint moves > 0.2 rad from home.
    home = dict(zip(target.keys(), [0.0] * len(target)))
    max_delta = max(abs(target[name] - home[name]) for name in target)
    assert max_delta >= 0.2, (
        "long-motion target produces no meaningful motion "
        f"(max joint delta {max_delta:.3f} rad)"
    )


# --------------------------------------------------------------------------- #
# G1 — execute-pose target placement.
#
# The round-2 target ``[0.55, 0.0, 0.99]`` put the generated approach pose
# (declared + POSE_APPROACH_Z_OFFSET) at ``[0.55, 0.0, 1.09]`` — directly above
# the Stage-D pedestal (box [0.7,0.7,0.85] at [0.8,0,0.425], x in 0.45..1.15).
# The top-down gripper geometry therefore overlapped the pedestal volume and
# OMPL reported "Unable to sample any valid states for goal tree".  The fix
# moves ``sim_fixture/public_target`` clear of the pedestal footprint (in free
# space ahead of it) at a pose the xarm7 can reach with the fixed z-down
# approach orientation.
# --------------------------------------------------------------------------- #

def _g1_execute_pose_scenario():
    """Return (declared_xyz, approach_xyz, approach_quat) for execute-pose."""
    import json

    from integrated_gate_executor import POSE_APPROACH_Z_OFFSET
    from ompl_goal_builders import POSE_APPROACH_QUATERNION_XYZW

    scenario = json.loads(
        (ROOT / "simulation" / "scenarios" / "qualification-moveit-execute-pose.json")
        .read_text(encoding="utf-8")
    )
    ps = scenario["planning_scene"]
    target = next(record for record in ps["objects"] if record.get("id") == ps["target_source_id"])
    xyz = [float(value) for value in target["pose"]["xyz"]]
    approach = [xyz[0], xyz[1], xyz[2] + POSE_APPROACH_Z_OFFSET]
    return xyz, approach, list(POSE_APPROACH_QUATERNION_XYZW)


def _quat_to_mat(q):
    w, x, y, z = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _mat_to_quat(R):
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        S = (tr + 1.0) ** 0.5 * 2
        return (0.25 * S, (R[2][1] - R[1][2]) / S, (R[0][2] - R[2][0]) / S, (R[1][0] - R[0][1]) / S)
    if R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        S = (1.0 + R[0][0] - R[1][1] - R[2][2]) ** 0.5 * 2
        return ((R[2][1] - R[1][2]) / S, 0.25 * S, (R[1][0] + R[0][1]) / S, (R[0][2] + R[2][0]) / S)
    if R[1][1] > R[2][2]:
        S = (1.0 + R[1][1] - R[0][0] - R[2][2]) ** 0.5 * 2
        return ((R[0][2] - R[2][0]) / S, (R[1][0] + R[0][1]) / S, 0.25 * S, (R[2][1] + R[1][2]) / S)
    S = (1.0 + R[2][2] - R[0][0] - R[1][1]) ** 0.5 * 2
    return ((R[1][0] - R[0][1]) / S, (R[0][2] + R[2][0]) / S, (R[2][1] + R[1][2]) / S, 0.25 * S)


def _numeric_ik_to_pose(transforms, target_R, target_p, q0=None, iters=400):
    """Damped-least-squares IK for the 7 arm joints.  Returns (q, pos_err, quat_err_deg)."""
    import math

    order = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    limits = {
        "joint1": (-6.283185307179586, 6.283185307179586),
        "joint2": (-2.059, 2.0944),
        "joint3": (-6.283185307179586, 6.283185307179586),
        "joint4": (-0.192, 3.927),
        "joint5": (-6.283185307179586, 6.283185307179586),
        "joint6": (-1.693, 3.142),
        "joint7": (-6.283185307179586, 6.283185307179586),
    }
    q = {name: (q0.get(name, 0.0) if q0 else 0.0) for name in order}

    def vsub(a, b):
        return [a[i] - b[i] for i in range(3)]

    def vnorm(v):
        return sum(v[i] * v[i] for i in range(3)) ** 0.5

    def mat_mul(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(len(B))) for j in range(len(B[0]))]
                for i in range(len(A))]

    def mat_vec(A, v):
        return [sum(A[i][k] * v[k] for k in range(len(v))) for i in range(len(A))]

    def rotvec_from_mat(R):
        ang = math.acos(max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2)))
        if ang < 1e-9:
            return [0.0, 0.0, 0.0]
        s = math.sin(ang)
        axis = [(R[2][1] - R[1][2]) / (2 * s), (R[0][2] - R[2][0]) / (2 * s), (R[1][0] - R[0][1]) / (2 * s)]
        return [axis[i] * ang for i in range(3)]

    def solve6(A, b):
        M = [A[i][:] + [b[i]] for i in range(6)]
        for col in range(6):
            piv = max(range(col, 6), key=lambda r: abs(M[r][col]))
            if abs(M[piv][col]) < 1e-12:
                raise ZeroDivisionError("singular")
            M[col], M[piv] = M[piv], M[col]
            for r in range(col + 1, 6):
                f = M[r][col] / M[col][col]
                for c in range(col, 7):
                    M[r][c] -= f * M[col][c]
        x = [0.0] * 6
        for r in range(5, -1, -1):
            x[r] = (M[r][6] - sum(M[r][c] * x[c] for c in range(r + 1, 6))) / M[r][r]
        return x

    eps = 1e-6
    lam = 0.5
    for _ in range(iters):
        Rc, pc = _fk_pose(transforms, "link_tcp", q)
        Rt = [[Rc[0][0], Rc[1][0], Rc[2][0]],
              [Rc[0][1], Rc[1][1], Rc[2][1]],
              [Rc[0][2], Rc[1][2], Rc[2][2]]]
        Re = mat_mul(target_R, Rt)
        er = rotvec_from_mat(Re)
        ep = vsub(target_p, pc)
        e = ep + er
        Jp, Jr = [], []
        for name in order:
            qe = dict(q)
            qe[name] += eps
            Rp, pp = _fk_pose(transforms, "link_tcp", qe)
            Jp.append([(pp[i] - pc[i]) / eps for i in range(3)])
            qa, qb = _mat_to_quat(Rc), _mat_to_quat(Rp)
            iw, ix, iy, iz = qa[0], -qa[1], -qa[2], -qa[3]
            w = qb[0] * iw - qb[1] * ix - qb[2] * iy - qb[3] * iz
            x = qb[0] * ix + qb[1] * iw - qb[2] * iz + qb[3] * iy
            y = qb[0] * iy + qb[1] * iz + qb[2] * iw - qb[3] * ix
            z = qb[0] * iz - qb[1] * iy + qb[2] * ix + qb[3] * iw
            n = (x * x + y * y + z * z) ** 0.5
            ang = 2.0 * math.atan2(n, w) / eps
            Jr.append([ang * x / n, ang * y / n, ang * z / n] if n > 1e-12 else [0.0, 0.0, 0.0])
        J = [[Jp[j][i] for j in range(7)] for i in range(3)] + \
            [[Jr[j][i] for j in range(7)] for i in range(3)]
        Jt = [[J[r][c] for r in range(6)] for c in range(7)]
        JJt = mat_mul(J, Jt)
        for i in range(6):
            JJt[i][i] += lam * lam
        try:
            d6 = solve6(JJt, e)
        except ZeroDivisionError:
            break
        dq = mat_vec(Jt, d6)
        for i, name in enumerate(order):
            q[name] = max(limits[name][0], min(limits[name][1], q[name] + dq[i]))
        lam = max(0.01, lam * 0.9)

    Rf, pf = _fk_pose(transforms, "link_tcp", q)
    qa, qb = _mat_to_quat(target_R), _mat_to_quat(Rf)
    dotp = abs(sum(a * b for a, b in zip(qa, qb)))
    quat_err = math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dotp))))
    pos_err = vnorm(vsub(target_p, pf))
    return q, pos_err, quat_err


def test_g1_execute_pose_target_is_pedestal_clear_and_reachable():
    """RED (G1): the execute-pose declared target must be clear of the pedestal
    footprint (the round-2 target hovered the top-down gripper directly over the
    pedestal volume, so OMPL could not sample any valid goal states) and the
    generated approach pose must be reachable with the fixed approach
    orientation, with every arm/gripper frame outside the pedestal box."""
    import math

    from ompl_goal_builders import POSE_APPROACH_QUATERNION_XYZW

    declared, approach, approach_quat = _g1_execute_pose_scenario()
    box = _safety_pedestal_box()
    margin = 0.05

    in_footprint = (
        box["min"][0] - margin <= approach[0] <= box["max"][0] + margin
        and box["min"][1] - margin <= approach[1] <= box["max"][1] + margin
    )
    assert not in_footprint, (
        "execute-pose approach pose is positioned over the pedestal footprint: "
        f"approach={tuple(round(v, 3) for v in approach)}, pedestal x in "
        f"[{box['min'][0]}, {box['max'][0]}], y in [{box['min'][1]}, {box['max'][1]}]"
    )

    # Reachability: numeric IK must converge to the approach pose.
    transforms = _urdf_fk_links()
    target_R = _quat_to_mat(approach_quat)
    q, pos_err, quat_err = _numeric_ik_to_pose(transforms, target_R, approach)
    assert pos_err <= 0.02, f"approach pose not reachable: pos error {pos_err:.4f} m"
    assert quat_err <= 5.0, f"approach orientation not reachable: {quat_err:.2f} deg"

    # Frame clearance: every arm/gripper frame plus finger tips clear the box.
    lo = [box["min"][i] + 1e-6 for i in range(3)]
    hi = [box["max"][i] - 1e-6 for i in range(3)]

    def in_box(x, y, z):
        return lo[0] <= x <= hi[0] and lo[1] <= y <= hi[1] and lo[2] <= z <= hi[2]

    frames = [
        "link_tcp", "xarm_gripper_base_link", "link_eef",
        "left_finger", "right_finger", "left_outer_knuckle", "right_outer_knuckle",
        "left_inner_knuckle", "right_inner_knuckle",
        "link7", "link6", "link5", "link4", "link3", "link2", "link1", "link_base",
    ]
    inside = {}
    for frame in frames:
        res = _fk_pose(transforms, frame, q)
        if res is None:
            continue
        Rf, p = res
        if in_box(*p):
            inside[frame] = (round(p[0], 3), round(p[1], 3), round(p[2], 3))
        zaxis = [Rf[i][2] for i in range(3)]
        tip = [p[i] - 0.09 * zaxis[i] for i in range(3)]
        if in_box(*tip):
            inside[f"{frame}_tip"] = (round(tip[0], 3), round(tip[1], 3), round(tip[2], 3))
    assert not inside, (
        "execute-pose approach IK places frames inside the pedestal: "
        f"{inside} for declared {declared} approach {approach}"
    )


def test_j_operator_subscription_qos_matches_transient_local_publisher():
    """RED (J): the driver's live operator observer must subscribe with the
    same TRANSIENT_LOCAL/RELIABLE durability+reliability the executor publishes
    on ``/sim/safety/operator``.  A VOLATILE observer misses the latched False
    baseline on a cold start (execute-joint/cancel/safety readiness-gate
    regression: ``operator_input: publisher count is 0`` / ``no sample
    received``), so the subscription spec must agree with the publisher
    contract in ``REQUIRED_TOPICS``."""
    from integrated_gate_executor import OPERATOR_TOPIC, REQUIRED_TOPICS

    spec = d._OPERATOR_SUB_QOS_SPEC
    contract = REQUIRED_TOPICS[OPERATOR_TOPIC]["qos"]
    assert spec["reliability"] == contract["reliability"]
    assert spec["durability"] == contract["durability"]
    assert spec["depth"] >= 1
    assert spec["durability"] == "transient_local"


# --------------------------------------------------------------------------- #
# R3 — cartesian-retreat environment-cloud provider resilience
# --------------------------------------------------------------------------- #

class _FakePointCloud:
    """ROS-free stand-in for ``sensor_msgs/msg/PointCloud2`` (structural only)."""

    def __init__(self, *, frame_id: str = "livox360", width: int = 3):
        self.header = types.SimpleNamespace(frame_id=frame_id)
        self.width = width
        self.height = 1
        self.data = b"\x00" * (width * 12)


class _CloudSpinningExecutor:
    """ROS-free executor double whose ``_spin_once`` delivers the first cloud.

    Mirrors the live observer's late-join race: ``latest_cloud`` is None until
    the executor has spun ``deliver_after_spins`` times (the dev lidar at
    ``/livox/lidar`` publishes best-effort/volatile sensor-data QoS at 10 Hz, so
    a late-joining observer misses every pre-join frame).  ``deliver_after_spins
    is None`` means the cloud is never delivered (bounded-fail closed case).
    """

    def __init__(self, *, deliver_after_spins: int | None = 2):
        self.spins = 0
        self.deliver_after_spins = (
            None if deliver_after_spins is None else int(deliver_after_spins)
        )
        self._driver_observer = types.SimpleNamespace(
            latest_cloud=None,
            cloud_received_mono=None,
            tf_buffer=types.SimpleNamespace(
                lookup_transform=lambda *_args, **_kwargs: types.SimpleNamespace()
            ),
        )

    def _spin_once(self) -> None:
        time.sleep(0.001)  # mirror the live spinner's ~ms block per spin
        self.spins += 1
        if self.deliver_after_spins is not None and self.spins >= self.deliver_after_spins:
            self._driver_observer.latest_cloud = _FakePointCloud()
            self._driver_observer.cloud_received_mono = time.monotonic()


def _install_cloud_transform_fakes(monkeypatch):
    """Inject ROS-free ``tf2_ros``/``tf2_sensor_msgs`` fakes into sys.modules.

    The provider imports these modules at call time (the driver module itself is
    import-safe without rclpy), so a ROS-free test can shadow them to exercise
    the bounded first-cloud wait without a live TF tree.
    """
    class _LookupException(Exception):
        pass

    class _ConnectivityException(Exception):
        pass

    class _ExtrapolationException(Exception):
        pass

    fake_tf2_ros = types.ModuleType("tf2_ros")
    fake_tf2_ros.LookupException = _LookupException
    fake_tf2_ros.ConnectivityException = _ConnectivityException
    fake_tf2_ros.ExtrapolationException = _ExtrapolationException

    def _do_transform_cloud(cloud, transform):  # noqa: ARG001 - ROS-free stub
        cloud.header = types.SimpleNamespace(frame_id="base_link")
        return cloud

    fake_tf2_sensor_msgs = types.ModuleType("tf2_sensor_msgs")
    fake_tf2_sensor_msgs.do_transform_cloud = _do_transform_cloud

    monkeypatch.setitem(sys.modules, "tf2_ros", fake_tf2_ros)
    monkeypatch.setitem(sys.modules, "tf2_sensor_msgs", fake_tf2_sensor_msgs)
    monkeypatch.setattr(d, "_tf_zero_time", lambda: types.SimpleNamespace(sec=0, nanosec=0))


def test_r3_environment_cloud_provider_waits_for_the_first_frame(monkeypatch):
    """RED (R3): the cartesian-retreat environment-cloud provider must bounded-
    wait for the first live ``/livox/lidar`` frame instead of failing closed the
    instant ``latest_cloud`` is still None.

    rerun-8 cartesian-retreat: ``environment_cloud_provider raised: no live
    environment PointCloud2 is available`` in 0.0099 s — the observer had not
    yet received a cloud (best-effort/volatile dev-lidar delivery races the
    late-joining observer).  rerun-6/7 the cloud flowed; rerun-8 it flaked.  The
    provider must spin the shared executor/observer for a bounded window so a
    frame that arrives a moment later is accepted rather than treated as absent.
    """
    _install_cloud_transform_fakes(monkeypatch)
    executor = _CloudSpinningExecutor(deliver_after_spins=2)
    provider = d._environment_cloud_provider(executor, first_cloud_wait_s=1.0)
    cloud = provider()
    assert executor.spins >= 1, "provider must spin while waiting for the first cloud"
    assert getattr(cloud.header, "frame_id", "") == "base_link"
    assert cloud.width >= 1 and len(cloud.data) >= 1


def test_r3_environment_cloud_provider_still_fails_closed_when_no_frame_arrives(monkeypatch):
    """R3 companion: the bounded first-cloud wait must still fail closed (with
    the exact existing error) when no frame ever arrives within the window —
    the resilience must not weaken the missing/stale/empty/wrong-frame
    rejections."""
    _install_cloud_transform_fakes(monkeypatch)
    executor = _CloudSpinningExecutor(deliver_after_spins=None)
    provider = d._environment_cloud_provider(executor, first_cloud_wait_s=0.05)
    with pytest.raises(d.DriverError, match="no live environment PointCloud2 is available"):
        provider()
    assert executor.spins >= 1, "provider must have bounded-spun before failing closed"
