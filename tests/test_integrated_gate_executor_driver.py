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
from pathlib import Path
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

import integrated_gate_executor_driver as d  # noqa: E402
from integrated_gate_executor import (  # noqa: E402
    STAGE_C_SCENARIOS,
    STAGE_D_KIND,
    STAGE_D_SCENARIOS,
    STAGE_E_SCENARIOS,
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

def test_dispatch_table_is_exactly_the_17_canonical_scenarios():
    table = d.canonical_dispatch()
    assert len(table) == 17
    assert len(set(table)) == 17
    expected = set(STAGE_C_SCENARIOS) | set(STAGE_D_SCENARIOS) | set(STAGE_E_SCENARIOS)
    assert set(table) == expected
    assert len(expected) == 17
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
