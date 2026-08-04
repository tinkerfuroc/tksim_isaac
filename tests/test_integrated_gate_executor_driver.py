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
sys.path.insert(0, str(ROOT / "simulation"))

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
