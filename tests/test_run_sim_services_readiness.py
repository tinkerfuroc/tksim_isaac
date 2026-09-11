"""Task #39: sim_control services must never be advertised before they can
be served.

``isaacsim.ros2.sim_control`` (the extension behind ``/spawn_entity``,
``/set_entity_state``, ``/delete_entity``, ...) registers its services the
instant it is enabled, but Kit only *serves* them from its own asyncio loop,
which nothing pumps regularly until the profile's main loop starts. Backend
construction and (on sensor-rich) camera-rig warm-up together can run for a
couple of minutes with almost no ``app.update()`` in between; enabling the
extension before that finishes leaves a window where a client's
``wait_for_service()`` succeeds long before a request can actually round-trip
-- the observed failure mode was rclpy's "failed to send response (timeout)".

Two things are covered here without any GPU/Kit process:

* ``run_sim._enable_sim_control_services`` -- the extracted helper -- with
  ``isaacsim.core.utils.extensions.enable_extension`` and the app/gateway
  faked, asserting the call order (enable -> app.update -> gateway marked
  ready) and that it tolerates ``gateway=None`` (the profiles/paths that run
  ``--ros`` without a ``RosStandardGateway``).
* A source-order regression test per sensor profile branch of ``main()``:
  backend construction, then (sensor-rich only) camera-rig warm-up, then
  gateway construction, must all precede the
  ``_enable_sim_control_services(...)`` call in that branch's source text.
  ``main()`` needs a live Kit process to actually run, so this is the
  practical boot-ordering assertion available under plain system Python
  (same rationale as the existing ``inspect.getsource``-based structural
  tests in this suite, e.g.
  ``test_manipulation_gate_executor.py::test_ros_executor_has_no_raw_truth_subscription``).
"""
from __future__ import annotations

import inspect
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

_STUB_PREFIXES = ("isaacsim",)


@pytest.fixture(autouse=True)
def _restore_sys_modules():
    saved = {
        name: mod
        for name, mod in sys.modules.items()
        if name == "run_sim" or name.split(".")[0] in _STUB_PREFIXES
    }
    try:
        yield
    finally:
        for name in [
            n for n in sys.modules if n == "run_sim" or n.split(".")[0] in _STUB_PREFIXES
        ]:
            sys.modules.pop(name, None)
        sys.modules.update(saved)


def _load_run_sim():
    sys.path.insert(0, str(ROOT / "validation"))
    sys.path.insert(0, str(ROOT / "simulation"))
    import importlib

    return importlib.import_module("run_sim")


def _install_fake_enable_extension(calls: list) -> None:
    """A fake ``isaacsim.core.utils.extensions`` module, package chain
    included, so ``from isaacsim.core.utils.extensions import
    enable_extension`` resolves without a real Isaac install."""
    isaacsim = types.ModuleType("isaacsim")
    isaacsim.__path__ = []
    sys.modules["isaacsim"] = isaacsim
    core = types.ModuleType("isaacsim.core")
    core.__path__ = []
    sys.modules["isaacsim.core"] = core
    utils = types.ModuleType("isaacsim.core.utils")
    utils.__path__ = []
    sys.modules["isaacsim.core.utils"] = utils
    extensions = types.ModuleType("isaacsim.core.utils.extensions")

    def enable_extension(name: str) -> None:
        calls.append(("enable_extension", name))

    extensions.enable_extension = enable_extension
    sys.modules["isaacsim.core.utils.extensions"] = extensions


class _FakeApp:
    def __init__(self, calls: list) -> None:
        self._calls = calls

    def update(self) -> None:
        self._calls.append("app.update")


class _FakeGateway:
    def __init__(self, calls: list) -> None:
        self._calls = calls

    def mark_services_ready(self) -> None:
        self._calls.append("gateway.mark_services_ready")


def test_enable_sim_control_services_enables_then_updates_then_marks_ready() -> None:
    calls: list = []
    _install_fake_enable_extension(calls)
    run_sim = _load_run_sim()

    run_sim._enable_sim_control_services(_FakeApp(calls), _FakeGateway(calls))

    assert calls == [
        ("enable_extension", "isaacsim.ros2.sim_control"),
        "app.update",
        "gateway.mark_services_ready",
    ]


def test_enable_sim_control_services_without_a_gateway_still_enables_and_updates() -> None:
    calls: list = []
    _install_fake_enable_extension(calls)
    run_sim = _load_run_sim()

    run_sim._enable_sim_control_services(_FakeApp(calls))

    assert calls == [
        ("enable_extension", "isaacsim.ros2.sim_control"),
        "app.update",
    ]


# --- Source-order regression: main() must not advertise the services until
# each branch's slow boot steps are behind it. ---------------------------

_BRANCH_MARKERS = [
    'if args.sensor_profile == "navigation-parity":',
    'elif args.sensor_profile == "sensor-rich":',
    'elif args.sensor_profile == "manipulation-core":',
    'elif args.sensor_profile == "physics-only":',
    'else:\n            raise RuntimeError(f"unsupported Isaac sensor profile',
]


def _branch_source(main_source: str, start_marker: str, end_marker: str) -> str:
    start = main_source.index(start_marker)
    end = main_source.index(end_marker, start + len(start_marker))
    return main_source[start:end]


def test_no_early_top_level_enable_extension_call() -> None:
    """Regression: enable_extension used to run immediately after
    SimulationApp construction, before any profile branch and long before
    the backend/camera-rig/gateway it now waits on exist. The only call
    left before the first branch must be the pre-enable monkeypatch
    installer, not the extension enable itself."""
    run_sim = _load_run_sim()
    main_source = inspect.getsource(run_sim.main)
    preamble = main_source[: main_source.index(_BRANCH_MARKERS[0])]
    assert "_install_set_entity_state_physics(" in preamble
    assert "enable_extension(" not in preamble


def test_navigation_parity_enables_services_after_backend_and_gateway() -> None:
    run_sim = _load_run_sim()
    main_source = inspect.getsource(run_sim.main)
    branch = _branch_source(main_source, _BRANCH_MARKERS[0], _BRANCH_MARKERS[1])
    backend_idx = branch.index("IsaacNavigationBackend(")
    gateway_idx = branch.index("RosStandardGateway(")
    enable_idx = branch.index("_enable_sim_control_services(")
    assert backend_idx < gateway_idx < enable_idx


def test_sensor_rich_enables_services_after_backend_camera_rig_and_gateway() -> None:
    run_sim = _load_run_sim()
    main_source = inspect.getsource(run_sim.main)
    branch = _branch_source(main_source, _BRANCH_MARKERS[1], _BRANCH_MARKERS[2])
    backend_idx = branch.index("IsaacWholeRobotBackend(")
    camera_idx = branch.index("camera_rig.initialize(")
    gateway_idx = branch.index("RosStandardGateway(")
    enable_idx = branch.index("_enable_sim_control_services(")
    assert backend_idx < camera_idx < gateway_idx < enable_idx


def test_manipulation_core_enables_services_after_backend_and_gateway() -> None:
    run_sim = _load_run_sim()
    main_source = inspect.getsource(run_sim.main)
    branch = _branch_source(main_source, _BRANCH_MARKERS[2], _BRANCH_MARKERS[3])
    backend_idx = branch.index("IsaacWholeRobotBackend(")
    gateway_idx = branch.index("RosStandardGateway(")
    enable_idx = branch.index("_enable_sim_control_services(")
    assert backend_idx < gateway_idx < enable_idx
