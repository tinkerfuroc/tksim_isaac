"""Unit tests for run_sim's /set_entity_state physics-effective patch.

The patch makes NVIDIA's standard ``/set_entity_state`` move spawned rigid
bodies in physics (via the backend's rigid-body-view write) instead of the
USD-layer no-op it falls back to for a freshly spawned prim. These tests
exercise the wrapper logic -- field extraction, xyzw quaternion order, and the
free-rigid-body scope guard that excludes the robot articulation -- with the
isaacsim/omni/pxr dependencies stubbed, since the live path is GPU-only.
"""
from __future__ import annotations

import asyncio
import importlib
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

# Module-name prefixes the tests stub into sys.modules; snapshot + restore so a
# fake isaacsim/omni/pxr never leaks into other test files in the same session.
_STUB_PREFIXES = ("isaacsim", "omni", "pxr")


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
            n
            for n in sys.modules
            if n == "run_sim" or n.split(".")[0] in _STUB_PREFIXES
        ]:
            sys.modules.pop(name, None)
        sys.modules.update(saved)


def _load_run_sim():
    sys.path.insert(0, str(ROOT / "validation"))
    return importlib.import_module("run_sim")


class _FakePrim:
    def __init__(self, path: str, apis: set[str], parent: "_FakePrim | None") -> None:
        self._path = path
        self._apis = apis
        self._parent = parent

    def IsValid(self) -> bool:
        return True

    def HasAPI(self, api) -> bool:
        return api in self._apis

    def GetParent(self) -> "_FakePrim | None":
        return self._parent

    def GetPath(self):
        return types.SimpleNamespace(pathString=self._path)


def _install_fake_isaacsim() -> None:
    # A fake isaacsim package whose derived extension dir does not exist on disk
    # (so the path splice is skipped) but whose submodules resolve from the
    # stubs installed below.
    isaacsim = types.ModuleType("isaacsim")
    isaacsim.__file__ = "/nonexistent/isaacsim/__init__.py"
    isaacsim.__path__ = []
    sys.modules["isaacsim"] = isaacsim


def _install_fakes_for_guard(prim: _FakePrim) -> None:
    pxr = types.ModuleType("pxr")
    pxr.UsdPhysics = types.SimpleNamespace(
        RigidBodyAPI="RigidBodyAPI", ArticulationRootAPI="ArticulationRootAPI"
    )
    sys.modules["pxr"] = pxr

    stage = types.SimpleNamespace(GetPrimAtPath=lambda path: prim)
    omni = types.ModuleType("omni")
    omni.__path__ = []
    omni_usd = types.ModuleType("omni.usd")
    omni_usd.get_context = lambda: types.SimpleNamespace(get_stage=lambda: stage)
    omni.usd = omni_usd
    sys.modules["omni"] = omni
    sys.modules["omni.usd"] = omni_usd


def _install_fake_sim_control():
    seen = {"original_called": 0}

    class _SimulationControl:
        async def _handle_set_entity_state(self, request, response):
            seen["original_called"] += 1
            return response

    _install_fake_isaacsim()
    for name in (
        "isaacsim.ros2",
        "isaacsim.ros2.sim_control",
        "isaacsim.ros2.sim_control.impl",
    ):
        module = types.ModuleType(name)
        module.__path__ = []
        sys.modules[name] = module
    module = types.ModuleType("isaacsim.ros2.sim_control.impl.simulation_control")
    module.SimulationControl = _SimulationControl
    sys.modules["isaacsim.ros2.sim_control.impl.simulation_control"] = module
    sys.modules["isaacsim.ros2.sim_control.impl"].simulation_control = module
    return module, seen


def _request(entity: str):
    ns = types.SimpleNamespace
    return ns(
        entity=entity,
        state=ns(
            pose=ns(
                position=ns(x=1.0, y=2.0, z=0.05),
                orientation=ns(x=0.0, y=0.0, z=0.3826834, w=0.9238795),
            ),
            twist=ns(
                linear=ns(x=0.0, y=0.0, z=0.0),
                angular=ns(x=0.0, y=0.0, z=0.0),
            ),
        ),
    )


class _RecordingBackend:
    def __init__(self) -> None:
        self.calls = []

    def set_entity_pose_physics(self, path, position, quat_xyzw, lin, ang) -> bool:
        self.calls.append((path, position, quat_xyzw, lin, ang))
        return True


def _run(coro):
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


def test_installer_skips_when_extension_absent() -> None:
    # A fake isaacsim whose extension module is absent -> import fails -> the
    # installer skips gracefully without raising.
    _install_fake_isaacsim()
    run_sim = _load_run_sim()
    run_sim._install_set_entity_state_physics({"backend": None})


def test_patched_handler_writes_physics_pose_for_free_rigid_body() -> None:
    run_sim = _load_run_sim()
    module, seen = _install_fake_sim_control()
    # A free rigid body: RigidBodyAPI present, no ArticulationRootAPI anywhere.
    root = _FakePrim("/", set(), None)
    world = _FakePrim("/World", set(), root)
    prim = _FakePrim("/World/Scenario/bench_bottle_100", {"RigidBodyAPI"}, world)
    _install_fakes_for_guard(prim)

    holder = {"backend": None}
    run_sim._install_set_entity_state_physics(holder)
    backend = _RecordingBackend()
    holder["backend"] = backend

    response = _run(
        module.SimulationControl._handle_set_entity_state(
            module.SimulationControl(),
            _request("/World/Scenario/bench_bottle_100"),
            types.SimpleNamespace(ok=True),
        )
    )
    assert response.ok is True
    assert seen["original_called"] == 1  # original still runs (USD + response)
    assert len(backend.calls) == 1
    path, position, quat_xyzw, lin, ang = backend.calls[0]
    assert path == "/World/Scenario/bench_bottle_100"
    assert position == [1.0, 2.0, 0.05]
    assert quat_xyzw == [0.0, 0.0, 0.3826834, 0.9238795]  # xyzw, no reorder
    assert lin == [0.0, 0.0, 0.0]
    assert ang == [0.0, 0.0, 0.0]


def test_patched_handler_skips_articulation_root() -> None:
    run_sim = _load_run_sim()
    module, seen = _install_fake_sim_control()
    # The robot: RigidBodyAPI on the link but an ArticulationRootAPI ancestor.
    root = _FakePrim("/", set(), None)
    robot = _FakePrim("/World/Tinker", {"ArticulationRootAPI"}, root)
    link = _FakePrim("/World/Tinker/base_link", {"RigidBodyAPI"}, robot)
    _install_fakes_for_guard(link)

    holder = {"backend": None}
    run_sim._install_set_entity_state_physics(holder)
    backend = _RecordingBackend()
    holder["backend"] = backend

    _run(
        module.SimulationControl._handle_set_entity_state(
            module.SimulationControl(),
            _request("/World/Tinker/base_link"),
            types.SimpleNamespace(ok=True),
        )
    )
    assert seen["original_called"] == 1  # original still runs
    assert backend.calls == []  # but no physics-view write on the articulation
