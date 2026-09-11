"""PhysX scene queries must be switched on whenever the raycast lidar is built.

IsaacLab's ``SimulationCfg.enable_scene_query_support`` defaults to **False**,
and its own docstring is explicit about what that means:

    If set to False, the physics engine does not create the scene query
    manager and the scene query functionality will not be available.

A raycast lidar is nothing but scene queries, so with the default the sensor
constructs cleanly, reports ``is_valid``, returns a full-length reading, casts
full-length rays from the correct world origin -- and every single ray misses.
It publishes correctly-timed EMPTY clouds. That cost a whole live Nav2 battery
to localise, because nothing anywhere logs an error.

Measured in an isolated probe, one flag flipped and nothing else changed:

    enable_scene_query_support=False -> raw raycast_closest: MISS, sensor hits 0
    enable_scene_query_support=True  -> raw raycast_closest: /World/Ground @1.0,
                                        sensor hits 2, correct hit prim paths

IsaacLab also force-enables the flag when a GUI is attached
(``physx_manager.py``: ``if has_gui: cfg.enable_scene_query_support = True``),
which is why this never reproduced interactively and only ever bit headless
runs.

These tests pin the wiring by parsing the source, because the failure is
silent: a run with the flag lost publishes empty clouds rather than crashing.
``run_sim`` defers every Isaac import into ``main()``, so this needs no
simulator.
"""

from __future__ import annotations

import ast
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "tools"))

import run_sim as rs  # noqa: E402

RUN_SIM = ROOT / "validation/run_sim.py"
BACKEND = ROOT / "simulation/tinker_sim_isaac/backend.py"

BACKEND_NAMES = {"IsaacNavigationBackend", "IsaacWholeRobotBackend"}


def _tree(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"))


def _backend_constructions(tree: ast.Module) -> list[ast.Call]:
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id in BACKEND_NAMES
    ]


def _keyword(call: ast.Call, name: str) -> ast.keyword | None:
    for kw in call.keywords:
        if kw.arg == name:
            return kw
    return None


class SceneQuerySupportContractTest(unittest.TestCase):
    def test_backend_accepts_the_flag(self) -> None:
        """The backend must expose it: SimulationCfg is built inside __init__."""
        tree = _tree(BACKEND)
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and node.name == "__init__":
                names = {a.arg for a in node.args.kwonlyargs} | {
                    a.arg for a in node.args.args
                }
                if "scene_query_support" in names:
                    return
        self.fail(
            "IsaacWholeRobotBackend.__init__ must accept 'scene_query_support'; "
            "the flag is only settable at SimulationCfg construction, so it "
            "cannot be turned on later by the lidar rig"
        )

    def test_backend_forwards_it_to_simulation_cfg(self) -> None:
        """Accepting the argument is useless unless SimulationCfg receives it."""
        tree = _tree(BACKEND)
        cfgs = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "SimulationCfg"
        ]
        self.assertTrue(cfgs, "no SimulationCfg construction found in backend.py")
        for call in cfgs:
            self.assertIsNotNone(
                _keyword(call, "enable_scene_query_support"),
                "SimulationCfg must set enable_scene_query_support explicitly; "
                "IsaacLab defaults it to False, which silently disables every "
                "raycast the lidar makes",
            )

    def test_every_backend_construction_passes_it(self) -> None:
        """All three run_sim profiles construct a backend; none may forget."""
        calls = _backend_constructions(_tree(RUN_SIM))
        self.assertGreaterEqual(
            len(calls), 3, "expected every profile's backend construction"
        )
        for call in calls:
            kw = _keyword(call, "scene_query_support")
            self.assertIsNotNone(
                kw,
                f"backend construction at run_sim.py:{call.lineno} must pass "
                "scene_query_support, or a --raycast-lidar run there returns "
                "empty clouds with no error logged",
            )

    def test_scene_queries_are_enabled_exactly_when_the_lidar_is_built(self) -> None:
        """The flag and the rig must be driven by the SAME decision.

        Enabling scene queries without the rig pays PhysX cost for nothing;
        building the rig without them publishes empty clouds. Both bugs are
        silent, so they are pinned to one variable.
        """
        tree = _tree(RUN_SIM)
        calls = _backend_constructions(tree)
        flag_names = set()
        for call in calls:
            kw = _keyword(call, "scene_query_support")
            self.assertIsNotNone(kw, "covered by test_every_backend_construction")
            self.assertIsInstance(
                kw.value,
                ast.Name,
                "scene_query_support must be a named variable shared with the "
                "rig-construction guard, not an inline expression",
            )
            flag_names.add(kw.value.id)

        # Every `if <name>: ... build_lidar_rig(...)` must test one of them.
        guards = set()
        for node in ast.walk(tree):
            if not isinstance(node, ast.If):
                continue
            builds = any(
                isinstance(inner, ast.Call)
                and isinstance(inner.func, ast.Name)
                and inner.func.id == "build_lidar_rig"
                for inner in ast.walk(node)
            )
            if builds and isinstance(node.test, ast.Name):
                guards.add(node.test.id)

        self.assertTrue(guards, "no `if <name>:` guard around build_lidar_rig found")
        self.assertTrue(
            guards <= flag_names,
            f"build_lidar_rig is guarded by {sorted(guards)} but scene queries "
            f"are enabled by {sorted(flag_names)}; they must be the same "
            "decision or the two can silently disagree",
        )

    def test_one_predicate_decides_both(self) -> None:
        """Scene queries and the rig are ONE decision, in both directions.

        This used to assert the gate defaulted OFF. That was a statement about
        the default, not about this file's invariant, and it went stale when
        the live sensor became the default on 2026-09-11; the default itself is
        owned by ``test_raycast_lidar_gate.py``.

        What matters here is the coupling, and it is worth stating as a value
        and not only as the AST shape checked above. Queries without a rig pay
        PhysX for nothing; a rig without queries publishes empty clouds while
        reporting ``is_valid`` -- the silent failure this whole contract exists
        to prevent. So the same predicate must answer both questions, whatever
        it currently returns.
        """
        for profile in ("navigation-parity", "sensor-rich", "manipulation-core"):
            for qualification in (False, True):
                for map_lidar in (False, True):
                    decision = rs.raycast_lidar_enabled(
                        profile, qualification, map_lidar
                    )
                    self.assertIsInstance(
                        decision,
                        bool,
                        "the rig guard and the scene-query flag are handed "
                        "this one value; it must be a plain bool so neither "
                        "can coerce it differently",
                    )
                    if map_lidar:
                        self.assertFalse(
                            decision,
                            "--map-lidar must leave scene queries off as well "
                            "as the rig: paying for queries with no rig is "
                            "pure cost",
                        )


if __name__ == "__main__":
    unittest.main()
