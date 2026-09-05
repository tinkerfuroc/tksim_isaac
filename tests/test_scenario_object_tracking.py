from __future__ import annotations

import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
RUN_SIM = ROOT / "validation/run_sim.py"

# Constructor names that stand up a robot backend for a sensor-profile branch
# in run_sim.py's main loop.  Every one of these must pass expected_objects=
# and scenario= through to the backend, or /sim/internal/physics_truth can
# never see scenario-spawned objects for that profile: the backend defaults
# expected_objects to None (-> {}), and _refresh_object_views() returns on
# its first line when self._expected_objects is empty (backend.py:927-928).
BACKEND_CONSTRUCTOR_NAMES = frozenset({"IsaacNavigationBackend", "IsaacWholeRobotBackend"})
REQUIRED_KEYWORDS = frozenset({"expected_objects", "scenario"})


class ScenarioObjectTrackingContractTest(unittest.TestCase):
    """Guard the 2026-08-20 defect: physics_truth reported no scenario
    objects under navigation-parity across 499 live messages because that
    branch never passed expected_objects/scenario to the backend.

    This is AST-only so it runs under system python without Isaac/torch, per
    the pattern in test_safety_contract.py and
    ActorPathDriverNodeAttributeTest in test_actor_paths.py.
    """

    def setUp(self) -> None:
        self.assertTrue(RUN_SIM.is_file(), f"{RUN_SIM} does not exist")
        self.tree = ast.parse(RUN_SIM.read_text(encoding="utf-8"))

    def _backend_constructor_calls(self) -> list[ast.Call]:
        calls = []
        for node in ast.walk(self.tree):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            name = func.id if isinstance(func, ast.Name) else getattr(func, "attr", None)
            if name in BACKEND_CONSTRUCTOR_NAMES:
                calls.append(node)
        return calls

    def test_at_least_three_backend_constructor_sites_exist(self) -> None:
        # Locks in the shape described by the 2026-08-20 root cause: three
        # sensor-profile branches (navigation-parity, sensor-rich,
        # manipulation-core) each construct a robot backend. If a branch is
        # added or removed this count should be revisited deliberately.
        calls = self._backend_constructor_calls()
        self.assertGreaterEqual(
            len(calls),
            3,
            "expected at least 3 IsaacNavigationBackend/IsaacWholeRobotBackend "
            "construction sites in run_sim.py (one per sensor-profile branch)",
        )

    def test_every_backend_constructor_call_passes_expected_objects_and_scenario(self) -> None:
        calls = self._backend_constructor_calls()
        self.assertTrue(calls, "no backend constructor calls found in run_sim.py")
        for node in calls:
            keywords = {kw.arg for kw in node.keywords if kw.arg is not None}
            missing = REQUIRED_KEYWORDS - keywords
            self.assertFalse(
                missing,
                f"backend constructor call at run_sim.py:{node.lineno} is missing "
                f"keyword(s) {sorted(missing)}; without expected_objects/scenario "
                "the backend's _refresh_object_views() returns immediately and "
                "physics_truth never reports scenario objects for this profile",
            )


if __name__ == "__main__":
    unittest.main()
