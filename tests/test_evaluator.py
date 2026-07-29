from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.evaluator import PostconditionEvaluator


class PostconditionEvaluatorTest(unittest.TestCase):
    def test_rejects_claim_when_world_postcondition_is_false(self) -> None:
        conditions = json.loads(
            (ROOT / "simulation/scenarios/pick-deliver-place.json").read_text()
        )["postconditions"]
        truth = {
            "delivery": {
                "object_in_target": False,
                "retained_by_gripper": False,
                "object_speed": 0.0,
            },
            "robot": {"safety_stop": False},
        }
        evaluation = PostconditionEvaluator().evaluate(conditions, truth)
        self.assertFalse(evaluation.success)
        self.assertEqual(evaluation.failed, ("correct object in target region",))

    def test_accepts_only_complete_reception_state(self) -> None:
        conditions = json.loads(
            (ROOT / "simulation/scenarios/reception-seat-assignment.json").read_text()
        )["postconditions"]
        truth = {
            "reception": {
                "identified_guests": ["guest_2", "guest_1"],
                "assigned_guests": ["guest_1", "guest_2"],
                "confirmed": True,
            }
        }
        self.assertTrue(PostconditionEvaluator().evaluate(conditions, truth).success)


if __name__ == "__main__":
    unittest.main()
