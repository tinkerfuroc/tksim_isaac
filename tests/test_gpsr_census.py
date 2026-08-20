"""Regression guard for the GPSR interface census's action-name matching.

`_action_present` must normalize both bare (``listen_action``) and
already-slashed (``/xarm_gripper/gripper_action``) ACTIONS entries to the
leading-slash form that the ROS graph reports actions under, since a prior
version of this check could never match a bare-named entry and always
reported it missing. This test needs no ROS: it only imports the pure
function, which the module keeps free of any rclpy import at module scope.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from gpsr_interface_census import _action_present


class ActionPresentTest(unittest.TestCase):
    def test_bare_name_matches_slashed_graph_entry(self):
        self.assertTrue(_action_present("listen_action", {"/listen_action"}))

    def test_already_slashed_name_matches(self):
        self.assertTrue(
            _action_present(
                "/xarm_gripper/gripper_action", {"/xarm_gripper/gripper_action"}
            )
        )

    def test_missing_action_reported_absent(self):
        self.assertFalse(_action_present("listen_action", set()))


if __name__ == "__main__":
    unittest.main()
