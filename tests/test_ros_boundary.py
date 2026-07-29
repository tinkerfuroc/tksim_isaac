from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.ros_boundary import contamination


class RosBoundaryTest(unittest.TestCase):
    def test_rejects_system_ros_and_python_310_paths(self) -> None:
        dirty = contamination(
            {
                "PYTHONPATH": "/project:/opt/ros/humble/lib/python3.10/site-packages",
                "AMENT_PREFIX_PATH": "/opt/ros/humble",
            }
        )
        self.assertEqual(set(dirty), {"PYTHONPATH", "AMENT_PREFIX_PATH"})

    def test_accepts_isolated_paths(self) -> None:
        self.assertEqual(
            contamination({"PYTHONPATH": "/project/tools", "LD_LIBRARY_PATH": "/usr/lib"}),
            {},
        )


if __name__ == "__main__":
    unittest.main()
