from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "validation"))

import run_sim
from tinker_sim_deploy import cli


def _parse_cli_launch(argv):
    return cli._parser().parse_args(argv)


class SensorRichLaunchTest(unittest.TestCase):
    def test_sensor_rich_implies_ros(self) -> None:
        self.assertTrue(run_sim.sensor_rich_implies_ros("sensor-rich", False))
        self.assertFalse(run_sim.sensor_rich_implies_ros("sensor-rich", True))
        self.assertFalse(run_sim.sensor_rich_implies_ros("physics-only", False))

    def test_sensor_rich_enables_development_lidar(self) -> None:
        self.assertTrue(run_sim.gateway_lidar_enabled("sensor-rich", False))
        self.assertTrue(run_sim.gateway_lidar_enabled("sensor-rich", True))

    def test_launcher_forwards_camera_flags(self) -> None:
        args = _parse_cli_launch(
            [
                "launch",
                "--sensor-profile",
                "sensor-rich",
                "--ros",
                "--camera-pointcloud",
                "--arena-colors",
            ]
        )
        self.assertEqual(
            cli._camera_stream_arguments(args),
            ["--camera-pointcloud", "--arena-colors"],
        )

    def test_launcher_omits_camera_flags_by_default(self) -> None:
        args = _parse_cli_launch(
            ["launch", "--sensor-profile", "sensor-rich", "--ros"]
        )
        self.assertEqual(cli._camera_stream_arguments(args), [])


if __name__ == "__main__":
    unittest.main()
