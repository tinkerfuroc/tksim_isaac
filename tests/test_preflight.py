from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.config import Config
from tinker_sim_deploy.preflight import collect
from tinker_sim_deploy.process import CommandResult


class PreflightTest(unittest.TestCase):
    def test_old_driver_is_warning_not_hidden_failure(self) -> None:
        def command_runner(command: list[str]) -> CommandResult:
            if command[0] == "nvidia-smi":
                output = "NVIDIA GeForce RTX 5070 Ti, 16384, 570.211.01\n"
            else:
                output = " V..... h264_nvenc NVIDIA NVENC H.264 encoder\n"
            return CommandResult(tuple(command), 0, output, "")

        with (
            mock.patch(
                "tinker_sim_deploy.preflight._os_release",
                return_value={"ID": "ubuntu", "VERSION_ID": "22.04"},
            ),
            mock.patch("tinker_sim_deploy.preflight.platform.machine", return_value="x86_64"),
            mock.patch("tinker_sim_deploy.preflight._glibc", return_value="2.35"),
            mock.patch("tinker_sim_deploy.preflight._ram_gb", return_value=64.0),
            mock.patch(
                "tinker_sim_deploy.preflight.shutil.disk_usage",
                return_value=SimpleNamespace(free=120 * 1024**3),
            ),
            mock.patch(
                "tinker_sim_deploy.preflight.shutil.which",
                side_effect=lambda name: f"/usr/bin/{name}",
            ),
        ):
            checks = {check.name: check for check in collect(Config.load(ROOT), command_runner)}
        self.assertEqual(checks["gpu"].status, "pass")
        self.assertEqual(checks["vram"].status, "pass")
        self.assertEqual(checks["driver"].status, "warn")
        self.assertIn("experimental", checks["driver"].detail)


if __name__ == "__main__":
    unittest.main()
