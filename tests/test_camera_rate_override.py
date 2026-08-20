"""The sensor-rich camera cadence must be overridable without editing parity.

`camera_hz` drives two of the three dominant costs in the sensor-rich loop:
Kit is pumped (and both RTX cameras rendered) once per `camera_stride` physics
frames, and the camera payloads are published on the same beat. At the parity
file's 30 Hz and a 120 Hz physics dt that is stride 4 -- 30 renders and ~9 MB
of image payload per simulated second.

Measured live on 2026-08-20 (domain 71, gpsr-rcw2026, full GPSR stack up):
the simulator ran at RTF 0.296 alone but ~0.06 with the stack subscribed, and
Nav2's lifecycle manager tore its own stack down ("Switch controller timed out
after 2.000000 seconds") because bond heartbeats could not be serviced.

`simulation/sensors/hardware-parity.json` states what the hardware does and
must stay authoritative for hardware runs, so the override is an explicit
opt-in env knob rather than an edit to that file -- the same split already
used for the arm profile (config/controllers.sim-clock.yaml).
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "simulation"))

RUN_SIM = ROOT / "validation/run_sim.py"
PARITY = ROOT / "simulation/sensors/hardware-parity.json"


def _load_resolver():
    """Import just the resolver without importing Isaac."""
    import importlib.util

    spec = importlib.util.spec_from_file_location("_run_sim_probe", RUN_SIM)
    module = importlib.util.module_from_spec(spec)
    # run_sim imports Isaac lazily inside main(); module level is safe.
    spec.loader.exec_module(module)
    return module


class CameraRateOverrideTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_resolver()

    def test_default_is_the_parity_rate(self):
        """With no override the hardware value must win, unchanged."""
        resolve = self.module._resolve_camera_hz
        self.assertEqual(resolve(30.0, None), 30.0)
        self.assertEqual(resolve(30.0, ""), 30.0)

    def test_override_replaces_the_parity_rate(self):
        resolve = self.module._resolve_camera_hz
        self.assertEqual(resolve(30.0, "15"), 15.0)
        self.assertEqual(resolve(30.0, "15.0"), 15.0)

    def test_override_may_not_exceed_the_hardware_rate(self):
        """Publishing faster than the real camera would break parity."""
        resolve = self.module._resolve_camera_hz
        with self.assertRaises(ValueError):
            resolve(30.0, "60")

    def test_override_rejects_nonsense(self):
        resolve = self.module._resolve_camera_hz
        for bad in ("0", "-5", "abc", "nan", "inf"):
            with self.assertRaises(ValueError, msg=f"accepted {bad!r}"):
                resolve(30.0, bad)

    def test_fifteen_hz_halves_the_kit_pump_rate(self):
        """The point of the knob: half the renders and half the payload."""
        stride = self.module._streaming_update_stride
        dt = 1.0 / 120.0
        self.assertEqual(stride(dt, update_hz=30.0), 4)
        self.assertEqual(stride(dt, update_hz=15.0), 8)

    def test_parity_file_still_declares_the_hardware_rate(self):
        """The override must not have been implemented by editing parity."""
        import json

        spec = json.loads(PARITY.read_text(encoding="utf-8"))
        self.assertEqual(spec["head_camera"]["tick_rate_hz"], 30)
        self.assertEqual(spec["wrist_camera"]["tick_rate_hz"], 30)


if __name__ == "__main__":
    unittest.main()
