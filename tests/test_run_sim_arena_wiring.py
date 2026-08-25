"""Task 3: ``run_sim``'s opt-in wiring of the arena observer camera.

Covers ``run_sim._with_arena_camera``, the pure helper the sensor-rich
camera block calls right after the head-aim correction: when the arena
camera is disabled it returns the original specs unchanged; when enabled
(``TINKER_SIM_ARENA_CAMERA``) it appends the arena camera's
``CameraStreamSpec`` without lowering the robot cameras' resolved cadence
(``robot_min_hz`` is always ``min(tick_rate_hz)`` over the *original*
specs only).
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "simulation"))

from run_sim import _with_arena_camera  # noqa: E402
from tests.test_arena_camera import _Occ  # noqa: E402


class _S:  # minimal stand-in with tick_rate_hz
    def __init__(self, hz):
        self.tick_rate_hz = hz


def test_disabled_returns_originals():
    specs = (_S(30.0), _S(30.0))
    out, robot_hz = _with_arena_camera(specs, _Occ, {})
    assert out == specs and robot_hz == 30.0


def test_enabled_appends_arena_and_keeps_robot_hz():
    specs = (_S(30.0),)
    out, robot_hz = _with_arena_camera(specs, _Occ, {"TINKER_SIM_ARENA_CAMERA": "1"})
    assert len(out) == 2 and out[-1].name == "arena_camera"
    assert robot_hz == 30.0
