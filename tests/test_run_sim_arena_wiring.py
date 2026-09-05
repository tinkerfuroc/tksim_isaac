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

from run_sim import (  # noqa: E402
    STABLE_AA_CAMERAS,
    _arena_camera_enabled,
    _stable_aa_requested,
    _with_arena_camera,
    _without_wrist_camera,
)
from tests.test_arena_camera import _Occ  # noqa: E402


class _S:  # minimal stand-in with tick_rate_hz/name
    def __init__(self, hz, name="robot_camera"):
        self.tick_rate_hz = hz
        self.name = name


def test_disabled_returns_originals():
    specs = (_S(30.0), _S(30.0))
    out, robot_hz = _with_arena_camera(specs, _Occ, {})
    assert out == specs and robot_hz == 30.0


def test_enabled_appends_arena_and_keeps_robot_hz():
    specs = (_S(30.0),)
    out, robot_hz = _with_arena_camera(specs, _Occ, {"TINKER_SIM_ARENA_CAMERA": "1"})
    assert len(out) == 2 and out[-1].name == "arena_camera"
    assert robot_hz == 30.0


def test_arena_camera_enabled_false_when_absent():
    assert _arena_camera_enabled((_S(30.0), _S(30.0))) is False


def test_arena_camera_enabled_true_when_appended():
    specs = (_S(30.0),)
    out, _ = _with_arena_camera(specs, _Occ, {"TINKER_SIM_ARENA_CAMERA": "1"})
    assert _arena_camera_enabled(out) is True


# --- Fix round 3: TINKER_SIM_DISABLE_WRIST_CAMERA (two-render-product cap) -

def test_wrist_camera_disabled_removes_only_wrist():
    specs = (_S(30.0, "head_camera"), _S(30.0, "wrist_camera"))
    out = _without_wrist_camera(specs, {"TINKER_SIM_DISABLE_WRIST_CAMERA": "1"})
    assert [s.name for s in out] == ["head_camera"]


def test_wrist_camera_disabled_accepts_truthy_literals():
    specs = (_S(30.0, "head_camera"), _S(30.0, "wrist_camera"))
    for literal in ("1", "true", "True", "yes", "YES"):
        out = _without_wrist_camera(specs, {"TINKER_SIM_DISABLE_WRIST_CAMERA": literal})
        assert [s.name for s in out] == ["head_camera"], literal


def test_wrist_camera_unset_leaves_specs_unchanged():
    specs = (_S(30.0, "head_camera"), _S(30.0, "wrist_camera"))
    out = _without_wrist_camera(specs, {})
    assert out == specs


def test_wrist_camera_falsy_leaves_specs_unchanged():
    specs = (_S(30.0, "head_camera"), _S(30.0, "wrist_camera"))
    out = _without_wrist_camera(specs, {"TINKER_SIM_DISABLE_WRIST_CAMERA": "0"})
    assert out == specs


def test_wrist_camera_disabled_then_arena_append_still_works():
    specs = (_S(30.0, "head_camera"), _S(30.0, "wrist_camera"))
    filtered = _without_wrist_camera(specs, {"TINKER_SIM_DISABLE_WRIST_CAMERA": "1"})
    out, robot_hz = _with_arena_camera(
        filtered, _Occ, {"TINKER_SIM_ARENA_CAMERA": "1"}
    )
    assert [s.name for s in out] == ["head_camera", "arena_camera"]
    # robot_min_hz is over the (already wrist-filtered) specs handed to
    # _with_arena_camera, not the original three-camera set.
    assert robot_hz == 30.0


def test_enabled_arena_honours_size_env():
    specs = (_S(30.0),)
    out, _ = _with_arena_camera(
        specs, _Occ,
        {"TINKER_SIM_ARENA_CAMERA": "1", "TINKER_SIM_ARENA_CAMERA_SIZE": "640x360"},
    )
    assert (out[-1].width, out[-1].height) == (640, 360)


def test_stable_aa_follows_arena_unless_forced():
    assert _stable_aa_requested(True, {}) is True
    assert _stable_aa_requested(False, {}) is False
    assert _stable_aa_requested(False, {"TINKER_SIM_STABLE_AA": "1"}) is True
    assert _stable_aa_requested(False, {"TINKER_SIM_STABLE_AA": "0"}) is False
    # the env can force DLAA *off* with the arena on, for the A/B only
    assert _stable_aa_requested(True, {"TINKER_SIM_STABLE_AA": "0"}) is False


def test_stable_aa_cameras_is_arena_only():
    # the DLAA pin, when applied, must be scoped to the arena render
    # product only -- not the head/wrist parity cameras (Task 4a's finding
    # that a global pin taxed them ~50ms/pump for no benefit).
    assert STABLE_AA_CAMERAS == frozenset({"arena_camera"})


def test_stable_aa_cameras_scope_follows_arena_presence():
    from run_sim import STABLE_AA_CAMERAS, _stable_aa_cameras

    assert _stable_aa_cameras(True) == STABLE_AA_CAMERAS == frozenset({"arena_camera"})
    assert _stable_aa_cameras(False) is None
