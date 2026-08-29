import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.arena_camera import (  # noqa: E402
    ARENA_CAMERA_DEFAULT_HZ, arena_camera_pose, arena_camera_spec,
    look_at_wxyz, resolve_arena_camera,
)

class _Occ:
    width, height, resolution, origin_x, origin_y = 100, 80, 0.1, -5.0, -4.0

def _rotate(q, v):
    w, x, y, z = q
    # quaternion-vector rotation q v q*
    t = (2*(y*v[2]-z*v[1]), 2*(z*v[0]-x*v[2]), 2*(x*v[1]-y*v[0]))
    return (v[0]+w*t[0]+y*t[2]-z*t[1], v[1]+w*t[1]+z*t[0]-x*t[2], v[2]+w*t[2]+x*t[1]-y*t[0])

def test_look_at_points_optical_z_at_target():
    eye, target = (0.0, 0.0, 5.0), (2.0, 1.0, 0.0)
    fwd = _rotate(look_at_wxyz(eye, target), (0.0, 0.0, 1.0))  # optical +Z
    want = [t-e for t, e in zip(target, eye)]
    n = math.sqrt(sum(c*c for c in want))
    for got, exp in zip(fwd, [c/n for c in want]):
        assert got == pytest.approx(exp, abs=1e-6)

def test_look_at_rejects_degenerate():
    with pytest.raises(ValueError):
        look_at_wxyz((1.0, 1.0, 1.0), (1.0, 1.0, 1.0))

def test_resolve_disabled_by_default():
    assert resolve_arena_camera({}) is None
    assert resolve_arena_camera({"TINKER_SIM_ARENA_CAMERA": "0"}) is None

def test_resolve_enabled_and_rate_only_lowers():
    assert ARENA_CAMERA_DEFAULT_HZ == 2.0
    assert resolve_arena_camera({"TINKER_SIM_ARENA_CAMERA": "1"}) == ARENA_CAMERA_DEFAULT_HZ
    env = {"TINKER_SIM_ARENA_CAMERA": "1", "TINKER_SIM_ARENA_CAMERA_HZ": "2"}
    assert resolve_arena_camera(env) == 2.0
    env["TINKER_SIM_ARENA_CAMERA_HZ"] = "30"   # may only lower
    assert resolve_arena_camera(env) == ARENA_CAMERA_DEFAULT_HZ
    env["TINKER_SIM_ARENA_CAMERA_HZ"] = "junk"
    with pytest.raises(ValueError):
        resolve_arena_camera(env)

def test_spec_shape():
    spec = arena_camera_spec(_Occ, hz=4.0)
    assert spec.name == "arena_camera"
    assert spec.color_topic == "/sim/arena_camera/image_raw"
    assert spec.depth_topic == ""            # color-only stream
    assert spec.camera_info_topics == ("/sim/arena_camera/camera_info",)
    assert spec.mount_prim == "/World/ArenaCamera"
    assert spec.mount_translation is not None
    assert (spec.width, spec.height) == (640, 360)
    assert spec.tick_rate_hz == 4.0

def test_pose_matches_run_sim_contract():
    eye, target, bounds = arena_camera_pose(_Occ)
    assert bounds == [-5.0, -4.0, 5.0, 4.0]
    assert eye[2] > target[2]


def test_spec_rotation_points_usd_camera_forward_at_target():
    # The rig authors mount_rotation_wxyz directly onto the USD camera prim,
    # which renders along its local -Z. The spec's rotation must therefore
    # map (0, 0, -1) onto the eye->target direction — look_at_wxyz alone
    # (optical convention, +Z forward) faces exactly backward.
    from tinker_sim_isaac.arena_camera import arena_camera_pose

    spec = arena_camera_spec(_Occ, hz=4.0)
    eye, target, _ = arena_camera_pose(_Occ)
    fwd = _rotate(spec.mount_rotation_wxyz, (0.0, 0.0, -1.0))  # USD cam fwd
    import math as _math

    d = [t - e for t, e in zip(target, eye)]
    n = _math.sqrt(sum(c * c for c in d))
    d = [c / n for c in d]
    assert all(abs(a - b) < 1e-9 for a, b in zip(fwd, d))


def test_spec_rotation_keeps_image_upright():
    # USD camera up (+Y) must map to a world direction with positive Z:
    # the bird's-eye view renders the arena with the sky up, not rolled
    # 180 degrees (people head-down, observed live 2026-08-27 with the
    # x-flip composition).
    spec = arena_camera_spec(_Occ, hz=4.0)
    up = _rotate(spec.mount_rotation_wxyz, (0.0, 1.0, 0.0))
    assert up[2] > 0.0


def test_resolve_size_default_and_only_lowers():
    from tinker_sim_isaac.arena_camera import (
        ARENA_CAMERA_DEFAULT_SIZE, resolve_arena_camera_size,
    )
    assert resolve_arena_camera_size({}) == ARENA_CAMERA_DEFAULT_SIZE == (640, 360)
    assert resolve_arena_camera_size({"TINKER_SIM_ARENA_CAMERA_SIZE": "640x360"}) == (640, 360)
    # may only lower: a larger request is clamped to the default
    assert resolve_arena_camera_size({"TINKER_SIM_ARENA_CAMERA_SIZE": "960x540"}) == (640, 360)
    for bad in ("640", "640x", "axb", "0x0", "-1x10"):
        with pytest.raises(ValueError):
            resolve_arena_camera_size({"TINKER_SIM_ARENA_CAMERA_SIZE": bad})


def test_spec_takes_size():
    spec = arena_camera_spec(_Occ, hz=2.0, size=(640, 360))
    assert (spec.width, spec.height) == (640, 360)
    assert spec.horizontal_fov_deg == 70.0   # FOV is independent of size
