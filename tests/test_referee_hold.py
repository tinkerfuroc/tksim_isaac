"""Referee hand-off retention (RefereeHold wiring, kinematic follow).

fix5 (2026-09-30) runs 003/004: the referee-handed banana fell to the floor
as soon as the arm lifted. The backend now pins the handed object to
link_tcp (+) the hand-off offset every physics step until the tree commands
the gripper open. The pose writes themselves are Isaac-only; here the pure
state machine, the follow-pose math and the backend wiring are driven with
the stage/physics writers stubbed on the instance.
"""
from __future__ import annotations

import json
import math
import random
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.command_mux import JointCommand  # noqa: E402
from tinker_sim_isaac.backend import (  # noqa: E402
    IsaacWholeRobotBackend,
    RefereeHold,
    pose_from_frame,
    pose_in_frame,
    rotate_vector_xyzw,
)
from tinker_sim_isaac import scoped_reset  # noqa: E402

PATH = "/World/Scenario/cmd_banana_0"
OPEN, CLOSED = 0.0, 0.78


def _yaw(deg):
    half = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def _same_rotation(q1, q2, tol=1e-9):
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    return abs(dot - 1.0) <= tol


# ------------------------------------------------------------- pure math

def test_pose_from_frame_inverts_pose_in_frame():
    rng = random.Random(7)
    for _ in range(50):
        q = [rng.uniform(-1, 1) for _ in range(4)]
        n = math.sqrt(sum(v * v for v in q))
        frame_q = tuple(v / n for v in q)
        frame_p = tuple(rng.uniform(-3, 3) for _ in range(3))
        world_p = tuple(rng.uniform(-3, 3) for _ in range(3))
        world_q = _yaw(rng.uniform(-180, 180))
        local_p, local_q = pose_in_frame(frame_p, frame_q, world_p, world_q)
        back_p, back_q = pose_from_frame(frame_p, frame_q, local_p, local_q)
        assert _close(back_p, world_p, 1e-9)
        assert _same_rotation(back_q, world_q, 1e-9)


def test_follow_pose_tracks_tcp_translation_and_rotation():
    hold = RefereeHold()
    tcp0 = ((1.0, 2.0, 1.15), _yaw(90))
    obj0 = ((1.0, 2.0, 1.15), _yaw(30))  # teleported to the TCP, own yaw kept
    hold.attach(PATH, OPEN, tcp_pose=tcp0, object_pose=obj0)
    # Unmoved TCP -> object stays exactly where it was handed.
    p, q = hold.follow_pose(*tcp0)
    assert _close(p, obj0[0]) and _same_rotation(q, obj0[1])
    # Lift 0.45 m and swing the TCP another +45 deg about z.
    p, q = hold.follow_pose((1.0, 2.0, 1.60), _yaw(135))
    assert _close(p, (1.0, 2.0, 1.60))
    assert _same_rotation(q, _yaw(75))


def test_follow_pose_rotates_a_nonzero_offset_with_the_tcp():
    hold = RefereeHold()
    hold.attach(PATH, OPEN, tcp_pose=((0, 0, 1), _yaw(0)), object_pose=((0.05, 0, 1), _yaw(0)))
    p, _ = hold.follow_pose((0.3, 0.0, 1.0), _yaw(90))
    assert _close(p, (0.3, 0.05, 1.0), 1e-9)


def test_follow_pose_none_when_not_held_and_defaults_to_tcp():
    hold = RefereeHold()
    assert hold.follow_pose((0, 0, 0), (0, 0, 0, 1)) is None
    hold.attach(PATH, OPEN)  # no poses: rides at the TCP origin
    p, q = hold.follow_pose((1, 1, 1), _yaw(10))
    assert _close(p, (1, 1, 1)) and _same_rotation(q, _yaw(10))
    assert hold.clear() == PATH and not hold.held
    assert hold.local_position == (0.0, 0.0, 0.0)


# ------------------------------------------------------ state machine stream

@pytest.mark.parametrize("attach_target, stream, release_at", [
    # fix5 sequence: open -> hand-off -> close -> (keepalives) -> open (place)
    (OPEN, [CLOSED, CLOSED, CLOSED, OPEN], 3),
    # hand-off with the gripper already closed; re-send, then open
    (CLOSED, [CLOSED, CLOSED, 0.5], 2),
    # partial close jitter below the delta never releases
    (OPEN, [CLOSED, CLOSED - 0.03, CLOSED - 0.049, CLOSED], None),
    # unknown target at attach: first command becomes the peak
    (None, [CLOSED, CLOSED - 0.2], 1),
])
def test_command_stream_release_point(attach_target, stream, release_at):
    hold = RefereeHold()
    hold.attach(PATH, attach_target)
    released = [i for i, t in enumerate(stream) if hold.on_drive_target(t)]
    assert (released[:1] or [None])[0] == release_at


def test_not_held_never_releases():
    assert RefereeHold().on_drive_target(0.0) is False


# ------------------------------------------------------------ backend wiring

class _Targets:
    dtype = "f32"
    device = "cpu"

    def __setitem__(self, key, value):
        pass

    def __getitem__(self, key):
        raise IndexError


def _backend(tcp_pose=((1.0, 2.0, 1.15), _yaw(90)), object_quat=_yaw(30)):
    b = object.__new__(IsaacWholeRobotBackend)
    state = {"tcp": tcp_pose}
    b._tcp_world_pose = lambda: None if state["tcp"] is None else (
        list(state["tcp"][0]), list(state["tcp"][1]))
    b._rigid_body_world_pose = lambda path: (
        None if object_quat is None else (tuple(state["tcp"][0]), object_quat))
    writes = []
    b._write_held_body_pose = lambda path, p, q: writes.append((path, p, q))
    b._joint_index = {"drive_joint": 0}
    b._torch = SimpleNamespace(tensor=lambda values, dtype, device: values)
    b._position_targets = _Targets()
    b._velocity_targets = _Targets()
    b._referee_hold_now = lambda: 12.5
    return b, state, writes


def _logs(capsys):
    return [json.loads(line) for line in capsys.readouterr().out.splitlines()
            if line.startswith("{")]


def test_hand_object_attaches_and_follow_writes_every_step(capsys):
    b, state, writes = _backend()
    b._resolve_handover_entity = lambda e: {"path": PATH}
    b._tcp_world_position = lambda: list(state["tcp"][0])
    b._teleport_rigid_body = lambda path, xyz: "usd+view"
    b._drive_command_target = OPEN
    ack = b.hand_object_to_gripper("banana")
    assert ack["ok"] is True
    logs = _logs(capsys)
    attach = [l for l in logs if l.get("referee_hold") == "attach"]
    assert attach and attach[0]["path"] == PATH and attach[0]["t"] == 12.5
    assert attach[0]["orientation_source"] == "physx"
    # The existing hand-off log is still the last JSON line.
    assert logs[-1]["referee_hand_object"] == PATH
    # Arm lifts 1.12 m over three physics steps: the object rides along.
    for z in (1.15, 1.6, 2.27):
        state["tcp"] = ((1.0, 2.0, z), _yaw(90))
        b._referee_hold_follow()
    assert [round(w[1][2], 6) for w in writes] == [1.15, 1.6, 2.27]
    assert all(w[0] == PATH and _same_rotation(w[2], _yaw(30)) for w in writes)


def test_joint_command_stream_releases_on_open(capsys):
    b, state, writes = _backend()
    b._drive_command_target = OPEN
    assert b._referee_hold_attach(PATH, state["tcp"][0]) is True
    for target in (CLOSED, CLOSED, CLOSED):
        b._apply_joint_command(JointCommand(names=("drive_joint",), positions=(target,)))
        b._referee_hold_follow()
    assert b._referee_hold.held
    before = len(writes)
    b._set_gripper_effort_limit = lambda value: None
    b._apply_joint_command(JointCommand(names=("drive_joint",), positions=(OPEN,), efforts=(50.0,)))
    assert not b._referee_hold.held
    # release wrote one zero-velocity (pose None) update, then follow is a no-op
    assert writes[before] == (PATH, None, None)
    b._referee_hold_follow()
    assert len(writes) == before + 1
    release = [l for l in _logs(capsys) if l.get("referee_hold") == "release"]
    assert release == [{"referee_hold": "release", "path": PATH, "t": 12.5,
                        "reason": "gripper_open", "drive_target": OPEN}]


def test_apply_joint_command_without_hold_is_unchanged():
    b, _, writes = _backend()
    b._apply_joint_command(JointCommand(names=("drive_joint",), positions=(CLOSED,)))
    assert b._drive_command_target == CLOSED and writes == []


def test_attach_fails_soft_without_tcp_pose(capsys):
    b, state, writes = _backend(tcp_pose=None)
    assert b._referee_hold_attach(PATH, (0, 0, 1)) is False
    assert not b._referee_hold.held
    assert _logs(capsys)[-1]["referee_hold"] == "attach_failed"


def test_attach_orientation_falls_back_to_tcp(capsys):
    b, state, writes = _backend(object_quat=None)
    b._referee_hold_attach(PATH, state["tcp"][0])
    assert _logs(capsys)[-1]["orientation_source"] == "tcp"
    b._referee_hold_follow()
    assert _same_rotation(writes[-1][2], _yaw(90))


def test_delete_entity_releases_by_path_or_name_only_for_held(capsys):
    b, state, writes = _backend()
    b._referee_hold_attach(PATH, state["tcp"][0])
    assert b.release_referee_hold_for("/World/Scenario/other") is None
    assert b.release_referee_hold_for("cmd_banana_0") == PATH
    assert writes == []  # no write to a prim that is about to be deleted
    assert _logs(capsys)[-1] == {"referee_hold": "release", "path": PATH, "t": 12.5,
                                 "reason": "deleted"}
    b._referee_hold_attach(PATH, state["tcp"][0])
    assert b.release_referee_hold_for(PATH) == PATH


def test_scoped_reset_delete_wrapper_releases_hold():
    calls = []
    backend = SimpleNamespace(release_referee_hold_for=lambda e: calls.append(e))
    reset = SimpleNamespace(gateway=SimpleNamespace(backend=backend))
    scoped_reset.release_referee_hold_for_delete(reset, SimpleNamespace(entity=PATH))
    assert calls == [PATH]
    # A backend without the hook (or a raising one) never blocks the delete.
    scoped_reset.release_referee_hold_for_delete(SimpleNamespace(gateway=None), None)

    def boom(entity):
        raise RuntimeError("x")

    backend.release_referee_hold_for = boom
    scoped_reset.release_referee_hold_for_delete(reset, SimpleNamespace(entity=PATH))


def test_follow_releases_when_prim_vanishes_or_writes_keep_failing(capsys):
    b, state, _ = _backend()
    b._referee_hold_attach(PATH, state["tcp"][0])

    def gone(path, p, q):
        raise RuntimeError(f"no prim at {path}")

    b._write_held_body_pose = gone
    b._referee_hold_follow()
    assert not b._referee_hold.held
    assert _logs(capsys)[-1]["reason"] == "prim_gone"

    b._write_held_body_pose = lambda *a: None
    b._referee_hold_attach(PATH, state["tcp"][0])

    def flaky(path, p, q):
        raise RuntimeError("Failed to set transforms")

    b._write_held_body_pose = flaky
    for _ in range(IsaacWholeRobotBackend.REFEREE_HOLD_MAX_WRITE_FAILURES - 1):
        b._referee_hold_follow()
    assert b._referee_hold.held
    b._referee_hold_follow()
    assert not b._referee_hold.held
    assert _logs(capsys)[-1]["reason"] == "write_failed"


def test_rehandoff_replaces_previous_hold(capsys):
    b, state, _ = _backend()
    b._referee_hold_attach(PATH, state["tcp"][0])
    b._referee_hold_attach("/World/Scenario/cmd_soup_0", state["tcp"][0])
    assert b._referee_hold.path == "/World/Scenario/cmd_soup_0"
    events = [(l["referee_hold"], l.get("reason")) for l in _logs(capsys)]
    assert events == [("attach", None), ("release", "re_handoff"), ("attach", None)]


def test_scoped_reset_restore_releases_hold():
    released = []
    backend = SimpleNamespace(
        _safety_stopped=True,
        release_referee_hold=lambda reason: released.append(reason),
    )
    baseline = object.__new__(scoped_reset.PhysicsBaseline)
    baseline.backend = backend
    with pytest.raises(AttributeError):  # stops at the (absent) robot handle
        baseline.restore()
    assert released == ["scoped_reset"]


def test_rotate_helper_convention_sanity():
    # +90 deg yaw maps +x to +y (scalar-last convention used by follow_pose)
    assert _close(rotate_vector_xyzw(_yaw(90), (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0), 1e-12)
