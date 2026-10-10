"""Unit tests for the GPSR run-truth probe's pure-Python parts.

The ROS node wrapper is exercised live; here we pin the accumulator, the
placements copy and the on-shutdown write that the tier-2 bench oracle depends
on. rclpy is imported lazily inside ``main()``, so the module imports standalone;
``main()`` itself is driven once with a stub ``rclpy`` whose ``spin`` raises
``KeyboardInterrupt`` to prove the summary lands on shutdown.
"""
from __future__ import annotations

import json
import math
import signal
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
_TOOLS = str(ROOT / "tools")
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)

import gpsr_run_truth_probe as probe  # noqa: E402
from gpsr_run_truth_probe import RunTruthTracker, load_placements, yaw_deg_from_quaternion  # noqa: E402

PLACEMENTS = ROOT / "simulation" / "scenarios" / "rcw2026-placements.json"


def _quat_z(yaw_deg: float) -> tuple[float, float, float, float]:
    h = math.radians(yaw_deg) / 2.0
    return 0.0, 0.0, math.sin(h), math.cos(h)


def test_no_samples_any_object_retained_is_none_and_schema_keys_present():
    s = RunTruthTracker().summary()
    assert s["schema_version"] == 1
    assert s["samples"] == 0
    assert s["any_object_retained"] is None
    assert s["objects"] == {}
    assert s["robot"] is None
    assert s["surfaces"] == {} and s["persons"] == {}


def test_closed_on_air_never_retained():
    t = RunTruthTracker()
    for i in range(20):
        t.update_object("cmd_soup_0", False, 0.43 + 0.00003 * i, 1.755, 0.612)
    s = t.summary()
    assert s["any_object_retained"] is False
    o = s["objects"]["cmd_soup_0"]
    assert o["ever_retained"] is False and o["retained_at_end"] is False
    assert o["max_xy_disp_m"] < 0.005 and o["samples"] == 20
    assert s["samples"] == 20


def test_retention_sticky_and_retained_at_end_tracks_last_sample():
    t = RunTruthTracker()
    t.update_object("cmd_soup_0", True, 0.43, 1.755, 0.70)
    t.update_object("cmd_soup_0", False, 0.43, 1.755, 0.612)
    o = t.summary()["objects"]["cmd_soup_0"]
    assert o["ever_retained"] is True
    assert o["retained_at_end"] is False
    assert t.summary()["any_object_retained"] is True

    t.update_object("cmd_soup_0", True, 0.43, 1.755, 0.70)
    assert t.summary()["objects"]["cmd_soup_0"]["retained_at_end"] is True


def test_any_retained_is_or_across_objects():
    t = RunTruthTracker()
    t.update_object("cmd_cheez_it_0", False, 0.61, 1.755, 0.612)
    t.update_object("cmd_soup_0", True, 0.43, 1.755, 0.70)
    assert t.summary()["any_object_retained"] is True


def test_lift_displacement_first_and_last_xyz():
    t = RunTruthTracker()
    for i in range(10):
        t.update_object("cmd_soup_0", i >= 5, 0.43 + 0.05 * i, 1.755, 0.612 + 0.02 * i)
    # Put it back down lower than it started: max_lift must stay at the peak, not go negative.
    t.update_object("cmd_soup_0", False, 2.5, -3.0, 0.60)
    o = t.summary()["objects"]["cmd_soup_0"]
    assert o["first_xyz"] == [0.43, 1.755, 0.612]
    assert o["last_xyz"] == [2.5, -3.0, 0.6]
    assert o["max_lift_m"] == pytest.approx(0.18, abs=1e-6)
    assert o["max_xy_disp_m"] == pytest.approx(math.hypot(2.5 - 0.43, -3.0 - 1.755), abs=1e-4)


@pytest.mark.parametrize("yaw", [0.0, 45.0, 90.0, -90.0, -135.0, 179.0, -179.0])
def test_yaw_from_quaternion_round_trips(yaw):
    assert yaw_deg_from_quaternion(*_quat_z(yaw)) == pytest.approx(yaw, abs=1e-6)


def test_robot_block_reports_last_pose_and_yaw():
    t = RunTruthTracker()
    t.update_robot(0.0, 0.0, 0.0, *_quat_z(0.0))
    t.update_robot(-3.227, 0.769, 0.0, *_quat_z(-135.0))
    r = t.summary()["robot"]
    assert r["last_xyz"] == [-3.227, 0.769, 0.0]
    assert r["last_yaw_deg"] == pytest.approx(-135.0, abs=1e-2)
    assert r["samples"] == 2
    # Robot samples do not count as object samples, so the retention verdict stays undecided.
    assert t.summary()["samples"] == 0
    assert t.summary()["any_object_retained"] is None


def test_surfaces_and_persons_copied_from_placements():
    placements = json.loads(PLACEMENTS.read_text())
    surfaces, persons = load_placements(placements)
    assert set(surfaces) == set(placements["spots"])
    assert set(persons) == set(placements["persons"]) | {"operator"}
    assert persons["operator"] == {"xyz": placements["operator"]["xyz"]}
    shelf = surfaces["shelf"]
    assert shelf["surface_xyz"] == placements["spots"]["shelf"]["surface_xyz"]
    # placements.json carries the measured extent; the grid*3 rule is only the fallback
    assert shelf["half_extent_xy"] == placements["spots"]["shelf"]["half_extent_xy"]
    legacy, _ = load_placements({"spots": {"x": {"surface_xyz": [0, 0, 0], "grid_dx": 0.18, "grid_dy": 0.15}},
                                 "persons": {}})
    assert legacy["x"]["half_extent_xy"] == [pytest.approx(0.18 * 3), pytest.approx(0.15 * 3)]
    assert persons["kitchen"] == {"xyz": placements["persons"]["kitchen"]["xyz"]}
    s = RunTruthTracker(surfaces=surfaces, persons=persons).summary()
    assert s["surfaces"]["shelf"]["half_extent_xy"] == shelf["half_extent_xy"]
    assert s["persons"]["living_room"]["xyz"] == placements["persons"]["living_room"]["xyz"]


def test_extents_json_overrides_grid_half_extent():
    placements = json.loads(PLACEMENTS.read_text())
    audit = {"furniture": {"shelf": {"half_extent_xy": [0.4275, 0.1485]}}}
    surfaces, _ = load_placements(placements, audit)
    assert surfaces["shelf"]["half_extent_xy"] == [0.4275, 0.1485]
    assert surfaces["sofa"]["half_extent_xy"] == placements["spots"]["sofa"]["half_extent_xy"]


def test_output_path_dir_vs_file(tmp_path):
    assert probe.output_path(str(tmp_path)) == tmp_path / "run_truth.json"
    assert probe.output_path(str(tmp_path / "x.json")) == tmp_path / "x.json"


def _install_fake_rclpy(monkeypatch, spin_exc):
    """Stub the lazily-imported ROS modules; ``spin`` raises ``spin_exc``."""
    calls = {"destroyed": 0, "shutdown": 0, "subs": []}

    class _Node:
        def __init__(self, name):
            calls["name"] = name

        def create_subscription(self, msg_type, topic, cb, qos):
            calls["subs"].append((msg_type, topic))
            return object()

        def destroy_node(self):
            calls["destroyed"] += 1

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None

    def _spin(node):
        raise spin_exc

    rclpy.spin = _spin
    rclpy.try_shutdown = lambda: calls.__setitem__("shutdown", calls["shutdown"] + 1)
    executors = types.ModuleType("rclpy.executors")

    class ExternalShutdownException(Exception):
        pass

    executors.ExternalShutdownException = ExternalShutdownException
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = _Node
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kw: kw
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    tsi = types.ModuleType("tinker_sim_interfaces")
    tsi_msg = types.ModuleType("tinker_sim_interfaces.msg")
    tsi_msg.ObjectTruth = type("ObjectTruth", (), {})
    tsi_msg.RobotTruth = type("RobotTruth", (), {})
    for name, mod in {"rclpy": rclpy, "rclpy.executors": executors, "rclpy.node": node_mod,
                      "rclpy.qos": qos, "tinker_sim_interfaces": tsi,
                      "tinker_sim_interfaces.msg": tsi_msg}.items():
        monkeypatch.setitem(sys.modules, name, mod)
    calls["ExternalShutdownException"] = ExternalShutdownException
    return calls


@pytest.fixture
def restore_signals():
    """``main()`` installs a SIGTERM handler and ignores SIGINT/SIGTERM while
    shutting down; undo that so later tests (and their subprocesses, which
    inherit ignored dispositions) see the pytest process's original handlers."""
    saved = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    yield
    for s, h in saved.items():
        signal.signal(s, h)


def test_main_writes_run_truth_json_on_keyboard_interrupt(tmp_path, monkeypatch, restore_signals):
    calls = _install_fake_rclpy(monkeypatch, KeyboardInterrupt())
    rc = probe.main(["--out", str(tmp_path)])
    assert rc == 0
    out = tmp_path / "run_truth.json"
    data = json.loads(out.read_text())
    assert data["schema_version"] == 1
    assert data["any_object_retained"] is None
    assert data["samples"] == 0
    assert data["robot"] is None
    assert "shelf" in data["surfaces"] and "kitchen" in data["persons"]
    assert not (tmp_path / "run_truth.json.tmp").exists()
    assert calls["destroyed"] == 1 and calls["shutdown"] == 1
    assert sorted(t for _, t in calls["subs"]) == ["/sim/truth/object_state", "/sim/truth/robot_state"]


def test_main_writes_on_external_shutdown_too(tmp_path, monkeypatch, restore_signals):
    calls = _install_fake_rclpy(monkeypatch, Exception("placeholder"))
    exc = calls["ExternalShutdownException"]()
    sys.modules["rclpy"].spin = lambda node: (_ for _ in ()).throw(exc)
    assert probe.main(["--out", str(tmp_path)]) == 0
    assert (tmp_path / "run_truth.json").exists()


def test_write_summary_is_atomic_and_overwrites(tmp_path):
    t = RunTruthTracker()
    out = tmp_path / "run_truth.json"
    probe.write_summary(out, t)
    assert json.loads(out.read_text())["samples"] == 0
    t.update_object("cmd_soup_0", True, 0.0, 0.0, 0.0)
    probe.write_summary(out, t)
    assert json.loads(out.read_text())["samples"] == 1
    assert not out.with_name("run_truth.json.tmp").exists()
