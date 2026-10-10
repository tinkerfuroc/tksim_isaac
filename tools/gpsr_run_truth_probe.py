#!/usr/bin/env python3
"""GPSR run-truth probe: a bench ORACLE that records, from sim physics truth, what
physically happened during one tier-2 GPSR run -- independently of the robot's
own beliefs.

It subscribes (rclpy, sim ros2_ws sourced) to the topics republished by
``ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/truth_evaluator.py``:

* ``/sim/truth/object_state`` -- ``tinker_sim_interfaces/msg/ObjectTruth``
  (``object_id``, ``class_name``, ``pose``, ``twist``, ``retained_by_gripper``),
  one message per object per physics frame. Scenario-declared objects come
  first; harness-spawned ``/World/Scenario/*`` rigid bodies (``tools/gpsr_spawn.py``)
  are appended by ``backend._spawned_object_states()`` and carry the LAST PATH
  SEGMENT as ``object_id`` (``/World/Scenario/cmd_sugar_box_0`` -> ``cmd_sugar_box_0``),
  i.e. exactly the ``id`` from the spawn plan / manifest.
* ``/sim/truth/robot_state`` -- ``tinker_sim_interfaces/msg/RobotTruth``
  (``base_pose`` in the world frame).

On SIGTERM/SIGINT (the bench sends SIGINT to the recorder's whole process group,
then SIGKILL after 15 s) it writes ``<out_dir>/run_truth.json``::

    {
      "schema_version": 1,
      "samples": <int>,                       # ObjectTruth samples seen
      "any_object_retained": true|false|null, # null when no ObjectTruth sample arrived
      "objects": {
        "<object_id as published>": {
          "first_xyz": [x,y,z], "last_xyz": [x,y,z],
          "ever_retained": bool, "retained_at_end": bool,
          "max_lift_m": float, "max_xy_disp_m": float, "samples": int
        }, ...
      },
      "robot": {"last_xyz": [x,y,z], "last_yaw_deg": float, "samples": int} | null,
      "surfaces": {"<spot>": {"surface_xyz": [x,y,z], "half_extent_xy": [hx, hy]}, ...},
      "persons": {"<room>": {"xyz": [x,y,z]}, ...}
    }

``surfaces`` / ``persons`` are copied at startup from
``simulation/scenarios/rcw2026-placements.json``. ``half_extent_xy`` defaults to
``[grid_dx * 3, grid_dy * 3]`` (three placement grid cells each side of the
surface point -- the placement grid, not the furniture, is what bounds where the
bench spawns objects); pass ``--extents-json <pose_audit.json>`` (output of
``tools/gpsr_pose_audit.py``) to use the real furniture bounding-box half extents
measured from the arena USD instead.

The summary is also re-written every ``--flush-every`` samples, atomically, so a
hard kill still leaves a valid file. Strictly post-run SCORING input: never a
runtime input of the manipulation stack or the behaviour tree. The msg types live
only in the sim's install, so this probe is the single place that touches them;
everything downstream reads plain JSON. ``RunTruthTracker`` and the placements
loader are pure Python (no ROS) and are unit-tested in
``tests/test_gpsr_run_truth_probe.py``.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
from pathlib import Path
from typing import Any, Mapping, Optional

SCHEMA_VERSION = 1
DEFAULT_PLACEMENTS = Path(__file__).resolve().parent.parent / "simulation" / "scenarios" / "rcw2026-placements.json"
DEFAULT_OBJECT_TOPIC = "/sim/truth/object_state"
DEFAULT_ROBOT_TOPIC = "/sim/truth/robot_state"
OUTPUT_NAME = "run_truth.json"
GRID_CELLS_PER_SIDE = 3


def yaw_deg_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Z-axis yaw (degrees, in (-180, 180]) of an XYZW quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def _r(value: float, digits: int = 4) -> float:
    return round(float(value), digits)


def _xyz(values) -> list[float]:
    return [_r(v) for v in values]


class RunTruthTracker:
    """Pure-Python accumulator (no ROS) for ObjectTruth / RobotTruth samples.

    Feed ``update_object(object_id, retained, x, y, z)`` per ObjectTruth sample and
    ``update_robot(x, y, z, qx, qy, qz, qw)`` per RobotTruth sample; ``summary()``
    is the JSON-able dict described in the module docstring. ``any_object_retained``
    is None until at least one object sample arrives, so a probe that saw no data
    leaves the verdict gate graceful (it must only downgrade on an explicit False).
    """

    def __init__(self, surfaces: Optional[Mapping[str, Any]] = None,
                 persons: Optional[Mapping[str, Any]] = None) -> None:
        self._objects: dict[str, dict] = {}
        self._object_samples = 0
        self._robot: Optional[dict] = None
        self._surfaces = dict(surfaces or {})
        self._persons = dict(persons or {})

    @property
    def samples(self) -> int:
        return self._object_samples

    def update_object(self, object_id: str, retained: bool, x: float, y: float, z: float) -> None:
        self._object_samples += 1
        o = self._objects.get(object_id)
        if o is None:
            o = {"first": (x, y, z), "last": (x, y, z), "ever_retained": False,
                 "retained_at_end": False, "max_xy_disp": 0.0, "max_lift": 0.0, "samples": 0}
            self._objects[object_id] = o
        o["samples"] += 1
        o["last"] = (x, y, z)
        o["retained_at_end"] = bool(retained)
        if retained:
            o["ever_retained"] = True
        fx, fy, fz = o["first"]
        dxy = math.hypot(x - fx, y - fy)
        if dxy > o["max_xy_disp"]:
            o["max_xy_disp"] = dxy
        lift = z - fz
        if lift > o["max_lift"]:
            o["max_lift"] = lift

    def update_robot(self, x: float, y: float, z: float,
                     qx: float, qy: float, qz: float, qw: float) -> None:
        if self._robot is None:
            self._robot = {"samples": 0}
        self._robot["samples"] += 1
        self._robot["last"] = (x, y, z)
        self._robot["yaw_deg"] = yaw_deg_from_quaternion(qx, qy, qz, qw)

    def summary(self) -> dict:
        any_retained = (any(o["ever_retained"] for o in self._objects.values())
                        if self._objects else None)
        robot = None
        if self._robot is not None:
            robot = {"last_xyz": _xyz(self._robot["last"]),
                     "last_yaw_deg": _r(self._robot["yaw_deg"], 2),
                     "samples": self._robot["samples"]}
        return {
            "schema_version": SCHEMA_VERSION,
            "samples": self._object_samples,
            "any_object_retained": any_retained,
            "objects": {
                oid: {
                    "first_xyz": _xyz(o["first"]),
                    "last_xyz": _xyz(o["last"]),
                    "ever_retained": o["ever_retained"],
                    "retained_at_end": o["retained_at_end"],
                    "max_lift_m": _r(o["max_lift"]),
                    "max_xy_disp_m": _r(o["max_xy_disp"]),
                    "samples": o["samples"],
                }
                for oid, o in self._objects.items()
            },
            "robot": robot,
            "surfaces": {k: dict(v) for k, v in self._surfaces.items()},
            "persons": {k: dict(v) for k, v in self._persons.items()},
        }


def load_placements(placements: Mapping[str, Any],
                    extents: Optional[Mapping[str, Any]] = None) -> tuple[dict, dict]:
    """Build the ``surfaces`` / ``persons`` blocks from a parsed placements JSON.

    ``extents`` (optional) is a parsed ``tools/gpsr_pose_audit.py`` output; when it
    has a ``furniture.<spot>.half_extent_xy`` entry that real bbox half extent is
    used, otherwise ``[grid_dx * 3, grid_dy * 3]``.
    """
    surfaces: dict[str, dict] = {}
    audited = (extents or {}).get("furniture", {}) if extents else {}
    for name, spot in (placements.get("spots") or {}).items():
        xyz = [float(v) for v in spot["surface_xyz"]]
        real = audited.get(name, {}).get("half_extent_xy") if isinstance(audited, Mapping) else None
        if real is None:
            real = spot.get("half_extent_xy")  # explicit measured extent in placements.json
        if real is not None:
            half = [_r(real[0]), _r(real[1])]
        else:
            half = [_r(float(spot.get("grid_dx", 0.0)) * GRID_CELLS_PER_SIDE),
                    _r(float(spot.get("grid_dy", 0.0)) * GRID_CELLS_PER_SIDE)]
        surfaces[name] = {"surface_xyz": xyz, "half_extent_xy": half}
    persons = {room: {"xyz": [float(v) for v in p["xyz"]]}
               for room, p in (placements.get("persons") or {}).items()}
    # The operator at the command point (spawned by gpsr_scene for delivery
    # commands): the bench's delivered(..., operator) measures against it.
    operator = placements.get("operator")
    if isinstance(operator, Mapping) and operator.get("xyz"):
        persons["operator"] = {"xyz": [float(v) for v in operator["xyz"]]}
    return surfaces, persons


def write_summary(out_path: Path, tracker: RunTruthTracker) -> None:
    """Atomic write (tmp + os.replace) so a kill mid-write never leaves a torn file."""
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = out_path.with_name(out_path.name + ".tmp")
    tmp.write_text(json.dumps(tracker.summary(), indent=2) + "\n", encoding="utf-8")
    os.replace(tmp, out_path)


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Record sim physics truth for one GPSR run.")
    parser.add_argument("--out", required=True,
                        help=f"run directory; {OUTPUT_NAME} is written inside it "
                             "(a path ending in .json is used verbatim)")
    parser.add_argument("--placements", default=str(DEFAULT_PLACEMENTS),
                        help="rcw2026-placements.json to copy surfaces/persons from")
    parser.add_argument("--extents-json", default=None,
                        help="optional tools/gpsr_pose_audit.py output; its furniture "
                             "half extents replace the grid-derived half_extent_xy")
    parser.add_argument("--object-topic", default=DEFAULT_OBJECT_TOPIC)
    parser.add_argument("--robot-topic", default=DEFAULT_ROBOT_TOPIC)
    parser.add_argument("--flush-every", type=int, default=50,
                        help="re-write the summary every N object samples (survives SIGKILL)")
    return parser


def output_path(out: str) -> Path:
    p = Path(out)
    return p if p.suffix == ".json" else p / OUTPUT_NAME


def build_tracker(args: argparse.Namespace) -> RunTruthTracker:
    placements = json.loads(Path(args.placements).read_text(encoding="utf-8"))
    extents = None
    if args.extents_json:
        extents = json.loads(Path(args.extents_json).read_text(encoding="utf-8"))
    surfaces, persons = load_placements(placements, extents)
    return RunTruthTracker(surfaces=surfaces, persons=persons)


def main(argv: Optional[list[str]] = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    out = output_path(args.out)
    tracker = build_tracker(args)
    # Write an (empty) summary immediately so the bench always finds the file,
    # even if the sim never publishes a single truth frame.
    write_summary(out, tracker)

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from tinker_sim_interfaces.msg import ObjectTruth, RobotTruth

    flush_every = max(1, int(args.flush_every))

    class _Probe(Node):
        def __init__(self) -> None:
            super().__init__("gpsr_run_truth_probe")
            # Match the truth publisher's reliable QoS (truth_evaluator.py).
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=100)
            self.create_subscription(ObjectTruth, args.object_topic, self._on_object, qos)
            self.create_subscription(RobotTruth, args.robot_topic, self._on_robot, qos)

        def _on_object(self, msg) -> None:
            p = msg.pose.position
            tracker.update_object(str(msg.object_id), bool(msg.retained_by_gripper), p.x, p.y, p.z)
            if tracker.samples % flush_every == 0:
                write_summary(out, tracker)

        def _on_robot(self, msg) -> None:
            p = msg.base_pose.position
            q = msg.base_pose.orientation
            tracker.update_robot(p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def _on_sigterm(*_: object) -> None:
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, _on_sigterm)
    rclpy.init(args=None)
    node = _Probe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A second SIGINT/SIGTERM (bench killpg + wrapper forwarding) must not
        # raise inside the final write and leave no summary behind.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        write_summary(out, tracker)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
