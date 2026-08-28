#!/usr/bin/env python3
"""Per-run GPSR frame recorder.

Saves JPEG frames from one or more sensor_msgs/Image topics (arena camera,
head camera, ...) at a bounded rate during a tier-2 GPSR run, then writes a
``recorder-meta.json`` summary on shutdown.

``FrameSink`` is pure (no ROS) and is exercised directly by
``tests/test_gpsr_run_recorder.py``. ``main()`` builds the rclpy node and
imports rclpy lazily so unit tests never touch ROS.
"""
from __future__ import annotations

import argparse
import datetime
import json
import sys
from pathlib import Path
from typing import Optional

from PIL import Image

JPEG_QUALITY = 85


class FrameSink:
    """Accepts candidate frames for one topic/label and writes accepted ones as JPEG.

    A frame is accepted when it is the first offered frame, or when
    ``stamp_s >= last_accepted_stamp + interval_s``, and the sink is still
    under ``max_frames``. Accepted files land at
    ``out_dir/frames/<label>/<seq:04d>_<int(stamp_s*1000)>.jpg``.
    """

    def __init__(self, out_dir: Path, label: str, interval_s: float, max_frames: int) -> None:
        self.out_dir = Path(out_dir)
        self.label = label
        self.interval_s = interval_s
        self.max_frames = max_frames
        self._frames_dir = self.out_dir / "frames" / label
        self._index_path = self.out_dir / "frames" / "index.jsonl"
        self._count = 0
        self._last_accepted: Optional[float] = None
        self._first_stamp: Optional[float] = None
        self._last_stamp: Optional[float] = None
        self._index_write_errors = 0

    def offer(
        self,
        stamp_s: float,
        rgb_bytes: bytes,
        width: int,
        height: int,
        wall_iso: Optional[str] = None,
    ) -> Optional[Path]:
        if self._count >= self.max_frames:
            return None
        if self._last_accepted is not None and stamp_s < self._last_accepted + self.interval_s:
            return None

        self._frames_dir.mkdir(parents=True, exist_ok=True)
        seq = self._count
        name = f"{seq:04d}_{int(stamp_s * 1000)}.jpg"
        path = self._frames_dir / name

        img = Image.frombytes("RGB", (width, height), rgb_bytes)
        img.save(path, "JPEG", quality=JPEG_QUALITY)

        self._count += 1
        self._last_accepted = stamp_s
        if self._first_stamp is None:
            self._first_stamp = stamp_s
        self._last_stamp = stamp_s

        self._append_index_line(name, stamp_s, wall_iso)
        return path

    def _append_index_line(self, name: str, stamp_s: float, wall_iso: Optional[str]) -> None:
        entry = {
            "label": self.label,
            "file": f"frames/{self.label}/{name}",
            "stamp_s": stamp_s,
            "wall": wall_iso,
        }
        try:
            self._index_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self._index_path, "a") as f:
                f.write(json.dumps(entry) + "\n")
        except OSError:
            self._index_write_errors += 1

    def summary(self) -> dict:
        return {
            "frames": self._count,
            "first_stamp": self._first_stamp,
            "last_stamp": self._last_stamp,
            "index_write_errors": self._index_write_errors,
        }


def _parse_topic_arg(value: str) -> tuple[str, str]:
    if "=" not in value:
        raise argparse.ArgumentTypeError(f"expected TOPIC=label, got {value!r}")
    topic, label = value.split("=", 1)
    topic, label = topic.strip(), label.strip()
    if not topic or not label:
        raise argparse.ArgumentTypeError(f"expected TOPIC=label, got {value!r}")
    return topic, label


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Record GPSR run frames to JPEG.")
    parser.add_argument("--out", required=True, help="Output run directory.")
    parser.add_argument(
        "--topic",
        action="append",
        default=[],
        type=_parse_topic_arg,
        metavar="TOPIC=label",
        help="sensor_msgs/Image topic to record, mapped to a label. Repeatable.",
    )
    parser.add_argument("--interval", type=float, default=1.0, help="Minimum seconds between accepted frames.")
    parser.add_argument("--max-frames", type=int, default=900, help="Maximum frames per label.")
    return parser


def main(argv=None) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import Image as ImageMsg

    args = _build_arg_parser().parse_args(argv)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    sinks: dict[str, FrameSink] = {
        label: FrameSink(out_dir, label, args.interval, args.max_frames) for _, label in args.topic
    }

    rclpy.init(args=None)
    node = Node("gpsr_run_recorder")
    qos = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )
    skipped = {"count": 0}

    def _make_callback(label: str):
        sink = sinks[label]

        def _callback(msg: ImageMsg) -> None:
            if msg.encoding != "rgb8":
                skipped["count"] += 1
                node.get_logger().warn(
                    f"gpsr_run_recorder: skipping {label} frame with encoding {msg.encoding!r} (want rgb8)"
                )
                return
            stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            wall_iso = datetime.datetime.now(datetime.timezone.utc).isoformat()
            sink.offer(stamp_s, bytes(msg.data), msg.width, msg.height, wall_iso=wall_iso)

        return _callback

    subs = []
    for topic, label in args.topic:
        subs.append(node.create_subscription(ImageMsg, topic, _make_callback(label), qos))

    started_wall = datetime.datetime.now(datetime.timezone.utc).isoformat()

    def _write_meta() -> None:
        meta = {
            "labels": {label: sink.summary() for label, sink in sinks.items()},
            "started_wall": started_wall,
            "ended_wall": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        }
        with open(out_dir / "recorder-meta.json", "w") as f:
            json.dump(meta, f, indent=2)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _write_meta()
        node.destroy_node()
        rclpy.try_shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
