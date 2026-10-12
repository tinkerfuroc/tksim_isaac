#!/usr/bin/env python3
"""Sample /clock and record the simulator's real-time factor (RTF).

RTF = simulated seconds per wall second. Below ~0.5 the decision layer's
wall-clock budgets start cancelling healthy Nav2 goals, so every GPSR run
records it (docs/superpowers/specs/2026-10-11-gpsr-sim-rtf-budget-design.md).

    python3 tools/rtf_monitor.py [--out PATH]          # run (needs ROS sourced)
    python3 tools/rtf_monitor.py --summarise PATH --from WALL --to WALL

Run mode appends one JSON line every 10 s wall to ``$GPSR_RUN_DIR/rtf.jsonl``
(``{"wall", "sim", "rtf_10s", "rtf_30s"}``) and prints a WARN line when the
30 s RTF drops below 0.5. It never stops the run. Summarise mode prints
``{"mean", "min_30s", "samples"}`` for one trial's wall window -- the call a
battery runner makes to stamp per-trial RTF.

The maths is pure and importable without rclpy (tested in the sim venv);
rclpy is imported only in run mode.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections import deque
from pathlib import Path
from typing import Iterable, Mapping, Sequence

RTF_WARN_THRESHOLD = 0.5
RECORD_PERIOD_S = 10.0
KEEP_S = 35.0


def _after_last_rewind(samples: Sequence[tuple[float, float]]) -> Sequence[tuple[float, float]]:
    """The tail of *samples* after the last backward sim-time jump (a reset)."""
    start = 0
    for i in range(1, len(samples)):
        if samples[i][1] < samples[i - 1][1]:
            start = i
    return samples[start:]


def window_rtf(
    samples: Sequence[tuple[float, float]], now_wall: float, window_s: float
) -> float | None:
    """RTF over the last *window_s* wall seconds of ``(wall, sim)`` samples.

    Measured up to *now_wall*, not up to the last sample, so a stalled /clock
    decays toward 0 instead of going silent. The anchor is the last sample at
    or before the window start (else the first sample in the window); samples
    before the last backward sim jump (a reset) are never used.
    """
    usable = list(_after_last_rewind([s for s in samples if s[0] <= now_wall]))
    if not usable:
        return None
    window_start = now_wall - window_s
    older = [s for s in usable if s[0] <= window_start]
    anchor = older[-1] if older else usable[0]
    wall = now_wall - anchor[0]
    if wall <= 0.0:
        return None
    return (usable[-1][1] - anchor[1]) / wall


class WarnState:
    """Turns a stream of 30 s RTF values into rare, readable log lines."""

    def __init__(self, threshold: float = RTF_WARN_THRESHOLD, remind_s: float = 60.0) -> None:
        self.threshold = threshold
        self.remind_s = remind_s
        self._below_since_warn: float | None = None

    def update(self, rtf_30s: float | None, now_wall: float) -> str | None:
        if rtf_30s is None:
            return None
        if rtf_30s < self.threshold:
            if self._below_since_warn is None or now_wall - self._below_since_warn >= self.remind_s:
                self._below_since_warn = now_wall
                return f"WARN rtf_30s={rtf_30s:.2f} < {self.threshold:.2f}"
            return None
        if self._below_since_warn is not None:
            self._below_since_warn = None
            return f"OK rtf_30s={rtf_30s:.2f} >= {self.threshold:.2f}"
        return None


def summarise(records: Iterable[Mapping], t_from: float, t_to: float) -> dict:
    """Time-weighted mean RTF and worst 30 s RTF of records in [t_from, t_to]."""
    rows = [r for r in records if t_from <= float(r["wall"]) <= t_to]
    sim_total = wall_total = 0.0
    for a, b in zip(rows, rows[1:]):
        d_sim = float(b["sim"]) - float(a["sim"])
        d_wall = float(b["wall"]) - float(a["wall"])
        if d_sim < 0.0 or d_wall <= 0.0:
            continue  # a reset rewound the clock between these records
        sim_total += d_sim
        wall_total += d_wall
    lows = [float(r["rtf_30s"]) for r in rows if r.get("rtf_30s") is not None]
    return {
        "mean": (sim_total / wall_total) if wall_total > 0.0 else None,
        "min_30s": min(lows) if lows else None,
        "samples": len(rows),
    }


def _is_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def load_records(lines: Iterable[str]) -> tuple[list[dict], int]:
    """Parse jsonl lines, skipping blanks silently and counting unusable ones."""
    records: list[dict] = []
    skipped = 0
    for line in lines:
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except ValueError:
            skipped += 1
            continue
        if not isinstance(record, dict) or not (
            _is_number(record.get("wall")) and _is_number(record.get("sim"))
        ):
            skipped += 1
            continue
        records.append(record)
    return records, skipped


def append_record(out: Path, record: dict) -> str | None:
    """Append one JSON line, creating the directory. Returns an error message or None."""
    try:
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "a", encoding="utf-8") as handle:
            handle.write(json.dumps(record) + "\n")
    except OSError as exc:
        return str(exc)
    return None


def default_out_path(env: Mapping[str, str]) -> Path:
    run_dir = env.get("GPSR_RUN_DIR", "").strip()
    return Path(run_dir) / "rtf.jsonl" if run_dir else Path("rtf.jsonl")


def _run(out: Path) -> int:  # pragma: no cover - needs a live ROS graph
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock

    try:
        out.parent.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        print(f"[rtf_monitor] cannot create {out.parent}: {exc}", flush=True)

    rclpy.init()
    node = Node("rtf_monitor")
    samples: deque[tuple[float, float]] = deque()
    warn = WarnState()
    last_write_error: str | None = None

    def on_clock(msg) -> None:
        now = time.time()
        samples.append((now, msg.clock.sec + msg.clock.nanosec * 1e-9))
        # Never empty the deque: a stalled /clock still needs its anchor.
        while len(samples) > 1 and now - samples[0][0] > KEEP_S:
            samples.popleft()

    def on_timer() -> None:
        nonlocal last_write_error
        now = time.time()
        if not samples:
            return
        r10 = window_rtf(samples, now, 10.0)
        r30 = window_rtf(samples, now, 30.0)
        record = {"wall": round(now, 3), "sim": round(samples[-1][1], 3),
                  "rtf_10s": None if r10 is None else round(r10, 4),
                  "rtf_30s": None if r30 is None else round(r30, 4)}
        error = append_record(out, record)
        if error is not None and error != last_write_error:
            print(f"[rtf_monitor] write failed: {error}", flush=True)
        last_write_error = error
        line = warn.update(r30, now)
        if line:
            print(f"[rtf_monitor] {line}", flush=True)

    node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
    node.create_timer(RECORD_PERIOD_S, on_timer)
    print(f"[rtf_monitor] writing {out} every {RECORD_PERIOD_S:.0f} s", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--summarise", type=Path, default=None)
    parser.add_argument("--from", dest="t_from", type=float, default=None)
    parser.add_argument("--to", dest="t_to", type=float, default=None)
    args = parser.parse_args(argv)
    if args.summarise is not None:
        if args.t_from is None or args.t_to is None:
            parser.error("--summarise needs --from and --to (unix wall seconds)")
        try:
            lines = args.summarise.read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeDecodeError) as exc:
            print(f"[rtf_monitor] cannot read {args.summarise}: {exc}", file=sys.stderr)
            return 1
        records, skipped = load_records(lines)
        if skipped:
            print(f"[rtf_monitor] skipped {skipped} malformed line(s)", file=sys.stderr)
        print(json.dumps(summarise(records, args.t_from, args.t_to)))
        return 0
    return _run(args.out or default_out_path(os.environ))


if __name__ == "__main__":
    sys.exit(main())
