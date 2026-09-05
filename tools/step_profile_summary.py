#!/usr/bin/env python3
"""Summarise ``TINKER_SIM_PROFILE=1`` ``step_profile`` lines from a sim log.

Usage: step_profile_summary.py LABEL LOGFILE [--skip-first-s 30]
Prints one Markdown table row (see ``main``). Pure; no Isaac imports.
"""
from __future__ import annotations

import argparse
import json
import statistics
from collections.abc import Iterable

BUCKETS = ("physics", "publish", "kit_pump", "cameras", "spin", "unaccounted", "wall")


def _records(lines: Iterable[str]) -> list[dict]:
    out = []
    for line in lines:
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            rec = json.loads(line)
        except json.JSONDecodeError:
            continue
        if "step_profile" in rec:
            out.append(rec["step_profile"])
    return out


def summarize(lines: Iterable[str], *, skip_first_s: float = 30.0) -> dict:
    recs = _records(lines)
    if not recs:
        return {"windows": 0, "median_ms_per_cycle": {}, "rtf_estimate": None}
    t0 = recs[0]["wall_time"]
    steady = [r for r in recs if r["wall_time"] - t0 >= skip_first_s]
    medians = {
        b: statistics.median(r["ms_per_cycle"][b] for r in steady) for b in BUCKETS
    } if steady else {}
    rtf = None
    pairs = [
        (b["wall_time"] - a["wall_time"], (b["sim_time"] or 0.0) - (a["sim_time"] or 0.0))
        for a, b in zip(steady, steady[1:])
        if a.get("sim_time") is not None and b.get("sim_time") is not None
    ]
    pairs = [(w, s) for w, s in pairs if w > 0]
    if pairs:
        rtf = statistics.median(s / w for w, s in pairs)
    return {"windows": len(steady), "median_ms_per_cycle": medians, "rtf_estimate": rtf}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("label")
    parser.add_argument("logfile")
    parser.add_argument("--skip-first-s", type=float, default=30.0)
    args = parser.parse_args(argv)
    with open(args.logfile, encoding="utf-8", errors="replace") as fh:
        out = summarize(fh, skip_first_s=args.skip_first_s)
    m = out["median_ms_per_cycle"]
    rtf = "n/a" if out["rtf_estimate"] is None else f"{out['rtf_estimate']:.2f}"
    cell = lambda k: f"{m[k]:.1f}" if k in m else "n/a"  # noqa: E731
    print(
        f"| {args.label} | {out['windows']} | {cell('kit_pump')} | {cell('cameras')} | "
        f"{cell('physics')} | {cell('wall')} | {rtf} |"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
