#!/usr/bin/env python3
"""Map every "GoToApproach ABORTED" line to its bench run and verdict (section 2).

    python3 approach_aborts.py [SESSION ...]     (default: all sessions with an abort)

For each abort: session, epoch, batch, run entry (matched by the run's
orchestrator.log time range), run verdict and detail, abort message. About
half of the aborts fall outside any recorded run window ("NO RUN").
"""

import glob
import json
import os
import re
import sys

from navtrace import BENCH_DIR, STACK_LOGS, batch_for_session

ABORT = re.compile(r"\[(\d+\.\d+)\] \[approach_planner\]: GoToApproach ABORTED status=(\d+) phase=(\S+) msg='(.*)'")


def run_windows(batch):
    out = []
    for log in glob.glob(os.path.join(BENCH_DIR, batch, "runs", "*", "orchestrator.log")):
        stamps = [float(x) for x in re.findall(r"\[(1\d{9}\.\d+)\]", open(log, errors="replace").read())]
        if stamps:
            out.append((min(stamps), max(stamps), os.path.dirname(log)))
    return out


def main() -> None:
    sessions = sys.argv[1:]
    if not sessions:
        sessions = sorted(os.path.basename(os.path.dirname(p))
                          for p in glob.glob(os.path.join(STACK_LOGS, "*", "05-nav_extras-0.log"))
                          if "GoToApproach ABORTED" in open(p, errors="replace").read())
    for s in sessions:
        log = os.path.join(STACK_LOGS, s, "05-nav_extras-0.log")
        batch = batch_for_session(s)
        windows = run_windows(batch) if batch else []
        for line in open(log, errors="replace"):
            m = ABORT.search(line)
            if not m:
                continue
            t, status, phase, msg = float(m.group(1)), m.group(2), m.group(3), m.group(4)
            hits = [w for w in windows if w[0] - 5 <= t <= w[1] + 5]
            if not hits:
                print(f"{s} {t:.0f} {batch or '-'} NO RUN | status={status} {phase} | {msg[:120]}")
            for _a, _b, run in hits:
                j = json.load(open(os.path.join(run, "run.json")))
                print(f"{s} {t:.0f} {batch} {os.path.basename(run)} {j.get('verdict')} | "
                      f"{str(j.get('detail', ''))[:90]} | status={status} {phase} | {msg[:120]}")


if __name__ == "__main__":
    main()
