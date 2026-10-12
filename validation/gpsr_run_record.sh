#!/usr/bin/env bash
# Combined per-run GPSR recorder: one process for the bench's single --recorder-cmd.
#
#   validation/gpsr_run_record.sh [--dry-run] <run_dir> [recorder args...]
#
# Starts BOTH
#   python3 validation/gpsr_run_recorder.py --out <run_dir> --interval 1.0 [recorder args...]
#   python3 tools/gpsr_run_truth_probe.py   --out <run_dir>
# (extra args go to the frame recorder only, e.g. --topic /arena/image=arena; a
# caller-supplied --interval wins over the default 1.0 because argparse keeps the
# last value), forwards SIGTERM/SIGINT to both exactly once, waits for both, and
# exits 0.
#
# The tier-2 bench (tk25_decision GPSR/bench/tier2.py) launches the recorder with
# start_new_session=True and stops it with killpg(SIGINT), escalating to
# killpg(SIGKILL) after 15 s. Each child is started with job control on (`set -m`,
# its own process group) so the bench's group signal reaches only this script,
# which then delivers ONE signal per child -- a child that has already begun its
# final JSON write must never get a second SIGINT (that would raise inside the write).
# If a child is still alive GRACE_S after the forward it is SIGKILLed here, well
# inside the bench's own 15 s escalation, so nothing is ever orphaned.
#
# GPSR_RECORD_PYTHON overrides the interpreter (tests use a PATH shim instead).
# --dry-run prints the two command lines and exits 0 without starting anything.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
PYTHON="${GPSR_RECORD_PYTHON:-python3}"
GRACE_S="${GPSR_RECORD_GRACE_S:-10}"

DRY_RUN=0
if [ "${1:-}" = "--dry-run" ]; then
  DRY_RUN=1
  shift
fi
if [ $# -lt 1 ]; then
  echo "usage: $(basename "$0") [--dry-run] <run_dir> [recorder args...]" >&2
  exit 2
fi
RUN_DIR="$1"
shift

RECORDER_CMD=("$PYTHON" "$ROOT/validation/gpsr_run_recorder.py" --out "$RUN_DIR" --interval 1.0 "$@")
PROBE_CMD=("$PYTHON" "$ROOT/tools/gpsr_run_truth_probe.py" --out "$RUN_DIR")
# GPSR_TRUTH_EXTENTS_JSON: a tools/gpsr_pose_audit.py report (e.g.
# simulation/scenarios/rcw2026-pose-audit.json) whose measured furniture
# half-extents replace the probe's grid-derived surfaces[].half_extent_xy.
if [ -n "${GPSR_TRUTH_EXTENTS_JSON:-}" ] && [ -f "$GPSR_TRUTH_EXTENTS_JSON" ]; then
  PROBE_CMD+=(--extents-json "$GPSR_TRUTH_EXTENTS_JSON")
fi

if [ "$DRY_RUN" = 1 ]; then
  printf 'recorder:'; printf ' %q' "${RECORDER_CMD[@]}"; printf '\n'
  printf 'probe:'; printf ' %q' "${PROBE_CMD[@]}"; printf '\n'
  exit 0
fi

mkdir -p "$RUN_DIR"

# Job control on: each background child gets its OWN process group (so the
# bench's killpg reaches only this script) and keeps the default SIGINT
# disposition. Without `set -m` a non-interactive bash starts `cmd &` with
# SIGINT ignored, and the children could never be interrupted at all.
set -m
"${RECORDER_CMD[@]}" &
REC_PID=$!
"${PROBE_CMD[@]}" &
PROBE_PID=$!
set +m
echo "[gpsr_run_record] recorder pid=$REC_PID probe pid=$PROBE_PID run_dir=$RUN_DIR"

FORWARDED=0
forward() {
  local sig="$1"
  if [ "$FORWARDED" = 1 ]; then
    return
  fi
  FORWARDED=1
  echo "[gpsr_run_record] forwarding SIG$sig to recorder and probe"
  kill -"$sig" "$REC_PID" 2>/dev/null || true
  kill -"$sig" "$PROBE_PID" 2>/dev/null || true
  (
    # Watchdog: SIGKILL whatever is still alive after the grace period. Its fds
    # are detached so it never holds the bench's log pipe open past our exit.
    sleep "$GRACE_S"
    kill -KILL "$REC_PID" "$PROBE_PID" 2>/dev/null || true
  ) >/dev/null 2>&1 </dev/null &
  WATCHDOG_PID=$!
}
WATCHDOG_PID=""
trap 'forward TERM' TERM
trap 'forward INT' INT

# `wait <pid>` returns >128 immediately when a trapped signal arrives, so re-wait
# until the child is really gone.
for pid in "$REC_PID" "$PROBE_PID"; do
  while kill -0 "$pid" 2>/dev/null; do
    wait "$pid" 2>/dev/null || true
  done
done
if [ -n "$WATCHDOG_PID" ]; then
  kill "$WATCHDOG_PID" 2>/dev/null || true
fi
echo "[gpsr_run_record] both recorders exited"
exit 0
