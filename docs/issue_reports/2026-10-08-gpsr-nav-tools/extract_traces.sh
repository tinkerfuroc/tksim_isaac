#!/bin/bash
# Extract pose traces for many bench runs (see extract_tf_pose.py).
#
#   ./extract_traces.sh [-o OUT_DIR] [-j JOBS] [BATCH_GLOB]
#
#   OUT_DIR     default /tmp/gpsr-nav-traces; files are <batch>__<entry>.csv
#   JOBS        parallel bags, default 2. Each bag decompresses to 1-15 GB in
#               /dev/shm/gpsr-nav, so JOBS x 15 GB of RAM must be free.
#               Throughput is roughly 3-6 bags per 10 min regardless; for
#               many runs prefer the Nav2 logs (nav_goal_census.py).
#   BATCH_GLOB  default 't2-*'; e.g. 't2-five-diverse-*'
#
# Existing non-empty CSVs are skipped; failures go to <csv>.err. Known corrupt
# bags (zstd fails): t2-20-live-rerun14 entries 005 and 016,
# t2-diverse15-rerun15 div15-06-communication-talkInfoToGestPrsInRoom.
# Never `pkill -f extract_tf_pose` from a shell whose own command line contains
# that string; kill by PID.
source /opt/ros/humble/setup.bash   # before set -u: the ROS setup reads unset vars
set -u
OUT=/tmp/gpsr-nav-traces
JOBS=2
while getopts "o:j:" opt; do
  case $opt in
    o) OUT=$OPTARG ;;
    j) JOBS=$OPTARG ;;
    *) sed -n '2,20p' "$0"; exit 2 ;;
  esac
done
shift $((OPTIND - 1))
GLOB=${1:-'t2-*'}
BENCH=/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"

for r in "$BENCH"/$GLOB/runs/*/; do
  [ -f "$r/rosbag/rosbag_0.db3.zstd" ] || continue
  b=$(basename "$(dirname "$(dirname "$r")")"); e=$(basename "$r")
  out="$OUT/${b}__${e}.csv"
  [ -s "$out" ] && continue
  printf '%s\n%s\n' "$r" "$out"
done | xargs -P "$JOBS" -n 2 -d '\n' bash -c \
  'python3 "'"$HERE"'/extract_tf_pose.py" "$0" "$1" >/dev/null 2>"$1.err" && rm -f "$1.err" || echo "FAIL $0"'
echo "done: $(ls "$OUT"/*.csv 2>/dev/null | wc -l) traces in $OUT"
