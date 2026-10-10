#!/usr/bin/env python3
"""Extract the robot pose stream from one bench run's rosbag into a CSV.

    source /opt/ros/humble/setup.bash
    python3 extract_tf_pose.py <run_dir> <out.csv> [--tmp-dir /dev/shm/gpsr-nav]

<run_dir> is gpsr_runs/bench/<batch>/runs/<entry>; its rosbag/rosbag_0.db3.zstd
is decompressed to a temporary .db3 (1-15 GB), the /tf rows for map->odom and
odom->base_link are written out, and the temporary file is deleted.

CSV columns: t (epoch s), parent, child, x, y, yaw (rad). Compose map->base_link
with navtrace.load_trace(); the bags carry no cmd_vel or odom topics.
"""

import argparse
import math
import os
import sqlite3
import subprocess
import sys

from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("out_csv")
    ap.add_argument("--tmp-dir", default="/dev/shm/gpsr-nav" if os.path.isdir("/dev/shm") else None,
                    help="where the decompressed .db3 goes (default /dev/shm/gpsr-nav; RAM is far faster than disk)")
    args = ap.parse_args()

    bag = os.path.join(args.run_dir, "rosbag", "rosbag_0.db3.zstd")
    if not os.path.exists(bag):
        print(f"no rosbag: {bag}", file=sys.stderr)
        return 2
    tmp_dir = args.tmp_dir or os.path.dirname(os.path.abspath(args.out_csv))
    os.makedirs(tmp_dir, exist_ok=True)
    tmp = os.path.join(tmp_dir, os.path.basename(args.out_csv) + ".db3")
    try:
        subprocess.run(["zstd", "-dqf", "-o", tmp, bag], check=True)
        db = sqlite3.connect(tmp)
        row = db.execute("select id from topics where name='/tf'").fetchone()
        if row is None:
            print("bag has no /tf topic", file=sys.stderr)
            return 3
        with open(args.out_csv + ".part", "w") as out:
            out.write("t,parent,child,x,y,yaw\n")
            for ts, data in db.execute("select timestamp, data from messages where topic_id=? order by timestamp", row):
                for tr in deserialize_message(data, TFMessage).transforms:
                    if tr.child_frame_id not in ("base_link", "odom", "base_footprint"):
                        continue
                    q = tr.transform.rotation
                    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                    out.write(f"{ts / 1e9},{tr.header.frame_id},{tr.child_frame_id},"
                              f"{tr.transform.translation.x:.4f},{tr.transform.translation.y:.4f},{yaw:.4f}\n")
        db.close()
        os.replace(args.out_csv + ".part", args.out_csv)
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)
    return 0


if __name__ == "__main__":
    sys.exit(main())
