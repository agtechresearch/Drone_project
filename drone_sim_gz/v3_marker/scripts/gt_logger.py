#!/usr/bin/env python3
"""Ground-truth pose log with the simulator's own timestamps (v3_marker, 2026-10-31, codex ground-truth-timestamps).
Run INSIDE starling-gz with GZ_PARTITION set:  python3 gt_logger.py <model> <out_csv>      stop: touch <out_csv>.STOP
Every /model/<model>/odometry message -> one row: t_sim (the message header stamp, sim seconds), t_wall (host clock when it
arrived), x y z (world, m), qw qx qy qz, yaw (ENU, rad). Nothing is resampled or re-stamped: gaps stay gaps, so an evaluator can
interpolate the truth at the exact time of an estimate and see where the truth is missing. tag_detect.py's tags.csv kept the
latest pose next to the IMAGE time (unknown offset); loc_compare.py / survey_eval.py prefer this file when it exists."""
import math, os, sys, threading, time
from gz.transport13 import Node
from gz.msgs10.odometry_pb2 import Odometry

model, out = sys.argv[1], sys.argv[2]
f = open(out, "w"); f.write("t_sim,t_wall,x,y,z,qw,qx,qy,qz,yaw\n")
lock = threading.Lock(); n = [0]


def cb(m):
    t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
    p, q = m.pose.position, m.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    with lock:
        f.write("%.4f,%.3f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%.6f\n" % (t, time.time(), p.x, p.y, p.z, q.w, q.x, q.y, q.z, yaw))
        n[0] += 1


node = Node()
if not node.subscribe(Odometry, "/model/%s/odometry" % model, cb):
    sys.exit("subscribe failed")
print("[gt_logger] /model/%s/odometry -> %s" % (model, out), flush=True)
while not os.path.exists(out + ".STOP"):
    time.sleep(0.5)
    with lock:
        f.flush()
with lock:
    f.close()
print("[gt_logger] stopped, rows %d" % n[0], flush=True)
