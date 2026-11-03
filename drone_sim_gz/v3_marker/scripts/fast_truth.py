#!/usr/bin/env python3
"""Ground-truth logger at the odometry rate (50 Hz) with sim time AND wall epoch, for system identification.
Run INSIDE starling-gz (GZ_PARTITION set):  python3 fast_truth.py <model> <out.csv>   (stop: touch <out.csv>.STOP)"""
import os, sys, time, math
from gz.transport13 import Node
from gz.msgs10.odometry_pb2 import Odometry
model, out = sys.argv[1], sys.argv[2]
f = open(out, "w"); f.write("t_sim,wall,x,y,z,yaw\n")
def cb(m):
    q = m.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    f.write("%.4f,%.4f,%.5f,%.5f,%.5f,%.5f\n" % (m.header.stamp.sec + m.header.stamp.nsec * 1e-9, time.time(),
            m.pose.position.x, m.pose.position.y, m.pose.position.z, yaw))
n = Node(); n.subscribe(Odometry, "/model/%s/odometry" % model, cb)
while not os.path.exists(out + ".STOP"):
    time.sleep(0.5); f.flush()
f.close()
