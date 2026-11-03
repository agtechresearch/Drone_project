#!/usr/bin/env python3
"""Raw gz IMU log (2026-11-02, IMU lever-arm check). Run INSIDE starling-gz with GZ_PARTITION set:
  python3 imu_logger.py <world> <model> <out_csv>      stop: touch <out_csv>.STOP
Every .../link/base_link/sensor/imu_sensor/imu message -> t_sim, t_wall, ax ay az (m/s^2), gx gy gz (rad/s), sensor frame (FLU),
exactly as gz publishes them (before PX4's FLU -> FRD turn in GZBridge)."""
import os, sys, threading, time
from gz.transport13 import Node
from gz.msgs10.imu_pb2 import IMU

world, model, out = sys.argv[1], sys.argv[2], sys.argv[3]
f = open(out, "w"); f.write("t_sim,t_wall,ax,ay,az,gx,gy,gz\n")
lock = threading.Lock(); n = [0]


def cb(m):
    t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
    a, g = m.linear_acceleration, m.angular_velocity
    with lock:
        f.write("%.4f,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n" % (t, time.time(), a.x, a.y, a.z, g.x, g.y, g.z))
        n[0] += 1


topic = "/world/%s/model/%s/link/base_link/sensor/imu_sensor/imu" % (world, model)
node = Node()
if not node.subscribe(IMU, topic, cb):
    sys.exit("subscribe failed")
print("[imu_logger] %s -> %s" % (topic, out), flush=True)
while not os.path.exists(out + ".STOP"):
    time.sleep(0.5)
    with lock:
        f.flush()
with lock:
    f.close()
print("[imu_logger] stopped, rows %d" % n[0], flush=True)
