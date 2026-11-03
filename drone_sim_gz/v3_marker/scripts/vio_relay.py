#!/usr/bin/env python3
"""VIO stand-in relay with error injection (v2_wallkeep, 2026-10-29). Run INSIDE starling-gz with GZ_PARTITION set.
  python3 vio_relay.py <model e.g. starling2_tof_2> <control.json>

The starling2_tof model publishes its ground-truth covariance odometry on /wallkeep/vio_raw (make_tof_model.py);
this relay republishes it on /model/<model>/odometry_with_covariance, which PX4's gz_bridge feeds to the EKF as
vehicle_visual_odometry. Without injection it is a pass-through (same header stamps -> PX4 lockstep clock unaffected).

Injection (read from <control.json>, re-read every 0.5 s wall, so a run can change it in flight):
  {"rot_deg": 1.5,            # VIO frame rotated about the spawn point: the estimate believes it faces exactly the route
                              #   heading while the true heading is rot_deg clockwise (NED) of it -> straight legs drift
                              #   sideways (real cause: vehicle placed off-axis at power-on, VIO yaw 0 = initial facing)
   "drift_n": 0.0, "drift_e": 0.004,   # constant position-drift rate (m/s, NED) added to the estimate once airborne
   "airborne_dz": 0.15,       # drift starts when the true height exceeds spawn height + this
   "step_e": 0.05, "step_at": 20.0}   # sudden estimate jump (m, NED E) step_at s after airborne (VIO relocalisation glitch)
Rotation convention: estimate = Rz_ENU(+rot) * (truth - spawn) + spawn; orientation likewise (ENU yaw + rot = NED yaw - rot).
Prints one status line per second (sim t, applied offset).
"""
import json, math, os, sys, threading, time
from gz.transport13 import Node
from gz.msgs10.odometry_with_covariance_pb2 import OdometryWithCovariance

model = sys.argv[1]; ctl_path = sys.argv[2]
OUT = "/model/%s/odometry_with_covariance" % model
cfg = {"rot_deg": 0.0, "drift_n": 0.0, "drift_e": 0.0, "airborne_dz": 0.15, "step_e": 0.0, "step_at": 1e9}
st = {"spawn": None, "t_air": None, "n": 0, "last_print": 0.0, "off": (0.0, 0.0)}
lock = threading.Lock()


def load_cfg():
    try:
        with open(ctl_path) as f:
            c = json.load(f)
        with lock:
            for k in cfg:
                if k in c:
                    cfg[k] = float(c[k])
    except (OSError, ValueError):
        pass


def qmul(a, b):
    aw, ax, ay, az = a; bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


node = Node()
pub = node.advertise(OUT, OdometryWithCovariance)


def cb(msg):
    p = msg.pose_with_covariance.pose.position
    q = msg.pose_with_covariance.pose.orientation
    t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    with lock:
        if st["spawn"] is None:
            st["spawn"] = (p.x, p.y, p.z)
        sx, sy, sz = st["spawn"]
        rot = math.radians(cfg["rot_deg"]); dn, de = cfg["drift_n"], cfg["drift_e"]
        step_e, step_at = cfg["step_e"], cfg["step_at"]
        if st["t_air"] is None and p.z > sz + cfg["airborne_dz"]:
            st["t_air"] = t
        age = 0.0 if st["t_air"] is None else t - st["t_air"]
    out = OdometryWithCovariance(); out.CopyFrom(msg)
    c, s = math.cos(rot), math.sin(rot)
    rx, ry = p.x - sx, p.y - sy
    ox = sx + c * rx - s * ry + de * age + (step_e if age >= step_at else 0.0)          # ENU x = NED E
    oy = sy + s * rx + c * ry + dn * age          # ENU y = NED N
    op = out.pose_with_covariance.pose
    op.position.x, op.position.y = ox, oy
    qz = (math.cos(rot / 2), 0.0, 0.0, math.sin(rot / 2))
    w, x, y, z = qmul(qz, (q.w, q.x, q.y, q.z))
    op.orientation.w, op.orientation.x, op.orientation.y, op.orientation.z = w, x, y, z
    if age > 0.0 and (dn or de):
        # drift also biases the velocity estimate: add the drift rate expressed in body FLU (twist is body frame)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        vx_w, vy_w = de, dn
        lin = out.twist_with_covariance.twist.linear
        lin.x += math.cos(yaw) * vx_w + math.sin(yaw) * vy_w
        lin.y += -math.sin(yaw) * vx_w + math.cos(yaw) * vy_w
    pub.publish(out)
    with lock:
        st["n"] += 1; st["off"] = (ox - p.x, oy - p.y)
        if time.time() - st["last_print"] >= 1.0:
            st["last_print"] = time.time()
            print("t=%.2f n=%d rot=%.2f drift=(%.4f,%.4f) air_age=%.1f offset_E=%+.4f offset_N=%+.4f" % (
                t, st["n"], cfg["rot_deg"], dn, de, age, st["off"][0], st["off"][1]), flush=True)


load_cfg()
if not node.subscribe(OdometryWithCovariance, "/wallkeep/vio_raw", cb):
    sys.exit("subscribe failed")
print("[vio_relay] /wallkeep/vio_raw -> %s, control %s, cfg %s" % (OUT, ctl_path, cfg), flush=True)
while True:
    time.sleep(0.5)
    load_cfg()
