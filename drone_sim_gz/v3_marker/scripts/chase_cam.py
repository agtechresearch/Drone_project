#!/usr/bin/env python3
"""Omniscient chase camera as a SIM SENSOR (2026-10-29), so its frames carry sim time and line up exactly with the front
camera / ToF / truth (a GUI screen recording is wall-clock and needs a display). Run INSIDE starling-gz with GZ_PARTITION:
  python3 chase_cam.py <model e.g. starling2_tof_2> [stop_file]
Spawns a static model "chase_cam" with an 800x450 15 Hz camera (topic /chase_cam) and moves it with /world/qvix/set_pose:
  - x  = centre of the aisle the vehicle started in (fixed), so a sideways drift shows as the vehicle leaving the image centre
  (visibility_mask 1: the orange GUI halo, visibility_flags 2, is not drawn)
  - y  = vehicle y + CHASE_BACK (north of it, looking south down the aisle), clamped inside the room
  - z  = vehicle z + CHASE_UP
  y/z low-passed (CHASE_TAU s, on the vehicle's sim time), always looking at the low-passed vehicle position, no roll.
Save the frames with cam_grab.py <dir> 15 3600 /chase_cam.
"""
import math, os, sys, time, threading
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import qvix_grid as G  # noqa: E402
from gz.transport13 import Node
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

model = sys.argv[1]; stop = sys.argv[2] if len(sys.argv) > 2 else "/tmp/chase_cam_%s.STOP" % model
BACK = float(os.environ.get("CHASE_BACK", "1.6")); UP = float(os.environ.get("CHASE_UP", "0.35"))
TAU = float(os.environ.get("CHASE_TAU", "0.6")); W = os.environ.get("CHASE_WORLD", "qvix")
SDF = """<sdf version="1.9"><model name="chase_cam"><static>true</static><link name="link">
<sensor name="chase" type="camera"><camera><horizontal_fov>1.0</horizontal_fov><image><width>800</width><height>450</height></image>
<clip><near>0.05</near><far>40</far></clip><visibility_mask>1</visibility_mask></camera><always_on>1</always_on><update_rate>15</update_rate><topic>chase_cam</topic></sensor>
</link></model></sdf>"""
node = Node()
st = {"p": None, "t": None}
lock = threading.Lock()


def on_odom(m):
    with lock:
        st["p"] = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        st["t"] = m.header.stamp.sec + m.header.stamp.nsec * 1e-9


node.subscribe(Odometry, "/model/%s/odometry" % model, on_odom)
t0 = time.time()
while st["p"] is None and time.time() - t0 < 30:
    time.sleep(0.1)
if st["p"] is None:
    sys.exit("no odometry for %s" % model)
x0, y0, z0 = st["p"]
mx, my, _ = G.world_to_model(x0, y0)
col = next((c for c, (a, b) in G.AISLES.items() if a <= mx <= b), None)
cx = G.model_to_world(0.5 * sum(G.AISLES[col]), 0)[0] if col else x0
room = "R1" if my < 11 else "R2"
ymax = G.model_to_world(0, G.ROOMS[room]["y"][1] - 0.2)[1]
req = EntityFactory(); req.sdf = SDF; req.name = "chase_cam"
req.pose.position.x = cx; req.pose.position.y = min(y0 + BACK, ymax); req.pose.position.z = z0 + UP
ok, rep = node.request("/world/%s/create" % W, req, EntityFactory, Boolean, 5000)
print("[chase_cam] spawn %s aisle %s x %.3f" % (ok and rep.data, col, cx), flush=True)
fy, fz, last_t = y0, z0, None
n = 0
while not os.path.exists(stop):
    with lock:
        (px, py, pz), t = st["p"], st["t"]
    if last_t is not None and t > last_t:
        k = min(1.0, (t - last_t) / TAU)
        fy += k * (py - fy); fz += k * (pz - fz)
    last_t = t
    cy = min(fy + BACK, ymax); cz = fz + UP
    dx, dy, dz = cx - cx, fy - cy, fz - cz           # look at the low-passed vehicle position (camera x = aisle centre)
    yaw = math.atan2(dy, dx if abs(dx) > 1e-6 else 0.0) if abs(dy) > 1e-6 else -math.pi / 2
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    cr, sr = 1.0, 0.0
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2); cyw, syw = math.cos(yaw / 2), math.sin(yaw / 2)
    p = Pose(); p.name = "chase_cam"
    p.position.x, p.position.y, p.position.z = cx, cy, cz
    p.orientation.w = cr * cp * cyw + sr * sp * syw; p.orientation.x = sr * cp * cyw - cr * sp * syw
    p.orientation.y = cr * sp * cyw + sr * cp * syw; p.orientation.z = cr * cp * syw - sr * sp * cyw
    node.request("/world/%s/set_pose" % W, p, Pose, Boolean, 1000)
    n += 1
    if n % 200 == 0:
        print("[chase_cam] t=%.1f cam (%.2f, %.2f, %.2f) veh (%.2f, %.2f, %.2f)" % (t, cx, cy, cz, px, py, pz), flush=True)
    time.sleep(0.05)
print("[chase_cam] stopped after %d pose updates" % n)
