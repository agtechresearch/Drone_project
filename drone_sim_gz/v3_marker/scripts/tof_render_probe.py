#!/usr/bin/env python3
"""Why does the real-spec ToF (front_tof_real, 172 x 224 portrait depth camera) see ~6 % valid pixels in some worlds
(sv2 -40 deg, sv4 60 deg) while the landscape front_tof is fine? (v3_marker, 2026-10-31). Run INSIDE starling-gz with a gz
server of <world> running in GZ_PARTITION:
  python3 tof_render_probe.py <world> <qvix_yaw_deg>
Spawns a static probe in aisle C (R1C5, 0.68 m, facing the W face) with three depth sensors and prints the valid fraction:
  portrait   172 x 224, hfov 85.1 deg (as front_tof_real)
  rolled     224 x 172 landscape, hfov 100.2 deg, rolled 90 deg about the optical axis (same field of view on the vehicle)
  landscape  224 x 172, hfov 106.5 deg (as front_tof)"""
import math, os, sys, time
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.boolean_pb2 import Boolean

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ["QVIX_YAW_DEG"] = sys.argv[2]
import qvix_grid as G  # noqa: E402

world = sys.argv[1]
mx, my = 0.5 * sum(G.AISLES["C"]), 4.6
wx, wy, wz = G.model_to_world(mx, my, 0.68)
yaw = math.radians(180.0 + G.MODEL_YAW_DEG)          # facing map W (-x of the module) in the world (ENU yaw)
SENS = """<sensor name="{n}" type="depth_camera"><pose>0 0 0 {r} 0 0</pose><camera><horizontal_fov>{h}</horizontal_fov>
<image><width>{w}</width><height>{ht}</height><format>R_FLOAT32</format></image><clip><near>0.1</near><far>6</far></clip></camera>
<always_on>1</always_on><update_rate>5</update_rate><topic>probe_{n}</topic></sensor>"""
sdf = """<sdf version="1.9"><model name="tofprobe"><static>true</static><pose>%.4f %.4f %.4f 0 0 %.6f</pose><link name="l">%s%s%s</link></model></sdf>""" % (
    wx, wy, wz, yaw, SENS.format(n="portrait", r=0, h=1.4853, w=172, ht=224), SENS.format(n="rolled", r=1.5708, h=1.7487, w=224, ht=172),
    SENS.format(n="landscape", r=0, h=1.8588, w=224, ht=172))
node = Node()
req = EntityFactory(); req.sdf = sdf
ok, rep = node.request("/world/%s/create" % world, req, EntityFactory, Boolean, 5000)
print("spawn", ok, rep.data if ok else "")
got = {}
for n in ("portrait", "rolled", "landscape"):
    def cb(m, n=n):
        if n not in got:
            got[n] = np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width)
    node.subscribe(Image, "/probe_" + n, cb)
t0 = time.time()
while len(got) < 3 and time.time() - t0 < 20:
    time.sleep(0.2)
for n in ("portrait", "rolled", "landscape"):
    d = got.get(n)
    if d is None:
        print("%-9s no frame" % n); continue
    f = np.isfinite(d) & (d > 0)
    print("%-9s %dx%d valid %.2f median %.3f m" % (n, d.shape[1], d.shape[0], f.mean(), np.median(d[f]) if f.any() else float("nan")))
