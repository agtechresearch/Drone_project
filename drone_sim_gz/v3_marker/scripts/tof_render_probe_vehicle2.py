#!/usr/bin/env python3
"""Vehicle model + an extra rolled-landscape ToF (logs/probe_vehicle_rolled.sdf) in a running world: valid fraction of
front_tof_real (portrait), front_tof_rolled (224x172 rolled 90 deg, same field of view) and front_tof (2026-10-31 bug hunt).
  python3 tof_render_probe_vehicle2.py <world> <qvix_yaw_deg> <sdf>"""
import math, os, sys, time
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.boolean_pb2 import Boolean
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ["QVIX_YAW_DEG"] = sys.argv[2]
import qvix_grid as G  # noqa: E402
world = sys.argv[1]; wx, wy, wz = G.model_to_world(0.5 * sum(G.AISLES["C"]), 4.6, 0.68)
node = Node(); req = EntityFactory(); req.sdf = open(sys.argv[3]).read(); req.name = "probeveh2"
req.pose.position.x, req.pose.position.y, req.pose.position.z = wx, wy, wz
yaw = math.radians(180.0 + G.MODEL_YAW_DEG); req.pose.orientation.w = math.cos(yaw / 2); req.pose.orientation.z = math.sin(yaw / 2)
ok, rep = node.request("/world/%s/create" % world, req, EntityFactory, Boolean, 5000); print("spawn", ok, rep.data if ok else "")
got = {}
TP = tuple(sys.argv[4].split(",")) if len(sys.argv) > 4 else ("/front_tof_real", "/front_tof_rolled", "/front_tof")
for tp in TP:
    node.subscribe(Image, tp, lambda m, tp=tp: got.setdefault(tp, np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width)))
t0 = time.time()
while len(got) < len(TP) and time.time() - t0 < 25:
    time.sleep(0.2)
for tp in TP:
    d = got.get(tp)
    print("%-18s %s" % (tp, "no frame" if d is None else "%dx%d valid %.2f median %.3f" % (d.shape[1], d.shape[0], (np.isfinite(d) & (d > 0)).mean(), np.nanmedian(np.where(np.isfinite(d), d, np.nan)))))
if "/front_tof_real" in got and "/front_tof_rolled" in got:
    a, b = got["/front_tof_real"], got["/front_tof_rolled"]
    for k in range(4):
        r = np.rot90(b, k)
        if r.shape == a.shape:
            m = np.isfinite(a) & np.isfinite(r)
            print("rot90 k=%d: mean |diff| %.4f over %d px" % (k, np.mean(np.abs(a[m] - r[m])) if m.any() else -1, m.sum()))
np.savez("/tmp/probe_rolled_%s.npz" % world, **{tp.strip("/"): got[tp] for tp in got})
