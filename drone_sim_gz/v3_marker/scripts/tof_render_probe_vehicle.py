#!/usr/bin/env python3
"""Spawn the vehicle model starling2_tof (static pose, aisle C R1C5 facing W) into a running world and print the valid fraction
of its depth sensors (2026-10-31 render bug hunt; see tof_render_probe.py). Run INSIDE starling-gz with GZ_PARTITION set:
  python3 tof_render_probe_vehicle.py <world> <qvix_yaw_deg> [z=0.68]"""
import math, os, sys, time
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.boolean_pb2 import Boolean
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ["QVIX_YAW_DEG"] = sys.argv[2]
import qvix_grid as G  # noqa: E402
world = sys.argv[1]; z = float(sys.argv[3]) if len(sys.argv) > 3 else 0.68
wx, wy, wz = G.model_to_world(0.5 * sum(G.AISLES["C"]), 4.6, z)
node = Node(); req = EntityFactory(); req.sdf_filename = "model://starling2_tof"; req.name = "probeveh"
req.pose.position.x, req.pose.position.y, req.pose.position.z = wx, wy, wz
yaw = math.radians(180.0 + G.MODEL_YAW_DEG); req.pose.orientation.w = math.cos(yaw / 2); req.pose.orientation.z = math.sin(yaw / 2)
ok, rep = node.request("/world/%s/create" % world, req, EntityFactory, Boolean, 5000); print("spawn", ok, rep.data if ok else "")
got = {}
for tp in ("/front_tof_real", "/front_tof"):
    node.subscribe(Image, tp, lambda m, tp=tp: got.setdefault(tp, np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width)))
t0 = time.time()
while len(got) < 2 and time.time() - t0 < 25:
    time.sleep(0.2)
for tp in ("/front_tof_real", "/front_tof"):
    d = got.get(tp)
    if d is None:
        print(tp, "no frame"); continue
    f = np.isfinite(d) & (d > 0); print("%-16s valid %.2f median %.3f" % (tp, f.mean(), np.median(d[f]) if f.any() else float("nan")))
