#!/usr/bin/env python3
"""Variant hunt for the front_tof_real render bug (2026-10-31): portrait depth sensors that differ from the working probe in one
thing each (gaussian noise, the vehicle's mount offset, visibility mask, update rate). Run like tof_render_probe.py:
  python3 tof_render_probe_variants.py <world> <qvix_yaw_deg>"""
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
wx, wy, wz = G.model_to_world(0.5 * sum(G.AISLES["C"]), 4.6, 0.68); yaw = math.radians(180.0 + G.MODEL_YAW_DEG)
NOISE = "<noise><type>gaussian</type><mean>0</mean><stddev>0.005</stddev></noise>"
VAR = {"plain": ("0 0 0 0 0 0", "", 5, ""), "noise": ("0 0 0 0 0 0", NOISE, 5, ""), "offset": ("0.066 -0.009 0.012 0 0 0", "", 5, ""),
       "vismask": ("0 0 0 0 0 0", "", 5, "<visibility_mask>1</visibility_mask>"), "rate10": ("0 0 0 0 0 0", "", 10, ""),
       "all": ("0.066 -0.009 0.012 0 0 0", NOISE, 10, "<visibility_mask>1</visibility_mask>")}
sens = ""
for n, (pose, noise, rate, vm) in VAR.items():
    sens += ('<sensor name="%s" type="depth_camera"><pose>%s</pose><camera><horizontal_fov>1.4853</horizontal_fov><image><width>172</width>'
             '<height>224</height><format>R_FLOAT32</format></image><clip><near>0.1</near><far>6.0</far></clip>%s%s</camera><always_on>1</always_on>'
             '<update_rate>%d</update_rate><topic>pv_%s</topic></sensor>' % (n, pose, noise, vm, rate, n))
req = EntityFactory(); req.sdf = ('<sdf version="1.9"><model name="pv"><static>true</static><pose>%.4f %.4f %.4f 0 0 %.6f</pose><link name="l">%s</link>'
                                  '</model></sdf>' % (wx, wy, wz, yaw, sens))
node = Node(); ok, rep = node.request("/world/%s/create" % world, req, EntityFactory, Boolean, 5000); print("spawn", ok, rep.data if ok else "")
got = {}
for n in VAR:
    node.subscribe(Image, "/pv_" + n, lambda m, n=n: got.setdefault(n, np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width)))
t0 = time.time()
while len(got) < len(VAR) and time.time() - t0 < 25:
    time.sleep(0.2)
for n in VAR:
    d = got.get(n)
    print("%-8s %s" % (n, "no frame" if d is None else "valid %.2f median %.3f" % ((np.isfinite(d) & (d > 0)).mean(), np.nanmedian(np.where(np.isfinite(d), d, np.nan)))))
