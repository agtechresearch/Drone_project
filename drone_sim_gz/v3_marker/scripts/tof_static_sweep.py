#!/usr/bin/env python3
"""Static check of the simulated ToF + wall_sense fit (2026-10-29): place a STATIC copy of starling2_tof at known
distances from the aisle-C west rack face and compare the fitted distance with the geometry.
Run INSIDE starling-gz in its own partition with a server already running:
  GZ_PARTITION=starling_3 gz sim -s -r qvix.sdf &   then   GZ_PARTITION=starling_3 python3 tof_static_sweep.py
"""
import math, os, sys, time, threading
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import qvix_grid as G
import wall_sense_fit as WF

sdf = open(os.path.expanduser("~/PX4-Autopilot/Tools/simulation/gz/models/starling2_tof/model.sdf")).read()
sdf = sdf.replace("<static>false</static>", "<static>true</static>")
node = Node()
st = {"img": None, "t": 0}
lock = threading.Lock()


def cb(msg):
    with lock:
        st["img"] = (msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9,
                     np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width).copy())


node.subscribe(Image, "/front_tof", cb)
face_x = G.model_to_world(G.AISLES["C"][0], 0)[0]       # world x of the C-west face
y = G.model_to_world(0, 5.0)[1]; z = 0.15 + 0.68
req = EntityFactory(); req.sdf = sdf; req.name = "probe"
req.pose.position.x = face_x + 0.375; req.pose.position.y = y; req.pose.position.z = z
req.pose.orientation.w = math.cos(math.pi / 2); req.pose.orientation.z = math.sin(math.pi / 2)   # yaw 180 = facing W
ok, rep = node.request("/world/qvix/create", req, EntityFactory, Boolean, 5000)
print("create", ok, rep.data)
time.sleep(3)
print("d_true  d_fit  alpha  conf  min_valid_depth  frac_inf")
for d in [0.40, 0.35, 0.30, 0.27, 0.25, 0.23, 0.21, 0.19, 0.17, 0.15]:
    p = Pose(); p.name = "probe"; p.position.x = face_x + d; p.position.y = y; p.position.z = z
    p.orientation.w = math.cos(math.pi / 2); p.orientation.z = math.sin(math.pi / 2)
    node.request("/world/qvix/set_pose", p, Pose, Boolean, 3000)
    time.sleep(1.5)
    with lock:
        t, img = st["img"]
    r, _ = WF.fit(img)
    fin = img[np.isfinite(img)]
    print("%.3f  %s  %.3f  %.3f" % (d, ("%.4f  %+.2f  %.2f" % (r["d"], r["alpha"], r["conf"])) if r else "none",
                                    fin.min() if fin.size else float("nan"), 1 - fin.size / img.size))
    np.save("/tmp/tof_sweep_%03d.npy" % int(d * 100), img)
