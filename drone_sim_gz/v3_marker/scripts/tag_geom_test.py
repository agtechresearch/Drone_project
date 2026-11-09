#!/usr/bin/env python3
"""Controlled AprilTag pose test: heading/position error vs viewing geometry (v3_marker, 2026-10-30). Run INSIDE
starling-gz with a gz server of world qvix_rot in partition GZ_PARTITION (QVIX_YAW_DEG must match the world):
  python3 tag_geom_test.py <out.json> [tag_id ...]          (default tags 30 38)
A static probe camera with the front camera's intrinsics (640x480, hfov 1.187 rad, no distortion) is placed by set_pose,
so there is no PX4/VIO error and no timing error: what is left is the detector + PnP (step 2) and the map/tag geometry
(step 4). Per tag, sweeps around the "reference" pose (camera on the tag normal, axis perpendicular to the tag plane,
tag at the image centre, distance 0.42 m = the scan distance):
  yaw      camera stays on the normal, turned by -25..+25 deg -> tag moves across the image, plane seen obliquely
  shift    camera moved along the face by s, axis still perpendicular -> tag off-centre, plane parallel to the image
  look     camera moved along the face by s and turned to centre the tag -> tag centred, plane seen obliquely
  dist     reference pose at 0.25..1.2 m
Every view is solved with two principal-point conventions (cx = (W-1)/2 as tag_detect.py, and W/2) and evaluated with
wk3.localize.body_pose_from_tag (the flight code path): heading error and camera position error vs the TRUE probe pose.
The probe camera is removed at the end."""
import json, math, os, sys, time, threading
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from pupil_apriltags import Detector
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import qvix_grid as G  # noqa: E402
from wk3 import localize as LZ  # noqa: E402

out = sys.argv[1]
ids = [int(v) for v in sys.argv[2:]] or [30, 38]
W, H, HFOV = 640, 480, 1.187
FX = (W / 2) / math.tan(HFOV / 2)
PP = {"cx_w-1": ((W - 1) / 2, (H - 1) / 2), "cx_w": (W / 2, H / 2)}
TAGS = json.load(open(os.path.join(HERE, "..", "gz", "tags_map.json")))
WORLD = "qvix_rot"
SDF = """<sdf version="1.9"><model name="probe_cam"><static>true</static><link name="link">
<sensor name="cam" type="camera"><camera><horizontal_fov>%.4f</horizontal_fov><image><width>%d</width><height>%d</height></image>
<clip><near>0.05</near><far>40</far></clip></camera><always_on>1</always_on><update_rate>30</update_rate><topic>probe_cam</topic></sensor>
</link></model></sdf>""" % (HFOV, W, H)
node = Node(); st = {"img": None, "n": 0}; lock = threading.Lock()


def cb(m):
    with lock:
        st["img"] = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3).copy(); st["n"] += 1


node.subscribe(Image, "/probe_cam", cb)
req = EntityFactory(); req.sdf = SDF; req.name = "probe_cam"
print("spawn", node.request("/world/%s/create" % WORLD, req, EntityFactory, Boolean, 5000), flush=True)
time.sleep(2)
det = Detector(families="tag36h11", nthreads=4, quad_decimate=1.0, refine_edges=1)


def view(cx_m, cy_m, cz_m, th_map):
    """place the camera at map point, axis angle th_map (CCW from +x, level); return a fresh frame"""
    wx, wy, wz = G.model_to_world(cx_m, cy_m, cz_m)
    th_w = th_map + math.radians(G.MODEL_YAW_DEG)
    p = Pose(); p.name = "probe_cam"; p.position.x, p.position.y, p.position.z = wx, wy, wz
    p.orientation.w = math.cos(th_w / 2); p.orientation.z = math.sin(th_w / 2)
    node.request("/world/%s/set_pose" % WORLD, p, Pose, Boolean, 2000)
    time.sleep(0.25)
    with lock:
        n0 = st["n"]
    for _ in range(100):                           # two new frames after the move (the first may be mid-update)
        time.sleep(0.02)
        with lock:
            if st["n"] >= n0 + 2:
                return st["img"]
    return None


res = []
for tid in ids:
    tg = TAGS["tags"][str(tid)]
    nrm = tg["normal"]                             # +1: tag faces +x (camera looks along -x)
    th0 = math.pi if nrm > 0 else 0.0
    poses = []
    for a in np.arange(-25, 25.01, 2.5):
        poses.append(("yaw", float(a), 0.42, 0.0, math.radians(a)))
    for s in (-0.20, -0.15, -0.10, -0.05, 0.05, 0.10, 0.15, 0.20):
        poses.append(("shift", s, 0.42, s, 0.0))
        poses.append(("look", s, 0.42, s, None))
    for d in (0.25, 0.30, 0.35, 0.42, 0.50, 0.60, 0.75, 0.90, 1.05, 1.20):
        poses.append(("dist", d, d, 0.0, 0.0))
    for kind, val, d, s, dth in poses:
        # camera position: d out along the normal, s along the face (+y map)
        cxm, cym, czm = tg["x"] + nrm * d, tg["y"] + s, tg["z"]
        if dth is None:                            # look-at: turn to centre the tag
            th = math.atan2(tg["y"] - cym, tg["x"] - cxm)
        else:
            th = th0 + dth
        img = view(cxm, cym, czm, th)
        if img is None:
            print("no frame", kind, val, flush=True); continue
        gray = (0.299 * img[:, :, 0] + 0.587 * img[:, :, 1] + 0.114 * img[:, :, 2]).astype(np.uint8)
        # view angle: tag plane normal vs camera axis; image angle: tag direction vs camera axis
        bearing = math.atan2(tg["y"] - cym, tg["x"] - cxm)
        rec = {"tag": tid, "kind": kind, "val": val, "cam_map": [cxm, cym, czm], "cam_th_deg": math.degrees(th),
               "img_angle_deg": math.degrees(LZ.wrap(bearing - th)), "plane_angle_deg": math.degrees(LZ.wrap(th - th0)), "pp": {}}
        for name, (pcx, pcy) in PP.items():
            ds = [x for x in det.detect(gray, estimate_tag_pose=True, camera_params=(FX, FX, pcx, pcy), tag_size=TAGS["size_m"])
                  if x.tag_id == tid]
            if not ds:
                rec["pp"][name] = None; continue
            x = ds[0]
            t = [float(v) for v in x.pose_t.ravel()]; R = np.asarray(x.pose_R).tolist()
            bx, by, bh = LZ.body_pose_from_tag(t, R, tg)
            camx, camy = bx + LZ.CAM_FWD * math.cos(bh), by + LZ.CAM_FWD * math.sin(bh)
            rec["pp"][name] = {"heading_err_deg": math.degrees(LZ.wrap(bh - th)), "pos_err_m": math.hypot(camx - cxm, camy - cym),
                               "along_err_m": camy - cym, "across_err_m": camx - cxm, "margin": float(x.decision_margin),
                               "range_m": math.hypot(t[0], t[2]), "corners": np.asarray(x.corners).tolist()}
        res.append(rec)
        a = rec["pp"]["cx_w-1"]
        print("tag %d %-5s %+6.2f  img %+6.1f plane %+6.1f  -> %s" % (tid, kind, val, rec["img_angle_deg"], rec["plane_angle_deg"],
              "none" if a is None else "head %+.2f deg pos %.4f m" % (a["heading_err_deg"], a["pos_err_m"])), flush=True)

rm = Entity(); rm.name = "probe_cam"; rm.type = Entity.MODEL
print("remove", node.request("/world/%s/remove" % WORLD, rm, Entity, Boolean, 5000))
json.dump({"fx": FX, "tags": ids, "views": res}, open(out, "w"), indent=1)
