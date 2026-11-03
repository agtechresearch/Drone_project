#!/usr/bin/env python3
"""Static AprilTag check in the turned QVIX world (v3_marker, 2026-10-29). Run INSIDE starling-gz with a gz server of
world qvix_rot in partition GZ_PARTITION (QVIX_YAW_DEG must match the world):
  python3 tag_static_test.py <out.json> [x_model y_model z_floor]
Spawns a static camera with the front camera's intrinsics (640x480, hfov 1.187 rad), turns it through 360 deg in 15 deg
steps at the given map point, detects tag36h11 with pupil_apriltags and records per view: ids, decision margin,
tag position in the camera frame, and the TRUE camera->tag geometry from the map, so the pose sign conventions and
accuracy can be checked."""
import json, math, os, sys, time, threading
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from pupil_apriltags import Detector
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import qvix_grid as G  # noqa: E402

out = sys.argv[1]
mx, my, mz = (float(v) for v in sys.argv[2:5]) if len(sys.argv) >= 5 else (0.5 * sum(G.AISLES["C"]), 8.6, 0.68)
W, H, HFOV = 640, 480, 1.187
FX = (W / 2) / math.tan(HFOV / 2); CX, CY = (W - 1) / 2, (H - 1) / 2
TAGS = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gz", "tags_map.json")))
SDF = """<sdf version="1.9"><model name="probe_cam"><static>true</static><link name="link">
<sensor name="cam" type="camera"><camera><horizontal_fov>%.4f</horizontal_fov><image><width>%d</width><height>%d</height></image>
<clip><near>0.05</near><far>40</far></clip></camera><always_on>1</always_on><update_rate>15</update_rate><topic>probe_cam</topic></sensor>
</link></model></sdf>""" % (HFOV, W, H)
node = Node(); st = {"img": None, "t": None}; lock = threading.Lock()


def cb(m):
    with lock:
        st["img"] = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3).copy()
        st["t"] = m.header.stamp.sec + m.header.stamp.nsec * 1e-9


node.subscribe(Image, "/probe_cam", cb)
wx, wy, wz = G.model_to_world(mx, my, mz)
req = EntityFactory(); req.sdf = SDF; req.name = "probe_cam"
req.pose.position.x, req.pose.position.y, req.pose.position.z = wx, wy, wz
print("spawn", node.request("/world/qvix_rot/create", req, EntityFactory, Boolean, 5000))
time.sleep(2)
det = Detector(families="tag36h11", nthreads=4, quad_decimate=1.0, refine_edges=1)
res = []
for k in range(24):
    th_map = math.radians(15 * k)                  # camera axis angle in the MAP (model) frame, CCW from +x
    th_w = th_map + math.radians(G.MODEL_YAW_DEG)
    p = Pose(); p.name = "probe_cam"; p.position.x, p.position.y, p.position.z = wx, wy, wz
    p.orientation.w = math.cos(th_w / 2); p.orientation.z = math.sin(th_w / 2)
    node.request("/world/qvix_rot/set_pose", p, Pose, Boolean, 2000)
    time.sleep(1.0)
    with lock:
        img = st["img"]
    if os.environ.get("SAVE_IMG"):
        from PIL import Image as PI
        PI.fromarray(img).save(os.path.join(os.path.dirname(out), "view_%03d.png" % (15 * k)))
    gray = (0.299 * img[:, :, 0] + 0.587 * img[:, :, 1] + 0.114 * img[:, :, 2]).astype(np.uint8)
    ds = det.detect(gray, estimate_tag_pose=True, camera_params=(FX, FX, CX, CY), tag_size=TAGS["size_m"])
    view = {"cam_axis_map_deg": 15 * k, "detections": []}
    for d in ds:
        tg = TAGS["tags"].get(str(d.tag_id))
        if tg is None:
            view["detections"].append({"id": int(d.tag_id), "unknown": True}); continue
        # truth: tag centre in the camera frame (x right, y down, z forward) from the map geometry
        dxm, dym, dzm = tg["x"] - mx, tg["y"] - my, tg["z"] - mz
        fwd = dxm * math.cos(th_map) + dym * math.sin(th_map); left = -dxm * math.sin(th_map) + dym * math.cos(th_map)
        view["detections"].append({"id": int(d.tag_id), "face": tg["face"], "margin": float(d.decision_margin), "hamming": int(d.hamming),
                                   "t_cam": [float(v) for v in d.pose_t.ravel()], "R": np.asarray(d.pose_R).tolist(),
                                   "t_true": [-left, -dzm, fwd], "tag_normal_map": tg["normal"]})
    res.append(view)
    print("axis %3d deg: %s" % (15 * k, [(v["id"], v.get("face"), round(v.get("margin", 0), 1)) for v in view["detections"]]), flush=True)
json.dump({"cam_map": [mx, my, mz], "yaw_world_deg": G.MODEL_YAW_DEG, "fx": FX, "views": res}, open(out, "w"), indent=1)
