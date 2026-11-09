#!/usr/bin/env python3
"""Simulated voxl-tag-detector (v3_marker, 2026-10-29). Run INSIDE starling-gz with GZ_PARTITION set:
  python3 tag_detect.py <model> <out_csv> [udp_port=14662] [camera topic=/front_camera]
Detects tag36h11 in every front-camera frame (640x480, hfov 1.187 rad) with pupil_apriltags and sends one UDP JSON per
frame to 127.0.0.1:<port>:  {"t": sim time, "dets": [{"id", "t_cam": [x right, y down, z fwd], "R": 3x3, "margin", "hamming"}]}
= the fields voxl-tag-detector publishes (tag pose in the camera frame), so the flight side is the same on the real drone
(only the input adapter differs). Also logs the TRUE body pose from /model/<model>/odometry for accuracy checks.
Stop: touch <out_csv>.STOP"""
import json, math, os, socket, sys, threading, time
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.odometry_pb2 import Odometry
from pupil_apriltags import Detector

model, out = sys.argv[1], sys.argv[2]
port = int(sys.argv[3]) if len(sys.argv) > 3 else 14662
topic = sys.argv[4] if len(sys.argv) > 4 else "/front_camera"
W, H, HFOV = 640, 480, 1.187
# WK_SENSORS=real (2026-10-30, cameras_2026-10-30.zip): the real tag detector's view = tracking_front undistorted at scale 0.6
# (fx 276.6 px on 1280x800, model sensor front_camera_real) in fast mode (quad_decimate 2), detecting on ~5 fps
# (30 fps, skip_n_frames 5). Frames in between are not detected; their row keeps the truth pose (n_det -1).
REAL = os.environ.get("WK_SENSORS", "") == "real"
QUAD_DECIMATE = 1.0
if REAL:
    W, H, HFOV, QUAD_DECIMATE = 1280, 800, 2.3268, 2.0
    if len(sys.argv) <= 4:
        topic = "/front_camera_real"
TAG_HZ = float(os.environ.get("WK_TAG_HZ") or ("5" if REAL else "0"))     # 0 = every frame
FX = (W / 2) / math.tan(HFOV / 2); CX, CY = (W - 1) / 2, (H - 1) / 2
TAGS = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gz", "tags_map.json")))
SIZE = TAGS["size_m"]
det = Detector(families="tag36h11", nthreads=2, quad_decimate=QUAD_DECIMATE, refine_edges=1)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
lock = threading.Lock(); st = {"img": None, "t": None, "pose": None, "busy": False}
# true_t (2026-10-31, codex ground-truth-timestamps): the odometry message's own stamp of the true pose in the same row; the row
# time t_sim is the IMAGE stamp, so true_t - t_sim is the (previously invisible) offset between the two
f = open(out, "w"); f.write("t_sim,n_det,ids,margins,true_x,true_y,true_yaw,true_t\n")
# every detection frame in full (survey_map.py): {"t", "dets": [{"id", "t_cam", "R", "margin", "hamming"}]} - the real tag_detections fields
raw = open(os.path.splitext(out)[0] + "_raw.jsonl", "w")


def on_odom(m):
    with lock:
        st["pose"] = m.pose; st["pose_t"] = m.header.stamp.sec + m.header.stamp.nsec * 1e-9


def on_img(m):
    with lock:
        st["img"] = (m.header.stamp.sec + m.header.stamp.nsec * 1e-9, bytes(m.data), m.width, m.height)


node = Node()
node.subscribe(Odometry, "/model/%s/odometry" % model, on_odom)
node.subscribe(Image, topic, on_img)
last_t = None
last_det = -1e9
while not os.path.exists(out + ".STOP"):
    with lock:
        item = st["img"]; st["img"] = None; pose = st["pose"]; pose_t = st.get("pose_t", float("nan"))
    if item is None or item[0] == last_t:
        time.sleep(0.005); continue
    t, data, w, h = item; last_t = t
    dets = None
    if TAG_HZ <= 0 or t - last_det >= 1.0 / TAG_HZ - 0.02:
        last_det = t
        img = np.frombuffer(data, np.uint8).reshape(h, w, 3)
        gray = (0.299 * img[:, :, 0] + 0.587 * img[:, :, 1] + 0.114 * img[:, :, 2]).astype(np.uint8)
        ds = det.detect(gray, estimate_tag_pose=True, camera_params=(FX, FX, CX, CY), tag_size=SIZE)
        dets = [{"id": int(d.tag_id), "t_cam": [float(v) for v in d.pose_t.ravel()], "R": np.asarray(d.pose_R).tolist(),
                 "margin": float(d.decision_margin), "hamming": int(d.hamming)} for d in ds]
        msg = json.dumps({"t": t, "dets": dets})
        sock.sendto(msg.encode(), ("127.0.0.1", port))
        if dets:
            raw.write(msg + "\n"); raw.flush()
    if pose is not None:
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        tx, ty, tyaw = pose.position.x, pose.position.y, yaw
    else:
        tx = ty = tyaw = float("nan")
    dl = dets or []
    f.write("%.3f,%d,%s,%s,%.4f,%.4f,%.5f,%.4f\n" % (t, -1 if dets is None else len(dl), " ".join(str(d["id"]) for d in dl),
            " ".join("%.0f" % d["margin"] for d in dl), tx, ty, tyaw, pose_t if pose is not None else float("nan")))
    f.flush()
f.close(); raw.close()
