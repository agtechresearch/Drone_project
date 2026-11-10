#!/usr/bin/env python3
"""시뮬 voxl-tag-detector (드리프트 실험용). starling-gz 컨테이너 안에서 GZ_PARTITION 을 맞춰 실행.

    GZ_PARTITION=starling_2 python3 sim_tag_detect.py <model> <out_csv> <udp_port> [camera topic=/front_camera_real] [tag_size=0.20]

카메라 토픽의 프레임마다 tag36h11 을 pupil_apriltags 로 검출해 UDP JSON 한 개를 127.0.0.1:<udp_port> 로 보낸다
(s2flight.sim.UdpTagFeed 가 받는다). 형식은 drone_sim_gz/v3_marker/scripts/tag_detect.py 와 같다:
    {"t": <이미지 sim 초>, "dets": [{"id", "t_cam": [x 우, y 하, z 전], "R": 3x3 (tag→cam), "margin", "hamming"}]}
tag_detect.py 와 다른 점: 태그 크기를 인자로 받고(tags_map.json 을 읽지 않음), 이미지 크기는 메시지에서 읽고, 초점거리는
토픽별 HFOV 표(또는 CAM_HFOV 환경변수)로 만든다. 검출 주기 TAG_HZ(기본 5 = 실기 skip_n_frames 5 와 비슷), QUAD_DECIMATE(기본 2).

같이 쓰는 진실값: /model/<model>/odometry 의 자세를 CSV 에 함께 적는다 (t_sim,n_det,ids,margins,true_x,true_y,true_yaw,true_t).
정지: touch <out_csv>.STOP
"""
import json
import math
import os
import socket
import sys
import threading
import time

import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.odometry_pb2 import Odometry
from pupil_apriltags import Detector

if len(sys.argv) < 4:
    print(__doc__)
    sys.exit(2)
model, out, port = sys.argv[1], sys.argv[2], int(sys.argv[3])
topic = sys.argv[4] if len(sys.argv) > 4 else "/front_camera_real"
SIZE = float(sys.argv[5]) if len(sys.argv) > 5 else 0.20

# 토픽별 수평 화각 (starling2_tof model.sdf): front_camera_real = 실기 tracking_front 와 같은 1280x800, 2.3268 rad
HFOV_BY_TOPIC = {"/front_camera_real": 2.3268, "/front_camera": 1.187}
HFOV = float(os.environ.get("CAM_HFOV") or HFOV_BY_TOPIC.get(topic, 0.0))
if HFOV <= 0:
    sys.exit("카메라 토픽 {} 의 HFOV 를 모른다: CAM_HFOV 환경변수로 줄 것".format(topic))
TAG_HZ = float(os.environ.get("TAG_HZ") or ("5" if topic == "/front_camera_real" else "0"))   # 0 = 매 프레임
QUAD_DECIMATE = float(os.environ.get("QUAD_DECIMATE") or ("2.0" if topic == "/front_camera_real" else "1.0"))

det = Detector(families="tag36h11", nthreads=2, quad_decimate=QUAD_DECIMATE, refine_edges=1)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
lock = threading.Lock()
st = {"img": None, "pose": None, "pose_t": float("nan")}
f = open(out, "w")
f.write("t_sim,n_det,ids,margins,true_x,true_y,true_yaw,true_t\n")
raw = open(os.path.splitext(out)[0] + "_raw.jsonl", "w")


def on_odom(m):
    with lock:
        st["pose"] = m.pose
        st["pose_t"] = m.header.stamp.sec + m.header.stamp.nsec * 1e-9


def on_img(m):
    with lock:
        st["img"] = (m.header.stamp.sec + m.header.stamp.nsec * 1e-9, bytes(m.data), m.width, m.height)


def to_gray(data, w, h):
    n = len(data)
    if n == w * h:
        return np.frombuffer(data, np.uint8).reshape(h, w)
    if n == w * h * 3:
        img = np.frombuffer(data, np.uint8).reshape(h, w, 3)
        return (0.299 * img[:, :, 0] + 0.587 * img[:, :, 1] + 0.114 * img[:, :, 2]).astype(np.uint8)
    if n == w * h * 4:
        img = np.frombuffer(data, np.uint8).reshape(h, w, 4)
        return (0.299 * img[:, :, 0] + 0.587 * img[:, :, 1] + 0.114 * img[:, :, 2]).astype(np.uint8)
    return None


node = Node()
node.subscribe(Odometry, "/model/%s/odometry" % model, on_odom)
node.subscribe(Image, topic, on_img)
print("[sim_tag_detect] model=%s topic=%s hfov=%.4f size=%.3f udp=%d tag_hz=%g decimate=%g" % (
    model, topic, HFOV, SIZE, port, TAG_HZ, QUAD_DECIMATE), flush=True)

last_t = None
last_det = -1e9
n_frames = n_sent = 0
intr = None
while not os.path.exists(out + ".STOP"):
    with lock:
        item = st["img"]
        st["img"] = None
        pose = st["pose"]
        pose_t = st["pose_t"]
    if item is None or item[0] == last_t:
        time.sleep(0.005)
        continue
    t, data, w, h = item
    last_t = t
    n_frames += 1
    if intr is None or intr[0] != w:
        fx = (w / 2.0) / math.tan(HFOV / 2.0)
        intr = (w, h, fx, (w - 1) / 2.0, (h - 1) / 2.0)
        print("[sim_tag_detect] image %dx%d fx=%.1f" % (w, h, fx), flush=True)
    dets = None
    if TAG_HZ <= 0 or t - last_det >= 1.0 / TAG_HZ - 0.02:
        gray = to_gray(data, w, h)
        if gray is not None:
            last_det = t
            ds = det.detect(gray, estimate_tag_pose=True, camera_params=(intr[2], intr[2], intr[3], intr[4]), tag_size=SIZE)
            dets = [{"id": int(d.tag_id), "t_cam": [float(v) for v in d.pose_t.ravel()], "R": np.asarray(d.pose_R).tolist(),
                     "margin": float(d.decision_margin), "hamming": int(d.hamming)} for d in ds]
            msg = json.dumps({"t": t, "dets": dets})
            sock.sendto(msg.encode(), ("127.0.0.1", port))
            n_sent += 1
            if dets:
                raw.write(msg + "\n")
                raw.flush()
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
f.close()
raw.close()
print("[sim_tag_detect] stopped: frames=%d sent=%d" % (n_frames, n_sent), flush=True)
