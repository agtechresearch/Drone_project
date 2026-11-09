#!/usr/bin/env python3
"""Rack-face distance/angle from the front depth sensor (v2_wallkeep, 2026-10-29). Run INSIDE starling-gz (GZ_PARTITION set).
  python3 wall_sense.py <model e.g. starling2_tof_2> <out_csv> [udp_port=14650] [vis_dir]

Per depth frame (/front_tof, R_FLOAT32 depth along the optical axis, 224x172, hfov 106.5 deg):
  1. every image column -> the NEAREST surface in that column (3rd smallest forward distance, robust to noise):
     the rack front (tray edges + posts) is the front-most thing; the gaps between trays are open, so a
     column-wise minimum finds the rack plane even when the camera looks into a gap,
  2. those points (forward X, left Y) -> RANSAC line + least squares on the inliers (tol 2 cm),
  3. d_sensor = perpendicular distance sensor -> plane, alpha = camera yaw CCW (ENU sense) relative to facing the plane
     squarely, d_body = d_sensor + 0.075 cos(alpha) (sensor sits 0.075 m ahead of the body origin),
  4. conf = inlier fraction of the valid columns; span = lateral extent of the inliers.
Map-free: deciding whether the plane is "the rack face" is the consumer's job (wall_keep layer gates with the QVIX map).
Also logs the GROUND TRUTH for the same frame from /model/<model>/odometry (truth; the PX4 VIO feed is relayed and may
carry injected errors): distance body -> nearest rack face in the camera direction and the true alpha, via qvix_grid.
Output: UDP JSON to 127.0.0.1:<port> per frame {t, d, alpha, conf, span, n} (the flight-side wall_keep layer listens)
and one CSV row per frame. vis_dir: false-colour depth frames with the fitted line + numbers (for the comparison videos).
"""
import csv, json, math, os, socket, sys, threading, time
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.odometry_pb2 import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import qvix_grid as G  # noqa: E402
from wall_sense_fit import fit, HFOV, SENSOR_X  # noqa: E402

model = sys.argv[1]; out_csv = sys.argv[2]
port = int(sys.argv[3]) if len(sys.argv) > 3 else 14650
vis_dir = sys.argv[4] if len(sys.argv) > 4 else ""
# WK_SENSORS=real (2026-10-30, cameras_2026-10-30.zip): model sensor front_tof_real = the real ToF mount (body 0.066 fwd on the vehicle, 0.080 in the sim: inside the body mesh at 0.066,
# 0.009 right; wide axis vertical -> hfov 85.1 deg, 172 x 224, 10 Hz). WK_TOF_HZ thins the frames (real measured 2.0 fps).
REAL = os.environ.get("WK_SENSORS", "") == "real"
TOPIC, FIT = ("/front_tof_real", dict(hfov=1.4853, sensor_x=0.080, sensor_y=-0.009)) if REAL else ("/front_tof", {})
TOF_HZ = float(os.environ.get("WK_TOF_HZ") or "0")      # 0 = every frame
# WK_TOF_RAW=1: keep every processed depth frame (float16, metres along the optical axis) for the survey map, in chunks
# <run>/tof_raw/chunk_NNNN.npz (t, depth[n, h, w]) + meta.json (topic, size, hfov, sensor offset) -> survey_map.py
# WK_TOF_MODEL=<json> (2026-10-31): degrade the ideal gz depth like a real ToF (tof_sim_model.py: range-proportional noise,
# far / grazing surfaces dropped, flying pixels). Every frame also reports its valid-pixel fraction (UDP "valid", csv column
# "valid") so the flight side can tell a blind ToF from an empty view.
TOF_MODEL = None
if os.environ.get("WK_TOF_MODEL"):
    from tof_sim_model import ToFModel
    TOF_MODEL = ToFModel(os.environ["WK_TOF_MODEL"], FIT.get("hfov", HFOV))
RAW_DIR = os.path.join(os.path.dirname(os.path.abspath(out_csv)), "tof_raw") if os.environ.get("WK_TOF_RAW") == "1" else ""
raw_buf = {"t": [], "d": [], "n": 0}


def raw_flush():
    if RAW_DIR and raw_buf["t"]:
        np.savez_compressed(os.path.join(RAW_DIR, "chunk_%04d.npz" % raw_buf["n"]), t=np.array(raw_buf["t"]), depth=np.stack(raw_buf["d"]))
        raw_buf["t"], raw_buf["d"] = [], []; raw_buf["n"] += 1
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
truth = {"pose": None}
lock = threading.Lock()
if vis_dir:
    os.makedirs(vis_dir, exist_ok=True)
    from PIL import Image as PILImage, ImageDraw
f = open(out_csv, "w", newline="")
w = csv.writer(f)
w.writerow(["t_sim", "wall", "d_meas", "alpha_meas_deg", "conf", "span", "n_cols", "rms",
            "x_true", "y_true", "z_true", "yaw_true_ned_deg", "face", "d_true", "alpha_true_deg", "frame", "valid"])
st = {"frame": 0, "last_print": 0.0, "last_t": -1e9}


def yaw_enu(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def truth_face(x, y, yaw):
    """nearest rack face in the camera direction (W or E heading only) -> (face label, d_body, alpha_true_deg)"""
    yaw = yaw - math.radians(G.MODEL_YAW_DEG)          # v3: heading relative to the (turned) rack = map frame
    yn = (90.0 - math.degrees(yaw)) % 360.0          # NED yaw
    mx, my, _ = G.world_to_model(x, y)
    ry0, ry1 = G.ROOMS["R1"]["racks_y"] if my < 11 else G.ROOMS["R2"]["racks_y"]
    for col, (x0, x1) in G.AISLES.items():
        if x0 <= mx <= x1:
            break
    else:
        return "", float("nan"), float("nan")
    if not (ry0 <= my <= ry1):
        return "", float("nan"), float("nan")
    if abs(((yn - 270.0) + 180) % 360 - 180) < 30 and col != "A":
        return col + "W", mx - x0, math.degrees(((yaw - math.pi) + math.pi) % (2 * math.pi) - math.pi)
    if abs(((yn - 90.0) + 180) % 360 - 180) < 30 and col != "E":
        return col + "E", x1 - mx, math.degrees((yaw + math.pi) % (2 * math.pi) - math.pi)
    return "", float("nan"), float("nan")


def save_vis(depth, r, extra, n):
    H, W = depth.shape
    v = np.clip(np.nan_to_num(depth, nan=6.0, posinf=6.0), 0.1, 1.5)
    g = ((1.5 - v) / 1.4 * 255).astype(np.uint8)        # near = bright
    rgb = np.stack([g, (g * 0.6).astype(np.uint8), (255 - g) // 3], axis=-1)
    im = PILImage.fromarray(rgb).resize((W * 3, H * 3), PILImage.NEAREST)
    dr = ImageDraw.Draw(im)
    X, Y, ok, inl = extra
    if inl is not None:
        cols = np.arange(W)[ok]
        for c, good in zip(cols, inl):
            dr.line([(c * 3, H * 3 - 14), (c * 3 + 2, H * 3 - 14)], fill=(0, 255, 0) if good else (255, 0, 0), width=6)
    txt = "ToF  " + ("d %.3f m  a %+.2f deg  conf %.2f" % (r["d"], r["alpha"], r["conf"]) if r else "no plane")
    dr.rectangle([0, 0, W * 3, 22], fill=(0, 0, 0)); dr.text((6, 5), txt, fill=(255, 255, 255))
    im.save(os.path.join(vis_dir, "f%06d.jpg" % n), quality=85)


def on_truth(msg):
    with lock:
        truth["pose"] = msg.pose


def on_depth(msg):
    if st.get("stop"):
        return
    t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    if TOF_HZ > 0 and t - st["last_t"] < 1.0 / TOF_HZ - 0.02:
        return
    st["last_t"] = t
    depth = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
    if TOF_MODEL is not None:
        depth = TOF_MODEL.apply(depth)
    valid = float(np.mean(np.isfinite(depth) & (depth > 0)))
    if RAW_DIR:
        if not os.path.isdir(RAW_DIR):
            os.makedirs(RAW_DIR)
            json.dump({"topic": TOPIC, "width": msg.width, "height": msg.height, "fit": FIT or {"hfov": HFOV, "sensor_x": SENSOR_X},
                       "units": "m along the optical axis, inf = no return"}, open(os.path.join(RAW_DIR, "meta.json"), "w"), indent=1)
        raw_buf["t"].append(t); raw_buf["d"].append(depth.astype(np.float16))
        if len(raw_buf["t"]) >= 50:
            raw_flush()
    r, extra = fit(depth, **FIT)
    with lock:
        p = truth["pose"]
    if p is not None:
        yw = yaw_enu(p.orientation)
        face, d_t, a_t = truth_face(p.position.x, p.position.y, yw)
        tx, ty, tz, yn = p.position.x, p.position.y, p.position.z, (90.0 - math.degrees(yw)) % 360.0
    else:
        face, d_t, a_t, tx, ty, tz, yn = "", float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan")
    n = st["frame"]; st["frame"] += 1
    if r:
        sock.sendto(json.dumps({"t": t, "d": r["d"], "alpha": r["alpha"], "conf": r["conf"], "span": r["span"],
                                "n": r["n"], "rms": r["rms"], "valid": round(valid, 3)}).encode(), ("127.0.0.1", port))
    else:
        sock.sendto(json.dumps({"t": t, "d": None, "valid": round(valid, 3)}).encode(), ("127.0.0.1", port))
    w.writerow(["%.3f" % t, "%.3f" % time.time()] +
               (["%.4f" % r["d"], "%.3f" % r["alpha"], "%.3f" % r["conf"], "%.3f" % r["span"], r["n"], "%.4f" % r["rms"]] if r else ["", "", "", "", "", ""]) +
               ["%.4f" % tx, "%.4f" % ty, "%.4f" % tz, "%.2f" % yn, face, "%.4f" % d_t, "%.3f" % a_t, n, "%.3f" % valid])
    if vis_dir:
        save_vis(depth, r, extra, n)
    if time.time() - st["last_print"] >= 2.0:
        st["last_print"] = time.time(); f.flush()
        print("t=%.1f frame=%d meas=%s truth face=%s d=%.3f a=%.2f" % (
            t, n, ("d %.3f a %+.2f conf %.2f" % (r["d"], r["alpha"], r["conf"])) if r else "none", face, d_t, a_t), flush=True)


node = Node()
if not node.subscribe(Odometry, "/model/%s/odometry" % model, on_truth) or not node.subscribe(Image, TOPIC, on_depth):
    sys.exit("subscribe failed")
print("[wall_sense] ToF model %s" % (os.environ.get("WK_TOF_MODEL") or "ideal (gz)"), flush=True)
print("[wall_sense] model %s %s (%s Hz) -> udp %d, csv %s, vis %s" % (model, TOPIC, TOF_HZ or "all", port, out_csv, vis_dir or "-"), flush=True)
stop = out_csv + ".STOP"
while not os.path.exists(stop):
    time.sleep(0.5)
st["stop"] = True; time.sleep(0.3)
f.flush(); f.close()
raw_flush()
print("[wall_sense] stopped, frames %d" % st["frame"])
