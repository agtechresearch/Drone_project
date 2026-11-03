#!/usr/bin/env python3
"""Live mini-maps of the vehicle for the Gazebo GUI (run INSIDE starling-gz; GZ_PARTITION selects the instance).
  python3 live_map.py [model=starling2_0] [rate_hz=5] [room=R1]
Publishes two gz.msgs.Image (RGB8) topics, shown by ImageDisplay panels in gz/gui_record.config and gz/gui_bench.config:
  /map/face  rack-face elevation, the view of the vehicle's front camera (like plot_rack_path.py): the whole rack face the
             vehicle looks at (trays, gaps 1-6, rows), vehicle position along the aisle and height, the last 60 s of path.
             Facing E: north on the left; facing W: mirrored. While turning or crossing it keeps the last face.
  /map/plan  top-down plan of the room: rack lines, aisles A-E, rows 1-10, vehicle position, heading arrow, path.
             Rotated so north is on the LEFT and east UP, matching the face view of an east face.
Stop: kill the process (grid_fly.sh / bench.sh do it).
"""
import math, os, sys, time, threading
from gz.transport13 import Node
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.image_pb2 import Image
from PIL import Image as PImage, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import qvix_grid as G  # noqa: E402

MODEL = sys.argv[1] if len(sys.argv) > 1 else "starling2_0"
RATE = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
ROOM = sys.argv[3] if len(sys.argv) > 3 else "R1"
MX = G.MODEL_TO_WORLD
Y0, Y1 = G.ROOMS[ROOM]["y"]; RY0, RY1 = G.ROOMS[ROOM]["racks_y"]
FONT_PATH = os.path.expanduser("~/.local/share/fonts/NotoSansCJK-Regular.ttc")


def font(sz):
    for p, idx in ((FONT_PATH, 1), ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 0)):
        try:
            return ImageFont.truetype(p, sz, index=idx)
        except OSError:
            continue
    return ImageFont.load_default()


F11, F12, F14 = font(11), font(12), font(14)
BG, TRAY, GAP, POST, TXT, PATH, DRONE, GRID = (248, 249, 247), (70, 118, 82), (222, 232, 222), (45, 80, 55), (40, 44, 42), (40, 120, 220), (230, 100, 20), (200, 205, 200)
state = {"p": None, "yaw": 0.0, "trail": [], "face": "E", "aisle": None}
lock = threading.Lock()


def cb(msg):
    p = msg.pose.position; q = msg.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    with lock:
        state["p"] = (p.x, p.y, p.z); state["yaw"] = yaw


def aisle_of(wx):
    mx = wx - MX[0]
    for k, (a, b) in G.AISLES.items():
        if a <= mx <= b:
            return k
    return None


# ---------- face (elevation) map ----------
FW, FH = 480, 230
FL, FR, FT, FB = 34, 64, 24, 30
ZMAX = 4.3


def face_xy(model_y, h, face):
    f = (Y1 - model_y) / (Y1 - Y0) if face == "E" else (model_y - Y0) / (Y1 - Y0)
    return FL + f * (FW - FL - FR), FT + (1 - h / ZMAX) * (FH - FT - FB)


def face_bg(face):
    im = PImage.new("RGB", (FW, FH), BG); d = ImageDraw.Draw(im)
    x0, _ = face_xy(RY0, 0, face); x1, _ = face_xy(RY1, 0, face); xa, xb = min(x0, x1), max(x0, x1)
    for zl, zh in G.TRAY_Z:
        d.rectangle([xa, face_xy(0, zh, face)[1], xb, face_xy(0, zl, face)[1]], fill=TRAY)
    for i, (zl, zh) in enumerate(G.GAP_Z):
        d.rectangle([xa, face_xy(0, zh, face)[1], xb, face_xy(0, zl, face)[1]], fill=GAP)
        d.text((xb + 4, face_xy(0, G.GAP_CENTRE_Z[i], face)[1] - 7), "gap%d" % (i + 1), font=F11, fill=(60, 90, 65))
    for py in (RY0, RY1 - 0.05):
        a, _ = face_xy(py, 0, face); b, _ = face_xy(py + 0.05, 0, face)
        d.rectangle([min(a, b), face_xy(0, G.TRAY_Z[-1][1], face)[1], max(a, b), face_xy(0, 0, face)[1]], fill=POST)
    gy = face_xy(0, 0, face)[1]
    d.line([FL, gy, FW - FR, gy], fill=(140, 140, 135), width=3)
    for r, (ya, yb) in G.rows(ROOM):
        x, _ = face_xy(0.5 * (ya + yb), 0, face)
        d.text((x - 6, gy + 4), str(r), font=F11, fill=TXT)
    for h in (0, 1, 2, 3, 4):
        d.text((4, face_xy(0, h, face)[1] - 7), "%dm" % h, font=F11, fill=TXT)
    left, right = ("N", "S") if face == "E" else ("S", "N")
    d.text((FL, 4), "%s ←" % left, font=F12, fill=TXT); d.text((FW - FR - 24, 4), "→ %s" % right, font=F12, fill=TXT)
    return im


# ---------- plan map ----------
PW, PH = 480, 290
PL, PR, PT, PB = 22, 12, 22, 22                       # north LEFT, east UP


def plan_xy(mx, my):
    sx = (PW - PL - PR) / (Y1 - Y0); sy = (PH - PT - PB) / 5.8
    s = min(sx, sy)
    return PL + (Y1 - my) * s, PH - PB - mx * s


def plan_bg():
    im = PImage.new("RGB", (PW, PH), BG); d = ImageDraw.Draw(im)
    a = plan_xy(0.1, Y0); b = plan_xy(5.7, Y1)
    d.rectangle([min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])], outline=(120, 120, 115), width=2)
    for r, (ya, yb) in G.rows(ROOM):
        p = plan_xy(0.1, ya); q = plan_xy(5.7, ya)
        d.line([p, q], fill=GRID, width=1)
        c = plan_xy(0.12, 0.5 * (ya + yb)); d.text((c[0] - 4, c[1] + 2), str(r), font=F11, fill=TXT)
    for x0, x1 in G.RACKS_X:
        p = plan_xy(x0, RY0); q = plan_xy(x1, RY1)
        d.rectangle([min(p[0], q[0]), min(p[1], q[1]), max(p[0], q[0]), max(p[1], q[1])], fill=TRAY)
    for k, (x0, x1) in G.AISLES.items():
        c = plan_xy(0.5 * (x0 + x1), Y1 - 0.15); d.text((c[0] + 2, c[1] - 7), k, font=F12, fill=TXT)
    d.text((4, 3), "N ←   ↑ E", font=F12, fill=TXT)
    return im


def to_msg(im, frame):
    m = Image(); m.width, m.height = im.size; m.pixel_format_type = 3; m.step = im.size[0] * 3
    m.data = im.tobytes(); m.header.data.add(key="frame_id", value=[frame])
    return m


def main():
    node = Node()
    if not node.subscribe(Odometry, "/model/%s/odometry" % MODEL, cb):
        sys.exit("subscribe failed")
    pub_f = node.advertise("/map/face", Image); pub_p = node.advertise("/map/plan", Image)
    bgs = {"E": face_bg("E"), "W": face_bg("W")}; pbg = plan_bg()
    last_tr = 0.0
    while True:
        time.sleep(1.0 / RATE)
        with lock:
            p, yaw = state["p"], state["yaw"]
        if p is None:
            continue
        mx, my, h = p[0] - MX[0], p[1] - MX[1], p[2] - MX[2]
        now = time.time()
        if now - last_tr > 0.25:
            last_tr = now; state["trail"].append((mx, my, h)); state["trail"] = state["trail"][-240:]
        if abs(math.cos(yaw)) > 0.8:                     # gz yaw: E = 0, W = pi; keep the last face while N/S
            state["face"] = "E" if math.cos(yaw) > 0 else "W"
        ai = aisle_of(p[0])
        if ai:
            state["aisle"] = ai
        face = state["face"]
        # face map
        im = bgs[face].copy(); d = ImageDraw.Draw(im)
        pts = [face_xy(t[1], t[2], face) for t in state["trail"] if aisle_of(t[0] + MX[0]) == state["aisle"]]
        if len(pts) > 1:
            d.line(pts, fill=PATH, width=2)
        x, y = face_xy(my, h, face)
        d.line([x, y, x, face_xy(0, 0, face)[1]], fill=DRONE, width=1)
        d.ellipse([x - 6, y - 6, x + 6, y + 6], fill=DRONE, outline=(255, 255, 255), width=2)
        side = "동쪽" if face == "E" else "서쪽"
        d.text((FL + 50, 4), "%s통로 %s 면   높이 %.2f m" % (state["aisle"] or "?", side, h), font=F12, fill=TXT)
        pub_f.publish(to_msg(im, "face"))
        # plan map
        im = pbg.copy(); d = ImageDraw.Draw(im)
        pts = [plan_xy(t[0], t[1]) for t in state["trail"]]
        if len(pts) > 1:
            d.line(pts, fill=PATH, width=2)
        x, y = plan_xy(mx, my)
        # heading arrow: world yaw -> plan (east up, north left)
        hx, hy = -math.sin(yaw), -math.cos(yaw)
        d.line([x, y, x + 16 * hx, y + 16 * hy], fill=DRONE, width=3)
        d.ellipse([x - 6, y - 6, x + 6, y + 6], fill=DRONE, outline=(255, 255, 255), width=2)
        cell = G.locate(p[0], p[1]) or "-"
        d.text((PW - 150, 3), "%s   %.2f m" % (cell, h), font=F12, fill=TXT)
        pub_p.publish(to_msg(im, "plan"))


if __name__ == "__main__":
    main()
