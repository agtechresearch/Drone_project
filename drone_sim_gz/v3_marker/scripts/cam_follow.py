#!/usr/bin/env python3
"""Overview ("god view") camera for the Gazebo GUI + visibility markers (run INSIDE starling-gz, instance 0 GUI).
  python3 cam_follow.py [view=oblique] [rate_hz=20] [max_seconds=3600]
Views (CAM_FOLLOW_VIEW or argv[1]):
  aisle (default)   camera in the same aisle, AISLE_BACK m (3.5) south of the vehicle (AISLE_DIR -1; +1 = north) at a fixed
                    height AISLE_Z (3.0 m), always looking the same way down the aisle: the line of sight stays inside the
                    open aisle, so racks never hide the vehicle; climbs/descents move the vehicle up/down in the frame.
                    The camera is not clamped along the aisle: near the south end it stands outside the room and looks
                    through the end wall, so grid_fly.sh makes the module semi-transparent for this view too.
  oblique            camera on the room-centre side of the vehicle, OBL_SIDE m across the aisle and OBL_BACK m south of it,
                    at a FIXED height (CAM_Z, world m) instead of following the vehicle, so climbs/descents show up as the
                    vehicle moving up/down in the frame against the semi-transparent rack trays; it trails the vehicle
                    with a slow low-pass (TAU_POS s), so along-aisle motion is seen against the background.
  side              like oblique but straight across the aisle (no back offset).
  chase             old behaviour: fixed world offset x,y,z (CAM_FOLLOW_OFFSET, default 0,-2.5,1.5) that follows the vehicle.
Roles (CAM_FOLLOW_ROLE): camera | markers | both. Marker service calls block (~0.3 s), so grid_fly.sh runs the camera
steering and the markers as two processes. The orange halo is a visual of the model (gui_halo, hidden from the front camera).
Markers (CAM_FOLLOW_MARKERS=0 to disable): optional marker halo (HALO diameter m, default 0 = off),
orange height pole floor->vehicle, floor ring under the vehicle, and a trail of the last TRAIL_S seconds (line strip). Height = above the QVIX floor (top z 0.15).
Stop: touch /tmp/cam_follow.STOP.  Env CAM_FOLLOW_BOX="xmin,xmax,ymin,ymax,zmin,zmax" clamps the camera inside the room.
History: v1 (2026-10-28 morning) followed at a body/world offset of 1.5-2.5 m: too close (every vehicle move dragged the
camera), ticking at 5 Hz, viewing direction changing on turns, racks hiding the vehicle. v2 = this file.
v3 (same day): camera motion by the GUI CameraTracking plugin (/gui/track FOLLOW_LOOK_AT, world-axis offset, per-frame
smoothing) steered at 10 Hz; markers in one /marker_array call. Earlier "pgain ignored, lags 2-4 m" was the 6 fps llvmpipe GUI.
"""
import math, os, sys, time, threading
from gz.transport13 import Node
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.gui_camera_pb2 import GUICamera
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.marker_v_pb2 import Marker_V
from gz.msgs10.cameratrack_pb2 import CameraTrack

view = (sys.argv[1] if len(sys.argv) > 1 else os.environ.get("CAM_FOLLOW_VIEW", "aisle")).lower()
rate = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
max_s = float(sys.argv[3]) if len(sys.argv) > 3 else 3600.0
E = os.environ.get
model = E("CAM_FOLLOW_MODEL", "starling2_0")
box = [float(v) for v in E("CAM_FOLLOW_BOX").split(",")] if E("CAM_FOLLOW_BOX") else None
off = [float(v) for v in E("CAM_FOLLOW_OFFSET", "0,-2.5,1.5").split(",")]
OBL_SIDE = float(E("OBL_SIDE", "2.0")); OBL_BACK = float(E("OBL_BACK", "1.6")) if view == "oblique" else 0.0
CAM_Z = float(E("CAM_Z", "2.6"))             # world z of the camera in oblique/side views (floor top 0.15)
CENTRE_X = float(E("CENTRE_X", "3.4"))
AISLE_BACK = float(E("AISLE_BACK", "3.5")); AISLE_Z = float(E("AISLE_Z", "3.0")); AISLE_DIR = float(E("AISLE_DIR", "-1"))   # aisle view       # world x of the room centre line (R1/R2: aisle C)
TAU_POS = float(E("TAU_POS", "0.8")); TAU_LOOK = float(E("TAU_LOOK", "1.0"))
MARKERS = E("CAM_FOLLOW_MARKERS", "1") != "0"; HALO = float(E("HALO", "0"));   # 0: the halo is now a visual of the model (gui_halo)
ROLE = E("CAM_FOLLOW_ROLE", "both")   # camera | markers | both (grid_fly.sh runs camera and markers as two processes)
TRAIL_S = float(E("TRAIL_S", "30")); FLOOR_Z = 0.15
STOP = "/tmp/cam_follow.STOP"   # one STOP file stops both roles (grid_fly.sh touches it after the flight)
if os.path.exists(STOP) and time.time() - os.path.getmtime(STOP) > 5:
    os.remove(STOP)                  # stale; a fresh one (< 5 s) may belong to the other role stopping
state = {"p": None, "n": 0}; lock = threading.Lock()

def cb(msg):
    with lock:
        p = msg.pose.position; state["p"] = (p.x, p.y, p.z); state["n"] += 1

def look_at_quat(cam, tgt):
    fx, fy, fz = tgt[0] - cam[0], tgt[1] - cam[1], tgt[2] - cam[2]
    n = math.sqrt(fx * fx + fy * fy + fz * fz) or 1.0
    fx, fy, fz = fx / n, fy / n, fz / n
    lx, ly = -fy, fx; m = math.sqrt(lx * lx + ly * ly) or 1.0; lx, ly, lz = lx / m, ly / m, 0.0
    ux, uy, uz = fy * lz - fz * ly, fz * lx - fx * lz, fx * ly - fy * lx
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = fx, lx, ux, fy, ly, uy, fz, lz, uz
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2; return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2; return (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2; return ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2; return ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)

def colour(mat, r, g, b, a=1.0):
    for c in (mat.ambient, mat.diffuse, mat.emissive):
        c.r, c.g, c.b, c.a = r, g, b, a

node = Node()
# Single thread only (gz python bindings crash with requests from several threads: "scoped_acquire::dec_ref"). All markers
# of one update go in ONE /marker_array call (it answers at once; the plain /marker service never answers and every call
# waited for its timeout). With the GUI on the GPU, /gui/move_to/pose also answers at once (on llvmpipe the GUI drew ~6 fps).
batch = []

def marker(mid, mtype, fill):
    m = Marker(); m.ns = "cam_follow"; m.id = mid; m.action = Marker.ADD_MODIFY; m.type = mtype; m.visibility = Marker.GUI
    fill(m); batch.append(m)

def flush_markers():
    if not batch:
        return
    v = Marker_V(); v.marker.extend(batch); del batch[:]
    node.request("/marker_array", v, Marker_V, Boolean, 300)
stats = {"sent": 0, "failed": 0, "skipped": 0}
track_pub = node.advertise("/gui/track", CameraTrack)
PGAIN = float(E("FOLLOW_PGAIN", "0.03")); TRACK_PGAIN = float(E("TRACK_PGAIN", "0.06"))   # per GUI frame (~20 fps)

def clear_markers():
    v = Marker_V(); m = v.marker.add(); m.ns = "cam_follow"; m.action = Marker.DELETE_ALL; node.request("/marker_array", v, Marker_V, Boolean, 300)

def draw(p, trail):
    h = p[2] - FLOOR_Z
    def pole(m):
        m.pose.position.x, m.pose.position.y, m.pose.position.z = p[0], p[1], FLOOR_Z + 0.5 * max(h - 0.06, 0.01)
        m.scale.x = m.scale.y = 0.012; m.scale.z = max(h - 0.06, 0.01); colour(m.material, 1.0, 0.45, 0.0)
    def ring(m):
        m.pose.position.x, m.pose.position.y, m.pose.position.z = p[0], p[1], FLOOR_Z + 0.005
        m.scale.x = m.scale.y = 0.30; m.scale.z = 0.004; colour(m.material, 1.0, 0.45, 0.0, 0.8)
    def label(m):
        m.pose.position.x, m.pose.position.y, m.pose.position.z = p[0], p[1], p[2] + 0.22
        m.scale.x = m.scale.y = m.scale.z = 0.16; m.text = "h %.2f m" % h; colour(m.material, 1.0, 1.0, 0.2)
    def halo(m):
        m.pose.position.x, m.pose.position.y, m.pose.position.z = p
        m.scale.x = m.scale.y = m.scale.z = HALO; colour(m.material, 1.0, 0.5, 0.0, 0.30)
    marker(1, Marker.CYLINDER, pole); marker(2, Marker.CYLINDER, ring)      # TEXT markers are not supported by ogre2 here
    if HALO > 0:
        marker(5, Marker.SPHERE, halo)
    if len(trail) >= 2:
        def tr(m):
            colour(m.material, 0.1, 0.6, 1.0)
            for q in trail:
                pt = m.point.add(); pt.x, pt.y, pt.z = q
        marker(4, Marker.LINE_STRIP, tr)

if not node.subscribe(Odometry, "/model/%s/odometry" % model, cb):
    sys.exit("subscribe failed")
if MARKERS and ROLE in ("markers", "both"):
    clear_markers()
t0 = time.time(); sent = fails = 0; filt = {"cam": None, "look": None}; trail = []; last_trail = 0.0; dt = 1.0 / rate
side_sign = None; aisle_sign = None
nxt = time.time()
while time.time() - t0 < max_s and not os.path.exists(STOP):
    nxt += dt; time.sleep(max(0.0, nxt - time.time()))           # fixed rate (request time included)
    with lock:
        p = state["p"]
    if p is None:
        continue
    if view == "chase":
        cam = [p[0] + off[0], p[1] + off[1], p[2] + off[2]]
    elif view == "aisle":
        # behind the vehicle ALONG the aisle, high up: the line of sight stays inside the open aisle, so racks never
        # hide the vehicle (no transparency needed); fixed camera height -> climbs show as motion in the frame.
        # ONE fixed viewing direction (AISLE_DIR -1 = camera south of the vehicle looking north, +1 = north looking south);
        # near the room end the room box clamp keeps the camera at the wall (the view then gets steeper, never flips).
        # Any camera point inside the aisle box keeps the line of sight inside the (convex) open aisle -> never occluded.
        aisle_sign = AISLE_DIR
        # turned world (qvix_rot, QVIX_YAW_DEG): the aisle runs along the module's +y, rotated CCW by the yaw (2026-10-30)
        yw = math.radians(float(os.environ.get("QVIX_YAW_DEG", "0")))
        cam = [p[0] - math.sin(yw) * aisle_sign * AISLE_BACK, p[1] + math.cos(yw) * aisle_sign * AISLE_BACK, AISLE_Z]
    else:
        want = 1.0 if p[0] < CENTRE_X else -1.0                        # look from the room-centre side (more room)
        if side_sign is None or abs(p[0] - CENTRE_X) > 0.3:             # hysteresis: aisle C (on the centre line) keeps its side
            side_sign = want
        cam = [p[0] + side_sign * OBL_SIDE, p[1] - OBL_BACK, CAM_Z]
    if box:
        # aisle view: no clamp ALONG the aisle (y) - the camera keeps its distance and looks through the (transparent)
        # end wall instead of ending up straight above the vehicle (run22: degenerate straight-down, rotated frames)
        cam = [cam[i] if (view == "aisle" and i == 1) else min(max(cam[i], box[2 * i]), box[2 * i + 1]) for i in range(3)]
    if filt["cam"] is None:
        filt["cam"], filt["look"] = list(cam), list(p)
    else:
        a, b = dt / (TAU_POS + dt), dt / (TAU_LOOK + dt)
        filt["cam"] = [filt["cam"][i] + a * (cam[i] - filt["cam"][i]) for i in range(3)]
        filt["look"] = [filt["look"][i] + b * (p[i] - filt["look"][i]) for i in range(3)]
    # The GUI's own camera tracking does the per-frame motion (FOLLOW_LOOK_AT: camera at target + follow_offset in WORLD
    # axes, always looking at the target, smoothed by follow_pgain every render frame). We only steer the offset over the
    # non-blocking /gui/track topic. (/gui/move_to/pose answers only after its ~0.3-0.5 s move animation -> 2-3 updates/s,
    # the "ticking" camera of v1/v2.) The offset is computed from the low-passed camera point, so it changes smoothly.
    # offset from SMOOTHED points only: computing it from the raw position (v3 first try) fed the vehicle's sway straight
    # into the camera (user feedback 2026-10-28: "the camera must keep distance/framing but not shake with the drone").
    o = [filt["cam"][i] - filt["look"][i] for i in range(3)]
    msg = CameraTrack(); msg.track_mode = CameraTrack.FOLLOW_LOOK_AT
    msg.follow_target.name = model; msg.track_target.name = model
    msg.follow_offset.x, msg.follow_offset.y, msg.follow_offset.z = o
    msg.follow_pgain = PGAIN; msg.track_pgain = TRACK_PGAIN
    if ROLE in ("camera", "both"):
        track_pub.publish(msg); stats["sent"] += 1
    if MARKERS and ROLE in ("markers", "both"):
        now = time.time()
        if now - last_trail >= 0.25:
            last_trail = now; trail.append(p); trail = trail[-int(TRAIL_S / 0.25):]
        draw(p, trail); flush_markers()
el = time.time() - t0
if ROLE in ("camera", "both"):
    stop = CameraTrack(); stop.track_mode = CameraTrack.NONE; track_pub.publish(stop)
print("cam_follow: view %s, %.0f s, track updates %d (%.1f/s), odometry msgs %d" % (
    view, el, stats["sent"], stats["sent"] / max(el, 1e-3), state["n"]))
