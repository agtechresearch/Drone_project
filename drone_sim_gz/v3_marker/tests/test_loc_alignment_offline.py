#!/usr/bin/env python3
"""--wk-loc ToF update: time alignment and turned faces (2026-11-01, codex tof-time-alignment, survey-face-observation).
No flight, no simulator: the real loc2d / wk_mission code with noise-free synthetic poses and ToF frames.
  1. tof-time-alignment: a frame measured at tm, the state at ts = tm + lag (POSE_LAG / arrival). Constant approach speed
     (0.1 m/s toward the CW face) and a constant turn (8 deg/s): the new update (pose AT tm) has no systematic residual,
     whatever the lag; the old one (pose at ts for a frame from tm) shows speed x lag
  2. the queue in WallKeep.loc_step: frames wait for the pose time, are applied with the pose at their stamp, old frames are
     dropped and counted; a duplicate stamp is refused by the receiver
  3. survey-face-observation: turned face lines (0 / +3 / -3 deg, W and E): map_face distance = brute-force perpendicular
     distance, its y derivative = -+k / sqrt(1 + k^2) (numerical vs analytic), and the loc2d prediction follows a y-only move
Run in starling-flight (Python 3.6):  cd ~/workspace/v3_marker && python3 tests/test_loc_alignment_offline.py   exit = failures"""
import json, math, os, socket, sys, tempfile, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import loc2d  # noqa: E402
import wk_mission as W  # noqa: E402
import qvix_grid as G  # noqa: E402
F = []
XF = G.AISLES["C"][0]                     # CW face x (design)


def check(name, cond, info=""):
    print("%s  %s%s" % ("PASS" if cond else "FAIL", name, ("  (" + info + ")") if info else ""))
    if not cond:
        F.append(name)


def new_wk(corr=None):
    wk = W.WallKeep(True, 0, os.path.join(tempfile.mkdtemp(), "wk.csv"), (2.9075, 8.6))
    wk.tag_map = json.load(open(os.path.join(HERE, "..", "gz", "tags_map.json")))["tags"]; wk.face_corr = corr or {}
    return wk


def new_loc(wk, x, y, comp):
    L = loc2d.Loc2D(wk.tag_map, wk.map_face)
    L.init((y, x, comp), (x, y), comp)            # vio (n, e, yaw) = (map y, map x, compass): cY = 0
    L.P = np.diag([0.02 ** 2, 0.02 ** 2, 0.5 ** 2] + [0.01 ** 2, 0.01 ** 2] * loc2d.DRIFTV)
    return L


# ---------------------------------------------------------------- 1. time alignment
wk = new_wk(); V = 0.1; Y0 = 4.0
xt = lambda tau: 2.95 - V * tau                   # approaching the CW face (map -x), facing W (compass 270)
for lag in (0.05, 0.10, 0.23):
    new_nu, old_nu = [], []
    for tm in np.arange(0.0, 2.0, 0.1):
        ts = tm + lag
        m = {"t": tm, "d": xt(tm) - XF, "alpha": 0.0, "conf": 0.9, "span": 0.8}
        vt, vn = (Y0, xt(tm), 270.0), (Y0, xt(ts), 270.0)
        L = new_loc(wk, xt(ts), Y0, 270.0); r = L.update_tof(m, vt, vn, 0.0); new_nu.append(r["nu_d"])
        L = new_loc(wk, xt(ts), Y0, 270.0); r = L.update_tof(m, vn, vn, 0.0); old_nu.append(r["nu_d"])   # old: pose of ts
    check("approach 0.1 m/s, lag %.2f s: residual at the frame's time ~0" % lag, max(abs(v) for v in new_nu) < 1e-6,
          "new mean %+.6f max|.| %.6f m; old (pose of the state time) mean %+.5f m (speed x lag %.5f)" % (
              np.mean(new_nu), max(abs(v) for v in new_nu), np.mean(old_nu), V * lag))
RATE = 8.0
for lag in (0.05, 0.23):
    new_a, old_a = [], []
    for tm in np.arange(0.0, 1.5, 0.1):
        ts = tm + lag; cm, cs = 266.0 + RATE * tm, 266.0 + RATE * ts
        m = {"t": tm, "d": 0.40, "alpha": 270.0 - cm, "conf": 0.9, "span": 0.8}
        L = new_loc(wk, XF + 0.40, Y0, cs); r = L.update_tof(m, (Y0, XF + 0.40, cm), (Y0, XF + 0.40, cs), RATE); new_a.append(r["nu_a"])
        L = new_loc(wk, XF + 0.40, Y0, cs); r = L.update_tof(m, (Y0, XF + 0.40, cs), (Y0, XF + 0.40, cs), RATE); old_a.append(r["nu_a"])
    check("turn %.0f deg/s, lag %.2f s: heading residual at the frame's time ~0" % (RATE, lag), max(abs(v) for v in new_a) < 1e-3,
          "new mean %+.5f deg; old mean %+.4f deg (rate x lag %.3f)" % (np.mean(new_a), np.mean(old_a), RATE * lag))

# ---------------------------------------------------------------- 2. the queue in loc_step


class Ctl(object):
    current_target_yaw = 270.0; active_stage = {"name": "s01"}

    def feedback_state(self):
        return {"cmd_n": 0.0, "cmd_e": 0.0, "cmd_d": -0.7, "rel_n": 0.0, "rel_e": 0.0, "rel_u": 0.7, "yaw_deg": 270.0}


wk = new_wk(); wk.tags = True; wk.ready = False; wk.origin = (0.0, 0.0)
wk.loc_f = open(os.path.join(tempfile.mkdtemp(), "loc.jsonl"), "w")
T = 10.0; xq = lambda t: xt(t - (T - 1.0)); hist_t = np.arange(T - 1.0, T + 1e-9, 0.05)
wk.hist = [(t, Y0, xq(t), math.radians(270.0)) for t in hist_t]
L = wk.loc = new_loc(wk, xq(hist_t[-2]), Y0, 270.0); L.vio_prev = (Y0, xq(hist_t[-2]), 270.0); wk.loc_t = hist_t[-2]
frames = [T - 0.9, T - 0.30, T - 0.12, T + 0.05, T + 0.15]               # one too old, two due, two newer than the pose time
wk.tof_q = [{"t": t, "d": xq(t) - XF, "alpha": 0.0, "conf": 0.9, "span": 0.8} for t in frames]
wk.loc_step(Ctl(), Ctl().feedback_state())
check("queue: frames newer than the pose time wait", [q["t"] for q in wk.tof_q] == frames[3:], str([q["t"] for q in wk.tof_q]))
check("queue: a frame %.1f s behind the pose time is dropped and counted" % 0.9, wk.n_tof_old == 1, "n_tof_old %d" % wk.n_tof_old)
check("queue: the due frames were applied at their own time (residual ~0)", L.n_tof_d == 2 and abs(L.last_tof["nu_d"]) < 2e-4
      and abs(L.last_tof["t"] - (T - 0.12)) < 1e-9, "n_tof_d %d, last %s" % (L.n_tof_d, L.last_tof))
wk2 = new_wk(); wk2.clock = lambda: 10.0; port = wk2.sock.getsockname()[1]; tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for t in (9.5, 9.5, 9.6):
    tx.sendto(json.dumps({"t": t, "d": 0.4, "alpha": 0.0, "conf": 0.9, "span": 0.8, "valid": 0.9}).encode(), ("127.0.0.1", port)); time.sleep(0.1)
check("receiver: a duplicate stamp is refused (counted), the queue holds 9.5 and 9.6 once",
      wk2.n_bad_tof == 1 and [q["t"] for q in wk2.tof_q] == [9.5, 9.6], "bad %d, queue %s" % (wk2.n_bad_tof, [q["t"] for q in wk2.tof_q]))


# ---------------------------------------------------------------- 3. turned faces
def brute(mx, my, x0, k, yr, sgn):
    ys = np.linspace(my - 2.0, my + 2.0, 400001); xs = x0 + k * (ys - yr)
    i = np.argmin((xs - mx) ** 2 + (ys - my) ** 2)
    return sgn * math.copysign(math.hypot(xs[i] - mx, ys[i] - my), mx - (x0 + k * (my - yr)))


for face, head, sgn, xd in (("CW", 270.0, 1.0, G.AISLES["C"][0]), ("CE", 90.0, -1.0, G.AISLES["C"][1])):
    for turn in (0.0, 3.0, -3.0):
        corr = {face: {"offset": 0.012, "turn_deg": turn, "y_ref": 4.75}}
        wk = new_wk(corr); k = math.tan(math.radians(turn)); x0 = xd + 0.012
        mx, my = 0.5 * sum(G.AISLES["C"]), 6.3
        mf = wk.map_face(mx, my, head)
        g = brute(mx, my, x0, k, 4.75, sgn)
        dy = 1e-4; num = (wk.map_face(mx, my + dy, head)[1] - wk.map_face(mx, my - dy, head)[1]) / (2 * dy)
        ana = -sgn * k / math.sqrt(1 + k * k)
        check("%s turn %+.0f deg: map_face d = brute-force perpendicular distance, d/dy numerical = analytic" % (face, turn),
              abs(mf[1] - g) < 2e-5 and abs(num - ana) < 1e-6 and abs(mf[3] - (head + turn)) < 1e-9,
              "d %.6f vs %.6f m, d/dy %.6f vs %.6f" % (mf[1], g, num, ana))
        # loc2d: measurement = the true distance at y; the state moved 0.10 m along y only -> predicted distance changes by
        # -(d/dy) 0.10 (0 for a face parallel to y; the old fixed x_face gave 0 for every face)
        L = new_loc(wk, mx, my + 0.10, head); m = {"t": 0.0, "d": g, "alpha": 0.0, "conf": 0.9, "span": 0.8}
        v = (my + 0.10, mx, head); r = L.update_tof(m, v, v, 0.0)
        check("%s turn %+.0f deg: y-only move of 0.10 m -> residual %+.5f m (expected %+.5f)" % (face, turn, r["nu_d"], -ana * 0.10),
              abs(r["nu_d"] - (-ana * 0.10)) < 2e-5)
print("%d failure(s)" % len(F)); sys.exit(len(F))
