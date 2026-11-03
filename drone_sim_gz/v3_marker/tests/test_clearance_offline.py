#!/usr/bin/env python3
"""Offline checks of loc_compare.clearance (2026-10-31, codex collision-coverage). Synthetic truth, no flight.
  QVIX_YAW_DEG=0 python3 tests/test_clearance_offline.py        exit code = number of failures"""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
os.environ.setdefault("QVIX_YAW_DEG", "0")
import loc_compare as LC  # noqa: E402
import qvix_grid as G  # noqa: E402

F = []


def run(xs, ys, comps, ts=None, shift=None):
    LC.SHIFT = shift
    n = len(xs); t = np.arange(n) * 0.02 if ts is None else np.array(ts, float)
    return LC.clearance(t, np.array(xs, float), np.array(ys, float), None, np.array(comps, float), (t[0], t[-1]), require_end=False)


def check(name, got, expect, info):
    ok = got == expect
    print("%s  %-55s -> %s  %s" % ("PASS" if ok else "FAIL", name, got, info))
    if not ok:
        F.append(name)


xc = 0.5 * sum(G.AISLES["C"]); cw = G.AISLES["C"][0]
v, w, n, bad, gap, inf = run([xc] * 50, [4.0] * 50, [270.0] * 50)
check("aisle centre facing W", v, "PASS", "min %.4f m" % w[0])
v, w, n, bad, gap, inf = run([cw - 0.01] * 50, [4.0] * 50, [270.0] * 50)
check("centre 1 cm inside the rack (was skipped before)", v, "FAIL", "min %.4f m, %s" % (w[0], w[2]))
v, w, n, bad, gap, inf = run([2.2] * 50, [4.0] * 50, [270.0] * 50)
check("centre in the middle of a rack line (outside aisles)", v, "FAIL", "min %.4f m" % w[0])
v, w, n, bad, gap, inf = run([cw + 0.10] * 50, [8.95] * 50, [270.0] * 50)
check("centre just past the rack end, rotors over the corner", v, "FAIL", "min %.4f m, %s" % (w[0], w[2]))
xb = 0.5 * sum(G.AISLES["B"])                                      # aisle C lines up with the door: no wall there
v, w, n, bad, gap, inf = run([xb] * 50, [10.0] * 50, [0.0] * 50)
check("near the north wall in aisle B (rotor reaches the wall)", v, "FAIL", "min %.4f m, %s" % (w[0], w[2]))
v, w, n, bad, gap, inf = run([xc] * 50, [10.0] * 50, [0.0] * 50)
check("same y in aisle C = in the door opening (no wall)", v, "PASS", "min %.4f m, %s" % (w[0], w[2]))
# moved rack line 1 (+3 cm toward aisle C, 0.5 deg): a pose with 2 cm clearance to the design face touches the moved one
shift = {"line": 1, "dx": 0.03, "dyaw_deg": 0.0, "centre": [0.5 * sum(G.RACKS_X[1]), 4.75]}
x_near = cw + 0.088 + 0.06 + 0.02                                   # rotor tip 2 cm from the design CW face (facing N)
v, w, n, bad, gap, inf = run([x_near] * 50, [4.0] * 50, [0.0] * 50)
check("2 cm from the design face", v, "PASS", "min %.4f m" % w[0])
v, w, n, bad, gap, inf = run([x_near] * 50, [4.0] * 50, [0.0] * 50, shift=shift)
check("same pose, rack moved 3 cm toward the aisle", v, "FAIL", "min %.4f m, %s" % (w[0], w[2]))
v, w, n, bad, gap, inf = run([xc] * 3, [4.0] * 3, [270.0] * 3, ts=[0.0, 0.02, 1.2])
check("truth gap 1.18 s in the air", v, "INCOMPLETE", "max gap %.2f s" % gap)
v, w, n, bad, gap, inf = run([xc, float("nan"), xc], [4.0] * 3, [270.0] * 3)
check("non-finite truth sample", v, "INCOMPLETE", "non-finite %d" % bad)
# ---- 2026-10-31 codex collision-coverage / ground-truth-timestamps reply-2
LC.SHIFT = None; v, w, n, bad, gap, inf = LC.clearance(1.0 + np.arange(50) * 0.02, np.array([xc] * 50), np.array([4.0] * 50), None, np.array([270.0] * 50), (0.0, 1.98), require_end=False)
check("truth starts 1.0 s after the interval start", v, "INCOMPLETE", "max gap %.2f s" % gap)
v, w, n, bad, gap, inf = LC.clearance(np.arange(50) * 0.02, np.array([xc] * 50), np.array([4.0] * 50), None, np.array([270.0] * 50), (0.0, 2.0), require_end=False)
check("truth ends 1.02 s before the interval end", v, "INCOMPLETE", "max gap %.2f s" % gap)
v, w, n, bad, gap, inf = LC.clearance(np.zeros(0), np.zeros(0), np.zeros(0), None, np.zeros(0), (0.0, 2.0), require_end=False)
check("no truth sample at all (min none, no exception)", (v, w[0]), ("INCOMPLETE", None), "n %d" % n)
check("no truth and no stages -> no interval", LC.interval_air(np.zeros(0), None, {}), None, "")
z = np.array([0.5] * 50); z[10] = np.nan
v, w, n, bad, gap, inf = LC.clearance(np.arange(50) * 0.02, np.array([xc] * 50), np.array([4.0] * 50), z, np.array([270.0] * 50), (0.0, 0.98), require_end=False)
check("NaN height: still evaluated 2-D (all 50) but the verdict is INCOMPLETE", (v, n, inf["nonfinite_height"]), ("INCOMPLETE", 50, 1), str(inf))
# file loader: a row with x 'nan' stays (non-finite sample), a row with time 'nan' is dropped and counted
import tempfile  # noqa: E402
d = tempfile.mkdtemp(); f = open(os.path.join(d, "gt.csv"), "w"); f.write("t_sim,t_wall,x,y,z,qw,qx,qy,qz,yaw\n")
wx, wy, _ = G.model_to_world(xc, 4.0)
for i in range(50):
    t = "nan" if i == 20 else "%.3f" % (i * 0.02)
    x = "nan" if i == 30 else "%.5f" % (wx if wx is not None else 0.0)
    f.write("%s,0,%s,%.5f,0.5,1,0,0,0,0\n" % (t, x, wy if wy is not None else 0.0))
f.close()
src, tt, TX, TY, TZ, TC, dropped = LC.load_truth(d)
check("loader: time-nan row dropped and counted, x-nan row kept", (len(tt), dropped["time not finite"], int(np.isnan(TX).sum())), (49, 1, 1), src)
v, w, n, bad, gap, inf = LC.clearance(tt, TX, TY, TZ, TC, (0.0, 0.98), dropped["time not finite"], require_end=False)
check("loader: the kept x-nan row makes the verdict INCOMPLETE", (v, bad), ("INCOMPLETE", 1), "")
# door jamb end: hub inside the door width, rotor disc over the end of the west jamb
d0 = G.PARTITION_DOOR_X[0]; y1 = G.ROOMS["R1"]["y"][1]
v, w, n, bad, gap, inf = run([d0 + 0.13] * 50, [y1 - 0.05] * 50, [0.0] * 50)
check("rotor over the door jamb's end (hub inside the door width)", v, "FAIL", "min %.4f m, %s" % (w[0], w[2]))
# a turned rack line (2 deg about its centre): the far end moves ~0.14 m sideways
ry = G.RACK_UNITS_Y["R1"][-1]; yend = ry[1] - 0.1
mins = []
for dyaw in (2.0, -2.0):
    sh = {"line": 1, "dx": 0.0, "dyaw_deg": dyaw, "centre": [0.5 * sum(G.RACKS_X[1]), 4.75]}
    v, w, n, bad, gap, inf = run([cw + 0.088 + 0.06 + 0.05] * 50, [yend] * 50, [0.0] * 50, shift=sh); mins.append((v, w[0]))
check("rack line turned +-2 deg: one side touches, the clearances differ", ("FAIL" in (mins[0][0], mins[1][0]), abs(mins[0][1] - mins[1][1]) > 0.1), (True, True), str(mins))
LC.SHIFT = None
# ---- 2026-11-01 codex reply-4: end of the flight, missing heights / times, yaw NaN
tt = np.arange(300) * 0.02; X = np.array([xc] * 300); Y = np.array([4.0] * 300); C = np.array([270.0] * 300)
Zg = np.concatenate((np.zeros(20), np.full(260, 0.5), np.zeros(20)))          # take-off, flight, touchdown
span = {"s01_x": [1.0, 5.0]}                                                  # last stage ends in the air (before the landing)
iv = LC.interval_air(tt, Zg, span)
v, w, n, bad, gap, inf = LC.clearance(tt, X, Y, Zg, C, iv)
check("normal ground tail: interval reaches the touchdown sample", (v, inf["end"], round(iv[1], 2)), ("PASS", "on the ground", 5.6), str(iv))
cut = slice(0, 270)                                                           # truth cut during the landing, still in the air
iv = LC.interval_air(tt[cut], Zg[cut], span)
v, w, n, bad, gap, inf = LC.clearance(tt[cut], X[cut], Y[cut], Zg[cut], C[cut], iv)
check("truth cut in the air during the landing (after the last stage)", (v, inf["end"]), ("INCOMPLETE", "truth ends in the air"), str(iv))
iv = LC.interval_air(tt, Zg, {})
v, w, n, bad, gap, inf = LC.clearance(tt, X, Y, Zg, C, iv)
check("no stages at all (landing only from the truth heights)", (v, round(iv[0], 2), round(iv[1], 2)), ("PASS", 0.4, 5.6), str(iv))
v, w, n, bad, gap, inf = LC.clearance(tt, X, Y, None, C, (0.0, 5.98))
check("tags.csv-like truth (no height): end cannot be checked", (v, inf["end"]), ("INCOMPLETE", "unverified (no height)"), "-")
v, w, n, bad, gap, inf = LC.clearance(tt, X, Y, Zg, C, LC.interval_air(tt, Zg, span), dropped_t=2)
check("rows without a finite time -> INCOMPLETE", (v, inf["dropped_time"]), ("INCOMPLETE", 2), "-")
u = LC.unwrap_finite([170.0, 179.0, float("nan"), -179.0, -170.0])
check("unwrap_finite: NaN stays NaN, the +-180 crossing stays continuous",
      (bool(np.isnan(u[2])), [round(x, 1) for x in u[[0, 1, 3, 4]]]), (True, [170.0, 179.0, 181.0, 190.0]), str(u))
g = LC.truth_at(np.array([0.0, 0.1, 0.2]), u[[1, 2, 3]], np.array([0.05, 0.15]))
check("heading truth next to a NaN yaw is excluded (not 0 deg)", bool(np.isnan(g).all()), True, str(g))
# ---- 2026-11-01 codex reply-6: +-inf heights, missing / empty truth files
for zv, name in ((-np.inf, "-inf"), (np.inf, "+inf")):
    Zi = Zg.copy(); Zi[100] = zv
    v, w, n, bad, gap, inf = LC.clearance(tt, X, Y, Zi, C, LC.interval_air(tt, Zi, span))
    check("height %s in the air: evaluated 2-D and counted -> INCOMPLETE" % name, (v, inf["nonfinite_height"], n), ("INCOMPLETE", 1, 260), str(inf))
for files, expect in (({}, "none (gt.csv missing, tags.csv missing)"), ({"gt.csv": "t_sim,t_wall,x,y,z,qw,qx,qy,qz,yaw\n"}, "none (gt.csv empty, tags.csv missing)"),
                      ({"gt.csv": "t_sim,t_wall,x,y,z,qw,qx,qy,qz,yaw\n", "tags.csv": "t_sim,true_x,true_y,true_yaw\n1.0,0,0,0\n"},
                       "tags.csv (image stamps, pose = latest odometry) (gt.csv empty)")):
    d = tempfile.mkdtemp()
    for k, v in files.items():
        open(os.path.join(d, k), "w").write(v)
    src, tt2, TX2, TY2, TZ2, TC2, dr = LC.load_truth(d)
    check("loader with %s" % (", ".join(files) or "no truth file"), src, expect, "rows %d" % len(tt2))
print("%d failure(s)" % len(F)); sys.exit(len(F))
