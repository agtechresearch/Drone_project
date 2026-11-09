#!/usr/bin/env python3
"""Offline tests for the survey fixes of 2026-11-01 (codex survey-uncertainty, survey-anchor-lines, survey-height-syntax,
survey-replay-consistency). Synthetic only (no flight, no simulator):
  1 tag covariance J C J^T vs parameter sampling             2 fixed turn R C R^T / fixed shift (sampling)
  3 face line moved as a line (two points on the new line)    4 line covariance vs sampling
  5 ToF points: shared function = the old solver code, mount rotation used (the old exporter ignored it)
  6 points stacked then turned = points stacked on the turned path (compass heading - CCW turn): anchor / replay agree
  7 survey route text: height lists, segments, bare start, errors (+ grid_mission.survey_route when it uses survey_spec)
usage: python3 tests/test_survey_geom.py [path of a grid_mission.py to test, default scripts/grid_mission.py]"""
import importlib.util, math, os, re, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); SC = os.path.join(os.path.dirname(HERE), "scripts")
sys.path.insert(0, SC)
import survey_geom as SG  # noqa: E402
import survey_spec as SS  # noqa: E402

FAIL = 0


def check(name, ok, info=""):
    global FAIL
    print("%s  %s  %s" % ("PASS" if ok else "FAIL", name, info))
    FAIL += 0 if ok else 1


rng = np.random.default_rng(7)

# 1 ------------------------------------------------------------------------------------------------------------------
ymid = 4.75
for case, (mean, sd, corr) in {
        "sv5-like (y far from y_ref, slope sd 0.6 deg)": ([0.01, 0.002, 0.03, 0.63, 1.2], [0.03, math.radians(0.6), 0.005, 0.013, 0.018], 0.0),
        "correlated b-phi-s-y": ([0.0, -0.004, 0.02, 7.9, 0.7], [0.02, math.radians(0.4), 0.004, 0.01, 0.01], 0.5)}.items():
    sd = np.array(sd); Cr = np.full((5, 5), corr); np.fill_diagonal(Cr, 1.0); Cr[4, :4] = Cr[:4, 4] = 0.0
    C5 = Cr * np.outer(sd, sd)
    P = rng.multivariate_normal(mean, C5, 400000)
    X = np.stack([P[:, 0] + P[:, 1] * (P[:, 3] - ymid) + 1.0 * P[:, 2], P[:, 3], P[:, 4]], 1)
    Cmc = np.cov(X.T); Cj = SG.tag_cov(C5, mean[3], ymid, 1.0, mean[1])
    rel = np.abs(Cj - Cmc).max() / np.abs(Cmc).max()
    old_sd_x = math.sqrt(C5[0, 0] + C5[2, 2])
    check("1 tag cov J C J^T vs sampling, %s" % case, rel < 0.02,
          "max |diff| / max |cov| %.4f; sd x: J %.5f, sampled %.5f, old formula sqrt(var b + var s) %.5f" % (
              rel, math.sqrt(Cj[0, 0]), math.sqrt(Cmc[0, 0]), old_sd_x))

# 2 ------------------------------------------------------------------------------------------------------------------
C3 = np.array([[4e-4, 1e-4, 0], [1e-4, 1e-4, 0], [0, 0, 2e-4]])
for a in (math.radians(30), math.radians(-0.0272), math.radians(-90)):
    Pm = rng.multivariate_normal([1.0, 2.0, 0.5], C3, 300000)
    f = SG.turn_about(2.9, 5.1, a); qx, qy = f(Pm[:, 0], Pm[:, 1])
    Cmc = np.cov(np.stack([qx, qy, Pm[:, 2]]))
    Ct = SG.turn_cov(C3, a)
    check("2 fixed turn %+.4f deg: R C R^T vs sampling" % math.degrees(a), np.abs(Ct - Cmc).max() / np.abs(C3).max() < 0.02,
          "max |diff| %.2e" % np.abs(Ct - Cmc).max())
g = SG.shift_by(0.0156, 0.0042); qx, qy = g(Pm[:, 0], Pm[:, 1])
check("2 fixed shift leaves the covariance", np.abs(np.cov(np.stack([qx, qy])) - np.cov(Pm[:, :2].T)).max() < 1e-15)

# 3 ------------------------------------------------------------------------------------------------------------------
worst = 0.0; old_worst = 0.0
for rot_deg in (0.5, -0.5, 5.0, -5.0, -0.0272):
    for sx, sy in ((0.0, 0.0), (0.02, 0.0), (0.0, -0.03), (-0.015, 0.04)):
        for off, k in ((0.01, 0.003), (-0.02, -0.01)):
            xd, yref = 2.6, ymid
            f1 = SG.turn_about(2.9, 5.1, math.radians(rot_deg)); f2 = SG.shift_by(sx, sy)
            o1, k1 = SG.xform_line(off, k, xd, yref, f1); o2, k2 = SG.xform_line(o1, k1, xd, yref, f2)
            for yy in (0.6, 4.75, 8.9):                       # points of the original line, moved, must lie on the new line
                px, py = f2(*f1(xd + off + k * (yy - yref), yy))
                worst = max(worst, abs(px - (xd + o2 + k2 * (py - yref))))
            # the old survey_map: offset = turned point's x (y' dropped), slope + turn, then offset + sx - tan(turn) sy
            xr_, _ = f1(xd + off, yref); ko = math.tan(math.atan(k) + math.radians(rot_deg))
            oo = xr_ - xd + sx - ko * sy
            for yy in (0.6, 8.9):
                px, py = f2(*f1(xd + off + k * (yy - yref), yy))
                old_worst = max(old_worst, abs(px - (xd + oo + ko * (py - yref))))
check("3 moved line: points of the old line on the new line (5 turns x 4 shifts x 2 lines x 3 points)", worst < 1e-9,
      "max |x error| %.2e m (old formula: %.4f m)" % (worst, old_worst))

# 4 ------------------------------------------------------------------------------------------------------------------
C2 = np.array([[9e-4, 2e-5], [2e-5, 1.1e-4]]); f = SG.turn_about(2.9, 5.1, math.radians(3.0))
S2 = rng.multivariate_normal([0.01, 0.004], C2, 100000)
moved = np.array([SG.xform_line(o, k, 2.6, ymid, f) for o, k in S2[:20000]])
Cl = SG.line_cov(0.01, 0.004, C2, 2.6, ymid, f); Cmc = np.cov(moved.T)
check("4 line cov through a 3 deg turn vs sampling", np.abs(Cl - Cmc).max() / np.abs(C2).max() < 0.05, "max |diff| %.2e" % np.abs(Cl - Cmc).max())

# 5 ------------------------------------------------------------------------------------------------------------------
TF = {"hfov_deg": 76.0, "range": [0.1, 6.0], "pos_body_flu": [0.08, -0.009, 0.012], "rpy_deg": [2.0, -3.0, 4.0]}
dep = rng.uniform(0.3, 5.0, (48, 64)).astype(np.float32); dep[3, 5] = np.nan; dep[10, 10] = 0.05


def old_solver_points(dep, TF, stride):            # survey_map.py before 2026-11-01 (inline code)
    H, W = dep.shape; fpx = (W / 2.0) / math.tan(math.radians(TF["hfov_deg"]) / 2.0)
    us, vs = np.meshgrid(np.arange(0, W, stride), np.arange(0, H, stride))
    ry = -(us - (W - 1) / 2.0) / fpx; rz = -(vs - (H - 1) / 2.0) / fpx
    rr, rp, rw = np.radians(TF["rpy_deg"])
    Rx = np.array([[1, 0, 0], [0, math.cos(rr), -math.sin(rr)], [0, math.sin(rr), math.cos(rr)]])
    Ry = np.array([[math.cos(rp), 0, math.sin(rp)], [0, 1, 0], [-math.sin(rp), 0, math.cos(rp)]])
    Rz = np.array([[math.cos(rw), -math.sin(rw), 0], [math.sin(rw), math.cos(rw), 0], [0, 0, 1]])
    dd = dep[::stride, ::stride].astype(np.float32); ok = np.isfinite(dd) & (dd > TF["range"][0]) & (dd < TF["range"][1])
    return Rz @ Ry @ Rx @ np.stack([dd[ok], dd[ok] * ry[ok], dd[ok] * rz[ok]]) + np.array(TF["pos_body_flu"])[:, None]


def old_exporter_points(dep, TF, stride, rmax):    # survey_web_export.py before 2026-11-01: no mount rotation
    H, W = dep.shape; fpx = (W / 2.0) / math.tan(math.radians(TF["hfov_deg"]) / 2.0)
    us, vs = np.meshgrid(np.arange(0, W, stride), np.arange(0, H, stride))
    ry = -(us - (W - 1) / 2.0) / fpx; rz = -(vs - (H - 1) / 2.0) / fpx
    dd = dep[::stride, ::stride].astype(np.float32); ok = np.isfinite(dd) & (dd > TF["range"][0]) & (dd < rmax)
    P0 = TF["pos_body_flu"]
    return np.stack([dd[ok] + P0[0], dd[ok] * ry[ok] + P0[1], dd[ok] * rz[ok] + P0[2]])


d_new = np.abs(SG.frame_points(dep, TF, 2) - old_solver_points(dep, TF, 2)).max()
check("5 shared frame_points = old solver code (rpy 2/-3/4 deg)", d_new < 1e-12, "max |diff| %.2e m" % d_new)
d_exp = np.abs(SG.frame_points(dep, TF, 4, rmax=4.0) - old_exporter_points(dep, TF, 4, 4.0)).max()
TF0 = dict(TF, rpy_deg=[0.0, 0.0, 0.0])
d_exp0 = np.abs(SG.frame_points(dep, TF0, 4, rmax=4.0) - old_exporter_points(dep, TF0, 4, 4.0)).max()
check("5 the old exporter differed with a mount rotation, matched without", d_exp > 0.05 and d_exp0 < 1e-6,
      "rpy 2/-3/4: max |diff| %.4f m; rpy 0: %.2e m" % (d_exp, d_exp0))

# 6 ------------------------------------------------------------------------------------------------------------------
a = math.radians(-0.0272) * 40; f = SG.turn_about(2.9, 5.1, a); g = SG.shift_by(0.0156, 0.0042)
B = SG.frame_points(dep, TF, 4); x0, y0, z0, c0 = 2.3, 3.4, 1.2, 263.0
P1 = SG.body_to_map(B, x0, y0, z0, c0); qx, qy = g(*f(P1[:, 0], P1[:, 1]))     # solver: stack, then move the points
xm, ym = g(*f(x0, y0)); P2 = SG.body_to_map(B, xm, ym, z0, c0 - math.degrees(a))  # replay: stack on the moved path (compass - CCW turn)
dd6 = max(np.abs(qx - P2[:, 0]).max(), np.abs(qy - P2[:, 1]).max(), np.abs(P1[:, 2] - P2[:, 2]).max())
check("6 points moved after stacking = stacked on the moved path", dd6 < 1e-9, "max |diff| %.2e m" % dd6)

# 7 ------------------------------------------------------------------------------------------------------------------
ok_cases = {
    "R1C W 10 1 0.68,1.265": ["R1C W 10 1 0.68,1.265"],
    "R1C W 10 1 0.68": ["R1C W 10 1 0.68"],
    "R1C W, R1C E": ["R1C W", "R1C E"],
    "R1C10, R1D W 10 1 0.68,1.265, R1D E": ["R1C10", "R1D W 10 1 0.68,1.265", "R1D E"],
    "R1C W 10 1 0.68, 1.265 ,2.0, R1C E 1 10": ["R1C W 10 1 0.68,1.265,2.0", "R1C E 1 10"]}
for spec, want in ok_cases.items():
    try:
        got = SS.split_segments(spec)
    except SS.SpecError as e:
        got = "error: %s" % e
    check("7 split %r" % spec, got == want, "-> %s" % (got,))
p = SS.parse_segment("R1C W 10 1 0.68,1.265")
check("7 parse heights", p == ("R1C", "W", [10, 1], [0.68, 1.265]), "-> %s" % (p,))
for spec in ("1.265", "R1C W 0.68,1.265", "R1C W, 1.265", "R1C W,, R1C E", "R1C W,"):
    try:
        SS.split_segments(spec); r = "no error"
    except SS.SpecError as e:
        r = "SpecError: %s" % e
    check("7 split %r refused" % spec, r.startswith("SpecError"), r[:110])
for seg in ("R1C W 10", "R1C W 10 1 0.68,abc", "R1C W 10 1 -0.5", "R1C X", "R1C W ten 1", "R1C W 10 1 0.68 extra", "R1C W 10 1 nan"):
    try:
        SS.parse_segment(seg); r = "no error"
    except SS.SpecError as e:
        r = "SpecError: %s" % e
    check("7 parse %r refused" % seg, r.startswith("SpecError"), r[:110])

gm = sys.argv[1] if len(sys.argv) > 1 else os.path.join(SC, "grid_mission.py")
spec_ = importlib.util.spec_from_file_location("grid_mission_under_test", gm); GM = importlib.util.module_from_spec(spec_)
sys.argv = sys.argv[:1]
spec_.loader.exec_module(GM)
if not hasattr(GM, "SS"):
    try:
        r = "route %s" % GM.survey_route("R1C W 10 1 0.68,1.265")
    except Exception as e:  # noqa: BLE001
        r = "%s: %s" % (type(e).__name__, e)
    print("SKIP  7 grid_mission.survey_route: %s does not use survey_spec yet (patch logs/1930_survey_height_syntax_grid_mission.diff);"
          " documented example now -> %s" % (gm, r[:120]))
else:
    for spec, want in (("R1C W 10 1 0.68,1.265", [0.68, 1.265]), ("R1C W 10 1 0.68", [0.68]), ("R1C W", None),
                       ("R1C10, R1D W 10 1 0.68,1.265", [0.68, 1.265])):
        try:
            route = GM.survey_route(spec); hs = sorted({float(h) for h in re.findall(r"@([0-9.]+)", route)})
            ok = (hs == want) if want else len(hs) == len(GM.G.GAP_CENTRE_Z)
            r = "%s heights %s: %s" % ("ok" if ok else "WRONG", hs, route)
        except Exception as e:  # noqa: BLE001
            r = "error %s: %s" % (type(e).__name__, e)
        check("7 survey_route %r" % spec, r.startswith("ok"), r[:170])
    for spec in ("R1C W 0.68,1.265", "1.265", "R1C W 10 1 0.68,abc"):
        try:
            GM.survey_route(spec); r = "no error"
        except GM.RouteError as e:
            r = "RouteError: %s" % e
        except Exception as e:  # noqa: BLE001
            r = "other %s: %s" % (type(e).__name__, e)
        check("7 survey_route %r refused with RouteError" % spec, r.startswith("RouteError"), r[:120])
print("%d failure(s)" % FAIL)
sys.exit(1 if FAIL else 0)
