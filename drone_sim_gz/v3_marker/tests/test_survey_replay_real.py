#!/usr/bin/env python3
"""Replay / solver point agreement on a REAL survey result (2026-11-01, codex survey-replay-consistency):
the solver's saved points (points.npz, after the anchor) are rebuilt from the saved inputs only - the run's tof_raw frames, the
solver's own config (survey_map.json "config_effective") and its final path (traj.csv) - with the shared survey_geom functions,
then voxelised like the solver. Also checks that the face lines in survey_map.json pass through the saved points the same way
(solved line vs points line, both in the final frame).
  python3 tests/test_survey_replay_real.py <run_dir> <survey out dir> [config file: negative control, expected FAIL]"""
import glob, json, math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(os.path.dirname(HERE), "scripts"))
import survey_geom as SG  # noqa: E402

run, sdir = sys.argv[1:3]
S = json.load(open(os.path.join(sdir, "survey_map.json"))); CFG = S["config_effective"]; TF, GR = CFG["tof"], CFG["grid"]
if len(sys.argv) > 3:                     # negative control: rebuild with another config file (e.g. the current one) instead
    TF = json.load(open(sys.argv[3]))["tof"]; print("NEGATIVE CONTROL: tof from %s" % sys.argv[3])
J = np.genfromtxt(os.path.join(sdir, "traj.csv"), delimiter=",", names=True)
comp_u = np.degrees(np.unwrap(np.radians(J["comp"])))
t0, t1 = J["t"][0], J["t"][-1]; pts = []; nxt = -1e9
for fn in sorted(glob.glob(os.path.join(run, "tof_raw", "chunk_*.npz"))):
    Z = np.load(fn)
    for t, dep in zip(Z["t"], Z["depth"]):
        if t < nxt or not (t0 < t < t1):
            continue
        nxt = t + 1.0 / TF["frames_hz"]
        B = SG.frame_points(dep, TF, TF["pixel_stride"])
        pts.append(SG.body_to_map(B, np.interp(t, J["t"], J["x"]), np.interp(t, J["t"], J["y"]), np.interp(t, J["t"], J["z"]),
                                  float(np.interp(t, J["t"], comp_u))))
P = np.concatenate(pts)
Q = np.load(os.path.join(sdir, "points.npz"))["xyz"].astype(np.float64)
# the solver keeps the FIRST point of each 2 cm voxel (voxels taken BEFORE the anchor turn / shift) and then moves those points,
# so its saved points are a subset of the rebuilt points: each saved point must have a rebuilt point at ~0 distance (only
# traj.csv rounding 0.1 mm / 0.001 deg and float32 storage between them), and every rebuilt point must lie within one voxel
# diagonal of a saved one
from scipy.spatial import cKDTree  # noqa: E402
d_sr = cKDTree(P).query(Q)[0]; d_rs = cKDTree(Q).query(P)[0]
diag = GR["cell"] * math.sqrt(3)
print("config_effective tof pos_body_flu %s rpy_deg %s (set %s)" % (TF["pos_body_flu"], TF["rpy_deg"], S.get("set")))
print("points: solver saved %d, rebuilt (all, before voxels) %d" % (len(Q), len(P)))
print("saved -> nearest rebuilt: median %.2e max %.2e m (limit 1e-3)" % (np.median(d_sr), d_sr.max()))
print("rebuilt -> nearest saved: median %.4f max %.4f m (limit voxel diagonal %.4f)" % (np.median(d_rs), d_rs.max(), diag))
ok = d_sr.max() < 1e-3 and d_rs.max() <= diag + 1e-3
# face lines vs saved points (both in the final frame): mean signed distance of the points near each line
for fn, f in S["faces"].items():
    xd = f["x_design"]; yr = f["y_ref"]
    for name, (off, k) in (("solved", (f["offset"], f["slope"])), ("points", (f["points"]["offset"], f["points"]["slope"]) if "points" in f else (None, None))):
        if off is None:
            continue
        d = Q[:, 0] - (xd + off + k * (Q[:, 1] - yr))
        m = (np.abs(d) < 0.03) & (Q[:, 2] > GR["z_min"]) & (Q[:, 2] < GR["z_max"])
        print("face %s %-6s line: offset %+.4f slope %+.6f; saved points within 3 cm: %d, median distance %+.4f m" % (
            fn, name, off, k, int(m.sum()), float(np.median(d[m])) if m.sum() else float("nan")))
print("RESULT %s" % ("PASS" if ok else "FAIL"))
sys.exit(0 if ok else 1)
