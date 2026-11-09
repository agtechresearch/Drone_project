#!/usr/bin/env python3
"""Compare a survey map (survey_map.py) with the simulator truth (v3_marker, 2026-10-31).
  QVIX_YAW_DEG=<world yaw> python3 survey_eval.py <run_dir> <truth.json from make_survey_world.py> [--json out.json]
Truth: tag poses of the survey world, the design faces / walls (unchanged in the survey worlds), the vehicle's true pose
(gt.csv odometry stamps when the run has it, else tags.csv; world -> map with QVIX_YAW_DEG). Prints, for the tags the flight saw: design error (what the flight assumed) vs
corrected error (survey result), per axis (x across = stand-off direction, y along the face, z height); faces, walls and
the solved trajectory vs truth (gt.csv when present, see loc_compare.load_truth).
2026-11-01 (codex survey-uncertainty / survey-validation-independence): the tag sd is the survey's linearised sd CONDITIONAL on
the fixed wall anchor (older survey_map.json: the marginal approximation without the slope / cross terms - labelled); the points
line is an internal check (same ToF, same solved path); walls after the anchor are not independent accuracy evidence."""
import argparse, csv, json, math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); V3 = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import qvix_grid as G  # noqa: E402

ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("truth"); ap.add_argument("--json")
ap.add_argument("--design", default=os.path.join(V3, "gz", "tags_map.json")); ap.add_argument("--dir", default="survey")
A = ap.parse_args()
S = json.load(open(os.path.join(A.run, A.dir, "survey_map.json"))); TJ = json.load(open(A.truth)); T = TJ["tags"]
FT = TJ.get("faces_truth", {})                 # moved rack faces (make_survey_world.py --rack-shift); others = design (0)
D = json.load(open(A.design))["tags"]
res = {"run": os.path.basename(os.path.normpath(A.run))}
seen = [k for k, t in S["tags"].items() if t.get("observed")]
de = np.array([[D[k][a] - T[k][a] for a in "xyz"] for k in seen]); ce = np.array([[S["tags"][k][a] - T[k][a] for a in "xyz"] for k in seen])
sd = np.array([S["tags"][k]["sd"] for k in seen])
sd_kind = "J COV J^T, conditional on the fixed anchor" if "uncertainty" in S else "OLD marginal approximation (no slope / cross terms)"
res["sd_kind"] = sd_kind
lines = ["%s: tags seen %d of %d (sd: %s)" % (res["run"], len(seen), len(T), sd_kind)]
for i, a in enumerate("xyz"):
    lines.append("  tag %s  design err rms %.4f max %.4f  ->  corrected rms %.4f max %.4f m   (reported sd median %.4f)" % (
        a, np.sqrt(np.mean(de[:, i] ** 2)), np.abs(de[:, i]).max(), np.sqrt(np.mean(ce[:, i] ** 2)), np.abs(ce[:, i]).max(), np.median(sd[:, i])))
d3, c3 = np.linalg.norm(de, axis=1), np.linalg.norm(ce, axis=1)
lines.append("  tag 3D  design err median %.4f max %.4f  ->  corrected median %.4f max %.4f m; better on %d / %d tags" % (
    np.median(d3), d3.max(), np.median(c3), c3.max(), int((c3 < d3).sum()), len(seen)))
res["tags"] = {k: {"design_err": de[j].round(4).tolist(), "corrected_err": ce[j].round(4).tolist(), "sd": sd[j].tolist(),
                   "n_obs": S["tags"][k]["n_obs"]} for j, k in enumerate(seen)}
for fn, f in S["faces"].items():          # truth = design (offset 0, turn 0) unless the rack line was moved
    pp = f.get("points", {}); tr = FT.get(fn, {"offset": 0.0, "turn_deg": 0.0})
    f["truth"] = tr; f["err"] = [round(f["offset"] - tr["offset"], 4), round(f["turn_deg"] - tr["turn_deg"], 3)]
    lines.append("  face %s  solved offset %+.4f m turn %+.3f deg (truth %+.4f, %+.3f -> err %+.4f m %+.3f deg; sd %.4f, %.3f)   points line (internal check) offset %s turn %s (n %s)" % (
        fn, f["offset"], f["turn_deg"], tr["offset"], tr["turn_deg"], f["err"][0], f["err"][1], f["sd"][0], f["sd"][1],
        "%+.4f" % pp["offset"] if pp else "-", "%+.3f" % pp["turn_deg"] if pp else "-", pp.get("n", "-")))
for wn, w in S["walls"].items():
    lines.append("  wall %s  measured %.4f vs design %.4f -> offset %+.4f m (n %d; anchor reference, not independent)" % (wn, w["measured"], w["design"], w["offset"], w["n"]))
if "observability" in S:
    ob_ = S["observability"]
    res["observability"] = {k_: ob_.get(k_) for k_ in ("eig_min", "n_eig_below_1", "eig_min_without_shift", "n_eig_below_1_without_shift", "eligible", "weakest")}
    lines.append("  observability: min eig %.3g (prior-sd units), %d below 1; without the common shift min %.3g, %d below 1; eligible %s" % (
        ob_["eig_min"], ob_["n_eig_below_1"], ob_.get("eig_min_without_shift", float("nan")), ob_.get("n_eig_below_1_without_shift", -1), ob_["eligible"]))
res["faces"] = S["faces"]; res["walls"] = S["walls"]
# trajectory: the same truth contract as loc_compare.py (2026-10-31 codex ground-truth-timestamps reply-2): gt.csv with the
# odometry's own stamps when the run has it (else tags.csv, image time + latest pose, labelled), no interpolation across a truth
# gap > MAX_GAP or a non-finite sample (those estimates are counted as excluded)
import loc_compare as LC  # noqa: E402
src, tt, TX, TY, _, TC, dropped = LC.load_truth(A.run)
if not len(tt):                                     # no truth at all: the path part is "not evaluated", the rest stays
    lines.append("  path  not evaluated: no truth samples (%s)" % src)
    res["path"] = {"batch_median": None, "batch_max": None, "online_median": None, "online_max": None, "truth": src,
                   "excluded": None, "status": "not evaluated: no truth"}
    print("\n".join(lines))
    if A.json:
        json.dump(res, open(A.json, "w"), indent=1)
    sys.exit(0)
TCm = LC.unwrap_finite(TC)
J = np.genfromtxt(os.path.join(A.run, A.dir, "traj.csv"), delimiter=",", names=True)
m = (J["t"] >= tt[0] + 2) & (J["t"] <= tt[-1]) if len(tt) else np.zeros(len(J["t"]), bool)
gx, gy, gc = LC.truth_at(tt, TX, J["t"][m]), LC.truth_at(tt, TY, J["t"][m]), LC.truth_at(tt, TCm, J["t"][m])
ok = np.isfinite(gx) & np.isfinite(gy) & np.isfinite(gc)
pe = np.hypot(J["x"][m] - gx, J["y"][m] - gy)[ok]
he = ((J["comp"][m] - gc + 180) % 360 - 180)[ok]
L = [json.loads(s) for s in open(os.path.join(A.run, "loc.jsonl")) if s.strip()] if os.path.exists(os.path.join(A.run, "loc.jsonl")) else []
L = [r for r in L if r["ready"] and len(tt) and tt[0] <= r["t"] <= tt[-1]]
lt = np.array([r["t"] for r in L], float); lgx, lgy = LC.truth_at(tt, TX, lt), LC.truth_at(tt, TY, lt); lok = np.isfinite(lgx) & np.isfinite(lgy)
lp = np.hypot(np.array([r["x"] for r in L], float) - lgx, np.array([r["y"] for r in L], float) - lgy)[lok]
lines.append("  truth %s, rows %d, dropped %s; excluded (no bracketing truth <= %.1f s): survey %d of %d, online %d of %d" % (
    src, len(tt), dropped, LC.MAX_GAP, int((~ok).sum()), len(ok), int((~lok).sum()), len(lok)))
st = lambda v: (float(np.median(v)), float(np.max(v))) if len(v) else (None, None)   # noqa: E731
f4 = lambda v: "%.4f" % v if v is not None else "not evaluated (no comparable sample)"   # noqa: E731
(pm, px), (hm, hx), (om, ox) = st(pe), st(np.abs(he)) if len(he) else (None, None), st(lp)
lines.append("  path  survey (batch) err median %s max %s m, heading median %s max |%s| deg;  online estimator median %s max %s m" % (
    f4(pm), f4(px), "%+.2f" % float(np.median(he)) if len(he) else "-", "%.2f" % hx if hx is not None else "-", f4(om), f4(ox)))
res["path"] = {"batch_median": pm, "batch_max": px, "online_median": om, "online_max": ox, "truth": src,
               "excluded": {"survey": int((~ok).sum()), "online": int((~lok).sum())}}
print("\n".join(lines))
if A.json:
    json.dump(res, open(A.json, "w"), indent=1)
