#!/usr/bin/env python3
"""Aggregate the Gazebo comparison batches (2026-11-04): logs/hg_evsur_b*/ -> logs/evsur/compare_all.{json,txt}
Same per-episode rules as experiments/eval_hover_gust.compare (outcome from hover_gust_eval summary.json, validity from
train_hover_gust.episode_data, reward recomputed from the truth), but over several batches and split by cond:
  gust : peak |e|/|F| per gust (|F| >= 0.15 N and all), return after clear (s, None = not returned), lateral max / RMS, reward
  none : lateral RMS (truth, about the reference), max, reward
paired: gust seeds on which every controller completed validly, reward / lat max difference to PX4 alone.
  venv/bin/python surrogate/agg_evsur.py [batch dirs ...]   (default logs/hg_evsur_* + logs/hg_evxp*)"""
import glob, json, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, "experiments"))
import numpy as np
import train_hover_gust as T
from eval_hover_gust import name
from rl_demo.contracts import load_config


def st(v):
    v = [x for x in v if x is not None]
    return {"n": len(v), "mean": float(np.mean(v)), "median": float(np.median(v)), "sd": float(np.std(v))} if v else None


def main(dirs):
    cfg = load_config(os.path.join(ROOT, "config/hover_gust_rl_wide.json")); G = {}; per = {}
    for B in dirs:
        if not os.path.exists(os.path.join(B, "summary.json")):
            print("skip (no summary): %s" % B); continue
        plan = json.load(open(os.path.join(B, "plan.json"))); summ = {e["id"]: e for e in json.load(open(os.path.join(B, "summary.json")))["episodes"]}
        gt = T.GT(os.path.join(B, "gt.csv")); gt.update()
        for ep in plan["episodes"]:
            k = name(ep["ctrl"]); c = ep["cond"]
            g = G.setdefault(k, {}).setdefault(c, {"planned": 0, "completed": 0, "task_failed": [], "invalid": [], "peak15": [], "peak": [],
                                                   "ret": [], "not_ret": 0, "lat_max": [], "lat_rms": [], "reward": []})
            g["planned"] += 1; s = summ.get(ep["id"], {}); oc = s.get("outcome", "invalid")
            try:
                ok, why, d = T.episode_data(B, ep, cfg, gt) if oc != "invalid" else (False, "; ".join(s.get("fail", ["no summary"])), None)
            except Exception as e:
                ok, why, d = False, "episode_data: %s" % e, None
            if oc == "invalid" or not ok:
                g["invalid"].append([os.path.basename(B), ep["id"], ep["seed"], why]); continue
            if oc == "task_failed":
                g["task_failed"].append([os.path.basename(B), ep["id"], ep["seed"]]); continue
            g["completed"] += 1; L = s["lateral_err_true"]
            g["lat_max"].append(L["max_abs"]); g["lat_rms"].append(L["rms"]); g["reward"].append(d["info"]["return"])
            for q in s.get("gusts", []):
                r = abs(q["peak_lateral_m"]) / abs(q["force_n"]); g["peak"].append(r)
                if abs(q["force_n"]) >= 0.15:
                    g["peak15"].append(r)
                if q["return_after_clear_s"] is None:
                    g["not_ret"] += 1
                else:
                    g["ret"].append(q["return_after_clear_s"])
            per[(k, c, ep["seed"])] = {"reward": d["info"]["return"], "lat_max": L["max_abs"], "lat_rms": L["rms"]}
    out = {}
    for k, byc in G.items():
        out[k] = {}
        for c, g in byc.items():
            out[k][c] = {"planned": g["planned"], "completed": g["completed"], "task_failed": g["task_failed"], "invalid": g["invalid"],
                         "not_returned": g["not_ret"], **{m: st(g[m]) for m in ("peak15", "peak", "ret", "lat_max", "lat_rms", "reward")}}
    ctrls = list(G); base = "PX4 alone"; paired = {}
    for c in ("gust", "none"):
        seeds = sorted(set(sd for (_, cc, sd) in per if cc == c)); common = [sd for sd in seeds if all((k, c, sd) in per for k in ctrls)]
        paired[c] = {"seeds": common}
        for k in ctrls:
            pk = {m: st([per[(k, c, sd)][m] for sd in common]) for m in ("reward", "lat_max", "lat_rms")}
            if k != base and base in ctrls:
                pk["diff_vs_px4"] = {m: st([per[(k, c, sd)][m] - per[(base, c, sd)][m] for sd in common]) for m in ("reward", "lat_max", "lat_rms")}
            paired[c][k] = pk
    out["_paired"] = paired
    os.makedirs(os.path.join(ROOT, "logs/evsur"), exist_ok=True)
    json.dump(out, open(os.path.join(ROOT, "logs/evsur/compare_all.json"), "w"), indent=1)
    f = lambda x, key="mean", fmt="%.3f": (fmt % x[key]) if x else "-"
    L = ["GUST | controller | planned/completed/failed/invalid | peak/F (|F|>=0.15) | peak/F all | return s median (not returned) | lat max | reward"]
    for k in ctrls:
        o = out[k].get("gust")
        if o:
            L.append("  %s | %d/%d/%d/%d | %s | %s | %s (%d) | %s | %s" % (k, o["planned"], o["completed"], len(o["task_failed"]), len(o["invalid"]),
                     f(o["peak15"]), f(o["peak"]), f(o["ret"], "median", "%.2f"), o["not_returned"], f(o["lat_max"]), f(o["reward"], "mean", "%.2f")))
    L.append("NONE | controller | planned/completed/failed/invalid | lateral RMS cm | lat max cm | reward")
    for k in ctrls:
        o = out[k].get("none")
        if o:
            L.append("  %s | %d/%d/%d/%d | %s | %s | %s" % (k, o["planned"], o["completed"], len(o["task_failed"]), len(o["invalid"]),
                     f(o["lat_rms"], "mean", "%.3f") if not o["lat_rms"] else "%.3f" % (100 * o["lat_rms"]["mean"]),
                     "-" if not o["lat_max"] else "%.2f" % (100 * o["lat_max"]["mean"]), f(o["reward"], "mean", "%.2f")))
    for c in ("gust", "none"):
        L.append("paired %s over %d seeds %s" % (c, len(paired[c]["seeds"]), paired[c]["seeds"]))
        for k in ctrls:
            pk = paired[c][k]; dv = pk.get("diff_vs_px4")
            L.append("  %s | reward %s | lat max %s%s" % (k, f(pk["reward"], "mean", "%.2f"), f(pk["lat_max"]),
                     (" | vs PX4: reward %s (sd %s), lat max %s" % (f(dv["reward"], "mean", "%+.2f"), f(dv["reward"], "sd", "%.2f"), f(dv["lat_max"], "mean", "%+.3f"))) if dv else ""))
    for k in ctrls:
        for c, o in out[k].items():
            for x in o["invalid"]:
                L.append("INVALID %s %s %s ep %d seed %d: %s" % (k, c, x[0], x[1], x[2], x[3]))
            for x in o["task_failed"]:
                L.append("TASK_FAILED %s %s %s ep %d seed %d" % (k, c, x[0], x[1], x[2]))
    open(os.path.join(ROOT, "logs/evsur/compare_all.txt"), "w").write("\n".join(L) + "\n"); print("\n".join(L))


if __name__ == "__main__":
    main(sys.argv[1:] or sorted(glob.glob(os.path.join(ROOT, "logs/hg_evsur_*")) + glob.glob(os.path.join(ROOT, "logs/hg_evxp*"))))
