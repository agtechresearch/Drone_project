#!/usr/bin/env python3
"""Common-seed re-aggregation for the README table (2026-11-09, Codex readme-simulation-denominators).
Same per-episode rules as agg_evsur.py, but only over seeds on which ALL listed controllers completed validly.
  venv/bin/python surrogate/agg_evsur_common.py   -> logs/evsur/compare_common6.{json,txt}"""
import glob, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, "experiments"))
import numpy as np
import train_hover_gust as T
from eval_hover_gust import name
from rl_demo.contracts import load_config
CTRLS = ["PX4 alone", "PPO PD", "PPO Gazebo v042", "PPO sur scratch", "PPO sur bc", "PPO sur bcrand"]


def main():
    cfg = load_config(os.path.join(ROOT, "config/hover_gust_rl_wide.json")); per = {}
    for B in sorted(glob.glob(os.path.join(ROOT, "logs/hg_evsur_*"))):
        if not os.path.exists(os.path.join(B, "summary.json")):
            continue
        plan = json.load(open(os.path.join(B, "plan.json"))); summ = {e["id"]: e for e in json.load(open(os.path.join(B, "summary.json")))["episodes"]}
        gt = T.GT(os.path.join(B, "gt.csv")); gt.update()
        for ep in plan["episodes"]:
            k = name(ep["ctrl"])
            if k not in CTRLS:
                continue
            s = summ.get(ep["id"], {}); oc = s.get("outcome", "invalid")
            if oc != "completed":
                continue
            try:
                ok, why, d = T.episode_data(B, ep, cfg, gt)
            except Exception:
                ok = False
            if not ok:
                continue
            per[(k, ep["cond"], ep["seed"])] = {"gusts": [(abs(q["peak_lateral_m"]), abs(q["force_n"])) for q in s.get("gusts", [])],
                                                "rms": s["lateral_err_true"]["rms"]}
    out = {}; L = []
    for c in ("gust", "none"):
        seeds = sorted(set(sd for (_, cc, sd) in per if cc == c)); common = [sd for sd in seeds if all((k, c, sd) in per for k in CTRLS)]
        out[c] = {"seeds": common}; L.append("%s common seeds (%d): %s" % (c, len(common), common))
        for k in CTRLS:
            if c == "gust":
                r15 = [p / f for sd in common for (p, f) in per[(k, c, sd)]["gusts"] if f >= 0.15]
                ra = [p / f for sd in common for (p, f) in per[(k, c, sd)]["gusts"]]
                out[c][k] = {"n_gust15": len(r15), "peak15_mean": float(np.mean(r15)), "n_gust": len(ra), "peak_mean": float(np.mean(ra))}
                L.append("  %s | peak/F |F|>=0.15: %.3f (n=%d) | all: %.3f (n=%d)" % (k, np.mean(r15), len(r15), np.mean(ra), len(ra)))
            else:
                r = [per[(k, c, sd)]["rms"] for sd in common]
                out[c][k] = {"n": len(r), "rms_cm_mean": 100 * float(np.mean(r))}
                L.append("  %s | lateral RMS cm: %.3f (n=%d)" % (k, 100 * np.mean(r), len(r)))
    json.dump(out, open(os.path.join(ROOT, "logs/evsur/compare_common6.json"), "w"), indent=1)
    open(os.path.join(ROOT, "logs/evsur/compare_common6.txt"), "w").write("\n".join(L) + "\n"); print("\n".join(L))


if __name__ == "__main__":
    main()
