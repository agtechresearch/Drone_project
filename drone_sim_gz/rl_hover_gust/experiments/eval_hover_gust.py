#!/usr/bin/env python3
"""Hover-gust comparison: PX4 alone vs velocity-proportional correction vs PPO, same gusts (2026-11-03, stage D). Host venv.
  eval_hover_gust.py --make-plan <out.json> --config <cfg> [--policy <abs path of policy_vXXX.json>] [--ks 0.1,0.2,0.4]
                     [--seeds 10 --seed-base 900000]
      every seed is flown once per controller, controllers interleaved per seed (zero, vel k..., ppo); gust seeds 900000+ never
      occur in training (train_hover_gust.make_plan uses 100000 + seed * 10000 + i). The policy path must be the path INSIDE
      starling-flight (/home/dronesim/workspace/...); it is copied into the plan as is.
      "selection" sets (to pick k / a policy version) and the final set must use different --seed-base.
  eval_hover_gust.py --compare <batch_dir> --config <cfg>   -> <batch_dir>/compare.json, compare.txt
      per controller over the gust episodes: lateral truth max |e| and RMS, per-gust peak |e|/|F| and return time
      (hover_gust_eval.py summary.json), and the training reward of each episode recomputed from the truth (same function)."""
import argparse, json, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE); sys.path.insert(0, ROOT); sys.path.insert(0, HERE)
from rl_demo.contracts import load_config, gust_schedule


def make_plan(cfg, policy, ks, n, base, n_none=0):
    ctrls = [{"type": "zero"}] + [{"type": "vel", "k": k} for k in ks]
    # 2026-11-04: several policies, comma separated, each "label=path" or "path" (label = file name)
    for tok in (policy.split(",") if policy else []):
        lab, _, pth = tok.rpartition("=")
        c = {"type": "ppo", "policy": pth, "deterministic": True}
        if lab:
            c["label"] = lab
        ctrls.append(c)
    eps = []
    # 2026-11-04: n_none still-air seeds (cond none, seed base + 5000 + s), flown after the gust seeds, same interleaving
    for cond, s0, m in (("gust", base, n), ("none", base + 5000, n_none)):
        for s in range(m):
            for c in ctrls:
                e = {"id": len(eps), "cond": cond, "seed": s0 + s, "ctrl": dict(c)}
                gust_schedule(cfg, e["cond"], e["seed"]); eps.append(e)
    return {"episodes": eps, "eval": {"seed_base": base, "n_seeds": n, "n_none": n_none, "controllers": ctrls}}


def name(c):
    if c["type"] == "zero" and c.get("label"):       # 2026-11-04: e.g. a tuned-PX4 batch (parameter set by the runner)
        return c["label"]
    return {"zero": "PX4 alone", "vel": "vel k=%g" % c.get("k", 0), "ppo": "PPO %s" % c.get("label", os.path.basename(c.get("policy", "")))}[c["type"]]


def stats(v):
    v = [x for x in v if x is not None and not (isinstance(x, float) and math.isnan(x))]
    if not v:
        return None
    m = sum(v) / len(v); sd = math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
    return {"n": len(v), "mean": m, "sd": sd, "min": min(v), "max": max(v)}


def compare(B, cfgp):
    """2026-11-03 v2 (codex hover-early-termination-evaluation): every planned episode is counted. outcome = completed |
    task_failed (terminated) | invalid (hover_gust_eval check failed, or the trainer's validity rules - yaw / estimate offset /
    timing - rejected it; the same rules for every controller). Performance statistics use completed + valid episodes only;
    'paired' = the seeds on which EVERY controller completed validly (same gusts), with the difference to PX4 alone."""
    import train_hover_gust as T
    cfg = load_config(cfgp); plan = json.load(open(os.path.join(B, "plan.json")))
    summ = {e["id"]: e for e in json.load(open(os.path.join(B, "summary.json")))["episodes"]}
    gt = T.GT(os.path.join(B, "gt.csv")); gt.update()
    groups = {}; per = {}
    for ep in plan["episodes"]:
        k = name(ep["ctrl"])
        g = groups.setdefault(k, {"planned": 0, "completed": [], "task_failed": [], "invalid": [], "lat_max": [], "lat_rms": [],
                                  "peak_per_n": [], "return_s": [], "n_returned_none": 0, "reward": []})
        g["planned"] += 1
        s = summ.get(ep["id"], {}); oc = s.get("outcome", "invalid")
        ok, why, d = T.episode_data(B, ep, cfg, gt) if oc != "invalid" else (False, "; ".join(s.get("fail", ["no summary"])), None)
        if oc == "invalid" or not ok:
            g["invalid"].append([ep["id"], ep["seed"], why]); continue
        if oc == "task_failed":
            g["task_failed"].append([ep["id"], ep["seed"], s.get("terminated"), s.get("window_s")]); continue
        g["completed"].append(ep["id"])
        L = s["lateral_err_true"]; g["lat_max"].append(L["max_abs"]); g["lat_rms"].append(L["rms"]); g["reward"].append(d["info"]["return"])
        for q in s.get("gusts", []):
            g["peak_per_n"].append(abs(q["peak_lateral_m"]) / abs(q["force_n"]))
            if q["return_after_clear_s"] is None:
                g["n_returned_none"] += 1
            else:
                g["return_s"].append(q["return_after_clear_s"])
        per[(k, ep["seed"])] = {"lat_max": L["max_abs"], "lat_rms": L["rms"], "reward": d["info"]["return"]}
    out = {}
    for k, g in groups.items():
        n_end = len(g["completed"]) + len(g["task_failed"])
        out[k] = {"planned": g["planned"], "completed": len(g["completed"]), "task_failed": g["task_failed"], "invalid": g["invalid"],
                  "task_failure_rate_of_planned": len(g["task_failed"]) / float(g["planned"]),
                  "task_failure_rate_of_valid": len(g["task_failed"]) / float(n_end) if n_end else None,
                  "n_returned_none": g["n_returned_none"]}
        out[k].update({m: stats(g[m]) for m in ("lat_max", "lat_rms", "peak_per_n", "return_s", "reward")})
    seeds = sorted(set(sd for (_, sd) in per)); ctrls = list(groups)
    common = [sd for sd in seeds if all((k, sd) in per for k in ctrls)]
    base = "PX4 alone" if "PX4 alone" in ctrls else None
    paired = {"seeds": common}
    for k in ctrls:
        pk = {m: stats([per[(k, sd)][m] for sd in common]) for m in ("lat_max", "lat_rms", "reward")}
        if base and k != base:
            pk["diff_vs_px4_alone"] = {m: stats([per[(k, sd)][m] - per[(base, sd)][m] for sd in common]) for m in ("lat_max", "lat_rms", "reward")}
        paired[k] = pk
    out["_paired"] = paired
    json.dump(out, open(os.path.join(B, "compare.json"), "w"), indent=1)
    f = lambda st, key="mean", fmt="%.4f": (fmt % st[key]) if st else "-"
    L = ["controller | planned | completed | task_failed | invalid | lat max mean (sd) | lat RMS mean | peak/|F| | return s (none) | reward mean"]
    for k in ctrls:
        o = out[k]
        L.append("%s | %d | %d | %d | %d | %s (%s) | %s | %s | %s (%d) | %s" % (k, o["planned"], o["completed"], len(o["task_failed"]),
                 len(o["invalid"]), f(o["lat_max"]), f(o["lat_max"], "sd"), f(o["lat_rms"]), f(o["peak_per_n"]),
                 f(o["return_s"], "mean", "%.2f"), o["n_returned_none"], f(o["reward"], "mean", "%.2f")))
    L.append("paired over %d common seeds %s:" % (len(common), common))
    for k in ctrls:
        pk = paired[k]; dv = pk.get("diff_vs_px4_alone")
        L.append("  %s | lat max %s | lat RMS %s | reward %s%s" % (k, f(pk["lat_max"]), f(pk["lat_rms"]), f(pk["reward"], "mean", "%.2f"),
                 (" | diff vs PX4 alone: lat max %s (sd %s), RMS %s" % (f(dv["lat_max"]), f(dv["lat_max"], "sd"), f(dv["lat_rms"]))) if dv else ""))
    for k in ctrls:
        for x in out[k]["invalid"]:
            L.append("INVALID %s ep %d seed %d: %s" % (k, x[0], x[1], x[2]))
    open(os.path.join(B, "compare.txt"), "w").write("\n".join(L) + "\n"); print("\n".join(L))
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True); ap.add_argument("--make-plan"); ap.add_argument("--compare")
    ap.add_argument("--policy"); ap.add_argument("--ks", default="0.1,0.2,0.4")
    ap.add_argument("--seeds", type=int, default=10); ap.add_argument("--seed-base", type=int, default=900000)
    ap.add_argument("--none-seeds", type=int, default=0)
    a = ap.parse_args()
    if a.make_plan:
        ks = [float(k) for k in a.ks.split(",")] if a.ks else []
        json.dump(make_plan(load_config(a.config), a.policy, ks, a.seeds, a.seed_base, a.none_seeds), open(a.make_plan, "w"), indent=1); sys.exit(0)
    sys.exit(compare(a.compare, a.config))
