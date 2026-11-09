#!/usr/bin/env python3
"""Split a full comparison plan (eval_hover_gust.py --make-plan) into one batch per seed (2026-11-09, release helper).
One boot flies one seed x all controllers (EKF2_EV_CTRL 7 heading drift exceeded the 3 deg validity tolerance within ~12
take-offs on one boot, worklog 2026-11-03), as in the 2026-11-04 comparison (logs/evsur/plan_evsur_*.json).
  venv/bin/python experiments/split_eval_plan.py <plan.json> <out_dir> [--prefix ev] [--rotate]
writes <out_dir>/plan_<prefix>_g<i>.json / plan_<prefix>_n<i>.json and <out_dir>/batches.txt ("name|" per line, for
scripts/run_eval_batches.sh). --rotate shifts the controller order by the batch index (order effects spread over controllers).
Episode ids restart at 0 per batch; global_id keeps the index in the full plan."""
import argparse, json, os
ap = argparse.ArgumentParser(); ap.add_argument("plan"); ap.add_argument("out_dir"); ap.add_argument("--prefix", default="ev")
ap.add_argument("--rotate", action="store_true"); a = ap.parse_args()
P = json.load(open(a.plan)); os.makedirs(a.out_dir, exist_ok=True); groups = {}
for gid, e in enumerate(P["episodes"]):
    groups.setdefault((e["cond"], e["seed"]), []).append(dict(e, global_id=gid))
names = []; count = {"gust": 0, "none": 0}
for b, ((cond, seed), eps) in enumerate(sorted(groups.items(), key=lambda kv: kv[1][0]["global_id"])):
    if a.rotate:
        r = b % len(eps); eps = eps[r:] + eps[:r]
    for i, e in enumerate(eps):
        e["id"] = i
    name = "%s_%s%d" % (a.prefix, "g" if cond == "gust" else "n", count[cond]); count[cond] += 1
    json.dump({"episodes": eps, "eval": P["eval"]}, open(os.path.join(a.out_dir, "plan_%s.json" % name), "w"), indent=1); names.append(name)
open(os.path.join(a.out_dir, "batches.txt"), "w").write("".join("%s|\n" % n for n in names))
print("%d batches -> %s" % (len(names), a.out_dir))
