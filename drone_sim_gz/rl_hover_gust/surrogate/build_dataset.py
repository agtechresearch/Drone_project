#!/usr/bin/env python3
"""Surrogate step 1 (2026-11-04): Gazebo hover-gust logs -> per-episode arrays on the 10 Hz decision grid.
  venv/bin/python surrogate/build_dataset.py --out surrogate/data/ds.npz logs/hg_v2_s1_u* logs/hg_v2wc_s1_u* ...
Uses experiments/train_hover_gust.episode_data (same validity rules and reference as training). Per valid episode:
  t (decision sim time), e_est / ve_est / roll (PX4 estimate in the observation), delta (target offset sent AFTER decision j),
  F (mean world east force over [t_j, t_j+1), from gust_events.csv sent times), e_true / v_true (truth at t_j, v by +-0.05 s
  central difference), rew (training reward), run, update dir, cond. Read-only on the logs."""
import argparse, csv, glob, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE); sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "experiments"))
import numpy as np
from rl_demo.contracts import load_config
from train_hover_gust import GT, episode_data


def force_fn(events, ep_id):
    """piecewise-constant force (N, + = east) from the sent apply/clear events of one episode"""
    tt, ff = [], []
    for r in events:
        if r["ep"] != str(ep_id) or r["published"] != "1":
            continue
        tt.append(float(r["t_sim_sent"])); ff.append(float(r["force_n"]) if r["event"] == "apply" else 0.0)
    o = np.argsort(tt, kind="stable"); tt = np.array(tt)[o]; ff = np.array(ff)[o]

    def mean_on(t0, t1, n=11):
        s = np.linspace(t0, t1, n); i = np.searchsorted(tt, s, side="right") - 1
        return float(np.mean(np.where(i >= 0, ff[np.maximum(i, 0)], 0.0)))
    return mean_on


def one_dir(BU, cfg, out):
    gp = os.path.join(BU, "gt.csv"); ev = os.path.join(BU, "gust_events.csv")
    if not (os.path.exists(gp) and os.path.exists(ev)):
        return 0
    gt = GT(gp); gt.update(); events = list(csv.DictReader(open(ev))); n = 0
    for D in sorted(glob.glob(os.path.join(BU, "ep_*"))):
        ej = os.path.join(D, "episode.json")
        if not os.path.exists(ej) or not os.path.exists(os.path.join(D, "policy.csv")):
            continue
        ep = json.load(open(ej))["episode"]
        try:
            ok, why, d = episode_data(BU, ep, cfg, gt)
        except Exception as e:
            ok, why = False, "%s" % e
        if not ok:
            continue
        rows = list(csv.DictReader(open(os.path.join(D, "policy.csv"))))
        t = np.array([float(r["t_sim"]) for r in rows]); t_stop = d["info"]["t_stop"]
        ends = np.concatenate([t[1:], [t_stop]]); Ff = force_fn(events, ep["id"])
        ref = d["info"]["ref_e"]
        e_true = gt.x_at(t) - ref; v_true = (gt.x_at(t + 0.05) - gt.x_at(t - 0.05)) / 0.1
        out.append({"t": t - t[0], "e_est": np.array([float(r["o0"]) for r in rows]) * 0.1,
                    "ve_est": np.array([float(r["o1"]) for r in rows]) * 0.3,
                    "roll": np.array([float(r["o2"]) for r in rows]) * 10.0,
                    "delta": np.array([float(r["delta"]) for r in rows]),
                    "F": np.array([Ff(a, b) for a, b in zip(t, ends)]), "e_true": e_true, "v_true": v_true,
                    "rew": d["rew"], "term": d["term"], "cond": ep["cond"], "seed": ep["seed"],
                    "dir": os.path.basename(BU.rstrip("/")), "ctrl": json.dumps(ep.get("ctrl", {}))})
        n += 1
    return n


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True)
    ap.add_argument("--config", default=os.path.join(ROOT, "config/hover_gust_rl_wide.json")); ap.add_argument("dirs", nargs="+")
    a = ap.parse_args(); cfg = load_config(a.config); out = []
    for BU in a.dirs:
        n = one_dir(BU, cfg, out); print("%s %d" % (os.path.basename(BU), n), flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    np.save(a.out, np.array(out, dtype=object), allow_pickle=True)
    print("episodes %d, steps %d -> %s" % (len(out), sum(len(o["t"]) for o in out), a.out))


if __name__ == "__main__":
    main()
