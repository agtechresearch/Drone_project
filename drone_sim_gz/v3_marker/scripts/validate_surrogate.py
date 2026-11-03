#!/usr/bin/env python3
"""Surrogate vs Gazebo: same error conditions as the C-face sweep runs, controller none (wall-keep off) or estimator (on).
  venv/bin/python scripts/validate_surrogate.py  -> table + validate.png"""
import os, sys, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, "/home/dronesim/starling_sim/v2_wallkeep/scripts")
from wk3 import surrogate as S, controllers as C
import wk_report as R
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.rcParams["font.family"] = "Noto Sans CJK JP"
L = "/home/dronesim/starling_sim/v2_wallkeep/logs/sweep_20261029"
runs = []
for k in ("yaw", "drift", "step"):
    for line in open(os.path.join(L, k + ".txt")):
        f, c, w, d = line.split()
        runs.append((c, w, d))
rows = []; fig, axs = plt.subplots(4, 2, figsize=(14, 11), dpi=90); axs = axs.ravel(); pi = 0
for c, w, d in runs:
    sc = S.Scenario(heading_deg=int(c[3:]) / 10 if c.startswith("yaw") else 0, drift_cms=int(c[5:]) / 10 if c.startswith("drift") else 0,
                    step_cm=int(c[4:]) if c.startswith("step") else 0, step_at=20.0 + 3.0, seed=1)
    m, o = S.run(sc, C.make("estimator" if w == "on" else "none"), record=True)
    info = R.load_run(d); g = R.metrics(info)
    rows.append((c, w, g["err_max_abs_legs"] * 100, m["max_legs"] * 100, g["err_max_abs"] * 100, m["max_all"] * 100, g["grid_verify"], m["contact"]))
    if pi < 8 and c in ("yaw30", "drift04", "drift08", "step10"):
        Sd = info["S"]; fi = R.flight_idx(info); ref = list(R.d_ref_for(info).values())[0]
        ta = Sd["t_sim"][fi] - info["t_arm"]
        axs[pi].plot(ta, (Sd["d_face"][fi] - ref) * 100, label="Gazebo")
        axs[pi].plot(np.array(o["t"]), -np.array(o["x"]) * 100, "--", label="대리 모델")
        axs[pi].set_title("%s · 보정 %s" % (c, "켬(C1)" if w == "on" else "끔")); axs[pi].set_ylim(-40, 15); axs[pi].grid(alpha=.3); axs[pi].legend(fontsize=8)
        pi += 1
fig.supxlabel("시동 후 시간 (s)"); fig.supylabel("면까지 거리 − 목표 (cm, − = 면에 가까움)"); fig.tight_layout(); fig.savefig(os.path.join(HERE, "..", "logs", "validate.png"))
print("%-8s %-3s | Gazebo legs / 대리 legs | Gazebo all / 대리 all | Gz verdict / 대리 contact" % ("cond", "wk"))
for r in rows:
    print("%-8s %-3s | %6.1f / %6.1f | %6.1f / %6.1f | %s / %s" % r)
