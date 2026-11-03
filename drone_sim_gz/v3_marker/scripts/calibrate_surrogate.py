#!/usr/bin/env python3
"""Grid-calibrate the surrogate's EKF lag (tau_ekf) and hover wobble (sig_a) against the Gazebo C-face sweep runs
(leg max error per run; contact runs excluded because Gazebo's metric saturates once the vehicle is inside the rack)."""
import os, sys, numpy as np, itertools
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, "/home/dronesim/starling_sim/v2_wallkeep/scripts")
from wk3 import surrogate as S, controllers as C
import wk_report as R
L = "/home/dronesim/starling_sim/v2_wallkeep/logs/sweep_20261029"
G = []
for k in ("yaw", "drift", "step", "none"):
    for line in open(os.path.join(L, k + ".txt")):
        f, c, w, d = line.split()
        m = R.metrics(R.load_run(d))
        if min(m["rotor_clear_min_front"], m["rotor_clear_min_back"]) < 0:
            continue
        G.append((c, w, m["err_max_abs"]))
def sc_of(c, ta, sa, seed):
    return S.Scenario(heading_deg=int(c[3:]) / 10 if c.startswith("yaw") else 0, drift_cms=int(c[5:]) / 10 if c.startswith("drift") else 0,
                      step_cm=int(c[4:]) if c.startswith("step") else 0, step_at=22.7, seed=seed, tau_ekf=ta, sig_a=sa)
best = None
for ta, sa in itertools.product((0.3, 0.5, 0.8, 1.2), (0.0, 0.03, 0.06, 0.1)):
    err = []
    for c, w, g in G:
        sims = [S.run(sc_of(c, ta, sa, sd), C.make("estimator" if w == "on" else "none"))[0]["max_all"] for sd in range(3)]
        err.append(np.mean(sims) - g)
    rmse = float(np.sqrt(np.mean(np.square(err))))
    print("tau_ekf %.1f sig_a %.2f -> RMSE %.2f cm" % (ta, sa, rmse * 100), flush=True)
    if best is None or rmse < best[0]:
        best = (rmse, ta, sa)
print("best", best)
ta, sa = best[1], best[2]
print("%-8s %-3s  Gazebo  surrogate(mean of 3)" % ("cond", "wk"))
for c, w, g in G:
    sims = [S.run(sc_of(c, ta, sa, sd), C.make("estimator" if w == "on" else "none"))[0]["max_all"] for sd in range(3)]
    print("%-8s %-3s  %6.1f  %6.1f" % (c, w, g * 100, np.mean(sims) * 100))
