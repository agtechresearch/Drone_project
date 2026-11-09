#!/usr/bin/env python3
"""Fit the lateral response of PX4 (sim, real gains) to target-point steps: x'' = wn^2 (s(t - tau) - x) - 2 zeta wn x'.
  python sysid_fit.py <sysid log dir>   -> prints wn, zeta, tau, fit RMS per step and overall; writes fit.png, sysid.json"""
import sys, re, json, datetime, numpy as np
from scipy.optimize import least_squares
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.rcParams["font.family"] = "Noto Sans CJK JP"
d = sys.argv[1]
T = np.genfromtxt(d + "/truth.csv", delimiter=",", names=True)
day = datetime.datetime(2026, 10, 29)
steps = []
for line in open(d + "/mission.log", errors="ignore"):
    m = re.match(r"\[(\d\d):(\d\d):(\d\d\.\d+)\] \[goto (step\S+)\] \[START\].*target rel N([+-][\d.]+) E([+-][\d.]+)", line)
    if m:
        w = (day + datetime.timedelta(hours=int(m.group(1)), minutes=int(m.group(2)), seconds=float(m.group(3)))).timestamp()
        steps.append((float(np.interp(w, T["wall"], T["t_sim"])), float(m.group(6)), m.group(4)))
x0 = np.median(T["x"][(T["t_sim"] > steps[0][0] - 3) & (T["t_sim"] < steps[0][0])])
t = T["t_sim"]; x = T["x"] - x0
def setpoint(tt):
    s = np.zeros_like(tt)
    for ts, e, _ in steps:
        s[tt >= ts] = e
    return s
def simulate(p, tt):
    wn, ze, tau = p
    s = setpoint(tt - tau)
    xs = np.zeros_like(tt); v = 0.0; xc = 0.0
    for i in range(1, len(tt)):
        dt = tt[i] - tt[i - 1]
        for _ in range(4):
            a = wn * wn * (s[i] - xc) - 2 * ze * wn * v
            v += a * dt / 4; xc += v * dt / 4
        xs[i] = xc
    return xs
sel = (t > steps[0][0] - 1) & (t < steps[-1][0] + 7)
tt, xx = t[sel], x[sel]
r = least_squares(lambda p: simulate(p, tt) - xx, [3.0, 0.7, 0.05], bounds=([0.5, 0.1, 0.0], [20, 3, 0.5]))
wn, ze, tau = r.x; fit = simulate(r.x, tt)
out = {"wn": wn, "zeta": ze, "tau": tau, "rms_all_m": float(np.sqrt(np.mean((fit - xx) ** 2))), "steps": []}
for k, (ts, e, nm) in enumerate(steps):
    w = (tt >= ts) & (tt < ts + 6)
    seg = xx[w]; prev = steps[k - 1][1] if k else 0.0; amp = e - prev
    over = (np.max(seg) - e) if amp > 0 else (e - np.min(seg))
    i90 = np.argmax(np.abs(seg - prev) >= 0.9 * abs(amp)) if abs(amp) > 0 else 0
    out["steps"].append({"name": nm, "amp_m": amp, "overshoot_m": float(over), "t90_s": float(tt[w][i90] - ts),
                         "fit_rms_m": float(np.sqrt(np.mean((fit[w] - seg) ** 2)))})
json.dump(out, open(d + "/sysid.json", "w"), indent=1)
print(json.dumps(out, indent=1))
fig, ax = plt.subplots(figsize=(12, 4), dpi=100)
ax.plot(tt - tt[0], xx * 100, label="Gazebo 진실 위치"); ax.plot(tt - tt[0], fit * 100, "--", label="대리 모델 (wn %.2f, ζ %.2f, 지연 %.0f ms)" % (wn, ze, tau * 1000))
ax.step(tt - tt[0], setpoint(tt) * 100, where="post", color="#999", lw=1, label="목표점")
ax.set_xlabel("s"); ax.set_ylabel("동쪽 위치 (cm)"); ax.legend(); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(d + "/fit.png")
