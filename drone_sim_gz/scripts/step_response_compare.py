#!/usr/bin/env python3
import sys, numpy as np
# Step response of horizontal moves from ulogs (real vs SITL): setpoint ramp speed, peak speed, overshoot, 90% time,
# settling (+-2 cm).  usage: python3 step_response_compare.py <a.ulg> <label> [<b.ulg> <label> ...]   (2026-10-28, log102 comparison)
from pyulog import ULog
def step_metrics(path, label, axis_hint=None):
    u = ULog(path, ['vehicle_local_position', 'trajectory_setpoint'])
    D = {d.name: d.data for d in u.data_list}
    lp = D['vehicle_local_position']; t = lp['timestamp'] * 1e-6
    ts = D['trajectory_setpoint']; st = ts['timestamp'] * 1e-6
    sp = np.hypot(lp['vx'], lp['vy'])
    mov = sp > 0.15; i = 0; out = []
    while i < len(mov):
        if mov[i]:
            j = i
            while j < len(mov) and mov[j]: j += 1
            if t[j - 1] - t[i] > 0.8: out.append((i, j - 1))
            i = j
        else: i += 1
    print("== %s (local_position %.0f Hz, setpoint log %.0f Hz)" % (label, (len(t) - 1) / (t[-1] - t[0]), (len(st) - 1) / (st[-1] - st[0])))
    for a, b in out[:4]:
        ax = 'y' if abs(lp['y'][b] - lp['y'][a]) > abs(lp['x'][b] - lp['x'][a]) else 'x'
        pos = lp[ax]; t0 = t[a] - 1.0
        w = (st >= t0) & (st <= t[b] + 4)
        spos = ts['position[%d]' % (0 if ax == 'x' else 1)][w]; stt = st[w]
        start = np.interp(t0, t, pos); target = spos[-1]; dist = target - start
        # setpoint ramp speed: slope of the setpoint while it moves
        dsp = np.diff(spos) / np.diff(stt); ramp = np.median(np.abs(dsp[np.abs(dsp) > 0.02])) if np.any(np.abs(dsp) > 0.02) else float('nan')
        win = (t >= t0) & (t <= t[b] + 4); tw = t[win]; pw = pos[win]
        sgn = np.sign(dist); prog = (pw - start) * sgn
        over = prog.max() - abs(dist)
        t90 = tw[np.argmax(prog >= 0.9 * abs(dist))] - tw[0] if np.any(prog >= 0.9 * abs(dist)) else float('nan')
        settled = np.where(np.abs(prog - abs(dist)) > 0.02)[0]
        ts_ = tw[settled[-1]] - tw[0] if len(settled) else 0.0
        print("  move %+.2f m on %s: setpoint ramp %.3f m/s | peak speed %.3f m/s | overshoot %+.3f m | 90%% at %.2f s | settled (+-2 cm) at %.2f s (from 1 s before motion)"
              % (dist, ax, ramp, sp[a:b + 1].max(), over, t90, ts_))
for k in range(1, len(sys.argv), 2):
    step_metrics(sys.argv[k], sys.argv[k + 1])
