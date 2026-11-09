"""Rack-plane fit used by wall_sense.py (and tof_static_sweep.py). See wall_sense.py for the method."""
import math
import numpy as np

HFOV = 1.8588; SENSOR_X = 0.075; TOL = 0.02
rng = np.random.default_rng(1)


def fit(depth, hfov=HFOV, sensor_x=SENSOR_X, sensor_y=0.0):
    """sensor_x, sensor_y: sensor position ahead of / LEFT of the body origin (m)"""
    H, W = depth.shape
    fx = (W / 2.0) / math.tan(hfov / 2.0)
    cx = (W - 1) / 2.0
    d = np.where(np.isfinite(depth) & (depth > 0.1) & (depth < 5.9), depth, np.inf)
    # per column: the front surface = samples within 2.5 cm of the 3rd nearest; its MEDIAN is the face depth.
    # (2026-10-29: the 3rd nearest alone read 0.95 cm short on every face - an order statistic of ~100 noisy samples, sigma 5 mm)
    srt = np.sort(d, axis=0)
    m3 = srt[2]
    front = np.where(srt <= m3 + 0.025, srt, np.nan)
    with np.errstate(all="ignore"):
        xs = np.where(np.isfinite(m3), np.nanmedian(np.where(np.isfinite(front), front, np.nan), axis=0), np.inf)
    us = np.arange(W)
    ok = np.isfinite(xs)
    X = xs[ok]; Y = -X * (us[ok] - cx) / fx
    if len(X) < 20:
        return None, (X, Y, ok, None)
    best = None
    for _ in range(60):
        i, j = rng.choice(len(X), 2, replace=False)
        if abs(Y[i] - Y[j]) < 0.05:
            continue
        b = (X[j] - X[i]) / (Y[j] - Y[i]); a = X[i] - b * Y[i]
        res = np.abs(X - (a + b * Y)) / math.sqrt(1 + b * b)
        inl = res < TOL
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 15:
        return None, (X, Y, ok, None)
    A = np.vstack([np.ones(best.sum()), Y[best]]).T
    (a, b), *_ = np.linalg.lstsq(A, X[best], rcond=None)
    res = X[best] - (a + b * Y[best])
    alpha = math.atan(b)
    d_s = a / math.sqrt(1 + b * b)
    return {"d": d_s + sensor_x * math.cos(alpha) - sensor_y * math.sin(alpha), "alpha": math.degrees(alpha), "conf": float(best.sum()) / len(X),
            "span": float(Y[best].max() - Y[best].min()), "n": int(best.sum()), "rms": float(np.sqrt(np.mean(res ** 2)))}, (X, Y, ok, best)
