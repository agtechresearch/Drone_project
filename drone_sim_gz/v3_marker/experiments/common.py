"""shared experiment settings (v3_marker, 2026-10-29)"""
import os, sys, numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
from wk3 import surrogate as S, controllers as C  # noqa: E402

RULE_KEYS = ["db", "n", "k", "smax", "cool", "ramp", "win"]
RULE_BOUNDS = [(0.0, 0.03), (1, 8), (0.2, 1.2), (0.01, 0.10), (0.0, 3.0), (0.03, 0.40), (1, 9)]


def train_set(n=48, seed=12345, sensing="tof"):
    rng = np.random.RandomState(seed)
    out = []
    for _ in range(n):
        sc = S.random_scenario(rng); sc.sensing = sensing
        out.append(sc)
    return out


def cost(ms):
    """lower is better: accuracy first (max and RMS error, contact), then over-reaction (corrections, reversals)"""
    mx = np.mean([m["max_all"] for m in ms]); rms = np.mean([m["rms"] for m in ms])
    con = np.mean([m["contact"] for m in ms]); nc = np.mean([m["n_corr"] for m in ms]); rev = np.mean([m["reversals"] for m in ms])
    return mx / 0.03 + rms / 0.01 + 20.0 * con + 0.5 * nc / 100.0 + 0.5 * rev / 10.0


def run_set(make_ctrl, scs):
    return [S.run(sc, make_ctrl())[0] for sc in scs]
