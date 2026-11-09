#!/usr/bin/env python3
"""Surrogate step 1b (2026-11-04): linear ARX fit on the 10 Hz decision grid.
state s_j = [e_est, ve_est, roll, e_true, v_true]; s_{j+1} = sum_l A_l s_{j-l} + B_l delta_{j-l} + C_l F_{j-l} + c + w_j.
Usage: venv/bin/python surrogate/sysid.py --lags 3 [--save surrogate/model_L3.npz]"""
import argparse, json, numpy as np
KEYS = ["e_est", "ve_est", "roll", "e_true", "v_true"]

def mats(d):
    return np.stack([d[k] for k in KEYS], 1), d["delta"], d["F"]

def regress(S, dl, F, L):
    X, Y = [], []
    for j in range(L - 1, len(S) - 1):
        X.append(np.concatenate([S[j - l] for l in range(L)] + [[dl[j - l] for l in range(L)], [F[j - l] for l in range(L)], [1.0]]))
        Y.append(S[j + 1])
    return np.array(X), np.array(Y)

def fit(D, L, ridge=1e-6):
    XS, YS = zip(*[regress(*mats(d), L) for d in D]); X = np.concatenate(XS); Y = np.concatenate(YS)
    sc = np.maximum(X.std(0), 1e-9); sc[-1] = 1.0; Xs = X / sc
    W = np.linalg.solve(Xs.T @ Xs + ridge * np.eye(X.shape[1]), Xs.T @ Y) / sc[:, None]
    R = Y - X @ W
    return {"W": W, "L": L, "resid_cov": np.cov(R.T), "resid_std": R.std(0)}

def rollout(M, S0, dl, F, noise=None, rng=None):
    """free run: first L states from data, then only delta / F inputs"""
    L, W = M["L"], M["W"]; n = len(dl); S = np.zeros((n, 5)); S[:L] = S0[:L]
    Lc = np.linalg.cholesky(M["resid_cov"] + 1e-12 * np.eye(5)) if noise else None
    for j in range(L - 1, n - 1):
        x = np.concatenate([S[j - l] for l in range(L)] + [[dl[j - l] for l in range(L)], [F[j - l] for l in range(L)], [1.0]])
        S[j + 1] = x @ W + (Lc @ rng.standard_normal(5) if noise else 0)
    return S

def evaluate(M, D):
    out = {"one_step_r2": {}, "free": {}}
    XS, YS = zip(*[regress(*mats(d), M["L"]) for d in D]); X = np.concatenate(XS); Y = np.concatenate(YS)
    R = Y - X @ M["W"]
    for i, k in enumerate(KEYS):
        out["one_step_r2"][k] = float(1 - R[:, i].var() / Y[:, i].var())
    # free run over whole episodes (open loop in the recorded delta / F)
    eg, en, sg = [], [], []
    for d in D:
        S, dl, F = mats(d); P = rollout(M, S, dl, F)
        err = P[:, 3] - S[:, 3]; (eg if d["cond"] == "gust" else en).append(np.sqrt(np.mean(err ** 2)))
        if d["cond"] == "gust": sg.append(np.sqrt(np.mean(S[:, 3] ** 2)))
    out["free"] = {"gust_rmse_cm": float(np.median(eg) * 100), "gust_signal_rms_cm": float(np.median(sg) * 100),
                   "gust_rel": float(np.median(np.array(eg) / np.array(sg))), "none_rmse_cm": float(np.median(en) * 100)}
    return out

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--lags", type=int, default=3); ap.add_argument("--save")
    a = ap.parse_args()
    T = list(np.load("surrogate/data/ds_train.npy", allow_pickle=True)); V = list(np.load("surrogate/data/ds_val.npy", allow_pickle=True))
    M = fit(T, a.lags); ev = {"train": evaluate(M, T), "val": evaluate(M, V)}
    print(json.dumps(ev, indent=1)); print("resid std", M["resid_std"])
    if a.save:
        np.savez(a.save, W=M["W"], L=M["L"], resid_cov=M["resid_cov"], resid_std=M["resid_std"], eval=json.dumps(ev))
