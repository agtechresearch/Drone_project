"""shared surrogate helpers: model / init states / eval set / metrics / PD expert"""
import os, sys, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE); sys.path.insert(0, ROOT); sys.path.insert(0, HERE)
from env import HoverGustSurrogate, load_model
from rl_demo.contracts import load_config
MODEL = os.path.join(HERE, "model_L5.npz"); CFG = os.path.join(ROOT, "config/hover_gust_rl_wide.json")
KEYS = ["e_est", "ve_est", "roll", "e_true", "v_true"]

def init_states(L):
    D = np.load(os.path.join(HERE, "data/ds_train.npy"), allow_pickle=True)
    return [np.stack([d[k][:L] for k in KEYS], 1) for d in D]

def make_env(rand=None, fixed=None, model=MODEL, cfg=None):
    M = load_model(model); cfg = cfg or load_config(CFG)
    return HoverGustSurrogate(cfg, M, init_states(M["L"]), rand=rand, fixed=fixed)

def eval_set(n=60, base=900000):
    """fixed evaluation episodes: 20 % none, rest gust; seeds disjoint from training (random 30-bit draws) by the offset"""
    r = np.random.default_rng(7); return [("none" if r.random() < 0.2 else "gust", base + i) for i in range(n)]

def metrics(es, sched):
    t = (np.arange(len(es)) + 1) * 0.1; pf, pc, ret = [], [], []
    for g in sched:
        if abs(g["force_n"]) < 0.15 or g["t_off"] + 5 > t[-1]: continue
        m = (t >= g["t_on"]) & (t <= g["t_off"] + 5); pf.append(np.max(np.abs(es[m])) / abs(g["force_n"]))
        pc.append(np.mean(np.abs(es[(t >= g["t_off"]) & (t <= g["t_off"] + 3)])) / abs(g["force_n"]))
        for i in np.flatnonzero(t >= g["t_off"]):
            w = (t >= t[i]) & (t <= t[i] + 1.0)
            if t[-1] >= t[i] + 1 and np.all(np.abs(es[w]) < 0.02): ret.append(t[i] - g["t_off"]); break
    return pf, pc, ret

def run_eval(act, n=60, reps=2, rand=None, seed=0):
    """act(obs) -> a_raw. Returns summary dict (deterministic controller, surrogate noise on)."""
    env = make_env(rand=rand, fixed=eval_set(n)); R = {"pf": [], "pc": [], "ret": [], "rms": [], "ret_sum": [], "term": 0, "eps": 0}
    env.reset(seed=seed); env.k_fixed = 0
    for _ in range(n * reps):
        o, _ = env.reset(); es = []; tot = 0.0
        while True:
            o, r, te, tr, info = env.step(np.array([act(o)])); es.append(info["e_true"]); tot += r
            if te or tr: break
        es = np.array(es); R["ret_sum"].append(tot); R["eps"] += 1; R["term"] += int(te)
        if env.cond == "none": R["rms"].append(np.std(es))
        else:
            a, b, c = metrics(es, env.sched); R["pf"] += a; R["pc"] += b; R["ret"] += c
    return {"return": float(np.mean(R["ret_sum"])), "peak_per_F": float(np.mean(R["pf"])), "postclear_per_F": float(np.mean(R["pc"])),
            "return_s_med": float(np.median(R["ret"])) if R["ret"] else None, "returned_frac": len(R["ret"]) / max(1, len(R["pf"])),
            "still_rms_cm": float(100 * np.median(R["rms"])), "terminated": R["term"], "episodes": R["eps"]}

def pd_act(kp, kd, dm=0.2):
    """PD expert on the observation (PX4 estimate): delta target = -kp e - kd ve, as a_raw (clip / rate limit in the env)"""
    return lambda o: (-kp * o[0] * 0.1 - kd * o[1] * 0.3) / dm
