"""Surrogate hover-gust environment (2026-11-04), Gymnasium API, same task contract as the Gazebo training:
  observation = rl_demo.policy_runtime.ObsBuilder (PX4 estimate e, ve, roll, previous delta, two past steps), 10 Hz
  action      = rl_demo.policy_runtime.apply_action (clip, *delta_max, rate limit)
  reward      = rl_demo.policy_runtime.reward_step from the TRUTH state (interval mean e, interval velocity)
  termination = |estimate e| > terminate.e_max_m -> terminal_penalty; window_s -> truncation (SB3 bootstraps)
  gusts       = rl_demo.contracts.gust_schedule(cfg, cond, seed) (cond none with p_none, as in make_plan)
Dynamics = linear ARX from surrogate/sysid.py (state [e_est, ve_est, roll, e_true, v_true]) + Gaussian residual noise.
Optional randomization (dict `rand`, all default off):
  gain_F   : force gain multiplier range, e.g. [0.8, 1.2]   (mass / drag / force uncertainty)
  gain_d   : multiplier on the delta input columns           (PX4 tracking response)
  noise_k  : multiplier on the residual noise                 (turbulence / sensor jitter)
  obs_noise: std of extra observation noise [e m, ve m/s, roll deg]
  obs_delay: max extra observation delay in policy steps (integer, uniform 0..n per episode)
  F_fluct  : relative std of a per-step force fluctuation (OU, 0.5 s correlation) while a gust is on
  F_ramp_s : max ramp-up / ramp-down time of each gust (uniform 0..x)"""
import math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE); sys.path.insert(0, ROOT)
import numpy as np, gymnasium as gym
from rl_demo.contracts import gust_schedule
from rl_demo.policy_runtime import OBS_DIM, POLICY_HZ, E_S, V_S, R_S, apply_action, reward_step

DT = 1.0 / POLICY_HZ


def load_model(path):
    z = np.load(path, allow_pickle=True)
    M = {"W": z["W"], "L": int(z["L"]), "resid_cov": z["resid_cov"], "noise_cal": 1.0, "F_cal": 1.0,
         "taus": list(z["slow_taus"]) if "slow_taus" in z else []}
    cal = path.replace(".npz", "_cal.json")
    if os.path.exists(cal):                              # gate_px4 calibration against the Gazebo PX4-alone batch
        import json; M.update(json.load(open(cal)))
    return M


def force_series(cfg, sched, n, rng, rand):
    """mean force over each 0.1 s step, with optional ramps and OU fluctuation"""
    t = (np.arange(n) + 0.5) * DT; F = np.zeros(n)
    ramp = rand.get("F_ramp_s", 0.0)
    for g in sched:
        r_on = rng.uniform(0, ramp) if ramp else 0.0; r_off = rng.uniform(0, ramp) if ramp else 0.0
        a = np.clip((t - g["t_on"]) / r_on, 0, 1) if r_on > 0 else (t >= g["t_on"]).astype(float)
        b = np.clip((g["t_off"] + r_off - t) / r_off, 0, 1) if r_off > 0 else (t < g["t_off"]).astype(float)
        F += g["force_n"] * np.minimum(a, b)
    fl = rand.get("F_fluct", 0.0)
    if fl:
        x = 0.0; ph = math.exp(-DT / 0.5); s = math.sqrt(1 - ph * ph); z = np.zeros(n)
        for i in range(n):
            x = ph * x + s * rng.standard_normal(); z[i] = x
        F = F * (1 + fl * z)
    return F


class HoverGustSurrogate(gym.Env):
    observation_space = gym.spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32)
    action_space = gym.spaces.Box(-1.0, 1.0, (1,), np.float32)

    def __init__(self, cfg, model, init_states, p_none=0.2, rand=None, fixed=None):
        """init_states: list of (L, 5) arrays (first L states of real episodes). fixed: optional list of (cond, seed) episodes
        played in order (evaluation); otherwise random cond/seed per reset."""
        self.cfg = cfg; self.M = model; self.L = model["L"]; self.init = init_states; self.p_none = p_none
        self.rand = rand or {}; self.fixed = fixed; self.k_fixed = 0
        self.n = int(round(cfg["window_s"] * POLICY_HZ)); P = cfg["policy"]; self.dm, self.rate = P["delta_max_m"], P["delta_rate_m_s"]
        self.e_max = cfg.get("terminate", {}).get("e_max_m"); self.pen = cfg["reward"]["terminal_penalty"]
        self.Lc = np.linalg.cholesky(model["resid_cov"] + 1e-12 * np.eye(5)); W = model["W"]; L = self.L
        self.WS = W[:5 * L]; self.WD = W[5 * L:6 * L]; self.WF = W[6 * L:7 * L]
        nz = 3 * len(model["taus"]); self.WZ = W[7 * L:7 * L + nz]; self.Wc = W[7 * L + nz]
        self.za = np.array([math.exp(-DT / t) for t in model["taus"]] * 3)    # filters of [F, delta, e_true] (sysid2.py)
        self.rng = np.random.default_rng()

    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        r = self.rng; R = self.rand
        if self.fixed is not None:
            cond, gseed = self.fixed[self.k_fixed % len(self.fixed)]; self.k_fixed += 1
        else:
            cond = "none" if r.random() < self.p_none else "gust"; gseed = int(r.integers(1 << 30))
        self.cond, self.gseed = cond, gseed
        self.sched = gust_schedule(self.cfg, cond, gseed)
        self.F = force_series(self.cfg, self.sched, self.n + 1, r, R)
        self.gF = r.uniform(*R["gain_F"]) if "gain_F" in R else 1.0
        self.gD = r.uniform(*R["gain_d"]) if "gain_d" in R else 1.0
        self.nk = r.uniform(*R["noise_k"]) if "noise_k" in R else 1.0
        self.delay = int(r.integers(0, R.get("obs_delay", 0) + 1))
        S0 = self.init[int(r.integers(len(self.init)))]
        self.S = [s.copy() for s in S0[::-1]]           # S[0] = newest
        self.D = [0.0] * self.L; self.Fh = [0.0] * self.L
        self.j = 0; self.delta = 0.0; self.hist = None; self.obs_q = []
        nt = len(self.M["taus"]); self.z = np.concatenate([np.zeros(2 * nt), np.full(nt, float(np.mean(S0[:, 3])))])
        return self._obs(), {}

    def _measure(self):
        s = self.S[0]; e, ve, roll = s[0], s[1], s[2]
        on = np.asarray(self.rand.get("obs_noise", [0, 0, 0]))
        if on.any():
            e, ve, roll = e + on[0] * self.rng.standard_normal(), ve + on[1] * self.rng.standard_normal(), roll + on[2] * self.rng.standard_normal()
        self.obs_q.append((e, ve, roll)); self.obs_q = self.obs_q[-(self.delay + 1):]
        return self.obs_q[0]

    def _obs(self):
        e, ve, roll = self._measure()
        if self.hist is None:
            self.hist = [(e, ve), (e, ve)]
        h1, h2 = self.hist[-1], self.hist[-2]
        o = [e / E_S, ve / V_S, roll / R_S, self.delta / self.dm, h1[0] / E_S, h1[1] / V_S, h2[0] / E_S, h2[1] / V_S]
        self.hist = [h1, (e, ve)]
        self.e_obs = e
        return np.array(o, dtype=np.float32)

    def step(self, a):
        a_raw = float(np.asarray(a).reshape(-1)[0]); d_prev = self.delta
        _, self.delta = apply_action(a_raw, d_prev, self.dm, self.rate)
        self.D = [self.delta] + self.D[:-1]; self.Fh = [self.M["F_cal"] * self.gF * self.F[self.j]] + self.Fh[:-1]
        x = np.concatenate(self.S); nt = len(self.M["taus"])
        if nt:
            src = np.repeat([self.Fh[0], self.gD * self.delta, self.S[0][3]], nt); self.z = self.za * self.z + (1 - self.za) * src
        s_new = x @ self.WS + (self.gD * np.array(self.D)) @ self.WD + np.array(self.Fh) @ self.WF + self.z @ self.WZ + self.Wc \
            + self.M["noise_cal"] * self.nk * (self.Lc @ self.rng.standard_normal(5))
        e0 = self.S[0][3]; self.S = [s_new] + self.S[:-1]; e1 = s_new[3]
        rew = reward_step(0.5 * (e0 + e1), (e1 - e0) / DT, self.delta, self.delta - d_prev, self.cfg)
        self.j += 1; obs = self._obs()
        term = bool(self.e_max is not None and abs(self.e_obs) > self.e_max)
        if term:
            rew += self.pen
        trunc = (not term) and self.j >= self.n
        return obs, float(rew), term, trunc, {"e_true": float(e1), "delta": self.delta, "F": float(self.Fh[0])}
