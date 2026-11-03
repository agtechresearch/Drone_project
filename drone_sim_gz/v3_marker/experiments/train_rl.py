#!/usr/bin/env python3
"""C4: PPO on the surrogate (v3_marker, 2026-10-29).
  venv/bin/python experiments/train_rl.py pure [steps]       C4b: no rule, the policy sets the offset rate directly
  venv/bin/python experiments/train_rl.py residual [steps]   C4a: policy adds an offset rate on top of the optimised rule (C3)
Observation = wk3.controllers.ObsBuilder (same code as deployment). Action in [-1,1] x 0.15 m/s = offset rate.
Reward per depth frame: -(x_true/3 cm)^2 * dt - 0.3 * |du_own|/1 cm - 0.05 * |a - a_prev|, contact -50 (episode ends).
x_true (privileged) is used only in the reward. Exports results/policy_<variant>.npz for wk3.controllers.MLPPolicy (numpy).
"""
import json, os, sys, time
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from common import ROOT, S, C, train_set
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv

ACT = 0.15
FRAME = 1.0 / 15


def rule_params():
    p = os.path.join(ROOT, "results", "rule_opt_tof.json")
    return json.load(open(p))["best"] if os.path.exists(p) else {}


class WallEnv(gym.Env):
    def __init__(self, variant, seed=0):
        self.variant = variant
        self.rng = np.random.RandomState(seed)
        self.observation_space = spaces.Box(-10, 10, (6,), np.float32)
        self.action_space = spaces.Box(-1, 1, (1,), np.float32)
        self.rp = rule_params() if variant == "residual" else None

    def reset(self, seed=None, options=None):
        sc = S.random_scenario(self.rng)
        self.g = S.episode(sc)
        self.ob = C.ObsBuilder(S.D_REF)
        self.rule = C.Rule(S.D_REF, **self.rp) if self.rp is not None else None
        self.u_own = 0.0; self.u = 0.0; self.a_prev = 0.0
        self.t, self.d, self.xe, self.xt = next(self.g)
        return np.array(self.ob(self.t, self.d, self.xe, self.u), np.float32), {}

    def step(self, action):
        a = float(np.clip(action[0], -1, 1))
        du = a * ACT * FRAME
        self.u_own = float(np.clip(self.u_own + du, -0.35, 0.35))
        base = self.rule.update(self.t, self.d, self.xe) if self.rule else 0.0
        self.u = float(np.clip(base + self.u_own, -0.35, 0.35))
        try:
            self.t, self.d, self.xe, self.xt = self.g.send(self.u)
        except StopIteration:
            return np.zeros(6, np.float32), 0.0, False, True, {}
        r = -(self.xt / 0.03) ** 2 * FRAME - 0.3 * abs(du) / 0.01 - 0.05 * abs(a - self.a_prev)
        self.a_prev = a
        if abs(self.xt) >= S.CLEAR:
            return np.zeros(6, np.float32), r - 50.0, True, False, {}
        return np.array(self.ob(self.t, self.d, self.xe, self.u), np.float32), float(r), False, False, {}


def export(model, path):
    pn = model.policy.mlp_extractor.policy_net
    lin = [m for m in pn if hasattr(m, "weight")] + [model.policy.action_net]
    w = {}
    for i, m in enumerate(lin):
        w["W%d" % i] = m.weight.detach().cpu().numpy().T.astype(np.float64)
        w["b%d" % i] = m.bias.detach().cpu().numpy().astype(np.float64)
    w["obs_mean"] = np.zeros(6); w["obs_std"] = np.ones(6); w["act_scale"] = np.array(ACT)
    np.savez(path, **w)


if __name__ == "__main__":
    variant = sys.argv[1]; steps = int(sys.argv[2]) if len(sys.argv) > 2 else 2_000_000
    t0 = time.time()
    env = SubprocVecEnv([lambda i=i: WallEnv(variant, seed=1000 + i) for i in range(16)])
    model = PPO("MlpPolicy", env, n_steps=1024, batch_size=4096, learning_rate=3e-4, gamma=0.995, gae_lambda=0.95,
                ent_coef=0.0, clip_range=0.2, policy_kwargs=dict(net_arch=dict(pi=[64, 64], vf=[64, 64])), verbose=0, seed=3)
    log = []
    chunk = 200_000
    done = 0
    while done < steps:
        model.learn(chunk, reset_num_timesteps=False)
        done += chunk
        rew = [ep["r"] for ep in model.ep_info_buffer] if model.ep_info_buffer else [float("nan")]
        log.append({"steps": done, "ep_reward": float(np.mean(rew)), "sec": time.time() - t0})
        print("steps %d  mean episode reward %.1f  (%.0f s)" % (done, log[-1]["ep_reward"], log[-1]["sec"]), flush=True)
    export(model, os.path.join(ROOT, "results", "policy_%s.npz" % variant))
    model.save(os.path.join(ROOT, "results", "ppo_%s" % variant))
    json.dump({"variant": variant, "steps": steps, "log": log, "rule_params": rule_params() if variant == "residual" else None},
              open(os.path.join(ROOT, "results", "train_%s.json" % variant), "w"), indent=1)
    print("exported policy_%s.npz" % variant)
