#!/usr/bin/env python3
"""PPO on the surrogate (2026-11-04). Same policy / PPO settings as experiments/train_hover_gust.py (net 64x64 Tanh, log_std -2,
lr 3e-4, gamma 0.99, lambda 0.95, clip 0.2, 10 epochs, batch 200), many parallel surrogate episodes.
  venv/bin/python surrogate/train_sur.py --out surrogate/runs/X --steps 2000000 --seed 1 [--bc 1.5,0.3] [--rand default]
--bc kp,kd : imitation warm start from the PD expert (DAgger-style data: expert + exploration noise, label = expert action),
             value net fitted to the expert's discounted returns, then PPO.
--rand     : 'default' = RAND below, or a JSON dict (surrogate/env.py)
Outputs: model_final.zip, policy_final.json (worker format), eval.csv (deterministic eval every --eval-every steps)."""
import argparse, json, os, sys, time, csv
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE); sys.path.insert(0, ROOT); sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "experiments"))
import numpy as np, torch
torch.set_num_threads(1)
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import configure
from sur_common import make_env, run_eval, pd_act
from train_hover_gust import export_policy

RAND = {"gain_F": [0.7, 1.3], "gain_d": [0.8, 1.2], "noise_k": [0.5, 2.0], "obs_noise": [0.003, 0.01, 0.3], "obs_delay": 1,
        "F_fluct": 0.2, "F_ramp_s": 1.0}


def det_act(model):
    return lambda o: float(model.predict(o, deterministic=True)[0][0])


class EvalCB(BaseCallback):
    def __init__(self, out, every, rand):
        super().__init__(); self.out = out; self.every = every; self.next = 0; self.rand = rand
        self.f = open(os.path.join(out, "eval.csv"), "w"); self.w = None; self.t0 = time.time()

    def do(self):
        r = run_eval(det_act(self.model), n=30, reps=1); r2 = run_eval(det_act(self.model), n=30, reps=1, rand=RAND, seed=1)
        row = {"steps": self.num_timesteps, "wall_s": round(time.time() - self.t0, 1), **r,
               **{"rand_" + k: v for k, v in r2.items() if k in ("return", "peak_per_F", "postclear_per_F", "still_rms_cm", "terminated")},
               "std": float(torch.exp(self.model.policy.log_std).mean())}
        if self.w is None:
            self.w = csv.DictWriter(self.f, list(row)); self.w.writeheader()
        self.w.writerow(row); self.f.flush()
        print("%8d %6.0fs ret %.2f pf %.3f pc %.3f rms %.2f | rand ret %.2f pf %.3f term %d | std %.3f" % (
            row["steps"], row["wall_s"], r["return"], r["peak_per_F"], r["postclear_per_F"], r["still_rms_cm"], r2["return"],
            r2["peak_per_F"], r2["terminated"], row["std"]), flush=True)

    def _on_step(self):
        if self.num_timesteps >= self.next:
            self.do(); self.next += self.every
        return True


def bc_warm_start(model, kp, kd, rand, seed, episodes=240, epochs=30):
    env = make_env(rand=rand); env.reset(seed=seed); rng = np.random.default_rng(seed); expert = pd_act(kp, kd)
    O, A, G = [], [], []
    for ep in range(episodes):
        o, _ = env.reset(); rews = []; sig = [0.0, 0.1, 0.3][ep % 3]
        while True:
            a_e = expert(o); O.append(o); A.append(np.clip(a_e, -1, 1))
            o, r, te, tr, _ = env.step(np.array([a_e + sig * rng.standard_normal()])); rews.append(r)
            if te or tr: break
        g = 0.0; ret = []
        for r in rews[::-1]:
            g = r + 0.99 * g; ret.append(g)
        G += ret[::-1]
    O = torch.tensor(np.array(O), dtype=torch.float32); A = torch.tensor(np.array(A), dtype=torch.float32)[:, None]
    G = torch.tensor(np.array(G), dtype=torch.float32)[:, None]; pol = model.policy
    opt = torch.optim.Adam([p for n, p in pol.named_parameters() if n != "log_std"], lr=1e-3); n = len(O)
    for e in range(epochs):
        perm = torch.randperm(n); la = lv = 0.0
        for i in range(0, n, 512):
            idx = perm[i:i + 512]; f = pol.extract_features(O[idx]); lp, lvv = pol.mlp_extractor(f)
            l_a = ((pol.action_net(lp) - A[idx]) ** 2).mean(); l_v = ((pol.value_net(lvv) - G[idx]) ** 2).mean()
            opt.zero_grad(); (l_a + 0.01 * l_v).backward(); opt.step(); la += float(l_a) * len(idx); lv += float(l_v) * len(idx)
        if e % 10 == 9 or e == epochs - 1:
            print("bc epoch %d: action mse %.5f value mse %.3f (n %d)" % (e + 1, la / n, lv / n, n), flush=True)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); ap.add_argument("--steps", type=int, default=2000000)
    ap.add_argument("--seed", type=int, default=1); ap.add_argument("--n-envs", type=int, default=8); ap.add_argument("--bc")
    ap.add_argument("--rand"); ap.add_argument("--eval-every", type=int, default=100000); ap.add_argument("--init-model")
    ap.add_argument("--lr", type=float, default=3e-4)
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    rand = None if not a.rand else (RAND if a.rand == "default" else json.loads(a.rand))
    json.dump({**vars(a), "rand_used": rand, "model": "surrogate/model_L5.npz + model_L5_cal.json"}, open(os.path.join(a.out, "args.json"), "w"), indent=1)
    torch.manual_seed(a.seed); np.random.seed(a.seed)
    venv = DummyVecEnv([lambda i=i: make_env(rand=rand) for i in range(a.n_envs)]); venv.seed(a.seed)
    model = PPO("MlpPolicy", venv, n_steps=400, batch_size=200, n_epochs=10, learning_rate=a.lr, gamma=0.99, gae_lambda=0.95,
                clip_range=0.2, ent_coef=0.0, seed=a.seed, device="cpu",
                policy_kwargs={"net_arch": {"pi": [64, 64], "vf": [64, 64]}, "activation_fn": torch.nn.Tanh, "log_std_init": -2.0})
    if a.init_model:
        model.set_parameters(a.init_model, device="cpu")
    if a.bc:
        kp, kd = map(float, a.bc.split(",")); bc_warm_start(model, kp, kd, rand, a.seed)
        model.save(os.path.join(a.out, "model_bc.zip")); export_policy(model, os.path.join(a.out, "policy_bc.json"), "bc")
    model.set_logger(configure(os.path.join(a.out, "sb3_log"), ["csv"]))
    model.learn(a.steps, callback=EvalCB(a.out, a.eval_every, rand))
    model.save(os.path.join(a.out, "model_final.zip")); export_policy(model, os.path.join(a.out, "policy_final.json"), "sur_final")
    print("done", flush=True)


if __name__ == "__main__":
    main()
