"""Policy runtime for the hover-gust experiment (2026-11-03, stage B/C). Python 3.6, standard library only, so the same code
runs in starling-flight and on the VOXL. Exactly the SB3 2.3.2 ActorCriticPolicy used by experiments/train_hover_gust.py:
  net_arch pi [64, 64] / vf [64, 64], Tanh, Gaussian action with a state-independent log_std, 1 action, OBS_DIM inputs.
Weights come as JSON exported by the trainer ({"version", "obs_dim", layers ..., "log_std"}). tests/test_policy_runtime.py
checks mean / value / log-prob against PyTorch.

Observation (all from the PX4 estimate, nothing from the simulator truth), at the policy rate (POLICY_HZ):
  [e_err/E_S, ve/V_S, roll/R_S, delta_prev/delta_max, e_err(-1)/E_S, ve(-1)/V_S, e_err(-2)/E_S, ve(-2)/V_S]
  e_err = east estimate - nominal east (m), ve = east velocity (m/s), roll (deg; vehicle faces north, so roll is the tilt
  about the north axis), (-1)/(-2) = the previous two policy steps (the first step repeats the current values).
Action: a_raw ~ N(mean, std) (training) or mean (deterministic); a = clip(a_raw, -1, 1); delta target = a * delta_max, then
rate-limited to delta_rate_m_s / POLICY_HZ per step. The buffer stores a_raw (SB3 convention: clipping only in the env).
Velocity baseline: delta target = clip(-k * ve / delta_max) * delta_max, same rate limit (same action limits as the policy)."""
import json, math, random

POLICY_HZ = 10.0
E_S, V_S, R_S = 0.1, 0.3, 10.0
OBS_DIM = 8
LOG2PI = math.log(2 * math.pi)


def _lin(W, b, x):
    return [sum(w * v for w, v in zip(row, x)) + bb for row, bb in zip(W, b)]


def _tanh(x):
    return [math.tanh(v) for v in x]


class MLPPolicy(object):
    def __init__(self, path_or_dict):
        d = json.load(open(path_or_dict)) if isinstance(path_or_dict, str) else path_or_dict
        self.version = d["version"]; self.d = d
        if d["obs_dim"] != OBS_DIM:
            raise ValueError("policy obs_dim %s != %d" % (d["obs_dim"], OBS_DIM))

    def forward(self, obs):
        d = self.d
        h = _tanh(_lin(d["pi0_w"], d["pi0_b"], obs)); h = _tanh(_lin(d["pi2_w"], d["pi2_b"], h))
        mean = _lin(d["act_w"], d["act_b"], h)[0]
        v = _tanh(_lin(d["vf0_w"], d["vf0_b"], obs)); v = _tanh(_lin(d["vf2_w"], d["vf2_b"], v))
        value = _lin(d["val_w"], d["val_b"], v)[0]
        return mean, value

    def log_prob(self, mean, a_raw):
        ls = self.d["log_std"][0]
        return -0.5 * ((a_raw - mean) / math.exp(ls)) ** 2 - ls - 0.5 * LOG2PI

    def act(self, obs, rng=None, deterministic=False):
        mean, value = self.forward(obs)
        a_raw = mean if deterministic else rng.gauss(mean, math.exp(self.d["log_std"][0]))
        return a_raw, self.log_prob(mean, a_raw), value, mean


class ObsBuilder(object):
    def __init__(self, delta_max):
        self.dm = delta_max if delta_max > 0 else 1.0; self.hist = []

    def build(self, e_err, ve, roll_deg, delta_prev):
        if not self.hist:
            self.hist = [(e_err, ve), (e_err, ve)]
        h1, h2 = self.hist[-1], self.hist[-2]
        obs = [e_err / E_S, ve / V_S, roll_deg / R_S, delta_prev / self.dm, h1[0] / E_S, h1[1] / V_S, h2[0] / E_S, h2[1] / V_S]
        self.hist = [h1, (e_err, ve)]
        return obs


def apply_action(a_raw, delta_prev, delta_max, delta_rate_m_s):
    a = max(-1.0, min(1.0, a_raw)); target = a * delta_max; step = delta_rate_m_s / POLICY_HZ
    return a, max(delta_prev - step, min(delta_prev + step, target))


def reward_step(e_true, ve_true, delta, ddelta, cfg, dt=None):
    """One policy step, from the simulator TRUTH (computed by the trainer after landing, never in the policy). dt = the actual
    sim length of the transition (rl_demo/hover_metrics.py); default 1/POLICY_HZ. The cost is integrated over dt."""
    R = cfg["reward"]; dm = cfg["policy"]["delta_max_m"]; step = cfg["policy"]["delta_rate_m_s"] / POLICY_HZ
    c = (e_true / R["e_scale_m"]) ** 2 + R["w_v"] * (ve_true / R["v_scale_m_s"]) ** 2 + R["w_delta"] * (delta / dm) ** 2 \
        + R["w_ddelta"] * (ddelta / step) ** 2
    return -c * (1.0 / POLICY_HZ if dt is None else dt)
