"""Lateral wall-distance controllers (v3_marker, 2026-10-29). Python 3.6 + numpy 1.19 compatible: the SAME file runs
in the surrogate model, in the Gazebo flight layer and on the real VOXL2 (Ubuntu 18.04, Python 3.6.9).

Common interface
  c = make(name, **params);  c.reset()
  u = c.update(t, d_meas, x_est)        called at every depth frame (~15 Hz) and when the frame is missing (d_meas None)
    t       : time (s)
    d_meas  : measured body-centre distance to the rack face (m) or None (no valid plane / dropout / not facing)
    x_est   : PX4's lateral position estimate relative to the planned line (m, + = away from the face), map frame
    return u: lateral target-point offset (m, + = away from the face) that the flight layer adds to the plan
  d_ref     : wanted distance (aisle centre) - set at construction
Controllers
  none      C0: u = 0
  estimator C1: v2_wallkeep frame correction: u follows (x_est - x_true_measured) with time constant tau, rate/cap limits
  rule      C2: user's proposal: act only when the FILTERED distance error leaves a dead band for N frames, then move the
                target by k * error (step limited), applied as a ramp, then wait a cooldown before the next decision
  rule_rel  C2r: like rule but against the distance at the last decision ("closer than before -> move away")
  mlp       C4: learned policy (numpy MLP exported from training), optionally residual on top of rule
"""
import math

try:
    import numpy as np
except ImportError:          # the rule controllers do not need numpy
    np = None


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class Base(object):
    name = "base"

    def __init__(self, d_ref=0.375, **kw):
        self.d_ref = d_ref
        self.p = kw
        self.reset()

    def reset(self):
        self.u = 0.0
        self.t_last = None
        self.n_actions = 0

    def info(self):
        return {"u": self.u, "n_actions": self.n_actions}


class NoControl(Base):
    name = "none"

    def update(self, t, d_meas, x_est):
        return 0.0


class Estimator(Base):
    """C1 = v2_wallkeep wk_mission.py position channel (tau 0.3 s, 0.15 m/s, cap 0.35 m, innovation gate 0.12 m)."""
    name = "estimator"

    def update(self, t, d_meas, x_est):
        tau = self.p.get("tau", 0.3); rate = self.p.get("rate", 0.15); cap = self.p.get("cap", 0.35); gate = self.p.get("gate", 0.12)
        dt = 0.0 if self.t_last is None else max(0.0, min(0.5, t - self.t_last))
        self.t_last = t
        if d_meas is None or dt <= 0:
            return self.u
        x_true = d_meas - self.d_ref                 # measured lateral position (+ = away from the face)
        nu = x_true - (x_est - self.u)               # measured minus believed (corrected) position
        if abs(nu) > gate:
            return self.u
        step = _clamp(min(1.0, dt / tau) * nu, -rate * dt, rate * dt)
        # believed = x_est - u must become x_true  ->  u moves by -nu
        self.u = _clamp(self.u - step, -cap, cap)
        return self.u


class Rule(Base):
    """C2. params: db (dead band, m), n (frames over the band), k (gain), smax (max step, m), cool (s after an action
    ends), ramp (m/s, how fast the new offset is applied), win (median filter length, frames), cap (total offset, m),
    rel (False: error = filtered distance - d_ref; True: error = filtered distance - distance at the last decision)."""
    name = "rule"
    DEFAULT = dict(db=0.015, n=3, k=0.8, smax=0.05, cool=1.5, ramp=0.10, win=5, cap=0.35, rel=False)

    def reset(self):
        Base.reset(self)
        q = dict(self.DEFAULT); q.update(self.p); self.q = q
        self.buf = []
        self.over = 0
        self.goal = 0.0            # offset being ramped to
        self.busy_until = -1e9
        self.d_anchor = None

    def update(self, t, d_meas, x_est):
        q = self.q
        dt = 0.0 if self.t_last is None else max(0.0, min(0.5, t - self.t_last))
        self.t_last = t
        # ramp the applied offset toward the goal
        if self.u != self.goal:
            st = q["ramp"] * dt
            self.u = self.goal if abs(self.goal - self.u) <= st else self.u + st * (1 if self.goal > self.u else -1)
        if d_meas is None:
            return self.u
        self.buf.append(d_meas)
        w = int(max(1, round(q["win"])))
        if len(self.buf) > w:
            self.buf = self.buf[-w:]
        df = sorted(self.buf)[len(self.buf) // 2]
        if self.d_anchor is None:
            self.d_anchor = self.d_ref if not q["rel"] else df
        ref = self.d_anchor if q["rel"] else self.d_ref
        e = df - ref                                  # + = too far from the face
        if abs(e) > q["db"]:
            self.over += 1
        else:
            self.over = 0
        if self.u == self.goal and t >= self.busy_until and self.over >= int(round(q["n"])):
            # too far (e > 0) -> move the target toward the face (u decreases)
            step = _clamp(-q["k"] * e, -q["smax"], q["smax"])
            self.goal = _clamp(self.u + step, -q["cap"], q["cap"])
            self.busy_until = t + abs(self.goal - self.u) / max(q["ramp"], 1e-3) + q["cool"]
            self.over = 0
            self.n_actions += 1
            if q["rel"]:
                self.d_anchor = df + step             # expected distance after the move
        return self.u


class ObsBuilder(object):
    """observation vector shared by RL training (surrogate env) and deployment (MLPPolicy): same numbers both places"""
    def __init__(self, d_ref):
        self.d_ref = d_ref
        self.reset()

    def reset(self):
        self.buf = []; self.hist = []; self.t_valid = None

    def __call__(self, t, d_meas, x_est, u):
        if d_meas is not None:
            self.buf.append(d_meas); self.buf = self.buf[-5:]; self.t_valid = t
        df = sorted(self.buf)[len(self.buf) // 2] if self.buf else self.d_ref
        e = df - self.d_ref
        self.hist.append((t, e)); self.hist = [h for h in self.hist if t - h[0] <= 0.6]
        de = e - self.hist[0][1]
        since = 1.0 if self.t_valid is None else min(1.0, t - self.t_valid)
        return [e / 0.05, de / 0.05, u / 0.1, (x_est - u) / 0.1, since, 1.0 if d_meas is not None else 0.0]


class MLPPolicy(Base):
    """C4: numpy MLP exported from training (weights npz: W0,b0,W1,b1,...,obs_mean,obs_std,act_scale).
    Observation (see obs()): filtered error, its change over ~0.5 s, current offset, believed position, time since last
    valid frame, valid flag. Action: offset rate (m/s) in [-1,1]*act_scale. residual=True adds the action to a Rule."""
    name = "mlp"

    def __init__(self, d_ref=0.375, weights=None, residual=False, rule_params=None, **kw):
        self.wts = dict(np.load(weights)) if isinstance(weights, str) else weights
        self.residual = residual
        self.rule = Rule(d_ref, **(rule_params or {})) if residual else None
        Base.__init__(self, d_ref, **kw)

    def reset(self):
        Base.reset(self)
        self.ob = ObsBuilder(self.d_ref); self.u_own = 0.0
        if self.rule:
            self.rule.reset()

    def obs(self, t, d_meas, x_est):
        return np.array(self.ob(t, d_meas, x_est, self.u), dtype=np.float32)

    def act(self, o):
        h = (o - self.wts["obs_mean"]) / self.wts["obs_std"]
        i = 0
        while "W%d" % i in self.wts:
            h = h.dot(self.wts["W%d" % i]) + self.wts["b%d" % i]
            if "W%d" % (i + 1) in self.wts:
                h = np.tanh(h)
            i += 1
        return float(np.clip(h[0], -1.0, 1.0)) * float(self.wts["act_scale"])

    def update(self, t, d_meas, x_est):
        dt = 0.0 if self.t_last is None else max(0.0, min(0.5, t - self.t_last))
        self.t_last = t
        a = self.act(self.obs(t, d_meas, x_est))
        self.u_own = _clamp(self.u_own + a * dt, -0.35, 0.35)
        base = self.rule.update(t, d_meas, x_est) if self.rule else 0.0
        self.u = _clamp(base + self.u_own, -0.35, 0.35)
        return self.u


def make(name, d_ref=0.375, **kw):
    if name == "none":
        return NoControl(d_ref)
    if name == "estimator":
        return Estimator(d_ref, **kw)
    if name == "rule":
        return Rule(d_ref, **kw)
    if name == "rule_rel":
        kw = dict(kw); kw["rel"] = True
        return Rule(d_ref, **kw)
    if name == "mlp":
        return MLPPolicy(d_ref, **kw)
    raise ValueError(name)
