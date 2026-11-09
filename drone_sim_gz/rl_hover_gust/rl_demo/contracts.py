"""Hover-gust experiment contract (2026-11-03, stage A of review/CODEX_PPO_HOVER_GUST_PLAN_2026-11-03.md).
Shared by the flight worker (starling-flight, Python 3.6), the gust scheduler (starling-gz, Python 3.10) and the evaluator
(host). Standard library only.

Axis (user 2026-11-03: "y축"): the disturbed / corrected axis is PX4 NED y (east). The vehicle spawns facing north
(gz yaw +90 deg), so NED y is the vehicle's LEFT-RIGHT axis = a cross wind. In the gz world (ENU) this is +x:
  PX4 NED (north, east, down) = (gz y, gz x, -gz z) for the spawn at the gz origin and zero VIO injection.
A positive force_n pushes the vehicle EAST (NED +y, gz +x).

Policy contract placeholders for stage B/C (not used by stage A, which always sends delta = 0):
  target sent = nominal + delta_y, |delta_y| <= delta_max_m, |delta_y(k) - delta_y(k-1)| <= delta_rate_m_s * dt.
"""
import json, math, random

AXIS = {"name": "ned_y_east", "ned_index": 1, "gz_force_dir": [1.0, 0.0, 0.0]}

REQUIRED = {
    "world": str, "instance": int, "link": str, "spawn_pose": str, "alt_m": float,
    "setpoint_hz": float, "window_s": float, "ready": dict, "gust": dict, "policy": dict, "pause_between_s": float,
}
GUST_KEYS = ("quiet_start_s", "quiet_end_s", "n_min", "n_max", "force_n", "dur_s", "min_gap_s")


def load_config(path):
    c = json.load(open(path))
    for k, t in REQUIRED.items():
        if k not in c:
            raise ValueError("config: missing %s" % k)
        if t is float and isinstance(c[k], int):
            c[k] = float(c[k])
        if not isinstance(c[k], t):
            raise ValueError("config: %s must be %s" % (k, t.__name__))
    g = c["gust"]
    for k in GUST_KEYS:
        if k not in g:
            raise ValueError("config: gust.%s missing" % k)
    if not (0 < g["force_n"][0] <= g["force_n"][1] <= 1.0):
        raise ValueError("config: gust.force_n must be 0 < lo <= hi <= 1.0 N")   # 1 N = 3.4 m/s^2 on 0.2975 kg
    if not (0 < g["dur_s"][0] <= g["dur_s"][1]):
        raise ValueError("config: gust.dur_s")
    if not (1 <= g["n_min"] <= g["n_max"]):
        raise ValueError("config: gust.n_min/n_max")
    # optional (2026-11-03 wide distribution): force_dist "uniform" (default) | "loguniform"; dur_mix [[weight, lo, hi], ...]
    # replaces dur_s for the duration draw (dur_s stays the overall [min, max] for the checks)
    if g.get("force_dist", "uniform") not in ("uniform", "loguniform"):
        raise ValueError("config: gust.force_dist must be uniform | loguniform")
    if "dur_mix" in g:
        m = g["dur_mix"]
        if not m or any(len(x) != 3 or x[0] <= 0 or not (g["dur_s"][0] <= x[1] <= x[2] <= g["dur_s"][1]) for x in m):
            raise ValueError("config: gust.dur_mix entries [weight > 0, lo, hi] inside dur_s")
    if g["quiet_start_s"] + g["dur_s"][1] + g["quiet_end_s"] > c["window_s"]:
        raise ValueError("config: window too short for one gust plus both quiet periods")
    if c["instance"] not in (2, 3, 4, 5):
        raise ValueError("config: instance must be 2..5")
    return c


def model_name(c):
    return "starling2_tof_%d" % c["instance"]


def link_scoped(c):
    return "%s::%s" % (model_name(c), c["link"])


def _dur(g, r):
    """one gust duration; without dur_mix exactly the original single draw r.uniform(dur_s) (same RNG sequence)"""
    if "dur_mix" not in g:
        return r.uniform(g["dur_s"][0], g["dur_s"][1])
    tot = sum(x[0] for x in g["dur_mix"]); u = r.random() * tot
    for w, lo, hi in g["dur_mix"]:
        if u < w:
            return r.uniform(lo, hi)
        u -= w
    return r.uniform(g["dur_mix"][-1][1], g["dur_mix"][-1][2])


def _force(g, r):
    """|F|; uniform (default: the original single draw) or log-uniform in force_n"""
    lo, hi = g["force_n"]
    if g.get("force_dist", "uniform") == "loguniform":
        return math.exp(r.uniform(math.log(lo), math.log(hi)))
    return r.uniform(lo, hi)


def gust_schedule(cfg, cond, seed):
    """Deterministic gust list for one episode, times relative to the window start (sim s).
    cond "none" -> []. cond "gust" -> n gusts in [quiet_start, window - quiet_end], non-overlapping with min_gap between
    the end of one and the start of the next. Each: {"t_on", "t_off", "force_n"} (signed, + = east)."""
    if cond == "none":
        return []
    if cond != "gust":
        raise ValueError("cond must be none|gust")
    g = cfg["gust"]; W = cfg["window_s"]
    r = random.Random(int(seed))
    lo_t, hi_t = g["quiet_start_s"], W - g["quiet_end_s"]
    n = g["n_min"] + int(math.floor(r.random() * (g["n_max"] - g["n_min"] + 1)))
    for n_try in range(n, 0, -1):
        for _ in range(200):
            durs = [_dur(g, r) for _ in range(n_try)]
            need = sum(durs) + g["min_gap_s"] * (n_try - 1)
            slack = (hi_t - lo_t) - need
            if slack < 0:
                break
            # split the slack into n_try + 1 random parts -> start times
            cuts = sorted(r.uniform(0, slack) for _ in range(n_try))
            out, t, prev = [], lo_t, 0.0
            for i in range(n_try):
                t += cuts[i] - prev; prev = cuts[i]
                f = _force(g, r) * (1 if r.random() < 0.5 else -1)
                out.append({"t_on": round(t, 4), "t_off": round(t + durs[i], 4), "force_n": round(f, 4)})
                t += durs[i] + g["min_gap_s"]
            return out
    raise ValueError("no gust schedule fits the window")


def parse_episodes(spec):
    """'none:0,gust:1,gust:2' -> [{"id": 0, "cond": "none", "seed": 0}, ...]"""
    eps = []
    for i, tok in enumerate(spec.split(",")):
        cond, seed = tok.split(":")
        if cond not in ("none", "gust"):
            raise ValueError("episode %d: cond %s" % (i, cond))
        eps.append({"id": i, "cond": cond, "seed": int(seed)})
    return eps
