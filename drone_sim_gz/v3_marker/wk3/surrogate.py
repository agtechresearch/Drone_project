"""Fast lateral surrogate of a C-face survey flight (v3_marker, 2026-10-29), for controller comparison, optimisation and RL.
One flight = takeoff + turn (no rack in view), leg 1 (8 m, 0.45 m/s ramp), climb at the far end, leg 2 back, hover.
Lateral axis only (x, + = away from the face, 0 = aisle centre). Everything below is identified or measured:
  PX4 response to the target point: x_est'' = wn^2 (u(t - tau) - x_est) - 2 zeta wn x_est'   (Gazebo step test
      logs/sysid_212519: wn 3.23 rad/s, zeta 0.385, tau 0.05 s, fit RMS 1.1 cm)
  VIO error (x_est - x_true): heading theta -> sin(theta) * along-track distance from the VIO origin, drift rate from
      airborne, step at a time, initial offset (marker localisation residual), slow random walk
  ToF: d = d_ref + x_true + bias + noise (sigma 0.15 cm, Gazebo), 15 Hz, 67 ms latency, dropout, only while facing the
      rack at survey height inside the rack span (y 0.5 .. 9.0 model)
  gusts: random lateral acceleration pulses
Contact: |x_true| >= 0.249 m (C aisle: 0.375 - 0.126 rotor reach).
"""
import math
import numpy as np

WN, ZETA, TAU = 3.2276, 0.3852, 0.05
TAU_EKF = 1.0      # s, calibrated by calibrate_surrogate.py (grid best 1.2, RMSE 1.56 cm vs 18 Gazebo runs; 1.0 used)
SIG_A = 0.03       # m/s^2, calibrated by calibrate_surrogate.py
D_REF = 0.375
CLEAR = 0.375 - 0.126
POSTS_Y = (0.6, 4.6, 4.9, 8.9)
DT = 0.02


class Scenario(object):
    def __init__(self, heading_deg=0.0, drift_cms=0.0, step_cm=0.0, step_at=20.0, noise=1.0, bias_cm=0.0, dropout=0.0,
                 init_off_cm=0.0, gust_rate=0.0, gust_ms2=0.0, rw_cm=0.02, seed=0, name="", tau_ekf=None, sig_a=None, sensing="tof"):
        self.__dict__.update(locals()); del self.__dict__["self"]


def timeline():
    """phases (start, end, kind, y_from, y_to), time from the arm command, measured on Gazebo C-face runs (2026-10-29):
    takeoff 2.7 s, turn done 13.5 s, leg 1 13.5-33.3 (8 m ramp 17.8 s + reach wait), stage gap, climb 35.3-37.5, gap,
    leg 2 39.6-60.0, preland descent (sensor invalid below 0.45 m after ~1 s), landed ~72 s.
    y = model y along the aisle (row 9 centre 8.6 -> row 1 centre 0.6)"""
    return [(0.0, 13.5, "turn", 8.6, 8.6), (13.5, 33.3, "leg1", 8.6, 0.6), (33.3, 35.3, "hold", 0.6, 0.6),
            (35.3, 37.5, "climb", 0.6, 0.6), (37.5, 39.6, "hold", 0.6, 0.6), (39.6, 60.0, "leg2", 0.6, 8.6),
            (60.0, 61.0, "preland", 8.6, 8.6), (61.0, 72.0, "end", 8.6, 8.6)]


def y_at(t, ph):
    for a, b, kind, y0, y1 in ph:
        if a <= t < b:
            if kind in ("leg1", "leg2"):
                f = min(1.0, (t - a) / (8.0 / 0.45))
                return y0 + (y1 - y0) * f, kind
            return y0, kind
    return ph[-1][4], "end"


def episode(sc, record=False, rng=None):
    """generator: yields (t, d_meas or None, x_est) at every depth frame (after latency), receives u (lateral target
    offset, m, + = away from the face); returns (metrics, trace) via StopIteration.value"""
    rng = rng or np.random.RandomState(sc.seed)
    ph = timeline()
    n = int(ph[-1][1] / DT)
    x_est = 0.0; v = 0.0
    u_hist = [0.0] * (int(TAU / DT) + 1)
    rw = 0.0; gust = 0.0; gust_left = 0.0
    lat_n = int(round(0.067 / DT))
    meas_q = []
    u = 0.0
    out = {"t": [], "x": [], "u": [], "phase": []}
    th = math.radians(sc.heading_deg)
    next_frame = 0.0
    t_air = 2.7
    tau_e = TAU_EKF if sc.tau_ekf is None else sc.tau_ekf
    sig_a = SIG_A if sc.sig_a is None else sc.sig_a
    err_ekf = None; a_ou = 0.0
    for i in range(n):
        t = i * DT
        y, kind = y_at(t, ph)
        s_along = 8.6 - y
        rw += rng.randn() * sc.rw_cm * 0.01 * math.sqrt(DT)
        err_vio = (math.sin(th) * s_along + sc.drift_cms * 0.01 * max(0.0, t - t_air)
                   + (sc.step_cm * 0.01 if t >= sc.step_at else 0.0) + sc.init_off_cm * 0.01 + rw)
        err_ekf = err_vio if err_ekf is None else err_ekf + (err_vio - err_ekf) * min(1.0, DT / tau_e)
        a_ou += -a_ou * DT + sig_a * math.sqrt(2 * DT) * rng.randn()
        if gust_left <= 0 and sc.gust_rate > 0 and rng.rand() < sc.gust_rate * DT:
            gust = rng.choice([-1, 1]) * sc.gust_ms2 * (0.5 + rng.rand()); gust_left = 0.3 + 0.4 * rng.rand()
        if gust_left > 0:
            gust_left -= DT
        else:
            gust = 0.0
        u_del = u_hist[0]
        for _ in range(2):
            a = WN * WN * (u_del - x_est) - 2 * ZETA * WN * v + gust + a_ou
            v += a * DT / 2; x_est += v * DT / 2
        x_true = x_est - err_ekf
        valid = kind in ("leg1", "hold", "climb", "leg2", "preland") and 0.5 <= y <= 9.0
        if t >= next_frame:
            next_frame += 1.0 / 15
            seen = valid
            if sc.sensing == "marker":       # only AprilTags on the unit-end posts, +-0.15 m in view
                seen = valid and min(abs(y - py) for py in POSTS_Y) <= 0.15
                sig = 0.003 * sc.noise
            else:
                sig = 0.0015 * sc.noise
            d = None
            if seen and rng.rand() >= sc.dropout:
                d = D_REF + x_true + sc.bias_cm * 0.01 + rng.randn() * sig
            meas_q.append((t + lat_n * DT, d))
        while meas_q and meas_q[0][0] <= t:
            _, d = meas_q.pop(0)
            u = yield (t, d, x_est, x_true)
        u_hist.append(u); u_hist.pop(0)
        out["t"].append(t); out["x"].append(x_true); out["u"].append(u); out["phase"].append(kind)
    return metrics(out, ph), out


def run(sc, ctrl, record=False, rng=None):
    ctrl.reset()
    g = episode(sc, record, rng)
    try:
        t, d, xe, _ = next(g)
        while True:
            t, d, xe, _ = g.send(ctrl.update(t, d, xe))
    except StopIteration as e:
        return e.value


def metrics(o, ph):
    t = np.array(o["t"]); x = np.array(o["x"]); u = np.array(o["u"]); kind = np.array(o["phase"])
    survey = np.isin(kind, ["leg1", "hold", "climb", "leg2", "preland"])
    leg1_start = ph[1][0]
    legs = survey & (t >= leg1_start + 5.0)
    du = np.diff(u)
    moves = np.abs(du) > 1e-5
    # number of distinct corrections = rising edges of "u moving"
    n_corr = int(np.sum(moves[1:] & ~moves[:-1])) + int(moves[0]) if len(moves) else 0
    sgn = np.sign(du[moves])
    return {"max_all": float(np.max(np.abs(x[survey]))), "max_legs": float(np.max(np.abs(x[legs]))),
            "rms": float(np.sqrt(np.mean(x[survey] ** 2))), "within3": float(np.mean(np.abs(x[survey]) <= 0.03)),
            "contact": bool(np.max(np.abs(x)) >= CLEAR), "n_corr": n_corr, "travel": float(np.sum(np.abs(du))),
            "reversals": int(np.sum(sgn[1:] != sgn[:-1])) if len(sgn) > 1 else 0}


# scenario sets ---------------------------------------------------------------------------------------------------
def scenario_grid(seeds=range(10), base_seed=0):
    """evaluation set: each error type over its sweep range, x seeds (noise realisations)"""
    S = []
    for s in seeds:
        sd = base_seed + s
        for h in (0.0, 1.0, 2.0, 3.0, 4.0):
            S.append(Scenario(heading_deg=h, init_off_cm=1.0, seed=sd * 100 + int(h * 10), name="heading %g" % h))
        for dr in (0.2, 0.4, 0.8):
            S.append(Scenario(drift_cms=dr, seed=sd * 100 + 50 + int(dr * 10), name="drift %g" % dr))
        for st in (5.0, 10.0):
            S.append(Scenario(step_cm=st, seed=sd * 100 + 70 + int(st), name="step %g" % st))
        S.append(Scenario(heading_deg=2.0, drift_cms=0.4, noise=3.0, dropout=0.2, gust_rate=0.1, gust_ms2=1.5,
                          seed=sd * 100 + 90, name="hard mix"))
        S.append(Scenario(noise=3.0, gust_rate=0.2, gust_ms2=1.5, seed=sd * 100 + 95, name="noise+gust only"))
    return S


def random_scenario(rng):
    """training distribution for optimisation / RL (wider than the evaluation grid)"""
    return Scenario(heading_deg=rng.uniform(-4.5, 4.5), drift_cms=rng.uniform(-0.9, 0.9) * (rng.rand() < 0.6),
                    step_cm=rng.uniform(-12, 12) * (rng.rand() < 0.3), step_at=rng.uniform(16, 50),
                    noise=rng.choice([1.0, 2.0, 3.0]), dropout=rng.choice([0.0, 0.1, 0.3]), bias_cm=rng.uniform(-0.5, 0.5),
                    init_off_cm=rng.uniform(-3, 3), gust_rate=rng.choice([0.0, 0.1, 0.3]), gust_ms2=rng.uniform(0.5, 2.0),
                    seed=int(rng.randint(1 << 30)))
