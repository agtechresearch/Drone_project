#!/usr/bin/env python3
"""East target-offset correction during v13's take-off hover on the REAL vehicle (2026-11-04). Python 3.6, standard library +
MAVSDK 0.12 (already on the VOXL). path_flight_phase1_v13.py is NOT edited: install(fs, opts) replaces DroneController.hover
at run time, and only the hover named "post_takeoff" (v13 STAGE 1, --takeoff-hover-time) is changed; every other hover runs the
original method. real_ctl_safe.py calls install() and then real_flight_safe.main() (the unchanged safety layer).

Inside the post_takeoff hover, at POLICY_HZ (10 Hz) on the loop clock:
  settle  (0 .. settle_s)                 delta = 0, nothing decided (the vehicle settles after the climb)
  active  (settle_s .. duration - end_s)  zero: delta = 0, observations logged (PX4 alone, same path)
                                          ppo : obs -> policy mean (deterministic) -> apply_action (clip +-delta_max, rate limit)
  ramp    (last end_s, or after "off")    delta back to 0 at RAMP_M_S, then exactly the hover target again
The target sent by v13's own setpoint streamer is self.target = base + (0, delta, 0), base = v13's target at hover start, yaw
unchanged. Observation exactly as in Gazebo (rl_demo/policy_runtime.ObsBuilder): e_err = east estimate - base east, ve, roll.
  east / ve: v13's reach pipe (px4_vehicle_local_position via voxl-inspect-pose, aligned to the MAVSDK command frame by v13),
             NOT v13's mav_* values (they lag 5-10 s on the vehicle: hover_20261104_073955 launcher.log)
  roll:      the real pipe carries yaw only (roll 0), so roll comes from MAVSDK telemetry.attitude_euler with ONE subscriber here
             (no other reader of that stream in real mode); with v13 --sim the sim reader already subscribes, so its roll is used
Correction OFF for the rest of the hover (delta ramps to 0, PX4 keeps holding the hover target) when: feedback missing or older
than stale_s, roll missing / older than stale_s, a non-finite value, |e_err| > e_stop_m, |yaw| > yaw_max_deg at activation.
Nothing here lands or kills: landing stays with v13 / safety_patch (pose loss 1 s -> land, STOP, operator take-over).
Log: <log_dir>/ctl.csv (one row per policy step) and ctl.json (controller, policy sha256, base, events)."""
import asyncio, hashlib, json, math, os, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (HERE, os.path.join(HERE, "..", "rl_demo")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
from policy_runtime import MLPPolicy, ObsBuilder, apply_action, POLICY_HZ  # noqa: E402

RAMP_M_S = 0.2           # delta back to 0 (ramp / off), m/s: 0.2 m in 1 s
HOVER_NAME = "post_takeoff"
DEFAULTS = {"ctrl": "zero", "policy": None, "delta_max_m": 0.2, "delta_rate_m_s": 1.0, "settle_s": 3.0, "end_s": 2.0,
            "e_stop_m": 0.3, "stale_s": 0.3, "yaw_max_deg": 10.0, "log_dir": "ctl"}


def _fin(*v):
    return all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


class Ctl(object):
    """zero | ppo (deterministic policy mean, as evaluated in Gazebo)"""
    def __init__(self, o):
        self.o = o; self.t = o["ctrl"]; self.dm, self.rate = o["delta_max_m"], o["delta_rate_m_s"]
        self.obs = ObsBuilder(self.dm); self.pol = None; self.version = ""
        if self.t == "ppo":
            self.pol = MLPPolicy(o["policy"]); self.version = str(self.pol.version)
        elif self.t != "zero":
            raise ValueError("ctrl must be zero or ppo, not %r" % self.t)

    def step(self, e_err, ve, roll, delta_prev):
        o = self.obs.build(e_err, ve, roll, delta_prev)
        if self.t == "zero":
            return o, 0.0, 0.0, 0.0, 0.0
        mean, value = self.pol.forward(o)
        a_c, delta = apply_action(mean, delta_prev, self.dm, self.rate)
        return o, mean, a_c, delta, value


def install(fs, opts):
    """patch fs.DroneController.hover in place; returns the merged options"""
    o = dict(DEFAULTS); o.update({k: v for k, v in opts.items() if v is not None})
    need = o["delta_max_m"] / RAMP_M_S + 0.5
    if o["end_s"] < need or o["settle_s"] < 0 or o["e_stop_m"] <= 0 or o["stale_s"] <= 0:
        raise ValueError("end_s %.2f must be >= %.2f s (delta_max back to 0 at %.2f m/s + 0.5 s); settle_s >= 0, e_stop_m and "
                         "stale_s > 0" % (o["end_s"], need, RAMP_M_S))
    ctl = Ctl(o)                                   # policy loaded and checked BEFORE anything flies
    C = fs.DroneController
    orig = getattr(C, "_ctl_orig_hover", None) or C.hover
    C._ctl_orig_hover = orig
    os.makedirs(o["log_dir"], exist_ok=True)
    info = {"ctrl": o["ctrl"], "options": o, "policy_sha256": sha256(o["policy"]) if o["policy"] else None,
            "policy_version": ctl.version, "events": [], "installed_wall": time.time()}
    jpath = os.path.join(o["log_dir"], "ctl.json")

    def wj():
        tmp = jpath + ".tmp"
        with open(tmp, "w") as f:
            json.dump(info, f, indent=1)
        os.replace(tmp, jpath)
    wj()

    async def hover(self, duration, name, hold_current=False):
        if name != HOVER_NAME or info.get("used"):
            return await orig(self, duration, name, hold_current)
        info["used"] = True
        try:
            await run(self, fs, ctl, o, info, wj, duration)
        finally:
            wj()
    C.hover = hover
    fs.log("[ctl] installed: %s%s, delta_max %.2f m, rate %.2f m/s, settle %.1f s, end %.1f s, e_stop %.2f m, stale %.2f s -> %s" % (
        o["ctrl"], " (%s, sha256 %s)" % (o["policy"], info["policy_sha256"][:16]) if o["policy"] else "", o["delta_max_m"],
        o["delta_rate_m_s"], o["settle_s"], o["end_s"], o["e_stop_m"], o["stale_s"], o["log_dir"]))
    return o


async def _att_reader(drone, att):
    try:
        async for a in drone.telemetry.attitude_euler():
            att["roll"] = a.roll_deg; att["yaw"] = a.yaw_deg; att["t"] = asyncio.get_event_loop().time(); att["n"] += 1
    except asyncio.CancelledError:
        raise
    except Exception as e:  # noqa: BLE001
        att["err"] = "%s: %s" % (type(e).__name__, e)


async def run(self, fs, ctl, o, info, wj, duration):
    if self.aborted:
        return
    loop = asyncio.get_event_loop(); base = self.target; sim = bool(getattr(self.args, "sim", False))
    PT = type(base)
    t0 = loop.time(); DT = 1.0 / POLICY_HZ; ramp = RAMP_M_S * DT
    info.update(base_ned_yaw=[base.north_m, base.east_m, base.down_m, base.yaw_deg], duration_s=duration, t0_mono=time.monotonic(),
                t0_wall=time.time(), roll_source="sim reader (MAVSDK)" if sim else "MAVSDK attitude_euler (own subscriber)")
    ev = info["events"]

    def event(kind, detail=""):
        ev.append({"t": round(loop.time() - t0, 3), "kind": kind, "detail": detail}); fs.log("[ctl] %s %s" % (kind, detail)); wj()
    att = {"roll": None, "yaw": None, "t": None, "n": 0, "err": None}
    att_task = None if sim else asyncio.ensure_future(_att_reader(self.drone, att))
    self.active_phase = "HOVER"
    event("start", "%s hover %.1f s, base N%+.3f E%+.3f D%+.3f yaw %.1f" % (o["ctrl"], duration, base.north_m, base.east_m,
                                                                          base.down_m, base.yaw_deg))
    f = open(os.path.join(o["log_dir"], "ctl.csv"), "w")
    f.write("j,t,mono,wall,phase,fb_age,att_age,e_err,ve,roll,yaw,%s,a_mean,a,delta,value,cmd_e\n" % ",".join("o%d" % i for i in range(8)))
    delta = 0.0; phase = "settle"; off = None; j = 0; nxt = t0; last_print = -1.0
    try:
        while True:
            if self.aborted:
                event("aborted", "safety / STOP took over, target left to the safety layer"); return
            now = loop.time(); el = now - t0
            if el > duration:
                break
            fb = self.feedback_state()
            fb_age = fb["age_sec"] if fb else float("inf")
            if sim:
                roll = fb.get("roll_deg") if fb else None; att_age = fb_age
            else:
                roll = att["roll"]; att_age = now - att["t"] if att["t"] is not None else float("inf")
            e_err = fb["cmd_e"] - base.east_m if fb else float("nan"); ve = fb.get("ve") if fb else float("nan")
            yaw = fb.get("yaw_deg") if fb else float("nan")
            obs = [float("nan")] * 8; a_mean = a_c = value = float("nan")
            if el < o["settle_s"]:
                phase = "settle"
            elif off is None and el < duration - o["end_s"]:
                why = None
                if fb is None or fb_age > o["stale_s"]:
                    why = "feedback missing / old (%.2f s)" % fb_age
                elif roll is None or att_age > o["stale_s"]:
                    why = "roll missing / old (%.2f s)%s" % (att_age, " %s" % att["err"] if att["err"] else "")
                elif not _fin(e_err, ve, roll):
                    why = "non-finite input e %r ve %r roll %r" % (e_err, ve, roll)
                elif abs(e_err) > o["e_stop_m"]:
                    why = "|e_err| %.3f > %.2f m" % (e_err, o["e_stop_m"])
                elif phase == "settle" and _fin(yaw) and abs(fs.angle_diff_deg(base.yaw_deg, yaw)) > o["yaw_max_deg"]:
                    why = "heading %.1f deg vs target %.1f (> %.0f) at activation" % (yaw, base.yaw_deg, o["yaw_max_deg"])
                if why:
                    off = why; phase = "off"; event("off", why)
                else:
                    if phase == "settle":
                        event("active", "e_err %+.3f ve %+.3f roll %+.2f yaw %.1f" % (e_err, ve, roll, yaw))
                    phase = "active"
                    obs, a_mean, a_c, delta, value = ctl.step(e_err, ve, roll, delta)
            else:
                if phase == "active":
                    event("ramp", "delta %+.3f -> 0" % delta)
                phase = "off" if off else "ramp"
            if phase in ("off", "ramp"):
                delta = math.copysign(max(0.0, abs(delta) - ramp), delta)
            cmd_e = base.east_m + delta
            self.target = PT(base.north_m, cmd_e, base.down_m, base.yaw_deg)
            f.write("%d,%.3f,%.4f,%.3f,%s,%.3f,%.3f,%.5f,%.5f,%.3f,%.2f,%s,%.5f,%.5f,%.5f,%.5f,%.5f\n" % (
                j, el, time.monotonic(), time.time(), phase, fb_age, att_age, e_err, ve if _fin(ve) else float("nan"),
                roll if _fin(roll) else float("nan"), yaw if _fin(yaw) else float("nan"), ",".join("%.5f" % v for v in obs),
                a_mean, a_c, delta, value, cmd_e)); f.flush()
            if el - last_print >= 1.0:
                last_print = el
                fs.log("       [ctl {}] t={:.1f}s e={:+.3f} ve={:+.3f} roll={:+.2f} delta={:+.3f} fb_age={:.2f} att_age={:.2f}".format(
                    phase, el, e_err, ve if _fin(ve) else float("nan"), roll if _fin(roll) else float("nan"), delta, fb_age, att_age))
            j += 1; nxt += DT
            await asyncio.sleep(max(0.0, nxt - loop.time()))
        if abs(delta) > 1e-9:
            event("end_jump", "delta %+.3f still set at the hover end -> target reset" % delta)
        self.target = base
        info.update(steps=j, off=off, att_msgs=att["n"])
        event("end", "%d steps, correction %s" % (j, "OFF (%s)" % off if off else "ran to the end"))
    finally:
        f.close()
        if att_task is not None:
            att_task.cancel()
            try:
                await att_task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
