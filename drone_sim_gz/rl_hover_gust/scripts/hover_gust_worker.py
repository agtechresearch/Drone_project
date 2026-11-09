#!/usr/bin/env python3
"""Hover-gust flight worker (2026-11-03; stage A: correction delta = 0; stage B/C: per-episode controller from plan.json
"ctrl": {"type": zero|vel|ppo, "k", "policy": "<file in batch dir>", "deterministic"} - a ppo episode waits until the trainer
wrote its policy file, so an update never overlaps a flight; ep_XX/policy.csv = one row per policy step). Runs in starling-flight (Python 3.6, MAVSDK 0.12)
against a running mavsdk_server:
  hover_gust_worker.py --port 50053 --clock-port 14602 --config <config.json> --batch <batch_dir>
For every episode of <batch_dir>/plan.json:
  arm -> Offboard position hold alt_m above the start point (nominal target = start n, e, d - alt, start yaw)
  -> settled (pos/vel within ready tolerances for hold_s sim s) -> writes ep_XX/ready (the gust scheduler starts from it)
  -> window_s sim s of setpoints at setpoint_hz: target + (0, delta, 0) with delta = 0 in stage A -> writes ep_XX/window_end
  -> waits for ep_XX/cleared from the scheduler (force removed) -> action.land -> not in air -> disarmed -> ep_XX/landed
One CSV row per control cycle (ep_XX/control.csv): sim time and its age, the latest estimate (PX4 position/velocity NED,
attitude) with the age of each, the target sent, delta. Episode result in ep_XX/episode.json; batch result as exit code
(0 all episodes ok, 1 an episode failed, 2 setup failed) and worker_done.
The sim clock comes from sim_clock_udp.py (UDP text "<sim s>"); a clock older than 1 s wall stops the batch (no extrapolation
over a stalled simulator). One subscriber per telemetry stream (MAVSDK 0.12: a second subscriber steals the stream).
2026-11-03 v2: decisions on sim-time boundaries (policy.csv columns b, pv_age), t_stop in window_end, terminal.json =
observation after the last action (rl_demo/hover_metrics.py)."""
import argparse, asyncio, json, math, os, random, socket, sys, threading, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from rl_demo.contracts import load_config, AXIS
from rl_demo.policy_runtime import MLPPolicy, ObsBuilder, apply_action, POLICY_HZ
from mavsdk import System
from mavsdk.offboard import PositionNedYaw

CLOCK_MAX_AGE = 1.0


def log(msg):
    print("%.3f %s" % (time.time(), msg), flush=True)


class SimClock(object):
    def __init__(self, port):
        self.t = None; self.wall = None; self.n = 0
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("127.0.0.1", port)); s.settimeout(0.5); self.s = s
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while True:
            try:
                b = self.s.recv(64)
            except socket.timeout:
                continue
            try:
                self.t = float(b.decode().split()[-1]); self.wall = time.time(); self.n += 1
            except ValueError:
                pass

    def now(self):
        """(sim t of the last datagram, its age in wall s). No extrapolation."""
        if self.t is None:
            return None, float("inf")
        return self.t, time.time() - self.wall


class Telem(object):
    """latest value + arrival wall time of each stream, one subscriber each"""
    def __init__(self, d):
        self.d = d; self.v = {}; self.w = {}

    async def _run(self, name, gen):
        async for x in gen:
            self.v[name] = x; self.w[name] = time.time()

    def start(self):
        t = self.d.telemetry
        for name, gen in (("pv", t.position_velocity_ned()), ("att", t.attitude_euler()), ("air", t.in_air()),
                          ("armed", t.armed()), ("mode", t.flight_mode()), ("bat", t.battery())):
            asyncio.ensure_future(self._run(name, gen))

    def age(self, name):
        return time.time() - self.w[name] if name in self.w else float("inf")


async def wait_for(pred, timeout, what):
    t0 = time.time()
    while not pred():
        if time.time() - t0 > timeout:
            raise RuntimeError("timeout waiting for %s" % what)
        await asyncio.sleep(0.02)


def wjson(path, obj):
    tmp = path + ".tmp"; json.dump(obj, open(tmp, "w"), indent=1); os.replace(tmp, path)


async def episode(d, tm, clk, cfg, ep, B):
    D = os.path.join(B, "ep_%02d" % ep["id"]); os.makedirs(D, exist_ok=True)
    res = {"episode": ep, "status": "error", "reason": "", "t_wall": {}}
    tw = res["t_wall"]; tw["start"] = time.time()
    try:
        await wait_for(lambda: "pv" in tm.v and "att" in tm.v and tm.v.get("armed") is False and tm.v.get("air") is False,
                       30, "disarmed on the ground with telemetry")
        ctl = make_controller(ep, cfg, B); res["controller"] = ctl.desc
        pv, att = tm.v["pv"], tm.v["att"]
        n0, e0, d0, yaw0 = pv.position.north_m, pv.position.east_m, pv.position.down_m, att.yaw_deg
        ref = [n0, e0, d0 - cfg["alt_m"]]; res["nominal_ned"] = ref; res["yaw_deg"] = yaw0
        bat = tm.v.get("bat"); res["battery_pct_before_arm"] = bat.remaining_percent if bat else None
        log("ep %d %s seed %d: start n %.3f e %.3f d %.3f yaw %.1f battery %s" % (ep["id"], ep["cond"], ep["seed"], n0, e0, d0, yaw0,
            "%.3f" % bat.remaining_percent if bat else "?"))
        await d.offboard.set_position_ned(PositionNedYaw(ref[0], ref[1], ref[2], yaw0))
        await d.action.arm(); tw["armed"] = time.time()
        await d.offboard.start(); tw["offboard"] = time.time()
        # settle: every tolerance holds for hold_s SIM seconds
        R = cfg["ready"]; ok_since = None; t_start = time.time()
        while True:
            t, age = clk.now()
            if age > CLOCK_MAX_AGE:
                raise RuntimeError("sim clock stalled (age %.2f s)" % age)
            p = tm.v["pv"]
            pe = max(abs(p.position.north_m - ref[0]), abs(p.position.east_m - ref[1]), abs(p.position.down_m - ref[2]))
            ve = math.sqrt(p.velocity.north_m_s ** 2 + p.velocity.east_m_s ** 2 + p.velocity.down_m_s ** 2)
            if pe <= R["pos_tol_m"] and ve <= R["vel_tol_m_s"]:
                ok_since = t if ok_since is None else ok_since
                if t - ok_since >= R["hold_s"]:
                    break
            else:
                ok_since = None
            if time.time() - t_start > R["timeout_s"]:
                raise RuntimeError("not settled within %.0f s (pos err %.3f vel %.3f)" % (R["timeout_s"], pe, ve))
            await asyncio.sleep(0.05)
        t_ready, _ = clk.now(); tw["ready"] = time.time()
        wjson(os.path.join(D, "ready"), {"t_sim": t_ready, "t_wall": tw["ready"]})
        log("ep %d ready at sim %.3f" % (ep["id"], t_ready))
        # window: fixed nominal target + delta (east) from the controller at POLICY_HZ; stage A = "zero"
        f = open(os.path.join(D, "control.csv"), "w")
        pf = open(os.path.join(D, "policy.csv"), "w")
        # 2026-11-03 v2 (codex hover-transition-time): decision j at the first cycle with sim t >= t_ready + j / POLICY_HZ
        # (b = boundary index, consecutive unless the sim jumped a boundary); a cycle on the same sim time never decides twice.
        # The previous delta keeps being sent between decisions. rl_demo/hover_metrics.py has the contract.
        pf.write("j,b,k,t_wall,t_sim,pv_age,%s,a_raw,a,delta,logp,value,mean,version\n" % ",".join("o%d" % i for i in range(8)))
        delta = 0.0; j = 0; term = None; DTP = 1.0 / POLICY_HZ; b_next = 0
        e_max = cfg.get("terminate", {}).get("e_max_m")
        f.write("k,t_wall,t_sim,clock_age,n,e,d,vn,ve,vd,pv_age,roll,pitch,yaw,att_age,cmd_n,cmd_e,cmd_d,cmd_yaw,delta,mode,bat\n")
        dt = 1.0 / cfg["setpoint_hz"]; k = 0; t_next = time.time(); t_end = t_ready + cfg["window_s"]
        max_clock_age = max_pv_age = 0.0; t_stop = None
        while True:
            t, age = clk.now()
            if age > CLOCK_MAX_AGE:
                raise RuntimeError("sim clock stalled in the window (age %.2f s)" % age)
            if t >= t_end - 1e-9:
                t_stop = t; break
            if t >= t_ready + b_next * DTP - 1e-9:
                b = int(math.floor((t - t_ready) / DTP + 1e-9))       # == b_next unless a boundary was jumped
                p, a = tm.v["pv"], tm.v["att"]; e_err = p.position.east_m - ref[1]; pva = tm.age("pv")
                if e_max is not None and abs(e_err) > e_max:
                    term = "estimate |east error| %.3f > %.3f m" % (e_err, e_max); t_stop = t; break
                obs, a_raw, a_c, delta, logp, value, mean = ctl.step(e_err, p.velocity.east_m_s, a.roll_deg, delta)
                pf.write("%d,%d,%d,%.4f,%.4f,%.4f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s\n" % (j, b, k, time.time(), t, pva,
                         ",".join("%.6f" % o for o in obs), a_raw, a_c, delta, logp, value, mean, ctl.version)); j += 1
                b_next = b + 1
            cmd = [ref[0], ref[1] + delta, ref[2]]       # AXIS ned_index 1 = east
            await d.offboard.set_position_ned(PositionNedYaw(cmd[0], cmd[1], cmd[2], yaw0))
            p, a = tm.v["pv"], tm.v["att"]; pa, aa = tm.age("pv"), tm.age("att")
            mode = str(tm.v.get("mode")).split(".")[-1]; bat = tm.v.get("bat")
            if mode != "OFFBOARD" and k > 0:
                raise RuntimeError("flight mode %s in the window (not OFFBOARD), cycle %d" % (mode, k))
            max_clock_age = max(max_clock_age, age); max_pv_age = max(max_pv_age, pa)
            f.write("%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.3f,%.5f,%s,%.4f\n" % (
                k, time.time(), t, age, p.position.north_m, p.position.east_m, p.position.down_m, p.velocity.north_m_s,
                p.velocity.east_m_s, p.velocity.down_m_s, pa, a.roll_deg, a.pitch_deg, a.yaw_deg, aa,
                cmd[0], cmd[1], cmd[2], yaw0, delta, mode, bat.remaining_percent if bat else float("nan")))
            k += 1; t_next += dt
            await asyncio.sleep(max(0.0, t_next - time.time()))
        f.close(); pf.close(); tw["window_end"] = time.time()
        # terminal observation at t_stop with the same ObsBuilder / history, no new action (codex hover-terminal-observation)
        p, a = tm.v["pv"], tm.v["att"]
        tobs, tval = ctl.terminal(p.position.east_m - ref[1], p.velocity.east_m_s, a.roll_deg, delta)
        wjson(os.path.join(D, "terminal.json"), {"t_sim": t_stop, "kind": "terminated" if term else "truncated", "obs": tobs,
                                                 "value": tval, "delta": delta, "pv_age": tm.age("pv"), "version": ctl.version})
        t_we = t_stop; wjson(os.path.join(D, "window_end"), {"t_sim": t_we, "t_wall": tw["window_end"], "cycles": k,
                                                             "policy_steps": j, "terminated": term})
        res.update(cycles=k, policy_steps=j, terminated=term, max_clock_age=max_clock_age, max_pv_age=max_pv_age)
        if term:
            log("ep %d TERMINATED: %s" % (ep["id"], term))
        log("ep %d window end at sim %.3f, %d cycles" % (ep["id"], t_we, k))
        try:
            await wait_for(lambda: os.path.exists(os.path.join(D, "cleared")), 5, "scheduler clear (ep_XX/cleared)")
        except RuntimeError as e:
            res["reason"] = str(e)                     # land anyway, the episode fails
        tw["cleared_seen"] = time.time()
        await d.action.land(); tw["land_cmd"] = time.time()
        await wait_for(lambda: tm.v.get("air") is False, 40, "landed (in_air false)"); tw["landed"] = time.time()
        await wait_for(lambda: tm.v.get("armed") is False, 20, "disarmed"); tw["disarmed"] = time.time()
        wjson(os.path.join(D, "landed"), {"t_sim": clk.now()[0], "t_wall": tw["disarmed"]})
        if not res["reason"]:
            res["status"] = "ok"
        log("ep %d %s, disarmed" % (ep["id"], res["status"]))
    except Exception as e:
        res["reason"] = "%s: %s" % (type(e).__name__, e); log("!! ep %d: %s" % (ep["id"], res["reason"]))
        try:
            await d.action.land(); await wait_for(lambda: tm.v.get("air") is False, 40, "landed after error")
        except Exception as e2:
            res["reason"] += "; land after error: %s" % e2
    wjson(os.path.join(D, "episode.json"), res)
    return res["status"] == "ok"


class Controller(object):
    """zero (stage A) | vel (delta = -k ve, same limits) | ppo (policy JSON, sampled or deterministic)"""
    def __init__(self, c, cfg, B, seed):
        self.t = c.get("type", "zero"); P = cfg["policy"]; self.dm, self.rate = P["delta_max_m"], P["delta_rate_m_s"]
        self.obs = ObsBuilder(self.dm); self.version = ""; self.pol = None; self.rng = random.Random(seed)
        self.det = bool(c.get("deterministic", False)); self.k = float(c.get("k", 0.0))
        if self.t == "ppo":
            self.pol = MLPPolicy(os.path.join(B, c["policy"])); self.version = str(self.pol.version)
        elif self.t not in ("zero", "vel"):
            raise ValueError("controller %s" % self.t)
        if self.t != "zero" and self.dm <= 0:
            raise ValueError("controller %s needs policy.delta_max_m > 0" % self.t)
        self.desc = {"type": self.t, "k": self.k, "version": self.version, "deterministic": self.det, "rng_seed": seed,
                     "delta_max_m": self.dm, "delta_rate_m_s": self.rate}

    def step(self, e_err, ve, roll, delta_prev):
        o = self.obs.build(e_err, ve, roll, delta_prev)
        if self.t == "zero":
            return o, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        if self.t == "vel":
            a_raw = -self.k * ve / self.dm; logp = value = mean = 0.0
        else:
            a_raw, logp, value, mean = self.pol.act(o, self.rng, self.det)
        a_c, delta = apply_action(a_raw, delta_prev, self.dm, self.rate)
        return o, a_raw, a_c, delta, logp, value, mean

    def terminal(self, e_err, ve, roll, delta_prev):
        """observation after the last action (time-limit bootstrap), value of the policy there; no action is sampled"""
        o = self.obs.build(e_err, ve, roll, delta_prev)
        return o, (self.pol.forward(o)[1] if self.t == "ppo" else 0.0)


def make_controller(ep, cfg, B):
    c = ep.get("ctrl", {"type": "zero"})
    return Controller(c, cfg, B, int(ep.get("seed", 0)) * 1000 + int(ep["id"]))


async def wait_policy(ep, B, timeout=1800):
    """training: the trainer writes the policy of the next version only after it updated on the previous episodes"""
    c = ep.get("ctrl", {})
    if c.get("type") != "ppo":
        return
    p = os.path.join(B, c["policy"]); t0 = time.time()
    if not os.path.exists(p):
        log("ep %d waits for %s" % (ep["id"], c["policy"]))
    while not os.path.exists(p):
        if os.path.exists(os.path.join(B, "trainer_failed")) or time.time() - t0 > timeout:
            raise RuntimeError("policy %s not written (trainer_failed or %d s)" % (c["policy"], timeout))
        await asyncio.sleep(0.5)


async def main(a):
    cfg = load_config(a.config); plan = json.load(open(os.path.join(a.batch, "plan.json")))
    clk = SimClock(a.clock_port)
    d = System(mavsdk_server_address="localhost", port=a.port)
    await d.connect()
    async for s in d.core.connection_state():
        if s.is_connected:
            break
    async for h in d.telemetry.health():
        if h.is_local_position_ok and h.is_home_position_ok:
            break
    tm = Telem(d); tm.start()
    await wait_for(lambda: clk.t is not None, 10, "sim clock datagrams")
    log("connected, axis %s, %d episodes" % (AXIS["name"], len(plan["episodes"])))
    ok_all = True
    for ep in plan["episodes"]:
        try:
            await wait_policy(ep, a.batch)
        except RuntimeError as e:
            log("!! %s" % e); ok_all = False; break
        ok = await episode(d, tm, clk, cfg, ep, a.batch)
        ok_all = ok_all and ok
        if not ok:
            log("!! stop the batch after a failed episode"); break
        await asyncio.sleep(cfg["pause_between_s"])
    return 0 if ok_all else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=50053)
    ap.add_argument("--clock-port", type=int, default=14602)
    ap.add_argument("--config", required=True)
    ap.add_argument("--batch", required=True)
    a = ap.parse_args()
    rc = 2
    try:
        rc = asyncio.get_event_loop().run_until_complete(main(a))
    finally:
        wjson(os.path.join(a.batch, "worker_done"), {"rc": rc, "t_wall": time.time()})
    sys.exit(rc)
