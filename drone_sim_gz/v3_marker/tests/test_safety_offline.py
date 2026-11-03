#!/usr/bin/env python3
"""Offline checks of the wrapper's safety paths (2026-10-31, answers to the codex review feedback/open/*). No flight, no
simulator: the real wk_mission / loc2d code runs against a stub flight controller. Run in starling-flight (Python 3.6):
  cd ~/workspace/v3_marker && python3 tests/test_safety_offline.py
Every case prints PASS / FAIL; the exit code is the number of failures."""
import asyncio, json, math, os, sys, tempfile, types
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import loc2d  # noqa: E402
import wk_mission as W  # noqa: E402
import safety_patch as SP  # noqa: E402

FAILS = []


def check(name, cond, info=""):
    print("%s  %s%s" % ("PASS" if cond else "FAIL", name, ("  (" + info + ")") if info else ""))
    if not cond:
        FAILS.append(name)


def new_wk():
    tmp = tempfile.mkdtemp()
    wk = W.WallKeep(True, 0, os.path.join(tmp, "wk.csv"), (2.9075, 8.6))
    wk.tag_map = json.load(open(os.path.join(HERE, "..", "gz", "tags_map.json")))["tags"]; wk.face_corr = {}
    wk.tags = True; wk.ready = False
    wk.loc = loc2d.Loc2D(wk.tag_map, wk.map_face); wk.loc_f = open(os.path.join(tmp, "loc.jsonl"), "w")
    return wk


# ---------------------------------------------------------------- 1. ToF watchdog on the current clock
def tof_case(frames, now, airborne_from=0.0):
    wk = new_wk(); wk.airborne_since = None
    wk.tof_blind(airborne_from, True)                  # the 20 Hz check sees the take-off moment
    for t, v in frames:
        wk.tof_rx_t = t; wk.tof_valid.append((t, v)); wk.tof_blind(t, t >= airborne_from)
    return wk.tof_blind(now, now >= airborne_from), wk


good = [(0.1 * i, 0.9) for i in range(50)]                               # 0..4.9 s good frames
b, wk = tof_case(good, 5.0); check("tof: good stream -> not blind", not b)
b, wk = tof_case(good, 4.9 + W.TOF_STALE_S + 0.2); check("tof: good then stopped -> blind (stale)", b, wk.tof_blind_why)
b, wk = tof_case([], W.TOF_NOFRAME_S + 0.1); check("tof: never any frame after take-off -> blind", b, wk.tof_blind_why)
b, wk = tof_case([], W.TOF_NOFRAME_S - 0.5); check("tof: no frame yet but < TOF_NOFRAME_S -> not yet", not b)
low2 = [(10.0, 0.05), (10.1, 0.05)]
b, wk = tof_case(good + low2, 10.1); check("tof: two low frames 0.1 s apart -> NOT blind (needs %.0f s)" % W.TOF_BLIND_S, not b)
lowrun = [(10.0 + 0.1 * i, 0.05) for i in range(25)]
b, wk = tof_case(good + lowrun, 12.4); check("tof: low for 2.4 s -> blind", b, wk.tof_blind_why)
b, wk = tof_case(good + lowrun[:10], 10.9 + W.TOF_STALE_S + 0.2); check("tof: low then stopped -> blind (stale keeps counting)", b, wk.tof_blind_why)
b, wk = tof_case(good + lowrun + [(12.5, 0.9)], 12.5); check("tof: recovery resets", not b and wk.tof_blind_since is None and wk.low_since is None)

# ---------------------------------------------------------------- 2. input validation
okdet = {"id": 30, "t_cam": [0.01, 0.02, 0.4], "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "margin": 55.0, "hamming": 0}
check("tag det: valid", W.valid_tag_det(okdet))
for name, mod in (("NaN right", {"t_cam": [float("nan"), 0, 0.4]}), ("Inf fwd", {"t_cam": [0, 0, float("inf")]}),
                  ("margin bool", {"margin": True}), ("t_cam size 2", {"t_cam": [0, 0.4]}), ("R 2x3", {"R": [[1, 0, 0], [0, 1, 0]]}),
                  ("R NaN", {"R": [[1, 0, 0], [0, float("nan"), 0], [0, 0, 1]]}), ("id str", {"id": "30"})):
    d = dict(okdet); d.update(mod); check("tag det rejected: " + name, not W.valid_tag_det(d))
# 2026-11-01 codex real-tag-quality-contract: a null margin is a valid record with UNAVAILABLE quality, rejected by the
# quality rule (default policy), never compared as a number
import tag_quality as TQ  # noqa: E402
nulld = dict(okdet, margin=None); nullh = dict(okdet, hamming=None)
check("tag det: margin null is a valid record", W.valid_tag_det(nulld))
check("tag quality: margin null -> unavailable, rejected (default policy)", TQ.status(nulld, 30) == "unavailable" and not TQ.ok(nulld, 30))
check("tag quality: hamming null -> unavailable", TQ.status(nullh, 30) == "unavailable")
check("tag quality: policy accept takes it and counts it", TQ.ok(nulld, 30, policy="accept") and TQ.COUNT["unavailable_accepted"] == 1)
check("tag quality: margin 12 < 30 -> low, rejected", TQ.status(dict(okdet, margin=12.0), 30) == "low" and not TQ.ok(dict(okdet, margin=12.0), 30))
check("tag quality: margin 'x' -> invalid", TQ.status(dict(okdet, margin="x"), 30) == "invalid")
Lq = loc2d.Loc2D(json.load(open(os.path.join(HERE, "..", "gz", "tags_map.json")))["tags"], lambda *a: None)
Lq.init((0.0, 0.0, 0.0), (2.9, 8.6), 0.0)
try:
    rq = Lq.update_tag(nulld, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0); exc = None
except Exception as e:  # noqa: BLE001
    rq, exc = None, e
check("loc2d.update_tag: margin null -> not used, no exception", exc is None and rq is None and Lq.n_tag == 0, repr(exc))
check("tof msg: valid plane", W.valid_tof_msg({"t": 1.0, "d": 0.37, "alpha": 0.2, "conf": 0.9, "span": 0.8, "valid": 0.9}))
check("tof msg: no plane is valid", W.valid_tof_msg({"t": 1.0, "d": None, "valid": 0.1}))
for name, m in (("NaN d", {"t": 1.0, "d": float("nan"), "alpha": 0, "conf": 1, "span": 1}), ("valid 1.5", {"t": 1.0, "d": None, "valid": 1.5}),
                ("t null", {"t": None, "d": None}), ("alpha missing", {"t": 1.0, "d": 0.4, "conf": 1, "span": 1})):
    check("tof msg rejected: " + name, not W.valid_tof_msg(m))

# ---------------------------------------------------------------- 3. estimator finiteness
L = loc2d.Loc2D(json.load(open(os.path.join(HERE, "..", "gz", "tags_map.json")))["tags"], lambda *a: None)
L.init((0.0, 0.0, 0.0), (2.9, 8.6), 0.0); x_before = L.x.copy()
ok, chi2, _ = L._update([float("nan")], lambda x: [x[0]], [[0.01]])
check("loc2d: NaN measurement rejected, state unchanged", (not ok) and L.n_bad == 1 and (L.x == x_before).all())
ok, chi2, _ = L._update([1.0], lambda x: [float("nan")], [[0.01]])
check("loc2d: NaN prediction rejected", (not ok) and L.healthy())
L.predict((float("nan"), 0.0, 0.0), 0.05)
check("loc2d: NaN VIO -> flagged bad", L.bad is not None and L.healthy())
L2 = loc2d.Loc2D(json.load(open(os.path.join(HERE, "..", "gz", "tags_map.json")))["tags"], lambda *a: None)
L2.init((0.0, 0.0, 0.0), (2.9, 8.6), 0.0); L2.P[0, 0] = float("nan"); L2.predict((0.01, 0.0, 0.0), 0.05)
check("loc2d: covariance NaN after predict -> flagged bad", L2.bad is not None and not L2.healthy(), str(L2.bad))
L2.P[0, 0] = -1e-4
check("loc2d: sd of a negative variance is NaN, no exception", math.isnan(L2.sd()[0]))
L3 = loc2d.Loc2D(json.load(open(os.path.join(HERE, "..", "gz", "tags_map.json")))["tags"], lambda *a: None)
L3.init((float("nan"), 0.0, 0.0), (2.9, 8.6), 0.0)
check("loc2d: init from a NaN VIO pose -> flagged bad", L3.bad is not None, str(L3.bad))


# ---------------------------------------------------------------- 4. wrapper paths against a stub controller
def stub_fs():
    fs = types.ModuleType("fs_stub"); fs.SETPOINT_HZ = 20; fs.MAX_STREAM_FAIL = 5; fs.LOG = []
    fs.log = lambda s: (fs.LOG.append(s), print("      log:", s))
    fs.yaw_to_360 = lambda a: a % 360.0                                       # the flight script's helpers
    fs.angle_diff_deg = lambda t, c: (t - c + 180.0) % 360.0 - 180.0

    class InAir(object):
        def __init__(self, seq): self.seq = list(seq)
        def __aiter__(self): return self
        async def __anext__(self):
            if not self.seq:
                await asyncio.sleep(10)
            await asyncio.sleep(0.01)
            return self.seq.pop(0)

    class C(object):
        def __init__(self):
            self.calls = []; self.streaming = True; self.aborted = False; self.current_target_yaw = 270.0; self.active_stage = None
            self.initial_x = self.initial_y = 0.0; self.fb_raise = False; self.fb_none = False
            self.land_mode = "ok"                     # ok | raise | hang
            self._emergency_lock = asyncio.Lock(); self._emergency_triggered = False
            self.voxl_latest = {}; self.reach_pipe = "qvio"; self.yaw_now = 0.0
            self.args = types.SimpleNamespace(yaw_tol=5.0, yaw_stable_time=1.0, sim=True)   # grid_mission always passes --sim
            tel = types.SimpleNamespace(in_air=lambda: InAir([True, True, False]))
            act = types.SimpleNamespace(kill=self._kill, land=self._land)
            self.drone = types.SimpleNamespace(telemetry=tel, action=act)
        async def _kill(self): self.calls.append("kill")
        async def _land(self):
            self.calls.append("land")
            if getattr(self, "land_fail_n", 0) > 0:
                self.land_fail_n -= 1; raise RuntimeError("synthetic: land rejected (n)")
            if self.land_mode == "raise":
                raise RuntimeError("synthetic: land rejected")
            if self.land_mode == "hang":
                await asyncio.sleep(100)
        def feedback_state(self):
            if self.fb_raise:
                raise ValueError("synthetic sensor-processing failure")
            if self.fb_none:
                return None
            return {"cmd_n": 0.0, "cmd_e": 0.0, "cmd_d": -0.7, "rel_n": 0.0, "rel_e": 0.0, "rel_u": 0.7, "yaw_deg": self.yaw_now}
        async def setpoint_streamer(self): pass
        async def arm_and_start_offboard(self): return True
        async def go_to_relative(self, *a, **k): return True
        async def go_yaw(self, *a, **k): return True
        async def wait_until_yaw_reached(self, name, target_yaw, timeout): return True, "NO_YAW_FEEDBACK_ASSUMED", 2.0
        async def land_and_disarm(self):                # like the flight script with no in_air message at all: never returns
            self.calls.append("land_and_disarm")
            await asyncio.sleep(100)
        async def _cancel_background_tasks(self): self.calls.append("cancel")
    fs.DroneController = C
    return fs


from mavsdk import telemetry as _T  # noqa: E402
FEED = {"pos": True, "att": True}                       # a stream sends at 20 Hz while True


def _fake_stream(key):
    async def gen(self_t):
        while True:
            await asyncio.sleep(0.05)
            if FEED[key]:
                yield ("same value", key)              # identical content every time: a normal steady hover
    return gen


_T.Telemetry.position_velocity_ned = _fake_stream("pos"); _T.Telemetry.attitude_euler = _fake_stream("att")


async def _consume(key):
    gen = _T.Telemetry.position_velocity_ned(None) if key == "pos" else _T.Telemetry.attitude_euler(None)
    async for _ in gen:
        pass


def inair_stream(c, box):
    """c's in_air: box["v"] every 0.05 s (box None: no message at all)"""
    async def g():
        while True:
            await asyncio.sleep(0.05)
            if box is not None:
                yield box["v"]
    c.drone.telemetry.in_air = lambda: g()


async def fly(c, wk, secs, events=()):
    """run the real correction task (wk_task_supervised via arm_and_start_offboard) with the two telemetry readers;
    events: (t, callable) applied at t s. -> seconds until the land command, or None"""
    readers = [asyncio.ensure_future(_consume(k)) for k in ("pos", "att")]
    t0 = asyncio.get_event_loop().time(); await c.arm_and_start_offboard(); ev = sorted(events, key=lambda e: e[0]); t_land = None
    while asyncio.get_event_loop().time() - t0 < secs:
        now = asyncio.get_event_loop().time() - t0
        while ev and ev[0][0] <= now:
            ev.pop(0)[1]()
        if t_land is None and "land" in c.calls:
            t_land = now
        await asyncio.sleep(0.02)
    for r in readers + [wk.task]:
        r.cancel()
    return t_land


async def wrapper_cases():
    # yaw without feedback
    fs = stub_fs(); wk = new_wk(); W.install(fs, wk); c = fs.DroneController()
    c.fb_none = True
    ok, reason, _ = await c.wait_until_yaw_reached("s00", 270.0, 5.0)
    check("yaw: no yaw feedback at all -> failure (NO_YAW_FEEDBACK)", (not ok) and reason == "NO_YAW_FEEDBACK", reason)
    c.fb_none = False
    # stop: land, wait for touchdown, no kill; the running correction task is cancelled and collected
    await c.arm_and_start_offboard(); await asyncio.sleep(0.2)
    await c.emergency_stop()
    check("stop: land requested, no motor kill", "land" in c.calls and "kill" not in c.calls, str(c.calls))
    check("stop: correction task collected", wk.task.done())
    # task supervision: the correction task dies -> land
    fs = stub_fs(); wk = new_wk(); W.install(fs, wk); c = fs.DroneController(); c.fb_raise = True
    await c.arm_and_start_offboard(); await asyncio.sleep(0.3)
    check("task: crash in the correction task -> land", "land" in c.calls and "correction task crashed" in (wk.land_reason or ""), str(c.calls))
    # scan-end gate after an early apply
    for sd, expect_land in ((0.08, True), (0.02, False), (float("nan"), True)):
        fs = stub_fs(); wk = new_wk(); W.install(fs, wk); c = fs.DroneController()
        L = wk.loc; L.init((0.0, 0.0, 0.0), (2.9, 8.6), 0.0); L.n_tag = 20
        L.P[0, 0] = L.P[1, 1] = sd ** 2; L.P[2, 2] = 1.0
        wk.ready = True                                  # as after "apply early"
        r = await c.go_yaw(0.0, "scan8_yaw0_last", "SCAN")
        landed = "land" in c.calls
        check("scan end after early apply, sd %.2f m -> %s" % (sd, "land" if expect_land else "pass"),
              landed == expect_land and (r is False) == expect_land and wk.scan_checked, str(c.calls))


asyncio.get_event_loop().run_until_complete(wrapper_cases())


# ---------------------------------------------------------------- 5. first correction is rate-limited
class Ctl(object):
    current_target_yaw = 270.0; active_stage = {"name": "s01"}
    def feedback_state(self):
        return {"cmd_n": 0.0, "cmd_e": 0.0, "cmd_d": -0.7, "rel_n": 0.0, "rel_e": 0.0, "rel_u": 0.7, "yaw_deg": 270.0}


async def first_step():
    wk = new_wk(); wk.origin = (0.0, 0.0); wk.ready = True; wk.loc_f = open(os.path.join(tempfile.mkdtemp(), "loc.jsonl"), "w")
    L = wk.loc; L.init((0.0, 0.0, 0.0), (2.9075, 8.6), 0.0)
    L.x[0] += 0.20; L.x[2] = 5.0                     # estimate 20 cm and 5 deg away from the uncorrected frame
    t0 = asyncio.get_event_loop().time()
    wk.hist = [(t0 - 0.05, 0.0, 0.0, 0.0), (t0, 0.0, 0.0, 0.0)]; wk.loc_t = t0 - 0.05
    wk.tof_rx_t = t0; wk.tof_valid = [(t0, 0.9)]
    wk.loc_step(Ctl(), Ctl().feedback_state())
    s = wk.loc_applied
    check("first correction starts at the current one and is rate-limited",
          abs(s[0] - 2.9075) <= W.LOC_RATE_POS * 0.05 + 1e-6 and abs(s[2]) <= W.LOC_RATE_YAW * 0.05 + 1e-6,
          "applied x %.4f (estimate %.4f), cY %.3f (estimate %.1f)" % (s[0], L.x[0], s[2], L.x[2]))

asyncio.get_event_loop().run_until_complete(first_step())


async def stream_cases():
    cases = [
        ("both streams, identical values every message, 2.5 s", 2.5, [], False, False, None),
        ("position stops at 0.5 s (attitude goes on)", 2.5, [(0.5, lambda: FEED.__setitem__("pos", False))], True, False, "position"),
        ("attitude stops at 0.5 s (position goes on)", 2.5, [(0.5, lambda: FEED.__setitem__("att", False))], True, False, "attitude"),
        ("both stop at 0.5 s", 2.5, [(0.5, lambda: (FEED.__setitem__("pos", False), FEED.__setitem__("att", False)))], True, False, "position/attitude"),
        ("feedback_state None, streams going, 2.5 s", 2.5, [], False, True, None),
        ("feedback_state None and position stops", 2.5, [(0.5, lambda: FEED.__setitem__("pos", False))], True, True, "position"),
    ]
    for name, secs, events, expect, fbn, why in cases:
        FEED["pos"] = FEED["att"] = True
        fs = stub_fs(); wk = new_wk(); W.install(fs, wk); c = fs.DroneController(); c.fb_none = fbn
        t = await fly(c, wk, secs, events)
        ok = (t is not None) == expect and (why is None or (wk.land_reason or "").startswith(why))
        check("feed: %s -> %s" % (name, "land" if expect else "fly on"), ok,
              "land at %s s, reason %s, messages pos %d att %d" % ("%.2f" % t if t is not None else "-", wk.land_reason, wk.rx_n["pos"], wk.rx_n["att"]))
    # yaw of a stopped attitude stream is no yaw: the wrapped feedback_state gives NaN after YAW_FRESH_S
    FEED["pos"] = FEED["att"] = True
    fs = stub_fs(); wk = new_wk(); W.install(fs, wk); c = fs.DroneController()
    readers = [asyncio.ensure_future(_consume(k)) for k in ("pos", "att")]; wk.rx_start = asyncio.get_event_loop().time()
    await asyncio.sleep(0.3); y0 = c.feedback_state()["yaw_deg"]; FEED["att"] = False
    await asyncio.sleep(W.YAW_FRESH_S + 0.2); y1 = c.feedback_state()["yaw_deg"]
    for r in readers:
        r.cancel()
    check("feed: yaw while attitude flows is a number, %.1f s after it stopped NaN" % (W.YAW_FRESH_S + 0.2), math.isfinite(y0) and math.isnan(y1), "%r -> %r" % (y0, y1))
    # ToF dead from the start, before `ready` (scan not finished): lands (limits shortened for the test)
    FEED["pos"] = FEED["att"] = True
    keep = (W.TOF_NOFRAME_S, W.TOF_BLIND_LAND_S); W.TOF_NOFRAME_S, W.TOF_BLIND_LAND_S = 0.5, 0.5
    fs = stub_fs(); wk = new_wk(); W.install(fs, wk); c = fs.DroneController(); wk.ready = False
    t = await fly(c, wk, 2.5)
    W.TOF_NOFRAME_S, W.TOF_BLIND_LAND_S = keep
    check("tof: no frame at all, not ready yet -> land", t is not None and "ToF blind" in (wk.land_reason or ""), "land at %s, %s" % (t, wk.land_reason))
    # sim clock datagrams lost
    sys.modules["grid_mission"].SIM_CLOCK_AGE = lambda: W.CLOCK_LOST_S + 0.5
    fs = stub_fs(); wk = new_wk(); W.install(fs, wk); c = fs.DroneController()
    t = await fly(c, wk, 1.0)
    sys.modules["grid_mission"].SIM_CLOCK_AGE = None
    check("clock: no sim clock datagram for %.1f s -> land" % (W.CLOCK_LOST_S + 0.5), t is not None and "sim clock" in (wk.land_reason or ""), str(wk.land_reason))
    # crash with a logger that raises too
    fs = stub_fs(); fs.log = lambda s: (_ for _ in ()).throw(IOError("synthetic: stdout closed"))
    wk = new_wk(); W.install(fs, wk); c = fs.DroneController(); c.fb_raise = True
    t = await fly(c, wk, 1.0)
    check("task crash while every log call raises -> still lands", t is not None and c.aborted and not c.streaming, str(c.calls))
    # land command fails / hangs: retried with a time limit, no kill unless WK_ALLOW_KILL
    keep = SP.LAND_RPC_TIMEOUT; SP.LAND_RPC_TIMEOUT = 0.2
    for mode, allow, expect_kill in (("raise", False, False), ("hang", False, False), ("raise", True, True)):
        SP.ALLOW_KILL = allow
        fs = stub_fs(); wk = new_wk(); W.install(fs, wk); c = fs.DroneController(); c.land_mode = mode
        await c.trigger_emergency_land("test"); wk.in_air = False      # (touchdown: ends the background retries of this case)
        check("land %s, WK_ALLOW_KILL=%d -> %d tries, kill %s" % (mode, allow, SP.LAND_TRIES, expect_kill),
              c.calls.count("land") == SP.LAND_TRIES and ("kill" in c.calls) == expect_kill and c.aborted, str(c.calls))
    SP.ALLOW_KILL = False; SP.LAND_RPC_TIMEOUT = keep

asyncio.get_event_loop().run_until_complete(stream_cases())


async def landing_cases():
    """2026-11-01 codex reply-4: landing ownership, stop entry with a failing logger, landing-step deadline, yaw hold reset"""
    # 1. SIGINT stop while the correction task is inside the landing's first delay (before the land command)
    fs = stub_fs(); wk = new_wk(); W.install(fs, wk); c = fs.DroneController()
    wk.main_task = asyncio.ensure_future(asyncio.sleep(30))            # the flight script's main flow, still running
    wk.task = asyncio.ensure_future(c.trigger_emergency_land("loc lost (test)"))
    await asyncio.sleep(0.05)                                          # inside the 0.2 s delay, land not sent yet
    await c.emergency_stop()                                           # cancels wk.task
    await asyncio.sleep(0.5)
    check("land in progress + SIGINT stop (cancels the correction task) -> land still sent once",
          c.calls.count("land") == 1 and wk.land_state == "sent" and wk.task.done(), "%s, state %s" % (c.calls, wk.land_state))
    wk.main_task.cancel()
    # 2. second stop while the land command is being retried: waits for the same landing, no second series
    keep = SP.LAND_RPC_TIMEOUT; SP.LAND_RPC_TIMEOUT = 0.2
    fs = stub_fs(); wk = new_wk(); W.install(fs, wk); c = fs.DroneController(); c.land_mode = "raise"
    t1 = asyncio.ensure_future(c.trigger_emergency_land("first")); await asyncio.sleep(0.3)
    t2 = asyncio.ensure_future(c.trigger_emergency_land("second")); await asyncio.gather(t1, t2); wk.in_air = False
    check("second stop during the land retries -> same landing, %d tries in total" % SP.LAND_TRIES,
          c.calls.count("land") == SP.LAND_TRIES and wk.land_reason == "first" and t2.done(), str(c.calls))
    SP.LAND_RPC_TIMEOUT = keep
    # 3. every log call raises: the SIGINT stop entry still lands
    fs = stub_fs(); fs.log = lambda s: (_ for _ in ()).throw(IOError("synthetic: stdout closed"))
    wk = new_wk(); W.install(fs, wk); c = fs.DroneController()
    await c.arm_and_start_offboard(); await asyncio.sleep(0.1)
    await c.emergency_stop()
    check("stop entry with a logger that raises -> land sent", "land" in c.calls and wk.land_state == "sent", str(c.calls))
    # 4. the flight script's landing step never sees an in_air message: outer deadline, failure recorded, no disarm
    keep = SP.LAND_CONFIRM_TIMEOUT; SP.LAND_CONFIRM_TIMEOUT = 0.5
    fs = stub_fs(); wk = new_wk(); W.install(fs, wk); c = fs.DroneController(); inair_stream(c, None)
    t0 = asyncio.get_event_loop().time(); await c.land_and_disarm(); dt = asyncio.get_event_loop().time() - t0
    SP.LAND_CONFIRM_TIMEOUT = keep; await SP.finish(wk)
    check("landing step without any in_air message -> ends after the deadline, TOUCHDOWN NOT CONFIRMED logged, no disarm",
          0.4 < dt < 1.2 and any("TOUCHDOWN NOT CONFIRMED" in x for x in fs.LOG) and wk.disarm_state is None and wk.touchdown is False,
          "%.2f s" % dt)
    # 5. yaw: in tolerance, the attitude stream stops long enough to go stale, then comes back in tolerance:
    #    the hold (yaw_stable_time 1.0 s) must restart after the gap
    FEED["pos"] = FEED["att"] = True
    fs = stub_fs(); wk = new_wk(); W.install(fs, wk); c = fs.DroneController(); c.yaw_now = 270.0
    readers = [asyncio.ensure_future(_consume(k)) for k in ("pos", "att")]; await asyncio.sleep(0.2)
    wk.rx_start = asyncio.get_event_loop().time()
    async def gap():
        await asyncio.sleep(0.4); FEED["att"] = False
        await asyncio.sleep(W.YAW_FRESH_S + 0.3); FEED["att"] = True
    g = asyncio.ensure_future(gap())
    ok, reason, el = await c.wait_until_yaw_reached("scan", 270.0, 6.0)
    for r in readers + [g]:
        r.cancel()
    t_back = 0.4 + W.YAW_FRESH_S + 0.3
    check("yaw in tolerance, stale gap, back: REACHED only a full hold after the gap (>= %.1f s)" % (t_back + 1.0 - 0.05),
          ok and reason == "REACHED" and el >= t_back + 1.0 - 0.05, "%s at %.2f s" % (reason, el))
    # 6. same without a gap: REACHED after one hold
    fs = stub_fs(); wk = new_wk(); W.install(fs, wk); c = fs.DroneController(); c.yaw_now = 270.0
    readers = [asyncio.ensure_future(_consume(k)) for k in ("pos", "att")]; await asyncio.sleep(0.2)
    wk.rx_start = asyncio.get_event_loop().time()
    ok, reason, el = await c.wait_until_yaw_reached("scan", 270.0, 6.0)
    for r in readers:
        r.cancel()
    check("yaw in tolerance, no gap: REACHED after one hold (1.0-1.3 s)", ok and 1.0 <= el <= 1.3, "%s at %.2f s" % (reason, el))

asyncio.get_event_loop().run_until_complete(landing_cases())


async def retry_cases():
    """2026-11-01 codex offboard-fallback-contract: a rejected land is retried until accepted or touchdown"""
    keep = (SP.LAND_RPC_TIMEOUT, SP.LAND_RETRY_S); SP.LAND_RPC_TIMEOUT, SP.LAND_RETRY_S = 0.2, 0.2
    fs = stub_fs(); wk = new_wk(); W.install(fs, wk); c = fs.DroneController(); c.land_fail_n = 4; box = {"v": True}; inair_stream(c, box)
    await c.trigger_emergency_land("test"); fast = list(c.calls); await asyncio.sleep(1.0)
    check("land rejected 4 x: fast 3 tries end the landing task, retry 1 rejected, retry 2 sends it",
          fast.count("land") == SP.LAND_TRIES and c.calls.count("land") == 5 and wk.land_state == "sent after retry 2"
          and [f[1] for f in wk.land_failures] == ["error"] * 4 and "kill" not in c.calls, "%s; %s" % (wk.land_state, [f[1] for f in wk.land_failures]))
    await SP.finish(wk)
    fs = stub_fs(); wk = new_wk(); W.install(fs, wk); c = fs.DroneController(); c.land_mode = "raise"; box = {"v": True}; inair_stream(c, box)
    await c.trigger_emergency_land("test"); await asyncio.sleep(0.7); n1 = c.calls.count("land")
    box["v"] = False; await asyncio.sleep(0.7); n2 = c.calls.count("land")
    check("land always rejected: retries go on until in_air False, then stop", n1 > SP.LAND_TRIES and n2 <= n1 + 1 and wk.land_retry_task.done(),
          "%d tries before, %d after touchdown" % (n1, n2))
    SP.LAND_RPC_TIMEOUT, SP.LAND_RETRY_S = keep

asyncio.get_event_loop().run_until_complete(retry_cases())


# ---------------------------------------------------------------- 6. numbers only, ToF stamps
check("tag det rejected: margin as a numeric string", not W.valid_tag_det(dict(okdet, margin="0.5")))
check("tof msg rejected: t as a numeric string", not W.valid_tof_msg({"t": "1.0", "d": None}))
check("tof msg rejected: valid True (bool)", not W.valid_tof_msg({"t": 1.0, "d": None, "valid": True}))
import json as _json, socket as _socket, time as _time  # noqa: E402
wk = new_wk(); wk.clock = lambda: 10.0; port = wk.sock.getsockname()[1]; tx = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
for t in (20.0, 9.9, 9.0):
    tx.sendto(_json.dumps({"t": t, "d": None, "valid": 0.9}).encode(), ("127.0.0.1", port)); _time.sleep(0.1)
check("tof stamps: future (20 s at clock 10) and backwards (9.0 after 9.9) rejected, 9.9 kept",
      wk.n_bad_tof == 2 and wk.tof_rx_t == 9.9 and wk.tof_rx_local == 10.0, "bad %d, last t %s" % (wk.n_bad_tof, wk.tof_rx_t))


# ---------------------------------------------------------------- 7. --wallkeep off = observe only (codex loc-observe-mode)
async def observe_case(enabled):
    fs = stub_fs(); wk = new_wk(); wk.enabled = enabled; W.install(fs, wk); c = fs.DroneController()
    wk.origin = (0.0, 0.0); wk.rotate = True; wk.cN, wk.cE, wk.cY = 0.10, -0.05, 2.0      # what the estimator would apply
    sent = []

    async def set_pos(p):
        sent.append((p.north_m, p.east_m, p.yaw_deg)); c.streaming = False
    c.drone.offboard = types.SimpleNamespace(set_position_ned=set_pos)
    from mavsdk.offboard import PositionNedYaw
    c.target = PositionNedYaw(1.0, 2.0, -0.7, 270.0)
    await c.setpoint_streamer()
    return sent[0], c.feedback_state()

sp_off, fb_off = asyncio.get_event_loop().run_until_complete(observe_case(False))
sp_on, fb_on = asyncio.get_event_loop().run_until_complete(observe_case(True))
check("wallkeep off: setpoint = the script target (correction estimated but not applied)",
      abs(sp_off[0] - 1.0) < 1e-12 and abs(sp_off[1] - 2.0) < 1e-12 and abs(sp_off[2] - 270.0) < 1e-12, str(sp_off))
check("wallkeep off: feedback uncorrected", abs(fb_off["cmd_n"]) < 1e-12 and abs(fb_off["cmd_e"]) < 1e-12 and abs(fb_off["yaw_deg"]) < 1e-9 and abs(fb_on["yaw_deg"] + 2.0) < 1e-9, "off %s, on yaw %s" % (fb_off, fb_on["yaw_deg"]))
check("wallkeep on: the same correction IS applied", abs(sp_on[2] - 272.0) < 1e-9 and abs(sp_on[0] - 1.0) > 0.01, str(sp_on))
wk = new_wk(); wk.enabled = False; wk.origin = (0.0, 0.0); wk.ready = True
wk.loc_f = open(os.path.join(tempfile.mkdtemp(), "loc.jsonl"), "w"); wk.loc.init((0.0, 0.0, 0.0), (2.9075, 8.6), 0.0)
t0 = 50.0; wk.hist = [(t0 - 0.05, 0.0, 0.0, 0.0), (t0, 0.0, 0.0, 0.0)]; wk.loc_t = t0 - 0.05
wk.loc_step(Ctl(), Ctl().feedback_state()); wk.f.flush()
import csv as _csv  # noqa: E402
rows = list(_csv.reader(open(wk.f.name)))
check("wallkeep off + loc: wk.csv enabled 0, state loc_observe", rows[-1][2] == "0" and rows[-1][3].startswith("loc_observe"), str(rows[-1][:5]))
print("\n%d failure(s)%s" % (len(FAILS), (": " + ", ".join(FAILS)) if FAILS else ""))
sys.exit(len(FAILS))
