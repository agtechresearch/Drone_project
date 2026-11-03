#!/usr/bin/env python3
"""STOP supervision, arm request state and result file (2026-11-01, codex decisions reply-7 S1/S2/S3, fallback reply-4,
preflight reply-4). path_flight_phase1_v13.py is imported unchanged and its OWN connect / wait_position_stable /
check_vio_quality_preflight / capture_initial_mavsdk_position / arm_and_start_offboard (1 s, reach pipe, 2 s, arm, 1 s, offboard
start) run for real. Only the transport is fake: the module's System name gives a fake MAVSDK system whose streams / RPCs can be
made to never answer. execute_mission is a stand-in only where a flight is needed (cases G, H, M, N). SIGINT is a real signal
to v13's own handler; the end goes through real_flight_safe.run() (prepare -> main -> report).
Run in starling-flight:  cd ~/workspace/v3_marker && python3 tests/test_stop_supervisor.py"""
import asyncio, hashlib, json, os, random, signal, sys, tempfile, time, types
HERE = os.path.dirname(os.path.abspath(__file__)); SC = os.path.join(HERE, "..", "scripts"); ROOT = os.path.join(HERE, "..", "..")
sys.path.insert(0, SC); sys.path.insert(0, ROOT)
V13 = os.path.join(ROOT, "path_flight_phase1_v13.py"); H0 = hashlib.sha256(open(V13, "rb").read()).hexdigest()
import path_flight_phase1_v13 as fs  # noqa: E402
import safety_patch as SP  # noqa: E402
import real_flight_safe as R  # noqa: E402
F = []; LOG = []; T0 = [0.0]
fs.log = lambda m: LOG.append("%7.2f %s" % (time.time() - T0[0], m))
loop = asyncio.get_event_loop(); C = fs.DroneController
SP.STOP_HANDOFF_S, SP.STOP_TOTAL_S, SP.STOP_END_S, SP.LAND_CONFIRM_TIMEOUT = 2.0, 6.0, 8.0, 3.0
SP.DISARM_CONFIRM_S = 0.5
SP.LAND_RPC_TIMEOUT, SP.LAND_RETRY_S, SP.FINISH_WAIT_S = 0.5, 0.3, 1.0
FOREVER = 3600.0


def check(name, cond, info=""):
    print("%s  %s%s" % ("PASS" if cond else "FAIL", name, ("  (" + info + ")") if info else ""), flush=True)
    if not cond:
        F.append(name)


class V(object):
    def reset(self, **k):
        self.calls = []; self.armed = False; self.flying = False; self.land_t = None
        self.connect = "ok"; self.health = "ok"; self.pos = "ok"; self.arm = "ok"; self.arm_s = 0.0; self.start = "ok"
        self.inair_true_forever = False; self.fc_arms_anyway_after = None
        self.touch_after = 0.3; self.disarm = "ok"
        self.__dict__.update(k)
VV = V()
NS = types.SimpleNamespace


async def _forever():
    await asyncio.sleep(FOREVER)


class FAct(object):
    async def arm(self):
        VV.calls.append("arm")
        if VV.fc_arms_anyway_after is not None:          # the FC arms whatever happens to this RPC (it was sent)
            loop.call_later(VV.fc_arms_anyway_after, lambda: setattr(VV, "armed", True))
        if VV.arm == "hang":
            await _forever()
        if VV.arm == "timeout":                           # mavsdk_server got no ACK from the FC: result unknown
            from mavsdk.action import ActionError, ActionResult
            raise ActionError(ActionResult(ActionResult.Result.TIMEOUT, "no ack (test)"), "arm()")
        await asyncio.sleep(VV.arm_s); VV.armed = True
    async def disarm(self):
        VV.calls.append("disarm")
        if VV.disarm == "hang":
            await _forever()
        VV.armed = False
    async def land(self):
        VV.calls.append("land")
        if VV.land_t is None:
            VV.land_t = loop.time()
    async def kill(self):
        VV.calls.append("kill")


class FOff(object):
    async def set_position_ned(self, sp):
        pass
    async def start(self):
        VV.calls.append("offboard_start")
        if VV.start == "hang":
            await _forever()
        VV.flying = True
    async def stop(self):
        pass


class FTel(object):
    def _gen(self, kind, make, period=0.05):
        async def g():
            while True:
                if getattr(VV, kind) == "hang":
                    await _forever()
                await asyncio.sleep(period); yield make()
        return g()
    def health(self):
        return self._gen("health", lambda: NS(is_local_position_ok=True))
    def position_velocity_ned(self):
        n = lambda: random.uniform(-0.001, 0.001)  # noqa: E731
        return self._gen("pos", lambda: NS(position=NS(north_m=n(), east_m=n(), down_m=n()),
                                          velocity=NS(north_m_s=0.0, east_m_s=0.0, down_m_s=0.0)))
    def attitude_euler(self):
        return self._gen("pos", lambda: NS(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0))
    def attitude_angular_velocity_body(self):
        return self._gen("pos", lambda: NS(roll_rad_s=0.0, pitch_rad_s=0.0, yaw_rad_s=0.0))
    def in_air(self):
        async def g():
            while True:
                await asyncio.sleep(0.05)
                yield VV.inair_true_forever or (VV.flying and not (VV.land_t is not None and loop.time() > VV.land_t + VV.touch_after))
        return g()
    def armed(self):
        async def g():
            while True:
                await asyncio.sleep(0.05); yield VV.armed
        return g()


class FCore(object):
    def connection_state(self):
        async def g():
            if VV.connect == "hang":
                await _forever()
            while True:
                await asyncio.sleep(0.05); yield NS(is_connected=True)
        return g()


class FakeSystem(object):
    def __init__(self, *a, **k):
        self.action = FAct(); self.offboard = FOff(); self.telemetry = FTel(); self.core = FCore()
    async def connect(self, *a, **k):
        VV.calls.append("connect")
fs.System = FakeSystem
REAL_MISSION = C._sp_orig["execute_mission"] if hasattr(C, "_sp_orig") else C.execute_mission


if len(sys.argv) > 2 and sys.argv[1] == "--child":      # case R: the launcher's own main(), in this separate process
    async def _deaf(self):
        while True:
            try:
                await asyncio.sleep(0.2)
            except Exception:
                pass
    VV.reset(); C.execute_mission = _deaf               # the "flight script's own" mission the layer wraps
    fs.log = print
    _orig_install = SP.install

    def _install(fs_, st_, monitor=False):              # the layer's class guards / feed stamps also on the fake classes
        r = _orig_install(fs_, st_, monitor)
        SP.guard_mavsdk(st_, FAct, FOff, FTel, r)
        for key, nm in (("pos", "position_velocity_ned"), ("att", "attitude_euler")):
            o = getattr(FTel, "_o_" + nm, None) or getattr(FTel, nm); setattr(FTel, "_o_" + nm, o)

            def mk(key, o):
                def gen(self_t):
                    async def g():
                        async for v in o(self_t):
                            st_.rx[key] = loop.time(); yield v
                    return g()
                return gen
            setattr(FTel, nm, mk(key, o))
        return r
    SP.install = _install
    sys.exit(R.main(["--v13", ROOT, "--result", os.path.join(sys.argv[2], "mission_result.json"), "--", "--sim"]))


async def mission_hover(self):                    # a stand-in mission: hover until stopped (v13: aborted -> return)
    for _ in range(100):
        if self.aborted:
            return
        await asyncio.sleep(0.1)


async def mission_deaf(self):                     # a flight-script loop that swallows every cancel and ignores aborted
    while True:
        try:
            await asyncio.sleep(0.2)
        except Exception:                         # Python 3.6: CancelledError is an Exception
            pass


CUR = {}


def case(name, expect_exit, sigints=(), mission=None, result_path=None, prep=None, within=None, want=(), never=(), consts=None, **veh):
    VV.reset(**veh); LOG[:] = []; T0[0] = time.time()
    keep = {k: getattr(SP, k) for k in (consts or {})}
    for k, v in (consts or {}).items():
        setattr(SP, k, v)
    st = SP.SafetyState(); SP.install(fs, st, monitor=False); SP.guard_mavsdk(st, FAct, FOff, FTel, fs.log); CUR["st"] = st
    C._sp_orig["execute_mission"] = mission or REAL_MISSION
    args = fs.build_arg_parser().parse_args(["--sim", "--stop-on-fail", "--skip-preland"])
    args.plan_steps = fs.build_mission_plan(args)
    out = result_path or os.path.join(tempfile.mkdtemp(), "mission_result.json")
    if prep:
        prep(out)

    def entry():
        for s_ in sigints:                        # (after "<log text>", +s) or (seconds,)
            if isinstance(s_, tuple):
                txt, dt = s_

                def arm_sig(txt=txt, dt=dt):
                    if any(txt in m for m in LOG):
                        loop.call_later(dt, os.kill, os.getpid(), signal.SIGINT)
                    else:
                        loop.call_later(0.02, arm_sig)
                loop.call_soon(arm_sig)
            else:
                loop.call_later(s_, os.kill, os.getpid(), signal.SIGINT)
        loop.run_until_complete(fs.main_async(args))
    t0 = time.time(); code = R.run(fs, SP, st, out, entry=entry); dt = time.time() - t0
    for k, v in keep.items():
        setattr(SP, k, v)
    # "within": seconds from the first STOP (SIGINT received) to the result line - v13's own pre-flight waits come before it
    ts = lambda txt: next((float(m.split()[0]) for m in LOG if txt in m), None)  # noqa: E731
    t_stop, t_res = ts("SIGINT received"), ts("[result] EXIT ")
    if t_stop is not None and t_res is not None:
        dt = t_res - t_stop
    loop.run_until_complete(asyncio.sleep(0.3))
    try:
        res = json.load(open(out))
    except (OSError, ValueError):
        res = {}
    n_exit = sum(1 for m in LOG if "[result] EXIT " in m)
    probs = []
    if code != expect_exit:
        probs.append("exit %s, expected %s" % (code, expect_exit))
    if n_exit != 1:
        probs.append("%d '[result] EXIT' lines (one expected)" % n_exit)
    if within is not None and dt > within:
        probs.append("took %.1f s > %.1f s" % (dt, within))
    if "kill" in VV.calls:
        probs.append("kill sent")
    for w in want:
        if callable(w):
            if not w(res, VV.calls, LOG, st):
                probs.append("condition: %s" % w.__doc__)
        elif not any(w in m for m in LOG):
            probs.append("missing: %s" % w)
    for w in never:
        if (w[1:] in VV.calls) if w.startswith("=") else any(w in m for m in LOG):
            probs.append("must not appear: %s" % w)
    check("%s -> exit %d, %.1f s from the STOP to the result" % (name, expect_exit, dt), not probs, "; ".join(probs) + " | %s | %s | land %s disarm %s arm %s by %s"
          % (res.get("outcome"), (res.get("detail") or "")[:60], res.get("landing"), res.get("disarm"), res.get("arm_state"),
             res.get("stop_finished_by")) + " | " + ",".join(VV.calls))
    if probs:
        for m in LOG[-25:]:
            print("      log:", m[:170])
    for t in asyncio.Task.all_tasks():                # nothing of this case may run into the next one
        if not t.done():
            t.cancel()
    loop.run_until_complete(asyncio.sleep(0.2))
    return res, st


def cond(doc, f):
    f.__doc__ = doc; return f


H = SP.STOP_HANDOFF_S
NOT_STARTED = cond("not_started, no arm RPC, armed_requested False, stop finished by the supervisor",
                   lambda r, c, l, st: r.get("outcome") == "not_started" and "arm" not in c and r.get("armed_requested") is False
                   and r.get("stop_finished_by") == "supervisor")
# ---- S1: a main flow that never comes back from a wait
case("A connect never completes, SIGINT 0.5 s: supervisor cancels the main flow after the hand-off time", 4, sigints=[0.5],
     connect="hang", within=H + 3, want=[NOT_STARTED, "STOP supervisor: the main flow did not reach its landing step",
                                         "Exception: ", "stop: finished by the STOP supervisor"])   # v13's except after the cancel
case("B health stream silent (wait_position_stable), SIGINT", 4, sigints=[("[2/7]", 0.3)], health="hang", within=H + 3,
     want=[NOT_STARTED])
case("C position stream silent (capture_initial_mavsdk_position), SIGINT", 4, sigints=[("[4/7]", 0.3)], pos="hang",
     within=H + 3, want=[NOT_STARTED])
case("I connect never completes, SIGINT x 3: one stop, one result", 4, sigints=[0.3, 0.8, 1.6], connect="hang", within=H + 3,
     want=[NOT_STARTED, cond("3 SIGINTs logged", lambda r, c, l, st: sum("SIGINT received" in m for m in l) == 3)])
# ---- S2: STOP inside v13's own arm preparation (1 s, reach pipe, 2 s) - the arm RPC never goes out
case("D SIGINT 1.5 s into v13's arm preparation: no arm / offboard RPC, armed_requested False, not_started",
     4, sigints=[("[5/7]", 1.5)], within=15,
     want=["ARM refused", cond("arm_prep True, arm RPC 0, armed_requested False, not_started",
           lambda r, c, l, st: r.get("arm_prep") is True and "arm" not in c and r.get("armed_requested") is False
           and r.get("outcome") == "not_started")], never=["=arm", "=offboard_start"])
# ---- arm RPC without answer / late FC effect
case("E arm RPC never answers (the FC arms anyway 0.5 s later), SIGINT: arm unknown, touchdown not counted, hand-over",
     10, sigints=[("[6/7]", 0.3)], arm="hang", fc_arms_anyway_after=0.5, within=H + SP.LAND_CONFIRM_TIMEOUT + 4,
     want=["arm RPC cancelled before its answer", "OPERATOR HAND-OVER",
           cond("arm_state sent, landing NOT confirmed, note 'no answer', supervisor", lambda r, c, l, st: r.get("arm_state") == "sent"
                and r.get("landing", "").startswith("NOT") and any("no answer" in n for n in r.get("notes", []))
                and r.get("stop_finished_by") == "supervisor")], never=["=disarm", "=offboard_start"])
case("F offboard start never answers (armed), SIGINT: supervisor lands it, touchdown after the arm answer, disarm", 3,
     sigints=[("Starting offboard", 0.3)], start="hang", within=H + SP.LAND_CONFIRM_TIMEOUT + 4,
     want=["Disarmed.", cond("arm acked, landing confirmed, disarm done, supervisor", lambda r, c, l, st: r.get("arm_state") == "acked"
           and r.get("landing") == "confirmed" and r.get("disarm") == "done" and r.get("stop_finished_by") == "supervisor")])
# ---- main in its landing step, then stuck; main that ignores its cancel
case("G landing step entered, touchdown never seen: whole-stop limit, hand-over", 10, sigints=[("Offboard active", 1.0)],
     mission=mission_hover, inair_true_forever=True, within=SP.STOP_TOTAL_S + 3, consts={"LAND_CONFIRM_TIMEOUT": 30.0},
     want=["did not finish the stop within", "OPERATOR HAND-OVER", cond("finished by the supervisor",
           lambda r, c, l, st: r.get("stop_finished_by") == "supervisor")])
res, st = case("H mission loop swallows the cancel and ignores aborted: supervisor lands it (land accepted), event loop stopped", 3,
               sigints=[("Offboard active", 1.0)], mission=mission_deaf, within=H + SP.LAND_CONFIRM_TIMEOUT + 4,
               want=["did not end after its cancel - event loop stopped", "Disarmed.", cond(
                   "finished by the supervisor, the deaf main flow still running", lambda r, c, l, st: r.get("stop_finished_by") ==
                   "supervisor" and st.flow_task is not None and not st.flow_task.done())])
# ---- S3: result file
d = tempfile.mkdtemp(); os.chmod(d, 0o555)
case("K result directory not writable: nothing flies, exit 21", 21, result_path=os.path.join(d, "mission_result.json"),
     want=["NOT starting (no flight)"], never=["=connect", "=arm"])
os.chmod(d, 0o755)
old = {}


def put_old(p):
    json.dump({"run_id": "OLD", "exit": 0}, open(p, "w")); old["p"] = p
r_, st_ = case("L an earlier result file exists: moved aside, the new one has this run's id", 0, mission=mission_hover,
               prep=put_old, sigints=[], want=["Done!"], connect="ok")
prev = [f for f in os.listdir(os.path.dirname(old["p"])) if f.startswith("mission_result.json.prev_")]
check("L the earlier file is kept as .prev_*, the new file is this run's", len(prev) == 1 and r_.get("run_id") == st_.run_id
      and json.load(open(os.path.join(os.path.dirname(old["p"]), prev[0])))["run_id"] == "OLD", "%s, run id %s" % (prev, r_.get("run_id")))


def block_write(p):                               # a directory where the temp file must go: the write fails at the end
    os.makedirs(p + ".tmp")
case("M normal flight, result file not writable at the end: exit 21, no success EXIT", 21, mission=mission_hover, prep=block_write,
     want=["RESULT FILE NOT WRITTEN", "Done!"], never=["[result] EXIT 0 "])
case("N hand-over AND result file not writable: exit 10 (hand-over not hidden), file error logged", 10, prep=block_write,
     sigints=[("Offboard active", 1.0)], mission=mission_hover, inair_true_forever=True,
     want=["RESULT FILE NOT WRITTEN", "OPERATOR HAND-OVER"])
# ---- 2026-11-01 codex decisions reply-9: one hard end limit, late arm effects, the real process end
async def mission_hover_stubborn_task(self):      # hover; a helper task that swallows its cancel is the layer's st.task
    async def deaf():
        while True:
            try:
                await asyncio.sleep(0.2)
            except Exception:
                pass
    CUR["st"].task = asyncio.ensure_future(deaf())
    await mission_hover(self)
END = SP.STOP_END_S
case("O touchdown just before the whole-stop limit, then the disarm RPC hangs and a task ignores its cancel: result within "
     "STOP_END_S", 10, sigints=[("Offboard active", 1.0)], mission=mission_hover_stubborn_task, touch_after=5.5, disarm="hang",
     within=END + 0.3, consts={"LAND_CONFIRM_TIMEOUT": 30.0, "LAND_RPC_TIMEOUT": 30.0},
     want=["did not finish the stop within", "left behind", cond("disarm failed, landing confirmed, supervisor",
           lambda r, c, l, st: r.get("disarm") == "failed" and r.get("landing") == "confirmed" and r.get("stop_finished_by") == "supervisor")])
case("P arm RPC: no ACK from the FC (result unknown); the FC arms 0.8 s later, after the disarm: the disarm check sees it, hand-over",
     10, arm="timeout", fc_arms_anyway_after=0.8, consts={"DISARM_CONFIRM_S": 1.5},
     want=["ARMED AGAIN", cond("arm unknown, disarm done, check ARMED AGAIN, note", lambda r, c, l, st: r.get("arm_state") == "unknown"
           and r.get("disarm") == "done" and r.get("disarm_check") == "ARMED AGAIN" and any("True again" in n for n in r.get("notes", [])))])
case("P2 arm RPC: no ACK, the FC does not arm: disarm ack + armed telemetry False resolve the unknown arm (error exit 1, no hand-over)",
     1, arm="timeout", consts={"DISARM_CONFIRM_S": 1.5},
     want=[cond("arm unknown resolved: disarm done, check disarmed, no hand-over", lambda r, c, l, st: r.get("arm_state") == "unknown"
                and r.get("disarm_check") == "disarmed" and r.get("operator_handover") is False)])


async def late_ack():
    """unit: the arm RPC answers AFTER the landing's disarm (an RPC whose answer comes late is not cancelled here)"""
    VV.reset(arm_s=1.0); st = SP.SafetyState(); st.log = fs.log; SP.guard_mavsdk(st, FAct, FOff, FTel, fs.log)
    t_arm = asyncio.ensure_future(FAct().arm()); await asyncio.sleep(0.1)
    st.stopped = True; st.land_t0 = loop.time(); st.in_air, st.in_air_t = False, loop.time() + 0.01; await asyncio.sleep(0.05)
    st.touchdown = True; await FAct().disarm(); st.disarm_state = "done"; st.disarm_t = loop.time(); st.disarm_check = "disarmed"
    await t_arm
    return st.result(), st
r, st = loop.run_until_complete(late_ack())
check("Q arm ACK after the disarm -> hand-over 10 with the note 'arm acknowledged AFTER the disarm'",
      r["exit"] == 10 and any("AFTER the disarm" in n for n in r["notes"]), "%s %s" % (r["exit"], r["notes"]))

# R: the real launcher main() in its own process: deaf main flow -> supervisor -> event loop stopped -> os._exit
if True:
    import subprocess, threading
    out_dir = tempfile.mkdtemp()
    p = subprocess.Popen([sys.executable, "-u", os.path.abspath(__file__), "--child", out_dir], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
    p.stdin.write("\n"); p.stdin.flush()
    lines = []; t_sig = [None]; t_res = [None]

    def reader():
        for ln in p.stdout:
            lines.append(ln.rstrip())
            if "Offboard active" in ln and t_sig[0] is None:
                time.sleep(1.0); t_sig[0] = time.time(); p.send_signal(signal.SIGINT)
            if "[result] EXIT " in ln and t_res[0] is None:
                t_res[0] = time.time()
    th = threading.Thread(target=reader, daemon=True); th.start()
    try:
        rc = p.wait(timeout=90)
    except subprocess.TimeoutExpired:
        p.kill(); rc = "timeout"
    th.join(2); t_end = time.time()
    res = {}
    try:
        res = json.load(open(os.path.join(out_dir, "mission_result.json")))
    except (OSError, ValueError):
        pass
    n_exit = sum("[result] EXIT " in x for x in lines)
    dt = (t_end - t_sig[0]) if t_sig[0] else -1
    check("R real_flight_safe.main() process: deaf main flow, SIGINT -> supervisor result, process ends by os._exit with the result "
          "code (rc %s, %.1f s from the SIGINT to the process end, limit STOP_END_S %.0f + 1)" % (rc, dt, END),
          rc == res.get("exit") == 3 and n_exit == 1 and 0 < dt <= END + 1.0 and res.get("stop_finished_by") == "supervisor"
          and any("event loop stopped" in x for x in lines), "%s | %s" % (res.get("outcome"), [x for x in lines if "[result]" in x][-3:]))
    if rc != 3:
        for x in lines[-30:]:
            print("      child:", x[:170])
check("path_flight_phase1_v13.py unchanged", hashlib.sha256(open(V13, "rb").read()).hexdigest() == H0, H0[:16])
print("%d failure(s)" % len(F)); sys.exit(len(F))
