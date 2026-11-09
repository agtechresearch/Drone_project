#!/usr/bin/env python3
"""Mission result and landing ownership through the REAL flight script's main_async (2026-11-01, codex
real-mission-result-contract / offboard-fallback-contract reply-2 / graceful-stop-kill reply-8 / preflight-stop-before-arm reply-2).
path_flight_phase1_v13.py is imported unchanged; its main_async, its SIGINT handler and its exception path run for real through
real_flight_safe.run() (run_until_complete + safety_patch.finish + result JSON). Only the vehicle is fake: connect() gives a fake
MAVSDK System whose action / offboard / telemetry classes carry the safety layer's class-level guards, the flight script's
arm_and_start_offboard / execute_mission / go_to_relative (the originals the layer wraps) are short stand-ins with v13's
stop_on_fail / aborted handling. A real SIGINT (os.kill) goes to v13's own handler.
Run in starling-flight (Python 3.6, MAVSDK 0.12):  cd ~/workspace/v3_marker && python3 tests/test_mission_result.py"""
import asyncio, hashlib, json, os, signal, sys, tempfile, types
HERE = os.path.dirname(os.path.abspath(__file__)); SC = os.path.join(HERE, "..", "scripts"); ROOT = os.path.join(HERE, "..", "..")
sys.path.insert(0, SC); sys.path.insert(0, ROOT)
V13 = os.path.join(ROOT, "path_flight_phase1_v13.py"); H0 = hashlib.sha256(open(V13, "rb").read()).hexdigest()
import path_flight_phase1_v13 as fs  # noqa: E402
import safety_patch as SP  # noqa: E402
import real_flight_safe as R  # noqa: E402
from mavsdk.action import ActionError, ActionResult  # noqa: E402
F = []; LOG = []; fs.log = lambda m: LOG.append(str(m))
loop = asyncio.get_event_loop(); C = fs.DroneController
SP.LAND_RPC_TIMEOUT, SP.LAND_RETRY_S, SP.LAND_CONFIRM_TIMEOUT = 0.3, 0.3, 4.0


def check(name, cond, info=""):
    print("%s  %s%s" % ("PASS" if cond else "FAIL", name, ("  (" + info + ")") if info else ""))
    if not cond:
        F.append(name)


def denied():
    return ActionError(ActionResult(ActionResult.Result.COMMAND_DENIED, "denied (test)"), "land()")


class V(object):
    """the fake vehicle of one case"""
    def reset(self, **k):
        self.calls = []; self.armed = False; self.flying = False; self.land_t = None
        self.land = "ok"; self.deny_n = 0; self.disarm = "ok"; self.arm_delay = 0.0; self.arm_gap = 0.0; self.connect_s = 0.1
        self.stages = ["takeoff", "forward_1", "right_1", "left_1"]; self.fail = set(); self.raise_at = None
        self.__dict__.update(k)
VV = V()


class FA(object):
    async def arm(self):
        VV.calls.append("arm"); await asyncio.sleep(VV.arm_delay); VV.armed = True
    async def land(self):
        VV.calls.append("land")
        if VV.land == "hang":
            await asyncio.sleep(100)
        if VV.land == "deny" or VV.deny_n > 0:
            VV.deny_n -= 1; raise denied()
        if VV.land_t is None:
            VV.land_t = loop.time()
    async def disarm(self):
        VV.calls.append("disarm")
        if VV.disarm == "fail":
            raise ActionError(ActionResult(ActionResult.Result.COMMAND_DENIED, "disarm denied (test)"), "disarm()")
        VV.armed = False
    async def kill(self):
        VV.calls.append("kill")


class FO(object):
    async def start(self):
        VV.calls.append("offboard_start"); VV.flying = True


class FT(object):
    def in_air(self):
        async def g():
            while True:
                await asyncio.sleep(0.05)
                yield VV.flying and not (VV.land_t is not None and loop.time() > VV.land_t + 0.3)
        return g()


async def connect(self):
    self.drone = types.SimpleNamespace(action=FA(), offboard=FO(), telemetry=FT()); await asyncio.sleep(VV.connect_s)
async def short(self, *a, **k):
    await asyncio.sleep(0.05)
async def arm_orig(self):                         # v13's arm_and_start_offboard: arm, (prepare), offboard start
    await self.drone.action.arm(); await asyncio.sleep(VV.arm_gap); await self.drone.offboard.start()
async def goto_orig(self, n, e, alt, name, label, speed_mps=None):
    if self.aborted:
        return False
    VV.calls.append("stage " + name); await asyncio.sleep(0.15)
    if name == VV.raise_at:
        raise RuntimeError("synthetic failure in %s" % name)
    return name not in VV.fail and not self.aborted
async def mission_orig(self):                     # v13's structure: stage, aborted -> return, stop_on_fail -> break
    for name in VV.stages:
        ok = await self.go_to_relative(0.0, 0.0, 1.0, name, "X")
        if self.aborted:
            return
        if self.args.stop_on_fail and not ok:
            fs.log("!!! stop_on_fail: {} did not pass. Mission will stop before next step.".format(name)); break
    fs.log("CUSTOM MISSION COMPLETE - position before preland descent")


for n, f in (("connect", connect), ("wait_position_stable", short), ("check_vio_quality_preflight", short),
             ("capture_initial_mavsdk_position", short)):
    setattr(C, n, f)


def case(name, expect_exit, sigint_at=None, argv=(), want=(), never=(), **vehicle):
    VV.reset(**vehicle); LOG[:] = []
    st = SP.SafetyState(); SP.install(fs, st, monitor=False); SP.guard_mavsdk(st, FA, FO, FT, fs.log)
    O = C._sp_orig; O["arm_and_start_offboard"] = arm_orig; O["execute_mission"] = mission_orig; O["go_to_relative"] = goto_orig
    args = fs.build_arg_parser().parse_args(["--sim"] + list(argv)); args.plan_steps = []
    out = os.path.join(tempfile.mkdtemp(), "mission_result.json")

    def entry():
        if sigint_at is not None:
            loop.call_later(sigint_at, os.kill, os.getpid(), signal.SIGINT)
        loop.run_until_complete(fs.main_async(args))
    code = R.run(fs, SP, st, out, entry=entry)
    loop.run_until_complete(asyncio.sleep(0.2))
    try:
        res = json.load(open(out))
    except (OSError, ValueError):
        res = {}
    probs = []
    if code != expect_exit or res.get("exit") != expect_exit:
        probs.append("exit %s / json %s, expected %s" % (code, res.get("exit"), expect_exit))
    if not any(("[result] EXIT %d" % expect_exit) in m for m in LOG):
        probs.append("no [result] EXIT line")
    if "kill" in VV.calls:
        probs.append("kill sent")
    for w in want:
        if callable(w):
            if not w(res, VV.calls, LOG):
                probs.append("condition: %s" % w.__doc__)
        elif not any(w in m for m in LOG + VV.calls):
            probs.append("missing: %s" % w)
    for w in never:                               # "=name": an exact vehicle call, else a text in the log
        if (w[1:] in VV.calls) if w.startswith("=") else any(w in m for m in LOG):
            probs.append("must not appear: %s" % w)
    leftover = [t for t in (st.land_retry_task, st.inair_task, st.land_task, st.task) if t is not None and not t.done()]
    if leftover:
        probs.append("%d task(s) of the layer still running after finish" % len(leftover))
    check("%s -> exit %d" % (name, expect_exit), not probs, "; ".join(probs) + " | %s | %s | land %s disarm %s" % (
        res.get("outcome"), res.get("detail"), res.get("landing"), res.get("disarm")) + " | " + ",".join(VV.calls))
    return res


def cond(doc, f):
    f.__doc__ = doc; return f


case("normal: all stages pass", 0, want=["Done!", cond("outcome complete, landing confirmed, disarm done",
     lambda r, c, l: r["outcome"] == "complete" and r["landing"] == "confirmed" and r["disarm"] == "done")],
     never=["EMERGENCY LAND"])
case("stage failure, --stop-on-fail --skip-preland: no later stage, failure kept after the landing", 2,
     argv=["--stop-on-fail", "--skip-preland"], fail={"forward_1"},
     want=[cond("first failed stage forward_1, landed, disarmed", lambda r, c, l: r["first_failed_stage"] == "forward_1"
                and r["landing"] == "confirmed" and r["disarm"] == "done"),
           cond("no stage after forward_1", lambda r, c, l: "stage right_1" not in c and "stage left_1" not in c)],
     never=["EMERGENCY LAND"])
case("stage failure without --stop-on-fail: goes on, the FIRST failure is kept", 2, fail={"forward_1", "left_1"},
     want=[cond("first failed forward_1, all four stages flown", lambda r, c, l: r["first_failed_stage"] == "forward_1"
                and r["detail"].startswith("forward_1") and "stage left_1" in c)])
case("SIGINT during the mission: STOP -> land -> touchdown -> disarm", 3, sigint_at=0.55,
     want=["SIGINT received", "Disarmed.", cond("safety_stop, landed", lambda r, c, l: r["outcome"] == "safety_stop" and r["landing"] == "confirmed")])
case("land DENIED by the FC for ever (in_air stays True): retries until the deadline, no disarm, hand-over", 10, land="deny",
     want=["TOUCHDOWN NOT CONFIRMED", "land retries stop here", "OPERATOR HAND-OVER",
           cond("failure kinds only fc_denied, > 3 tries, disarm not tried", lambda r, c, l: {f["kind"] for f in r["land_failures"]} == {"fc_denied"}
                and c.count("land") > SP.LAND_TRIES and r["disarm"] is None and r["outcome"] == "complete")],
     never=["Done!", "Disarmed."])
case("land RPC never answers: rpc_timeout (FC state unknown), hand-over", 10, land="hang",
     want=["FC state unknown", cond("failure kinds only rpc_timeout", lambda r, c, l: {f["kind"] for f in r["land_failures"]} == {"rpc_timeout"})],
     never=["Disarmed."])
case("land denied by the 3 fast tries and the 1st extra retry, accepted by the 2nd extra retry (late recovery): landed, disarmed, mission complete", 0, deny_n=4,
     want=["Land command sent (retry", cond("land request 'sent after retry', 4 failures recorded", lambda r, c, l:
           r["land_request"].startswith("sent after retry") and len(r["land_failures"]) == 4 and r["landing"] == "confirmed")])
case("touchdown confirmed but the FC refuses the disarm: hand-over, not success", 10, disarm="fail",
     want=["Disarm note", cond("landing confirmed, disarm failed", lambda r, c, l: r["landing"] == "confirmed" and r["disarm"] == "failed")],
     never=["Done!"])
case("exception inside the mission (v13 except -> emergency_stop -> sys.exit(1)): error kept, landed, disarmed", 1, raise_at="right_1",
     want=[cond("outcome error with the exception text, landed, disarmed", lambda r, c, l: r["outcome"] == "error"
                and "synthetic failure in right_1" in r["detail"] and r["landing"] == "confirmed" and r["disarm"] == "done")])
# STOP at every start-up boundary (codex preflight-stop-before-arm reply-2)
case("SIGINT while connecting: no arm / offboard RPC, not started", 4, sigint_at=0.1, connect_s=0.5,
     want=["arming skipped"], never=["=arm", "=offboard_start"])
case("SIGINT in the pre-arm waits (after connect): no arm / offboard RPC, not started", 4, sigint_at=0.16, connect_s=0.1,
     want=["arming skipped"], never=["=arm", "=offboard_start"])
case("SIGINT while the arm RPC is under way: armed, offboard start refused, land, touchdown, disarm", 3, sigint_at=0.4, arm_delay=0.5,
     want=["START refused", "Disarmed.", cond("arm sent, landed, disarmed", lambda r, c, l: "arm" in c and r["landing"] == "confirmed"
                                                and r["disarm"] == "done")], never=["=offboard_start"])
case("SIGINT after arming, before the offboard start: offboard start refused, land, disarm", 3, sigint_at=0.5, arm_gap=0.5,
     want=["START refused", "Disarmed."], never=["=offboard_start"])
# the cancel at the program end arrives WHILE a land RPC waits for its answer (1921p14 i4: Python 3.6's CancelledError is an
# Exception, the retry loop swallowed it and finish() waited for ever)
async def cancel_in_rpc():
    VV.reset(land="hang", flying=True); keep = (SP.LAND_RPC_TIMEOUT, SP.LAND_RETRY_S); SP.LAND_RPC_TIMEOUT, SP.LAND_RETRY_S = 30.0, 0.05
    st = SP.SafetyState(); SP.install(fs, st, monitor=False); SP.guard_mavsdk(st, FA, FO, FT, fs.log)
    c = C(fs.build_arg_parser().parse_args(["--sim"])); await connect(c); st.arm_sent = True
    t = asyncio.ensure_future(c.trigger_emergency_land("test")); await asyncio.sleep(0.5)      # fast try 1 is inside its 30 s RPC
    t0 = loop.time(); res = await SP.finish(st); dt = loop.time() - t0
    SP.LAND_RPC_TIMEOUT, SP.LAND_RETRY_S = keep
    return dt, st, t, res
dt, st, t, res = loop.run_until_complete(cancel_in_rpc())
check("program end while the land RPC hangs: the landing task ends at once (%.2f s), result NOT confirmed, exit 10" % dt,
      dt < 1.0 and st.land_task.done() and res["exit"] == 10, "landing done %s, exit %s" % (st.land_task.done(), res["exit"]))
async def cancel_in_retry_rpc():
    VV.reset(land="deny", flying=True); keep = (SP.LAND_RPC_TIMEOUT, SP.LAND_RETRY_S); SP.LAND_RPC_TIMEOUT, SP.LAND_RETRY_S = 0.05, 0.05
    st = SP.SafetyState(); SP.install(fs, st, monitor=False); SP.guard_mavsdk(st, FA, FO, FT, fs.log)
    c = C(fs.build_arg_parser().parse_args(["--sim"])); await connect(c); st.arm_sent = True
    await c.trigger_emergency_land("test")                     # the 3 fast tries are denied, the retry task runs
    VV.land = "hang"; SP.LAND_RPC_TIMEOUT = 30.0; await asyncio.sleep(0.4)   # the next retry is inside its 30 s RPC
    n = VV.calls.count("land"); t0 = loop.time(); res = await SP.finish(st); dt = loop.time() - t0
    await asyncio.sleep(0.3); n2 = VV.calls.count("land")
    SP.LAND_RPC_TIMEOUT, SP.LAND_RETRY_S = keep
    return dt, st, n, n2, res
dt, st, n, n2, res = loop.run_until_complete(cancel_in_retry_rpc())
check("program end while a land RETRY RPC hangs: retries stop (%d land calls before, %d after), finish %.2f s, exit 10" % (n, n2, dt),
      dt < 1.0 and n > SP.LAND_TRIES and st.land_retry_task.done() and n2 == n and res["exit"] == 10, "retry done %s" % st.land_retry_task.done())
async def stubborn():
    st = SP.SafetyState(); st.log = fs.log; keep = SP.FINISH_WAIT_S; SP.FINISH_WAIT_S = 0.3
    async def ignore_cancel():
        while True:
            try:
                await asyncio.sleep(10)
            except asyncio.CancelledError:
                pass
    st.task = asyncio.ensure_future(ignore_cancel()); await asyncio.sleep(0.05); LOG[:] = []
    t0 = loop.time(); await SP.finish(st); dt = loop.time() - t0; SP.FINISH_WAIT_S = keep
    return dt
dt = loop.run_until_complete(stubborn())
check("a task that ignores its cancel holds the program end only FINISH_WAIT_S (%.2f s) and is logged" % dt,
      dt < 0.6 and any("did not end within" in m for m in LOG))
# an in_air=False received BEFORE the land request (on the ground before take-off) is no touchdown: disarm refused; a False
# after the request: disarm sent (codex graceful-stop-kill reply-8)
async def stale_false():
    VV.reset(); st = SP.SafetyState(); SP.install(fs, st, monitor=False); SP.guard_mavsdk(st, FA, FO, FT, fs.log)
    st.in_air, st.in_air_t = False, loop.time(); await asyncio.sleep(0.05); st.land_t0 = loop.time()
    r1 = "sent"
    try:
        await FA().disarm()
    except RuntimeError:
        r1 = "refused"
    st.in_air, st.in_air_t = False, loop.time()
    r2 = "sent"
    try:
        await FA().disarm()
    except RuntimeError:
        r2 = "refused"
    return r1, r2, list(VV.calls)
r1, r2, calls = loop.run_until_complete(stale_false())
check("disarm guard: in_air False from before the land request -> refused; False after it -> sent",
      r1 == "refused" and r2 == "sent" and calls == ["disarm"], "%s / %s, %s" % (r1, r2, calls))
# report() itself interrupted (codex decisions reply-9 S3): the standard "RESULT FILE NOT WRITTEN" line, then the wk chain
# run_status.sh -> status.json -> batch_expect.py keeps the error next to a normal (0) or an expected (13) mission result
import subprocess  # noqa: E402
st = SP.SafetyState(); st.log = fs.log; LOG[:] = []; st.arm_sent = True; st.touchdown = True; st.disarm_state = "done"
st.set_outcome("complete", "test"); keep_finish = SP.finish
async def broken_finish(*a, **k):
    raise RuntimeError("synthetic: end step interrupted")
SP.finish = broken_finish
res = SP.report(st, fs.log, os.path.join(tempfile.mkdtemp(), "mission_result.json")); SP.finish = keep_finish
line = [m for m in LOG if "RESULT FILE NOT WRITTEN" in m]
check("report interrupted: standard RESULT FILE NOT WRITTEN line, exit 21 (not 0)", res["exit"] == 21 and len(line) == 1, "%s %s" % (res["exit"], line))
chain = []
for name, extra, want_rc in (("normal", "Done!", "16"), ("safety", "!!! EMERGENCY LAND TRIGGERED: test !!!", "13")):
    d = tempfile.mkdtemp(); open(os.path.join(d, "mission.log"), "w").write("[result] run id R9, result file x\n%s\n%s\n" % (extra, line[0] if line else ""))
    rc = subprocess.call(["bash", os.path.join(SC, "run_status.sh"), d, "0", "0", "qvix_rot", "ok", "PASS", "ok"], stdout=subprocess.DEVNULL)
    stj = json.load(open(os.path.join(d, "status.json")))
    chain.append((name, rc, stj.get("result_file")))
check("wk chain: that line -> run_status 16 next to a normal result, 13 + result_file not_written next to a safety landing",
      chain == [("normal", 16, "not_written"), ("safety", 13, "not_written")], str(chain))
check("path_flight_phase1_v13.py unchanged", hashlib.sha256(open(V13, "rb").read()).hexdigest() == H0, H0[:16])
print("%d failure(s)" % len(F)); sys.exit(len(F))
