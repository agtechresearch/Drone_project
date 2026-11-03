#!/usr/bin/env python3
"""The safety layer (safety_patch.py) on the REAL flight script's DroneController class (2026-11-01). No flight: the
controller's MAVSDK System is replaced by a stub after construction. Checks that the flight script FILE is not changed,
that the script's own stop kills the motors (the risk this layer removes) while the patched stop only lands, that the
real-vehicle monitor lands when the VOXL pipe stops, and that real_flight_safe.py passes the v13 arguments through.
Run in starling-flight (Python 3.6, MAVSDK 0.12):  cd ~/workspace/v3_marker && python3 tests/test_real_flight_safe.py"""
import asyncio, hashlib, os, subprocess, sys, types
HERE = os.path.dirname(os.path.abspath(__file__)); SC = os.path.join(HERE, "..", "scripts"); ROOT = os.path.join(HERE, "..", "..")
V13 = os.path.join(ROOT, "path_flight_phase1_v13.py")
sys.path.insert(0, SC); sys.path.insert(0, ROOT)
F = []


def check(name, cond, info=""):
    print("%s  %s%s" % ("PASS" if cond else "FAIL", name, ("  (" + info + ")") if info else ""))
    if not cond:
        F.append(name)


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


H0 = sha(V13)
import path_flight_phase1_v13 as fs  # noqa: E402
import safety_patch as SP  # noqa: E402
LOG = []; fs.log = lambda m: LOG.append(m)
C = fs.DroneController
orig_stop = C.emergency_stop                                  # the flight script's own, before the layer
args = fs.build_arg_parser().parse_args(["--sim"])


class Drone(object):
    def __init__(self):
        self.calls = []
        self.action = types.SimpleNamespace(land=self._land, kill=self._kill)
        self.telemetry = types.SimpleNamespace(in_air=self._in_air)
    async def _land(self): self.calls.append("land")
    async def _kill(self): self.calls.append("kill")
    def _in_air(self):
        async def g():
            for v in (True, True, False):
                await asyncio.sleep(0.05); yield v
        return g()


def ctrl():
    c = C(args); c.drone = Drone()
    async def nothing(): return None
    c._cancel_background_tasks = nothing
    return c


loop = asyncio.get_event_loop()
# 1. the flight script's own stop: land, then kill 0.5 s later (what the real vehicle does today on Ctrl+C)
c = ctrl(); loop.run_until_complete(orig_stop(c))
check("flight script's own emergency_stop: land AND motor kill (the risk)", c.drone.calls == ["land", "kill"], str(c.drone.calls))
# 2. with the layer: land, touchdown wait, no kill
st = SP.SafetyState(); SP.install(fs, st, monitor=True)
c = ctrl(); loop.run_until_complete(c.emergency_stop())
check("with safety_patch: emergency_stop lands and waits, no motor kill", c.drone.calls == ["land"] and st.land_state == "sent",
      "%s, %s" % (c.drone.calls, [m for m in LOG if "stop:" in m][-1:]))
check("every DroneController method replaced at run time keeps the original reachable",
      set(C._sp_orig) == {"feedback_state", "wait_until_yaw_reached", "land_and_disarm", "trigger_emergency_land",
                          "emergency_stop", "arm_and_start_offboard", "go_to_relative", "go_yaw", "execute_mission"})
# 3. monitor (real vehicle): the VOXL pipe's own timestamp stops changing -> land
st = SP.SafetyState(); SP.install(fs, st, monitor=True)
c = ctrl()
async def arm_stub(self): return True
C._sp_orig["arm_and_start_offboard"] = arm_stub                # no real arming in the test


async def pipe_then_stop():
    t0 = loop.time()
    while loop.time() - t0 < 0.5:                               # 0.5 s of pipe messages, then nothing
        c.voxl_latest[c.reach_pipe] = {"voxl_ts_ms": loop.time() * 1000.0, "pos_x": 0.0, "pos_y": 0.0, "pos_z": -0.7, "yaw_deg": 0.0}
        await asyncio.sleep(0.03)
    await asyncio.sleep(2.0)


async def run3():
    await c.arm_and_start_offboard()
    t0 = loop.time(); await pipe_then_stop()
    return [loop.time() - t0]
loop.run_until_complete(run3())
check("monitor: VOXL pipe stops after 0.5 s -> land (%.1f s stale limit)" % SP.FEED_STALE_S,
      "land" in c.drone.calls and "kill" not in c.drone.calls and "feedback" in (st.land_reason or ""), "%s, %s" % (c.drone.calls, st.land_reason))
# ---- 2026-11-01 codex reply-6 / new open items: guards, landing disarm, pre-arm STOP, real feedback source, kill authority
class FA(object):
    calls = []
    async def arm(self): FA.calls.append("arm")
    async def disarm(self): FA.calls.append("disarm")
    async def kill(self): FA.calls.append("kill")
    async def land(self): FA.calls.append("land")
class FO(object):
    async def start(self): FA.calls.append("offboard_start")
class FT(object):
    seq = []
    def in_air(self):
        async def g():
            for v in list(FT.seq):
                await asyncio.sleep(0.02); yield v
            while FT.forever is not None:
                await asyncio.sleep(0.02); yield FT.forever
        return g()
FT.forever = None
gst = SP.SafetyState(); glog = []
SP.guard_mavsdk(gst, FA, FO, FT, glog.append)


def rpc(coro):
    try:
        loop.run_until_complete(coro); return "ok"
    except RuntimeError as e:
        return "refused: %s" % e


async def drain(n):
    k = 0
    async for _ in FT().in_air():
        k += 1
        if k >= n:
            break
FA.calls = []
r1 = rpc(FA().disarm())
FT.seq = [True]; loop.run_until_complete(drain(1)); r2 = rpc(FA().disarm())
FT.seq = [True, True, False]; loop.run_until_complete(drain(3)); r3 = rpc(FA().disarm())
check("guard disarm: no in_air seen -> refused, last True -> refused, True..False -> sent",
      r1.startswith("refused") and r2.startswith("refused") and r3 == "ok" and FA.calls == ["disarm"], "%s | %s | %s" % (r1, r2, r3))
FA.calls = []; gst.stopped = False; ra = rpc(FA().arm()); gst.stopped = True; rb = rpc(FA().arm()); rc = rpc(FO().start())
check("guard arm / offboard start: allowed before a STOP, refused after", ra == "ok" and rb.startswith("refused") and rc.startswith("refused")
      and FA.calls == ["arm"], "%s | %s | %s" % (ra, rb, rc))
FA.calls = []; SP.ALLOW_KILL = False; k1 = rpc(FA().kill()); SP.ALLOW_KILL = True; k2 = rpc(FA().kill()); SP.ALLOW_KILL = False
check("guard kill: refused without ALLOW_KILL, sent with it", k1.startswith("refused") and k2 == "ok" and FA.calls == ["kill"], "%s | %s" % (k1, k2))

# the landing step (replaced, 2026-11-01 codex real-mission-result-contract) with in_air True forever: no disarm after the
# touchdown deadline; True..False: disarm after the touchdown. A fresh state per case (one flight = one state).
keep = SP.LAND_CONFIRM_TIMEOUT; SP.LAND_CONFIRM_TIMEOUT = 1.0


def ctrl_g():
    c = C(args); c.drone = types.SimpleNamespace(action=FA(), offboard=FO(), telemetry=FT())
    c.initial_x = c.initial_y = c.initial_z = 0.0; c.mav_x = c.mav_y = c.mav_z = 0.0
    async def nothing(): return None
    c._cancel_background_tasks = nothing; c._write_csv_if_needed = lambda: None; c._print_summary = lambda: None
    return c
for seq, forever, expect in (([], True, "refused"), ([True, True], False, "sent")):
    st = SP.SafetyState(); SP.install(fs, st, monitor=False); SP.guard_mavsdk(st, FA, FO, FT, fs.log); st.arm_sent = True
    FA.calls = []; FT.seq = seq; FT.forever = forever; LOG[:] = []
    c = ctrl_g(); c.aborted = True; st.set_outcome("safety_stop", "test")   # as after an emergency landing
    loop.run_until_complete(c.land_and_disarm()); FT.forever = None
    res = loop.run_until_complete(SP.finish(st))
    got = "sent" if "disarm" in FA.calls else "refused"
    check("landing step, in_air %s -> land 1 x, disarm %s%s, exit %d" % ("True for ever" if forever else "True..False", expect,
          " + TOUCHDOWN NOT CONFIRMED" if expect == "refused" else "", 10 if expect == "refused" else 3),
          got == expect and FA.calls.count("land") == 1 and (expect == "sent" or any("TOUCHDOWN NOT CONFIRMED" in m for m in LOG))
          and res["exit"] == (10 if expect == "refused" else 3) and res["operator_handover"] == (expect == "refused"),
          "%s; exit %s; %s" % (FA.calls, res["exit"], [m for m in LOG if "isarm" in m][-2:]))
SP.LAND_CONFIRM_TIMEOUT = keep

# STOP before arming (preflight): the flight script's arm_and_start_offboard must not reach action.arm / offboard.start
st = SP.SafetyState(); SP.install(fs, st, monitor=True); SP.guard_mavsdk(st, FA, FO, FT, fs.log)
C._sp_orig["arm_and_start_offboard"] = fs.DroneController._sp_orig_arm_real if hasattr(fs.DroneController, "_sp_orig_arm_real") else C._sp_orig["arm_and_start_offboard"]
FA.calls = []; c = ctrl_g(); FT.seq = [False]
loop.run_until_complete(c.emergency_stop())
r = rpc(c.arm_and_start_offboard())
check("STOP in preflight, then the flight script tries to arm -> refused, no arm / offboard RPC",
      r.startswith("refused") and "arm" not in FA.calls and "offboard_start" not in FA.calls, "%s; %s" % (r, FA.calls))

# real feedback source: MAVSDK position keeps coming (v13's own monitor), the VOXL pipe decides
def voxl_case(name, pipe, mav=True, expect_land=True, secs=2.0):
    st = SP.SafetyState(); SP.install(fs, st, monitor=True); SP.guard_mavsdk(st, FA, FO, FT, fs.log)
    async def arm_stub(self): return True
    C._sp_orig["arm_and_start_offboard"] = arm_stub
    c = ctrl(); c.args.sim = False
    async def run():
        await c.arm_and_start_offboard(); t0 = loop.time(); i = 0
        while loop.time() - t0 < secs:
            t = loop.time() - t0; i += 1
            if mav:
                st.rx["pos"] = loop.time(); st.rx_patched = True          # what the MAVSDK stamp does
            v = pipe(t, i)
            if v is not None:
                c.voxl_latest[c.reach_pipe] = v
            await asyncio.sleep(0.03)
    loop.run_until_complete(run())
    loop.run_until_complete(asyncio.sleep(0.5))                         # let a landing that started near the end send its command
    landed = "land" in c.drone.calls
    c.aborted = True; loop.run_until_complete(asyncio.sleep(0.1))      # end this case's monitor (one install per process in real use)
    check("real source: %s -> %s" % (name, "land" if expect_land else "no alarm"), landed == expect_land and st.source == "voxl",
          "source %s, %s, reason %s, bad stamps %d" % (st.source, c.drone.calls, st.land_reason, st.voxl_bad_ts))
P = lambda t, i, ts=None, yaw=0.0: {"voxl_ts_ms": i * 33.0 if ts is None else ts, "pos_x": 0.0, "pos_y": 0.0, "pos_z": -0.7, "yaw_deg": yaw}  # noqa: E731
voxl_case("VOXL and MAVSDK both fine (2 s)", lambda t, i: P(t, i), expect_land=False)
voxl_case("VOXL stops at 0.5 s, MAVSDK position goes on", lambda t, i: P(t, i) if t < 0.5 else None)
voxl_case("VOXL stamp repeats from 0.5 s (reader re-emits)", lambda t, i: P(t, i, ts=None if t < 0.5 else 999999.0) if t < 0.5 else P(t, i, ts=16.0 * 33.0))
voxl_case("VOXL stamp jumps back at 0.5 s (restart) and stays behind", lambda t, i: P(t, i) if t < 0.5 else P(t, i, ts=float(i)))
voxl_case("VOXL attitude NaN from 0.5 s (position fine)", lambda t, i: P(t, i) if t < 0.5 else P(t, i, yaw=float("nan")))

# kill authority of the real launcher: environment x command line
for envv in (None, "0", "1"):
    for cli in (False, True):
        env = dict(os.environ); env.pop("WK_ALLOW_KILL", None)
        if envv is not None:
            env["WK_ALLOW_KILL"] = envv
        p = subprocess.run([sys.executable, os.path.join(SC, "real_flight_safe.py"), "--v13", ROOT] + (["--allow-kill"] if cli else []) + ["--", "--help"],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, timeout=60, env=env)
        want = "software kill ALLOWED" if cli else "software kill never"
        check("kill authority: WK_ALLOW_KILL=%s, --allow-kill %s -> %s" % (envv, "yes" if cli else "no", want.split()[-1]),
              want in p.stdout and (envv is None or "IGNORED" in p.stdout))

# 4. the flight script file is unchanged by all of this
check("path_flight_phase1_v13.py unchanged (sha256 before = after)", sha(V13) == H0, H0[:16])
# 5. the launcher passes the flight script's own arguments through (--help: prints v13's usage, flies nothing)
p = subprocess.run([sys.executable, os.path.join(SC, "real_flight_safe.py"), "--v13", ROOT, "--", "--help"],
                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, timeout=60)
check("real_flight_safe.py -- --help: safety layer line, then the flight script's usage",
      "[safety] real_flight_safe" in p.stdout and "software kill never" in p.stdout and "--altitude" in p.stdout, p.stdout.splitlines()[0][:120] if p.stdout else "")
p = subprocess.run([sys.executable, os.path.join(SC, "real_flight_safe.py"), "--v13", ROOT, "--allow-kill", "--", "--help"],
                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, timeout=60)
check("--allow-kill is reported", "software kill ALLOWED" in p.stdout)

# 6. --own-server (2026-11-01): the launcher's own mavsdk_server ends with the launcher, kill -9 included (PR_SET_PDEATHSIG).
# Stand-in server: a script whose process is named mavsdk_server (exec -a) and only sleeps.
import tempfile, time, signal as _sig
T = tempfile.mkdtemp(); FAKE = os.path.join(T, "mavsdk_server")
open(FAKE, "w").write('#!/bin/bash\nexec -a "$0" sleep 300\n'); os.chmod(FAKE, 0o755)


def dead(pid):
    try:
        return open("/proc/%d/stat" % pid).read().rsplit(")", 1)[1].split()[0] in ("Z", "X")
    except OSError:
        return True


code = ("import sys, time; sys.path.insert(0, %r); import real_flight_safe as R; "
        "p = R.start_own_server('udp://:1', %r, 50999); print(p.pid, flush=True); time.sleep(300)") % (SC, FAKE)
L = subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE, universal_newlines=True)
spid = int(L.stdout.readline())
alive_before = not dead(spid)
os.kill(L.pid, _sig.SIGKILL); L.wait()
t0 = time.time()
while not dead(spid) and time.time() - t0 < 3:
    time.sleep(0.05)
check("own server: launcher killed with kill -9 -> its mavsdk_server ends too", alive_before and dead(spid),
      "server %d alive before %s, dead %.2f s after the kill" % (spid, alive_before, time.time() - t0))
O = subprocess.Popen([FAKE]); time.sleep(0.3)               # another mavsdk_server is running
p = subprocess.run([sys.executable, os.path.join(SC, "real_flight_safe.py"), "--v13", ROOT, "--own-server", "udp://:1",
                    "--server-bin", FAKE, "--", "--help"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, timeout=60)
check("own server: refused while another mavsdk_server runs (nothing started)",
      p.returncode != 0 and "another mavsdk_server runs" in p.stdout and "own mavsdk_server pid" not in p.stdout, p.stdout.strip()[-120:])
p = subprocess.run([sys.executable, os.path.join(SC, "real_flight_safe.py"), "--v13", ROOT, "--own-server", "udp://:1",
                    "--server-bin", FAKE, "--allow-other-server", "--", "--port", "50053", "--help"],
                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, timeout=60)
check("own server: refused with v13 --port given too", p.returncode != 0 and "do not pass them" in p.stdout, p.stdout.strip()[-100:])
p = subprocess.run([sys.executable, os.path.join(SC, "real_flight_safe.py"), "--v13", ROOT, "--own-server", "udp://:1",
                    "--server-bin", FAKE, "--server-port", "50998", "--allow-other-server", "--", "--help"],
                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, timeout=60)
import re as _re
m = _re.search(r"own mavsdk_server pid (\d+): .* -p 50998 udp://:1", p.stdout)
check("own server: started, v13 runs (usage printed), stopped at the normal end",
      p.returncode == 0 and m is not None and "--altitude" in p.stdout and "stopped (rc" in p.stdout and dead(int(m.group(1))),
      (m.group(0)[:80] if m else p.stdout.strip()[-150:]))
O.kill(); O.wait()
print("%d failure(s)" % len(F)); sys.exit(len(F))
