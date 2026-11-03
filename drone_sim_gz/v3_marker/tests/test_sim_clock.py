#!/usr/bin/env python3
"""The REAL sim-clock event loop (grid_mission.install_sim_clock_loop) through a clock gap and a resume (2026-11-01, codex
tof-watchdog reply-4). A thread plays sim_clock_udp.py: sim time at RTF 1, then a pause (no datagrams, sim time frozen), then
datagrams again from the frozen time (behind the loop's extrapolation). Checked: loop time never goes back and never stands
still, asyncio timers keep firing during the gap and after the resume, SIM_CLOCK_AGE grows in the gap, the loop meets the sim
again after the resume, and WallKeep.safety_step on this clock lands on a long gap ("sim clock").
Run in starling-flight (Python 3.6):  cd ~/workspace/v3_marker && python3 tests/test_sim_clock.py      exit = failures"""
import asyncio, os, socket, sys, tempfile, threading, time
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import grid_mission as GM  # noqa: E402
import wk_mission as W  # noqa: E402

PORT = 14697; F = []
ctl = {"send": True, "sim": 100.0}


def check(name, cond, info=""):
    print("%s  %s%s" % ("PASS" if cond else "FAIL", name, ("  (" + info + ")") if info else ""))
    if not cond:
        F.append(name)


def sender():
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); last = time.monotonic()
    while True:
        now = time.monotonic()
        if ctl["send"]:
            ctl["sim"] += now - last                 # RTF 1 while running; frozen (paused) while not sending
            sk.sendto(("%.6f\n" % ctl["sim"]).encode(), ("127.0.0.1", PORT))
        last = now; time.sleep(0.02)


threading.Thread(target=sender, daemon=True).start()
GM.install_sim_clock_loop(PORT)
loop = asyncio.get_event_loop()


async def run(plan, secs):
    """plan: [(wall s, send on/off)]; samples every asyncio.sleep(0.1): (wall, loop time, clock age, sim)"""
    t0 = time.monotonic(); S = []; p = list(plan)
    while time.monotonic() - t0 < secs:
        while p and time.monotonic() - t0 >= p[0][0]:
            ctl["send"] = p.pop(0)[1]
        w = time.monotonic(); await asyncio.sleep(0.1)
        S.append((time.monotonic() - t0, loop.time(), GM.SIM_CLOCK_AGE(), ctl["sim"], time.monotonic() - w))
    return S


S = loop.run_until_complete(run([(1.0, False), (3.5, True)], 9.0))
lt = [s[1] for s in S]
check("loop time never goes back", all(b >= a for a, b in zip(lt, lt[1:])))
check("loop time never stands still (every 0.1 s sleep moves it)", all(b > a for a, b in zip(lt, lt[1:])),
      "min step %.4f s" % min(b - a for a, b in zip(lt, lt[1:])))
check("asyncio.sleep(0.1) returns within 0.4 s wall, also in the gap and after the resume", max(s[4] for s in S) < 0.4,
      "max %.3f s" % max(s[4] for s in S))
gap = [s for s in S if 1.2 < s[0] < 3.4]
check("SIM_CLOCK_AGE grows in the 2.5 s gap", max(s[2] for s in gap) > 2.0, "max %.2f s" % max(s[2] for s in gap))
ahead_resume = [s[1] - s[3] for s in S if 3.6 < s[0] < 3.9]
end = S[-1][1] - S[-1][3]
check("after the resume the loop is ahead of the sim, then meets it again (|diff| < 0.3 s at the end)",
      ahead_resume and ahead_resume[0] > 1.0 and abs(end) < 0.3, "ahead %.2f s after resume, %.3f s at the end" % (ahead_resume[0], end))

check("the 2.5 s gap is logged with its length and the loop's lead (codex sim-clock-staleness)",
      len(GM.SIM_CLOCK_GAPS) == 1 and 2.0 < GM.SIM_CLOCK_GAPS[0][0] < 3.0 and GM.SIM_CLOCK_GAPS[0][1] > 2.0, str(GM.SIM_CLOCK_GAPS))

# safety_step on this clock: a 4 s gap -> lost "sim clock"
wk = W.WallKeep(True, 0, os.path.join(tempfile.mkdtemp(), "wk.csv"), (2.9, 8.6))


async def gap_safety():
    ctl["send"] = False; t0 = time.monotonic(); lost_at = None
    while time.monotonic() - t0 < 4.5:
        now = loop.time(); wk.rx = {"pos": now, "att": now}           # the telemetry itself is fresh here
        wk.safety_step(now, True)
        if wk.lost and lost_at is None:
            lost_at = time.monotonic() - t0
        await asyncio.sleep(0.05)
    ctl["send"] = True
    return lost_at

lost_at = loop.run_until_complete(gap_safety())
check("safety_step on the real clock: datagrams stop -> lost 'sim clock' after ~%.0f s wall" % W.CLOCK_LOST_S,
      lost_at is not None and W.CLOCK_LOST_S <= lost_at < W.CLOCK_LOST_S + 0.5 and "sim clock" in wk.lost, "%s at %s" % (wk.lost, lost_at))
print("%d failure(s)" % len(F)); sys.exit(len(F))
