#!/usr/bin/env python3
"""Offboard-loss / RC failsafe SITL test: judge one case (2026-11-01). fs_eval.py <case dir> <case>
Inputs in <case dir>: mon.csv (fs_rc.py mon: PX4 vehicle_status / land_detected, host wall time), gt.csv (gt_logger: simulator
truth, t_wall), events.txt ("<wall time> <name>" lines written by the harness). Prints the state sequence after the loss event
(times relative to it), truth z, each check with its raw numbers, and "CASE <case> OK|NOT OK"; exit 0 / 1.
Expected behaviour per case (PX4 1.15 failsafe framework, commander_params.c COM_FAIL_ACT_T / COM_RC_OVERRIDE):
  t_*  = COM_OBL_RC_ACT 4, COM_FAIL_ACT_T 3, NAV_RCL_ACT 3      b_* = the vehicle's current 0 / 0 / 2
  t_server    mavsdk_server killed -> Offboard lost after COM_OF_LOSS_T 1 s -> Land at once -> landed, disarmed. No Hold: PX4 1.15
              handles Offboard loss as a mode fallback (failsafe.cpp:567-569 checkModeFallback), and the COM_FAIL_ACT_T Hold delay
              (framework.cpp:469) is only given to actions added with user takeover "Auto" (framework.cpp:335-345) - seen in
              1930fs_try2: Offboard -> Land at +1.229 s
  t_pyexit    only the python flight script killed -> stays Offboard (the server keeps re-sending the setpoint); then server killed
              -> as t_server
  t_modesw    server killed, mode command Position 1.5 s later (during the Land) -> Position, keeps hovering
  t_stick     server killed, stick 60 % pitch 1 s after Land started -> Position (stick override)
  t_rcloss    RC stand-in stops (Offboard link alive) -> RC lost after COM_RC_LOSS_T 0.5 s -> Hold ~3 s -> Land -> landed
  b_server    server killed -> Position at once, hovers (pilot has to land)
  os_kill9    (logs/run_real_ownserver_sitl.sh) real_flight_safe.py --own-server, the launcher killed with kill -9 mid-flight
              -> its mavsdk_server dies with it -> judged as t_server; os_kill9_base the same with 0/0/2 -> judged as b_server
  b_rcloss    RC stand-in stops -> Return. Over home (1930fs) Return lands at once: rtl.cpp:533-534, distance <= NAV_ACC_RAD 10 m
              -> return altitude home + 2 x distance (= 0 here), so no climb; the climb check is for the *_far cases only
  *_far       the same 3 m north of home (fs_hover --north 3): b_rcloss_far expected to climb (rtl.cpp: home + 2 x 3 m = 6 m),
              t_rcloss_far Hold ~3 s -> Land where it is"""
import csv, sys

NAV = {"0": "MANUAL", "1": "ALTCTL", "2": "POSCTL", "4": "HOLD", "5": "RTL", "12": "DESCEND", "14": "OFFBOARD", "18": "LAND"}

d, case = sys.argv[1], sys.argv[2]
ev = {}
for line in open(d + "/events.txt"):
    t, name = line.split()[:2]
    ev.setdefault(name, float(t))
mon = [r for r in csv.DictReader(open(d + "/mon.csv")) if r["nav_state"]]
gt = [(float(r["t_wall"]), float(r["z"])) for r in csv.DictReader(open(d + "/gt.csv"))]
if "loss" not in ev:
    print("CASE %s NOT OK (no loss event - the case did not reach the injection)" % case)
    sys.exit(1)
t0 = ev["loss"]
fails = []


def check(ok, text):
    print("  [%s] %s" % ("ok" if ok else "FAIL", text))
    ok or fails.append(text)


def z_at(t):
    near = [z for tw, z in gt if abs(tw - t) <= 0.1]
    return near[-1] if near else float("nan")


def zs(a, b):
    return [z for tw, z in gt if a <= tw <= b]


# state changes from 5 s before the first event on
start = min(ev.values()) - 5
seq, prev = [], None
for r in mon:
    t = float(r["t_wall"])
    key = (r["nav_state"], r["arming_state"], r["failsafe"], r["user_took_over"], r["landed"])
    if t >= start and key != prev:
        seq.append((t, r))
        prev = key
print("case %s, events: %s" % (case, ", ".join("%s %+.3f s" % (k, v - t0) for k, v in sorted(ev.items(), key=lambda x: x[1]))))
for t, r in seq:
    print("  %+8.3f s  nav %-8s arm %s failsafe %s took_over %s landed %s  truth z %.5f" % (
        t - t0, NAV.get(r["nav_state"], r["nav_state"]), "ARMED" if r["arming_state"] == "2" else "disarmed", r["failsafe"],
        r["user_took_over"], r["landed"], z_at(t)))


def first(nav, after, before=1e18):
    for t, r in seq:
        if after <= t < before and r["nav_state"] == nav:
            return t
    return None


def left_offboard(after):
    for t, r in seq:
        if t >= after and r["nav_state"] != "14":
            return t, r["nav_state"]
    return None, None


z0 = z_at(t0)
last = mon[-1]
end_z = gt[-1][1]
print("  truth z at loss %.5f, max after loss %.5f, end %.5f; last state nav %s arm %s landed %s" % (
    z0, max(zs(t0, 1e18) or [float("nan")]), end_z, NAV.get(last["nav_state"]), last["arming_state"], last["landed"]))
check(0.3 <= z0 <= 3.5, "in the air at the loss: truth z %.5f m (0.3-3.5)" % z0)


def landed_end(limit_t):
    tl = first("18", t0)
    dis = [t for t, r in seq if t > t0 and r["arming_state"] == "1"]
    check(tl is not None and tl <= limit_t, "Land started %s" % ("%+.3f s" % (tl - t0) if tl else "never"))
    check(bool(dis), "disarmed %s" % ("%+.3f s" % (dis[0] - t0) if dis else "never"))
    check(end_z <= 0.25, "truth ends on the ground: z %.5f" % end_z)
    check(max(zs(t0, 1e18)) <= z0 + 0.3, "no climb after the loss: max z %.5f (loss %.5f)" % (max(zs(t0, 1e18)), z0))


def leave_then_land(dt_lo, dt_hi, hold):
    tl, nav = left_offboard(t0)
    want = "4" if hold else "18"
    check(tl is not None and dt_lo <= tl - t0 <= dt_hi and nav == want, "left Offboard to %s %s after the loss (expected %s, %.1f-%.1f s)" % (
        NAV.get(nav), "%+.3f s" % (tl - t0) if tl else "never", NAV[want], dt_lo, dt_hi))
    land = first("18", t0)
    if hold and tl and land:
        check(2.3 <= land - tl <= 4.0, "Hold lasted %.3f s before Land (COM_FAIL_ACT_T 3)" % (land - tl))
    landed_end(t0 + 10)


if case in ("t_server", "t_pyexit", "os_kill9"):
    if case == "t_pyexit":
        tp = ev["pykill"]
        moved = [r["nav_state"] for t, r in seq if tp <= t < t0 and r["nav_state"] != "14"]
        check(not moved and t0 - tp >= 10, "after the script was killed: Offboard kept for %.3f s until the server kill (other states: %s)"
              % (t0 - tp, moved or "none"))
        check(min(zs(tp, t0)) >= 0.7, "kept hovering meanwhile: truth z %.5f-%.5f" % (min(zs(tp, t0)), max(zs(tp, t0))))
    leave_then_land(0.8, 2.0, False)
elif case in ("t_rcloss", "t_rcloss_far"):
    leave_then_land(0.3, 1.5, True)
elif case == "t_modesw":
    tm = ev["mode"]
    tp = first("2", tm)
    check(tp is not None and tp - tm <= 1.0, "Position after the mode command %s" % ("%+.3f s" % (tp - tm) if tp else "never"))
    check(tp is not None and first("18", tp, ev.get("cleanup", 1e18)) is None, "no Land after the switch until the cleanup")
    z = zs(tm + 1, ev.get("cleanup", 1e18))
    check(z and min(z) >= 0.6 and max(z) <= 1.4, "kept hovering under the pilot: truth z %.5f-%.5f" % (min(z or [0]), max(z or [0])))
elif case == "t_stick":
    ts = ev["stick"]
    land = first("18", t0)
    check(land is not None and land < ts, "Land before the stick input %s" % ("%+.3f s" % (land - t0) if land else "never"))
    tp = first("2", ts)
    check(tp is not None and tp - ts <= 1.0, "Position after the stick %s" % ("%+.3f s" % (tp - ts) if tp else "never"))
    z = zs(ts + 2, ev.get("cleanup", 1e18))
    check(z and min(z) >= 0.3, "stopped descending under the pilot: truth z %.5f-%.5f" % (min(z or [0]), max(z or [0])))
elif case in ("b_server", "os_kill9_base"):
    tl, nav = left_offboard(t0)
    check(tl is not None and tl - t0 <= 2.0 and nav == "2", "Offboard -> %s %s after the loss" % (NAV.get(nav), "%+.3f s" % (tl - t0) if tl else "never"))
    check(first("18", t0, ev.get("cleanup", 1e18)) is None, "no Land before the cleanup")
    z = zs(t0, ev.get("cleanup", 1e18))
    check(z and min(z) >= z0 - 0.3, "kept flying: truth z %.5f-%.5f (loss %.5f, floor loss - 0.3)" % (min(z or [0]), max(z or [0]), z0))
elif case in ("b_rcloss", "b_rcloss_far"):
    tl, nav = left_offboard(t0)
    check(tl is not None and tl - t0 <= 1.5 and nav == "5", "Offboard -> %s %s after RC loss" % (NAV.get(nav), "%+.3f s" % (tl - t0) if tl else "never"))
    z = zs(t0, ev.get("cleanup", 1e18))
    if case == "b_rcloss_far":
        check(z and max(z) >= z0 + 1.0, "Return climbs away from home: truth z max %.5f (loss %.5f)" % (max(z or [0]), z0))
    else:
        print("  [info] over home: truth z max %.5f (loss %.5f) - no climb expected here" % (max(z or [0]), z0))
else:
    sys.exit("unknown case " + case)
print("CASE %s %s" % (case, "OK" if not fails else "NOT OK (%d)" % len(fails)))
sys.exit(1 if fails else 0)
