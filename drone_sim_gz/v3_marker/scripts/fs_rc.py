#!/usr/bin/env python3
"""Offboard-loss / RC failsafe SITL test: pilot stand-in and state recorder (2026-11-01). Runs INSIDE starling-gz (pymavlink).
  fs_rc.py emu <N> <ctl.json>   RC stand-in: MANUAL_CONTROL (joystick input, COM_RC_IN_MODE 3 takes it like RC) 
                                to PX4's GCS link udp 18570+N at 20 Hz (MANUAL_CONTROL only). ctl.json (re-read every cycle):
                                {"on": 1, "x": 0, "y": 0, "z": 500, "r": 0, "mode": "posctl"|"land"|"", "mode_seq": k, "exit": 0}
                                on 0 = stop sending (= RC signal lost); z 500 = throttle centre (hover in Position);
                                a new mode_seq sends MAV_CMD_DO_SET_MODE (stands in for the RC mode switch - same user-intention
                                path in commander, not the RC channel itself); exit 1 ends the process.
  fs_rc.py mon <N> <out.csv>    every 0.1 s: vehicle_status (nav_state, arming, failsafe, user took over) and vehicle_land_detected
                                (landed) via the PX4 shell client; stops when <out.csv>.STOP exists.
Event lines go to stdout with the host wall time (the harness and fs_eval.py line them up with gt.csv t_wall)."""
import json, os, re, subprocess, sys, time

BIN = os.path.expanduser("~/PX4-Autopilot/build/px4_sitl_default/bin")


def log(msg):
    print("%.3f %s" % (time.time(), msg), flush=True)


def emu(n, ctl):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection("udpout:127.0.0.1:%d" % (18570 + n), source_system=255, source_component=190)
    tgt = n + 1                                   # MAV_SYS_ID of instance N
    last, seq, sending = None, None, None
    while True:
        try:
            c = json.load(open(ctl))
        except Exception:
            c = last or {"on": 0}
        if c != last:
            log("ctl %s" % json.dumps(c, sort_keys=True))
            last = c
        if c.get("exit"):
            log("exit")
            return 0
        on = bool(c.get("on"))
        if on != sending:
            log("RC %s" % ("sending" if on else "STOPPED (signal lost)"))
            sending = on
        now = time.time()
        if on:
            # no heartbeat: PX4 forwards it to the Offboard link and mavsdk_server then takes sysid 255 for the vehicle
            # (1930fs_try: "New system on 127.0.0.1:14582 (with sysid: 255)"); MANUAL_CONTROL is addressed to the vehicle
            m.mav.manual_control_send(tgt, int(c.get("x", 0)), int(c.get("y", 0)), int(c.get("z", 500)), int(c.get("r", 0)), 0)
        if c.get("mode_seq") is not None and c.get("mode_seq") != seq:
            if seq is not None or c.get("mode"):
                mode = c.get("mode", "")
                main_sub = {"posctl": (3, 0), "land": (4, 6)}.get(mode)
                if main_sub:
                    for _ in range(3):
                        m.mav.command_long_send(tgt, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main_sub[0], main_sub[1], 0, 0, 0, 0)
                    log("mode command %s sent" % mode)
            seq = c.get("mode_seq")
        time.sleep(0.05)


def field(txt, name):
    r = re.search(r"\n\s*%s: (\S+)" % name, txt)
    return r.group(1) if r else ""


def mon(n, out):
    f = open(out, "w")
    f.write("t_wall,nav_state,arming_state,failsafe,user_took_over,nav_state_user_intention,landed\n")
    prev = None
    while not os.path.exists(out + ".STOP"):
        t = time.time()
        try:
            vs = subprocess.run([BIN + "/px4-listener", "--instance", str(n), "vehicle_status"], capture_output=True, text=True,
                                timeout=2).stdout
            ld = subprocess.run([BIN + "/px4-listener", "--instance", str(n), "vehicle_land_detected"], capture_output=True,
                                text=True, timeout=2).stdout
        except Exception as e:
            log("!! listener %s" % e)
            time.sleep(0.1)
            continue
        row = (field(vs, "nav_state"), field(vs, "arming_state"), field(vs, "failsafe"), field(vs, "failsafe_and_user_took_over"),
               field(vs, "nav_state_user_intention"), field(ld, "landed"))
        f.write("%.3f,%s\n" % (t, ",".join(row)))
        f.flush()
        if row != prev:
            log("state nav %s arm %s failsafe %s took_over %s intention %s landed %s" % row)
            prev = row
        time.sleep(max(0.0, 0.1 - (time.time() - t)))
    return 0


if __name__ == "__main__":
    what, n, path = sys.argv[1], int(sys.argv[2]), sys.argv[3]
    sys.exit(emu(n, path) if what == "emu" else mon(n, path))
