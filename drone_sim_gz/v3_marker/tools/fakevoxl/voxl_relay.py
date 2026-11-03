#!/usr/bin/env python3
"""SITL stand-in for the VOXL pose pipe px4_vehicle_local_position (2026-11-01, real-launcher tests). Run INSIDE starling-gz:
  python3 voxl_relay.py <instance> <udp_port>      pause: touch /tmp/voxl_relay_<N>.PAUSE   stop: touch /tmp/voxl_relay_<N>.STOP
Polls PX4's vehicle_local_position and vehicle_attitude with px4-listener and sends one line per sample in the format of
`voxl-inspect-pose -n px4_vehicle_local_position`:  timestamp(ms)| x y z | roll pitch yaw (deg) | vx vy vz | 0 0 0
While PAUSE exists nothing is sent (the pipe goes silent while MAVSDK keeps running). OFFSET "dx dy" (m, NED x / y) is added
to the reported position (a reach judged on the pipe then fails while the vehicle flies normally)."""
import math, os, re, socket, subprocess, sys, time
N, PORT = sys.argv[1], int(sys.argv[2]); BIN = "/home/dronesim/PX4-Autopilot/build/px4_sitl_default/bin"
sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); n = 0


def listen(topic):
    out = subprocess.run(["./px4-listener", "--instance", N, topic], cwd=BIN, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         universal_newlines=True, timeout=3).stdout
    return {k: v for k, v in re.findall(r"^\s+(\w+): (.*)$", out, re.M)}


while not os.path.exists("/tmp/voxl_relay_%s.STOP" % N):
    if os.path.exists("/tmp/voxl_relay_%s.PAUSE" % N):
        time.sleep(0.05); continue
    try:
        p = listen("vehicle_local_position"); a = listen("vehicle_attitude")
        rp = re.search(r"Roll: ([-\d.]+) deg, Pitch: ([-\d.]+) deg", a.get("q", ""))
        off = [0.0, 0.0]                                 # /tmp/voxl_relay_<N>.OFFSET "dx dy": shifts the reported position
        if os.path.exists("/tmp/voxl_relay_%s.OFFSET" % N):
            off = [float(v) for v in open("/tmp/voxl_relay_%s.OFFSET" % N).read().split()[:2]]
        line = "%.3f| %.5f %.5f %s | %s %s %.3f | %s %s %s | 0 0 0\n" % (
            int(p["timestamp"].split()[0]) / 1000.0, float(p["x"]) + off[0], float(p["y"]) + off[1], p["z"], rp.group(1) if rp else "0", rp.group(2) if rp else "0",
            math.degrees(float(p["heading"])), p["vx"], p["vy"], p["vz"])
        for k in (0, 1, 2):                              # one port per fake pipe reader (see voxl-inspect-pose)
            sk.sendto(line.encode(), ("127.0.0.1", PORT + k))
        n += 1
    except Exception as e:  # noqa: BLE001
        print("[voxl_relay] %r" % e, flush=True); time.sleep(0.2)
print("[voxl_relay] stopped after %d lines" % n, flush=True)
