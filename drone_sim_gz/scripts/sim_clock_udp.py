#!/usr/bin/env python3
"""Relay the Gazebo simulation clock as UDP text datagrams ("<sim seconds>\\n") so a process without gz bindings
(the mission script in starling-flight, same host network) can run on SIM time whatever the real-time factor.
Run INSIDE starling-gz:  python3 sim_clock_udp.py [world=qvix] [udp_port=14600] [rate_hz=50]   (GZ_PARTITION honoured)
Stop: touch /tmp/sim_clock_<port>.STOP
"""
import os, socket, sys, time, threading
from gz.transport13 import Node
from gz.msgs10.clock_pb2 import Clock

world = sys.argv[1] if len(sys.argv) > 1 else "qvix"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 14600
rate = float(sys.argv[3]) if len(sys.argv) > 3 else 50.0
STOP = "/tmp/sim_clock_%d.STOP" % port
if os.path.exists(STOP):
    os.remove(STOP)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state = {"t": None, "last_sent": -1.0, "n": 0}

def cb(msg):
    t = msg.sim.sec + msg.sim.nsec * 1e-9
    state["t"] = t; state["n"] += 1
    if t - state["last_sent"] >= 1.0 / rate - 1e-4:
        state["last_sent"] = t
        try:
            sock.sendto(("%.6f\n" % t).encode(), ("127.0.0.1", port))
        except OSError:
            pass

node = Node()
if not node.subscribe(Clock, "/world/%s/clock" % world, cb):
    sys.exit("subscribe failed")
while not os.path.exists(STOP):
    time.sleep(0.5)
print("sim_clock_udp: %d clock msgs, last sim t %s" % (state["n"], state["t"]))
