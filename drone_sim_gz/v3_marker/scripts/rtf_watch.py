#!/usr/bin/env python3
"""Print the achieved real-time factor of a Gazebo world every <window> s (sim-time delta / wall delta from
/world/<world>/stats). Run INSIDE starling-gz (GZ_PARTITION honoured):  python3 rtf_watch.py [world=qvix] [window=5] [max_s=600]"""
import sys, time, threading
from gz.transport13 import Node
from gz.msgs10.world_stats_pb2 import WorldStatistics
world = sys.argv[1] if len(sys.argv) > 1 else "qvix"; win = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
max_s = float(sys.argv[3]) if len(sys.argv) > 3 else 600.0
st = {"sim": None, "paused": False}; lock = threading.Lock()
def cb(m):
    with lock:
        st["sim"] = m.sim_time.sec + m.sim_time.nsec * 1e-9; st["paused"] = m.paused
node = Node()
if not node.subscribe(WorldStatistics, "/world/%s/stats" % world, cb): sys.exit("subscribe failed")
t0 = time.time(); prev = None
while time.time() - t0 < max_s:
    time.sleep(win)
    with lock: s = st["sim"]; p = st["paused"]
    if s is None: print(time.strftime("%H:%M:%S"), "no stats yet", flush=True); continue
    if prev: print("%s rtf %.2f%s" % (time.strftime("%H:%M:%S"), (s - prev[0]) / (time.time() - prev[1]), " (paused)" if p else ""), flush=True)
    prev = (s, time.time())
