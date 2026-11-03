#!/usr/bin/env python3
"""Ground-truth sampler (replaces truth_sampler.sh, 2026-10-28): subscribes to /model/<model>/odometry (gz.msgs.Odometry,
world-frame pose from the model's OdometryPublisher) and prints one line per period in the same format as the old
`gz model -p` poller:   HH:MM:SS.s  [x y z] [roll pitch yaw]
Why: `gz model -p` took >1-3 s under GUI load and produced empty lines / 4 s gaps (run13 2026-10-28), so stage [END]
readings fell back to stale samples. The subscription has no timeouts.
Run INSIDE starling-gz:  python3 truth_sampler.py [model=starling2_0] [period_s=1.0]
"""
import math, sys, time, threading
from gz.transport13 import Node
from gz.msgs10.odometry_pb2 import Odometry

model = sys.argv[1] if len(sys.argv) > 1 else "starling2_0"
period = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
state = {"pose": None}
lock = threading.Lock()

def rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z); cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    siny = 2.0 * (q.w * q.z + q.x * q.y); cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sinr, cosr), math.asin(sinp), math.atan2(siny, cosy)

def cb(msg):
    with lock:
        state["pose"] = msg.pose

node = Node()
if not node.subscribe(Odometry, "/model/%s/odometry" % model, cb):
    sys.exit("subscribe failed")
nxt = time.time()
while True:
    nxt += period
    time.sleep(max(0.0, nxt - time.time()))
    with lock:
        p = state["pose"]
    ts = time.strftime("%H:%M:%S") + ".%d" % (int(time.time() * 10) % 10)
    if p is None:
        print(ts, flush=True)
        continue
    r, pt, y = rpy(p.orientation)
    print("%s  [%.6f %.6f %.6f] [%.6f %.6f %.6f]" % (ts, p.position.x, p.position.y, p.position.z, r, pt, y), flush=True)
