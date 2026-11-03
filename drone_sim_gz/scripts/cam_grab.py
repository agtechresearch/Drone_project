#!/usr/bin/env python3
"""Save the vehicle's front camera stream (gz topic /front_camera, gz.msgs.Image RGB8) as JPEG frames.
Run INSIDE starling-gz:  python3 cam_grab.py <out_dir> [fps=15] [max_seconds=1800] [topic=/front_camera]   (stop early: touch <out_dir>/STOP)
(set GZ_PARTITION in the environment for a parallel instance)
fps = target save rate in SIM time; the sensor publishes at 15 Hz, so 15 = every frame. (2026-10-28: the old 5 fps throttle used a
1 ms tolerance and dropped every 3rd candidate because the sensor period is 0.066 s, 3 x 0.066 = 0.198 < 0.199 -> 3.8 fps saved;
tolerance is now half a sensor period.)
Frames: <out_dir>/f%06d.jpg + index.txt (frame, sim time s, wall time). Make a video on the host with
  ffmpeg -framerate 15 -i <out_dir>/f%06d.jpg -c:v libx264 -pix_fmt yuv420p out.mp4
(The gz CameraVideoRecorder plugin answered the record_video service but never wrote a file here, 2026-10-27.)
"""
import os, sys, time, threading
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from PIL import Image as PILImage

out = sys.argv[1]; fps = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0; max_s = float(sys.argv[3]) if len(sys.argv) > 3 else 1800.0
topic = sys.argv[4] if len(sys.argv) > 4 else "/front_camera"
os.makedirs(out, exist_ok=True)
idx = open(os.path.join(out, "index.txt"), "w")
SENSOR_HZ = 15.0
state = {"n": 0, "last": -1e9, "got": 0}
lock = threading.Lock()

def cb(msg):
    with lock:
        state["got"] += 1
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        if t - state["last"] < 1.0 / fps - 0.5 / SENSOR_HZ:   # tolerance = half a sensor period
            return
        state["last"] = t
        if msg.pixel_format_type != 3:      # RGB_INT8
            return
        im = PILImage.frombytes("RGB", (msg.width, msg.height), bytes(msg.data))
        n = state["n"]; state["n"] += 1
        im.save(os.path.join(out, "f%06d.jpg" % n), quality=90)
        idx.write("%d %.3f %s\n" % (n, t, time.strftime("%H:%M:%S"))); idx.flush()

node = Node()
if not node.subscribe(Image, topic, cb):
    sys.exit("subscribe failed")
t0 = time.time()
while time.time() - t0 < max_s and not os.path.exists(os.path.join(out, "STOP")):
    time.sleep(0.5)
idx.close()
print("frames saved: %d (messages received %d) -> %s" % (state["n"], state["got"], out))
