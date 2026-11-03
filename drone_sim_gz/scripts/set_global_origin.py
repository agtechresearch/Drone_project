#!/usr/bin/env python3
"""Give PX4 a global origin when flying on VIO/EV only (no GPS): sends MAVLink SET_GPS_GLOBAL_ORIGIN so
global position / home become valid and the boot-time Hold mode is armable. Run once after SITL boots.
usage: set_global_origin.py [udpout:127.0.0.1:14540|udpin:0.0.0.0:14540] [lat] [lon] [alt_m] [timeout_s=90]"""
import sys, time
from pymavlink import mavutil
addr = sys.argv[1] if len(sys.argv) > 1 else 'udpin:0.0.0.0:14540'
lat = float(sys.argv[2]) if len(sys.argv) > 2 else 37.5; lon = float(sys.argv[3]) if len(sys.argv) > 3 else 127.0; alt = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
m = mavutil.mavlink_connection(addr)
for _ in range(12):                      # udpout: PX4 only talks back after it has heard from us -> send a GCS heartbeat first
    if addr.startswith('udpout'):
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    if m.wait_heartbeat(timeout=10):
        break
print("vehicle sysid", m.target_system)
# Send the origin and listen for a valid global position; repeat every second until PX4 accepts it (it only does once
# the EKF runs on vision odometry, a few seconds after boot). Replaces the fixed "sleep 25" in run_sitl*.sh (2026-10-28).
t0 = time.time(); ok = False; g = None
TIMEOUT = float(sys.argv[5]) if len(sys.argv) > 5 else 90.0
while time.time() - t0 < TIMEOUT:
    m.mav.set_gps_global_origin_send(m.target_system, int(lat*1e7), int(lon*1e7), int(alt*1e3), int(time.time()*1e6))
    t1 = time.time()
    while time.time() - t1 < 1.0:
        g = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.3)
        if g and (g.lat != 0 or g.lon != 0):
            ok = True; break
    if ok:
        print("global position valid after %.1f s: lat=%.6f lon=%.6f rel_alt=%.2f" % (time.time() - t0, g.lat/1e7, g.lon/1e7, g.relative_alt/1000)); break
print("RESULT:", "origin set" if ok else "FAILED to get global position")
