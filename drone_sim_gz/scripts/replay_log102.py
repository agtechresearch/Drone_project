#!/usr/bin/env python3
"""Fly the real-vehicle log sess170_log102 pattern in SITL with the UNCHANGED flight script, for a like-for-like
step-response comparison (2026-10-28): takeoff to 1.2 m, hover, 1 m right (east, vehicle facing N) at 0.45 m/s, hover,
climb 0.5 m, 1 m left back at 0.45 m/s, hover, land. Run inside starling-flight:
  cd ~/workspace && python3 repo_export/scripts/replay_log102.py [--port 50052] [--sim-clock 14601]
"""
import argparse, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, ROOT)
import grid_mission as gm  # noqa: E402

PLAN = [
    {"name": "hover_after_takeoff", "command": "hover", "hover": 6.0},
    {"name": "right_1m", "command": "right", "distance": 1.0, "speed": 0.45, "hover": 12.0},
    {"name": "up_0p5", "command": "up", "distance": 0.5, "speed": 0.25, "hover": 4.0},
    {"name": "left_1m", "command": "left", "distance": 1.0, "speed": 0.45, "hover": 12.0},
]

ap = argparse.ArgumentParser(); ap.add_argument("--port", type=int, default=50052); ap.add_argument("--sim-clock", type=int, default=14601)
a = ap.parse_args()
if a.sim_clock:
    gm.install_sim_clock_loop(a.sim_clock)
os.chdir(ROOT)
import path_flight_phase1_v13 as fs  # noqa: E402
fs.MISSION_PLAN = PLAN; fs.USE_TOP_MISSION_PLAN = True; fs.input = lambda *x, **k: ""
sys.argv = ["path_flight_phase1_v13.py", "--sim", "--csv", "auto", "--skip-preland", "--yaw", "0", "--altitude", "1.2", "--port", str(a.port)]
fs.main()
