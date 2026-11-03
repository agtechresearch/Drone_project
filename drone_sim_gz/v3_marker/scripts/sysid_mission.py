#!/usr/bin/env python3
"""Lateral step-response identification flight (v3_marker, 2026-10-29), UNCHANGED flight script v13.
Hover at 0.65 m facing N, then direct (no ramp) east steps +2/+5/+10/+20 cm and back, 6 s hover each, so the PX4
position loop's sideways response to a target-point jump can be fitted for the surrogate model.
Run in starling-flight:  cd ~/workspace && python3 v3_marker/scripts/sysid_mission.py --port 50053 --sim-clock 14602
"""
import argparse, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "v2_wallkeep", "scripts")); sys.path.insert(0, ROOT)
import grid_mission as gm  # noqa: E402
PLAN = [("hover0", "hover", 6.0)]
for k, e in enumerate((0.02, 0.05, 0.10, 0.20)):
    PLAN.append(("step%02d_out" % k, "goto", 0.0, e, 0.0, 6.0))
    PLAN.append(("step%02d_back" % k, "goto", 0.0, 0.0, 0.0, 6.0))
ap = argparse.ArgumentParser(); ap.add_argument("--port", type=int, default=50053); ap.add_argument("--sim-clock", type=int, default=14602)
a = ap.parse_args()
if a.sim_clock:
    gm.install_sim_clock_loop(a.sim_clock)
os.chdir(ROOT)
import path_flight_phase1_v13 as fs  # noqa: E402
fs.MISSION_PLAN = PLAN; fs.USE_TOP_MISSION_PLAN = True; fs.input = lambda *x, **k: ""
sys.argv = ["path_flight_phase1_v13.py", "--sim", "--csv", "auto", "--skip-preland", "--yaw", "0", "--altitude", "0.65", "--port", str(a.port)]
fs.main()
