#!/usr/bin/env python3
"""Refuse unless v13 would fly a HOVER-ONLY mission with these arguments (2026-11-03). v13's --directions accepts only
forward/backward/right/left and its main() needs --distance > 0, so the hover is the takeoff hover (--takeoff-hover-time = the hold
time) plus one move_rel of at most 1 cm (below h_tol 0.02 m, same altitude); any other step (a move, a turn, an altitude change, the top-of-file MISSION_PLAN) is refused. Imports the vehicle's v13 unchanged
and calls its own build_arg_parser + build_mission_plan (no connection, nothing armed). Exit 0 = every step is a hover.
  python3 hover_plan_check.py <v13 dir> -- <v13 arguments>"""
import sys
a = sys.argv[1:]; d = a[0]; rest = a[a.index("--") + 1:]
sys.path.insert(0, d)
import path_flight_phase1_v13 as fs
ns = fs.build_arg_parser().parse_args(rest)
# the same checks as v13's main() before the flight (2026-11-03: --distance 0 passed this tool but v13 main refused it)
main_checks = [(ns.distance <= 0, "--distance must be positive"), (ns.altitude <= 0, "--altitude must be positive"),
               (ns.h_tol <= 0 or ns.v_tol <= 0, "--h-tol and --v-tol must be positive"), (ns.stable_time < 0, "--stable-time must be >= 0"),
               (ns.preland_alt < 0, "--preland-alt must be >= 0"), (ns.preland_hover_time < 0, "--preland-hover-time must be >= 0"),
               (ns.takeoff_hover_time < 0, "--takeoff-hover-time must be >= 0")]
for bad_, why in main_checks:
    if bad_:
        print("!! v13 main() would refuse: %s - refusing (nothing started)" % why); sys.exit(3)
steps = fs.build_mission_plan(ns)
src = "top-of-file MISSION_PLAN" if fs.USE_TOP_MISSION_PLAN and fs.MISSION_PLAN and not ns.ignore_mission_plan else "CLI"
print("plan source: %s, altitude %.2f m, %d step(s):" % (src, ns.altitude, len(steps)))
for s in steps:
    print("  %s kind=%s dn=%s de=%s distance=%s alt=%s hover=%s" % (s.get("label"), s.get("kind"), s.get("dn"), s.get("de"),
          s.get("distance"), s.get("alt"), s.get("hover")))


MAX_STEP_M = 0.01      # v13 needs --distance > 0: one move of at most 1 cm (below v13's h_tol 0.02 m) counts as staying in place


def still(s):
    return s.get("kind") == "hover" or (s.get("kind") == "move_rel" and 0.0 < (s.get("distance") or 0.0) <= MAX_STEP_M
                                        and abs(s.get("dn") or 0.0) <= MAX_STEP_M and abs(s.get("de") or 0.0) <= MAX_STEP_M
                                        and s.get("alt") == ns.altitude and s.get("yaw_target") is None and s.get("yaw_delta") is None)
bad = [s for s in steps if not still(s)]
if src != "CLI" or bad or not steps:
    print("!! NOT a hover-only plan - refusing (nothing started)"); sys.exit(3)
print("OK: takeoff to %.2f m -> hover %.1f s (takeoff hover) -> %s -> land" % (ns.altitude, ns.takeoff_hover_time,
      ", ".join("%s %.2f m + hover %.1f s" % (s.get("direction"), s.get("distance") or 0.0, s["hover"]) for s in steps))); sys.exit(0)
