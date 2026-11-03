#!/usr/bin/env python3
"""Grid route compiler for the QVIX flight grid -> path_flight_phase1_v13.py MISSION_PLAN (real-flight code untouched).

Route = whitespace-separated tokens  CELL[:HEADING][@HEIGHT]   e.g.
    R1C10:S@0.68  R1C1  R1C1@1.265  R1C10 ...
  - @HEIGHT = body height above the FLOOR in metres (the flight script's altitude is height above the rest position,
    i.e. HEIGHT - 0.03). Omitted -> keep the current height. Changing only the height = vertical "alt" step at the cell;
    changing cell and height together = one diagonal goto. The first token's height (or --alt) is the takeoff altitude.
  - heading = where the camera looks; it need not be the direction of travel (crab flight along the aisle while
    looking at a rack). Example survey: --survey "R1C W" or a chain "R1C W, R1C E, R1D W" (see survey_route()).
  old form without heights:
    R1C10:S  R1C1  R1C10:N  R1D10:E  R1D1:S  R1D10:N  R1C10:W  R1C10:S
  - first token = spawn cell + heading (must match the SITL spawn pose / the real drone's start pose)
  - a token with a new heading turns IN PLACE at the current cell first (absolute "yaw"), then flies to the cell
  - a token with the same cell and a new heading = turn only
  - legs must stay in one aisle (same column) or cross along a row whose rack-free band allows it (cross_ok)
Each move is an absolute "goto" relative to the spawn cell centre (EKF origin = world origin in SITL, and the
flight script captures its own initial position, so goto(n_rel, e_rel) = cell centre - spawn cell centre).
Height is NOT part of the grid: one --alt for the whole route (takeoff altitude of the flight script).

Modes
  --check            compile + geometry checks only, print the plan (no mavsdk needed; usable on the host)
  --plan-json FILE   also write the expected stages (for grid_verify.py)
  --spawn-pose       print "x,y,z,0,0,yaw" for run_sitl.sh and exit
  (default)          compile, then import path_flight_phase1_v13 from the project root, replace its
                     MISSION_PLAN, skip the "Type Enter" prompt and run it with --sim --csv auto --yaw <spawn heading>
                     (run inside starling-flight: cd ~/workspace && python3 repo_export/scripts/grid_mission.py ...)
Extra flight-script arguments go after "--" (e.g. -- --h-tol 0.03).
Python 3.6 compatible.
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
import qvix_grid as G  # noqa: E402

SPAWN_Z_WORLD = 0.18            # legs resting on the QVIX floor (floor top 0.15)
GZ_YAW_FOR_HEADING = {"N": 1.5708, "E": 0.0, "S": -1.5708, "W": 3.1416}


class RouteError(Exception):
    pass


def parse_token(tok):
    """'R1C10:S@0.68' -> (cell, heading or None, height above floor or None)"""
    tok = tok.strip().upper()
    if not tok:
        return None
    height = None
    if "@" in tok:
        tok, h = tok.split("@", 1)
        try:
            height = float(h)
        except ValueError:
            raise RouteError("bad height in %r" % tok)
        if not (0.2 <= height <= 4.8):
            raise RouteError("height %.2f m out of range (0.2..4.8 above the floor)" % height)
    if ":" in tok:
        name, heading = tok.split(":", 1)
    else:
        name, heading = tok, None
    if heading is not None and heading not in G.COMPASS_YAW:
        raise RouteError("bad heading in %r (use N/E/S/W)" % tok)
    try:
        c = G.cell(name)
    except KeyError:
        raise RouteError("unknown cell %r" % name)
    return c, heading, height


def cells_between(a, b):
    """Cells on the straight leg a -> b (inclusive), or raise if the leg is not aisle-run / row-crossing."""
    if a["room"] != b["room"]:
        raise RouteError("%s -> %s crosses rooms (not allowed)" % (a["name"], b["name"]))
    room_cells = G.cells(a["room"])
    if a["col"] == b["col"]:
        lo, hi = sorted((a["row"], b["row"]))
        return "aisle", [c for c in room_cells if c["col"] == a["col"] and lo <= c["row"] <= hi]
    if a["row"] == b["row"]:
        cols = list(G.AISLES)
        lo, hi = sorted((cols.index(a["col"]), cols.index(b["col"])))
        return "cross", [c for c in room_cells if c["row"] == a["row"] and lo <= cols.index(c["col"]) <= hi]
    raise RouteError("%s -> %s is diagonal: change only column OR row per leg" % (a["name"], b["name"]))


def leg_corridor(kind, cells_on_leg, heading):
    """Corridor the vehicle must stay in (world frame): axis along which it flies, centre line, half width, radius to use."""
    c0 = cells_on_leg[0]
    if kind == "aisle":
        x0, x1 = c0["x_range"]
        centre = G.model_to_world(0.5 * (x0 + x1), 0.0)[0]       # world x of the aisle centre line
        half = 0.5 * (x1 - x0)
        radius = G.ROTOR_RADIUS_SIDE if heading in ("N", "S") else G.SWEPT_RADIUS
        return {"axis": "y", "centre": round(centre, 3), "half_width": round(half, 3),
                "radius": round(radius, 3), "clearance": round(half - radius, 3)}
    # crossing: fly along x inside the rack-free band of the row
    blocked = [c["name"] for c in cells_on_leg if not c["cross_ok"]]
    if blocked:
        raise RouteError("row %d is not open for crossing at %s (open band < %.1f m)" % (
            c0["row"], ",".join(blocked), G.CROSS_MIN_BAND))
    band = min(c["open_band"] for c in cells_on_leg)
    centre = G.model_to_world(0.0, c0["cross_y_model"])[1]       # world y of the band centre
    half = 0.5 * band
    radius = G.ROTOR_RADIUS_SIDE if heading in ("E", "W") else G.SWEPT_RADIUS
    return {"axis": "x", "centre": round(centre, 3), "half_width": round(half, 3),
            "radius": round(radius, 3), "clearance": round(half - radius, 3)}


PRELAND_ALT = 0.25              # same as the flight script's DEFAULT_PRELAND_ALT
PRELAND_SPEED = 0.20


def compile_route(tokens, alt, speed, hover, yaw_speed, margin, final_heading="keep", preland_alt=PRELAND_ALT):
    toks = [parse_token(t) for t in tokens]
    toks = [t for t in toks if t is not None]
    if len(toks) < 2:
        raise RouteError("route needs at least spawn + one target")
    spawn, heading, h0 = toks[0]
    if heading is None:
        raise RouteError("first token needs a heading, e.g. %s:S" % spawn["name"])
    if h0 is not None:
        alt = round(h0 - G.FLOOR_TO_BODY_AT_REST, 3)      # takeoff altitude of the flight script
    cur_u = alt                                            # current planned altitude (above rest position)
    spawn_n, spawn_e = spawn["ned_ne"]
    plan, stages, problems = [], [], []
    cur, cur_h = spawn, "N"          # the vehicle boots facing N (see spawn_pose); the route's first heading is flown after takeoff
    idx = 0
    if heading != "N":
        name = "s00_yaw_%s_after_takeoff" % heading
        plan.append((name, "yaw", G.COMPASS_YAW[heading], yaw_speed, hover))
        stages.append({"name": name, "kind": "yaw", "cell": spawn["name"], "heading": heading,
                       "yaw_deg": G.COMPASS_YAW[heading], "world_xy": spawn["world_xy"],
                       "yaw_clearance": spawn["yaw_clearance"], "auto": "first heading after takeoff"})
        cur_h = heading
    for c, h, hz in toks[1:]:
        new_u = None if hz is None else round(hz - G.FLOOR_TO_BODY_AT_REST, 3)
        if h is not None and h != cur_h:
            idx += 1
            name = "s%02d_yaw_%s_at_%s" % (idx, h, cur["name"])
            if cur["yaw_clearance"] < margin:
                problems.append("%s: yaw clearance %.3f < margin %.3f" % (name, cur["yaw_clearance"], margin))
            plan.append((name, "yaw", G.COMPASS_YAW[h], yaw_speed, hover))
            stages.append({"name": name, "kind": "yaw", "cell": cur["name"], "heading": h,
                           "yaw_deg": G.COMPASS_YAW[h], "world_xy": cur["world_xy"],
                           "yaw_clearance": cur["yaw_clearance"], "alt": cur_u})
            cur_h = h
        if c["name"] != cur["name"]:
            idx += 1
            kind, on_leg = cells_between(cur, c)
            name = "s%02d_to_%s" % (idx, c["name"])
            corr = leg_corridor(kind, on_leg, cur_h)
            if corr["clearance"] < margin:
                problems.append("%s: corridor clearance %.3f < margin %.3f" % (name, corr["clearance"], margin))
            n_rel = round(c["ned_ne"][0] - spawn_n, 3)
            e_rel = round(c["ned_ne"][1] - spawn_e, 3)
            if new_u is not None and abs(new_u - cur_u) > 1e-6:
                plan.append((name, "goto", n_rel, e_rel, speed, hover, new_u))
                cur_u = new_u
            else:
                plan.append((name, "goto", n_rel, e_rel, speed, hover))
            stages.append({"name": name, "kind": "goto", "leg": kind, "from": cur["name"], "cell": c["name"],
                           "heading": cur_h, "n_rel": n_rel, "e_rel": e_rel, "world_xy": c["world_xy"],
                           "ned_ne": c["ned_ne"], "corridor": corr, "cells": [x["name"] for x in on_leg], "alt": cur_u})
            cur = c
        elif new_u is not None and abs(new_u - cur_u) > 1e-6:
            idx += 1
            name = "s%02d_alt_%.2f_at_%s" % (idx, hz, cur["name"])
            plan.append((name, "alt", new_u, speed, hover))
            stages.append({"name": name, "kind": "alt", "cell": cur["name"], "heading": cur_h, "alt": new_u,
                           "height_above_floor": hz, "world_xy": cur["world_xy"]})
            cur_u = new_u
    if final_heading and final_heading != "keep" and final_heading != cur_h:
        idx += 1
        name = "s%02d_yaw_%s_before_land" % (idx, final_heading)
        plan.append((name, "yaw", G.COMPASS_YAW[final_heading], yaw_speed, hover))
        stages.append({"name": name, "kind": "yaw", "cell": cur["name"], "heading": final_heading,
                       "yaw_deg": G.COMPASS_YAW[final_heading], "world_xy": cur["world_xy"],
                       "yaw_clearance": cur["yaw_clearance"], "auto": "final heading before land"})
        cur_h = final_heading
    # Precision landing: descend IN PLACE at the end cell, then land. The flight script's own preland stage is
    # go_to_relative(0, 0, preland_alt) = fly back to the START position at 0.25 m, which is a straight line through
    # the racks when the route ends in another cell (run13 2026-10-28 ended at R1D10 and landed at R1C10), so the
    # script is run with --skip-preland and this vertical stage replaces it.
    if preland_alt > 0 and preland_alt < cur_u:
        idx += 1
        name = "s%02d_preland_%.2f_at_%s" % (idx, preland_alt, cur["name"])
        plan.append((name, "alt", preland_alt, PRELAND_SPEED, hover))
        stages.append({"name": name, "kind": "alt", "cell": cur["name"], "heading": cur_h, "alt": preland_alt,
                       "height_above_floor": round(preland_alt + G.FLOOR_TO_BODY_AT_REST, 3), "world_xy": cur["world_xy"],
                       "auto": "in-place preland descent"})
        cur_u = preland_alt
    return {"spawn": {"cell": spawn["name"], "heading": heading, "yaw_deg": 0.0,   # boots facing N
                      "world_xy": spawn["world_xy"], "ned_ne": spawn["ned_ne"]},
            "alt": alt, "speed": speed, "hover": hover, "yaw_speed": yaw_speed, "margin": margin,
            "end": {"cell": cur["name"], "heading": cur_h, "alt": cur_u},
            "plan": plan, "stages": stages, "problems": problems}


def _survey_segment(seg):
    parts = seg.split()
    if len(parts) < 2:
        raise RouteError("survey segment: '<room><col> <W|E> [row_a row_b] [h1,h2,...]' (got %r)" % seg)
    rc, side = parts[0].upper(), parts[1].upper()
    if side not in ("W", "E"):
        raise RouteError("survey side must be W or E (the rack the camera looks at)")
    room, col = rc[:2], rc[2:]
    if room not in G.ROOMS or col not in G.AISLES:
        raise RouteError("unknown aisle %r" % rc)
    if (col == "A" and side == "W") or (col == "E" and side == "E"):
        raise RouteError("aisle %s has a wall, not a rack, on its %s side" % (col, side))
    rows = [int(parts[2]), int(parts[3])] if len(parts) > 3 else None
    heights = [float(x) for x in parts[4].split(",")] if len(parts) > 4 else list(G.GAP_CENTRE_Z)
    return room, col, side, rows, heights


def survey_route(spec):
    """Rack-face survey generator.  One segment: 'R1C W' or 'R1C W 10 1' or 'R1C W 10 1 0.68,1.265'
    = start at <col><row_a> facing <side> (the rack the camera looks at: W = rack on the aisle's west side, E = east),
    fly the aisle end to end at the first gap height, climb one gap at the end, fly back, ... (boustrophedon in the
    vertical plane), finishing at the last gap at whichever end that lands on (with the 6 default gaps: back at row_a).
    Heights default to the 6 rack gap centres. The vehicle flies sideways (heading perpendicular to the aisle) so
    the camera keeps looking at the rack.
    Several segments separated by commas are chained into ONE flight (2026-10-28), e.g. 'R1C W, R1C E, R1D W, R1D E'
    (a bare cell as the first segment, e.g. 'R1C10, R1D W', means the vehicle starts there and crosses first):
      - same aisle, other face: turn in place at the end cell; the next face is flown top-down if the previous one
        ended at the top gap (no height change between faces),
      - another aisle: crab across along the current row (must be a cross_ok row, i.e. row 10 in R1) at the current
        height, then turn to the new face. Rows default to 'current row -> other end'."""
    segs = [x.strip() for x in spec.split(",") if x.strip()]
    if not segs:
        raise RouteError("empty survey spec")
    toks, cur = [], None            # cur = (room, col, row, height) after the previous segment
    for seg in segs:
        if len(seg.split()) == 1:   # a bare cell name = "the vehicle starts here" (REUSE_SITL after a route that ended there)
            if cur is not None:
                raise RouteError("start cell %r must be the first segment" % seg)
            c = G.cell(seg.upper())
            cur = (c["room"], c["col"], c["row"], None)
            continue
        room, col, side, rows, heights = _survey_segment(seg)
        rws = [r for r, _ in G.rows(room)]
        if cur is None:
            row_a, row_b = rows if rows else (rws[-1], rws[0])
        else:
            room_c, col_c, row_c, h_c = cur
            if room != room_c:
                raise RouteError("survey segments must stay in one room (%s -> %s)" % (room_c, room))
            if rows:
                row_a, row_b = rows
            else:
                row_a = row_c if row_c in (rws[0], rws[-1]) else rws[-1]
                row_b = rws[0] if row_a == rws[-1] else rws[-1]
            if h_c is not None and abs(h_c - heights[-1]) < 1e-6 and abs(h_c - heights[0]) > 1e-6:
                heights = heights[::-1]           # continue top-down: no height change between faces
            if h_c is None:                       # start cell: take off there, facing the first face, at its first height
                toks.append("%s%s%d:%s@%.3f" % (room_c, col_c, row_c, side, heights[0]))
            if col != col_c:
                here = G.cell("%s%s%d" % (room_c, col_c, row_c))
                if not here["cross_ok"]:
                    raise RouteError("cannot cross from %s to aisle %s: row %d is not open for crossing" % (here["name"], col, row_c))
                if row_a != row_c:
                    raise RouteError("segment %r must start at row %d (where the crossing from %s lands)" % (seg, row_c, here["name"]))
                toks.append("%s%s%d" % (room, col, row_c))          # crab across at the current height, camera unchanged
        toks.append("%s%s%d:%s@%.3f" % (room, col, row_a, side, heights[0]))
        ends = [row_b, row_a]
        for i, hz in enumerate(heights):
            if i > 0:
                toks.append("%s%s%d@%.3f" % (room, col, ends[(i - 1) % 2], hz))     # climb at the end of the previous leg
            toks.append("%s%s%d" % (room, col, ends[i % 2]))                          # fly the aisle
        cur = (room, col, ends[(len(heights) - 1) % 2], heights[-1])
    return " ".join(toks)


def spawn_pose(compiled):
    """gz spawn pose: always facing NORTH (yaw 0 in PX4), whatever the route's first heading.
    Reason (worklog 2026-10-27): spawning e.g. facing S makes the EKF do a 180 deg heading reset at boot
    (EV yaw alignment); PX4 v1.15's auto flight task applies that stale reset delta to its yaw setpoint when
    action.land() activates it, so the vehicle spins ~120 deg on touchdown. Facing N at boot = delta 0 = no spin,
    and it matches the real Starling 2 whose VIO yaw 0 is the direction it faces at power-on."""
    s = compiled["spawn"]
    return "%.3f,%.3f,%.2f,0,0,%.4f" % (s["world_xy"][0], s["world_xy"][1], SPAWN_Z_WORLD, GZ_YAW_FOR_HEADING["N"])


def print_plan(compiled):
    s = compiled["spawn"]
    print("spawn %s (boots facing N, first heading %s flown after takeoff)  world(%.3f, %.3f)  NED(N %.3f, E %.3f)  gz pose %s" % (
        s["cell"], s["heading"], s["world_xy"][0], s["world_xy"][1], s["ned_ne"][0], s["ned_ne"][1], spawn_pose(compiled)))
    print("alt %.2f m  speed %.2f m/s  hover %.1f s  yaw %.0f deg/s  margin %.3f m" % (
        compiled["alt"], compiled["speed"], compiled["hover"], compiled["yaw_speed"], compiled["margin"]))
    print("%-24s %-6s %-14s %-8s %-18s %s" % ("stage", "kind", "target", "heading", "goto n_rel,e_rel", "corridor / clearance"))
    for st in compiled["stages"]:
        if st["kind"] == "alt":
            print("%-24s %-6s %-14s %-8s %-18s height above floor %.3f (U %.3f)" % (st["name"], "alt", st["cell"], st["heading"], "", st["height_above_floor"], st["alt"]))
            continue
        if st["kind"] == "yaw":
            print("%-24s %-6s %-14s %-8s %-18s yaw clr %.3f" % (st["name"], "yaw", st["cell"], st["heading"], "", st["yaw_clearance"]))
        else:
            c = st["corridor"]
            print("%-24s %-6s %-14s %-8s %-18s %s axis %s centre %.3f half %.3f clr %.3f  U %.3f" % (
                st["name"], "goto", "%s->%s" % (st["from"], st["cell"]), st["heading"],
                "%+.3f,%+.3f" % (st["n_rel"], st["e_rel"]), st["leg"], c["axis"], c["centre"], c["half_width"], c["clearance"], st["alt"]))
    print("ends at %s facing %s, then in-place preland descent + land there%s" % (compiled["end"]["cell"], compiled["end"]["heading"],
          "  (auto turn to %s before land)" % compiled["end"]["heading"]
          if any(st.get("auto") == "final heading before land" for st in compiled["stages"]) else ""))
    print("MISSION_PLAN =")
    for p in compiled["plan"]:
        print("    %r," % (p,))
    if compiled["problems"]:
        print("PROBLEMS:")
        for p in compiled["problems"]:
            print("  - " + p)
    else:
        print("checks: OK (no clearance / crossing problems)")


def install_scaled_loop(factor):
    """Make asyncio time run `factor` x faster than the wall clock, so the UNCHANGED flight script (which times its
    setpoint ramps, hovers and timeouts with loop.time()/asyncio.sleep) behaves in SIM time when Gazebo runs at
    real_time_factor = factor (grid_fly.sh SIM_SPEED). loop.time() is scaled, and the selector's wait is divided by
    the factor so a scheduled sleep(d) returns after d/factor wall seconds = d sim seconds."""
    import asyncio
    if factor <= 1.0 + 1e-9:
        return

    class _ScaledSelector(object):
        def __init__(self, sel):
            self._sel = sel

        def select(self, timeout=None):
            return self._sel.select(None if timeout is None else timeout / factor)

        def __getattr__(self, name):
            return getattr(self._sel, name)

    class ScaledLoop(asyncio.SelectorEventLoop):
        def __init__(self):
            super(ScaledLoop, self).__init__()
            self._selector = _ScaledSelector(self._selector)

        def time(self):
            return super(ScaledLoop, self).time() * factor

    asyncio.set_event_loop(ScaledLoop())
    print("[grid_mission] asyncio clock scaled x%.2f (sim speed)" % factor)


def install_sim_clock_loop(port):
    """Run asyncio on the Gazebo SIM clock received over UDP (sim_clock_udp.py in starling-gz): loop.time() is the
    latest sim time plus the wall time since, times the measured real-time factor; waits are divided by that factor
    and capped at 50 ms so the loop re-reads the clock. The flight script then flies exactly at its commanded speeds
    in sim time whatever the real-time factor (2026-10-28: a fixed x2 clock scale gave 0.80 m/s in sim because the
    RTF sagged to ~1.1 under camera + GUI load). If the clock stops (sim paused) the script's time stops too."""
    import asyncio, socket, threading
    st = {"sim": None, "wall": None, "rtf": 1.0}
    lock = threading.Lock()

    def rx():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", port))
        prev = None
        while True:
            data = sock.recv(64)
            try:
                t = float(data.decode().strip())
            except ValueError:
                continue
            w = _wall()
            with lock:
                if prev is not None and w - prev[1] >= 0.2:            # RTF estimate over >= 0.2 s wall windows
                    r = (t - prev[0]) / (w - prev[1])
                    st["rtf"] = max(0.0, min(50.0, 0.7 * st["rtf"] + 0.3 * r))
                    prev = (t, w)
                elif prev is None:
                    prev = (t, w)
                st["sim"], st["wall"] = t, w

    import time as _time
    _wall = _time.monotonic

    class _SimSelector(object):
        def __init__(self, sel):
            self._sel = sel

        def select(self, timeout=None):
            with lock:
                rtf = st["rtf"]
            if timeout is None:
                return self._sel.select(0.05)
            wait = timeout / rtf if rtf > 1e-3 else 0.05
            return self._sel.select(min(max(wait, 0.0), 0.05))

        def __getattr__(self, name):
            return getattr(self._sel, name)

    class SimClockLoop(asyncio.SelectorEventLoop):
        def __init__(self):
            super(SimClockLoop, self).__init__()
            self._selector = _SimSelector(self._selector)

        def time(self):
            with lock:
                if st["sim"] is None:
                    return 0.0
                return st["sim"] + (_wall() - st["wall"]) * st["rtf"]

    threading.Thread(target=rx, daemon=True).start()
    t0 = _wall()
    while st["sim"] is None and _wall() - t0 < 10.0:
        _time.sleep(0.05)
    if st["sim"] is None:
        sys.exit("[grid_mission] no sim clock datagrams on udp %d (start sim_clock_udp.py in starling-gz)" % port)
    asyncio.set_event_loop(SimClockLoop())
    print("[grid_mission] asyncio clock = Gazebo sim clock (udp %d), sim t %.1f" % (port, st["sim"]))


def run_flight(compiled, extra_args, csv_mode, sim_speed=1.0, sim_clock_port=0):
    if sim_clock_port:
        install_sim_clock_loop(sim_clock_port)
    else:
        install_scaled_loop(sim_speed)
    sys.path.insert(0, PROJECT_ROOT)
    os.chdir(PROJECT_ROOT)              # the flight script writes its CSV in the cwd
    import path_flight_phase1_v13 as fs
    fs.MISSION_PLAN = list(compiled["plan"])
    fs.USE_TOP_MISSION_PLAN = True
    fs.input = lambda *a, **k: ""       # skip "Type Enter when confirmed..."
    sys.argv = ["path_flight_phase1_v13.py", "--sim", "--csv", csv_mode, "--skip-preland",
                "--yaw", str(compiled["spawn"]["yaw_deg"]), "--altitude", str(compiled["alt"])] + list(extra_args)
    print("[grid_mission] running flight script with argv: %s" % " ".join(sys.argv[1:]))
    fs.main()


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    extra = []
    if "--" in argv:
        i = argv.index("--")
        argv, extra = argv[:i], argv[i + 1:]
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--route", help='e.g. "R1C10:S R1C1 R1C10:N" (tokens CELL[:HEADING][@HEIGHT])')
    p.add_argument("--survey", help='generate the route: "<room><col> <W|E> [row_a row_b] [h1,h2,..]" segments joined by commas, e.g. "R1C W, R1C E, R1D W"')
    p.add_argument("--alt", type=float, default=0.4, help="flight altitude above the start (m)")
    p.add_argument("--speed", type=float, default=0.45, help="setpoint ramp speed (m/s)")
    p.add_argument("--hover", type=float, default=2.0, help="hover after each stage (s)")
    p.add_argument("--yaw-speed", type=float, default=30.0, help="deg/s")
    p.add_argument("--margin", type=float, default=0.05, help="required clearance between rotor and rack/wall (m)")
    p.add_argument("--check", action="store_true", help="compile + checks only")
    p.add_argument("--final-heading", default="keep", choices=["N", "E", "S", "W", "keep"],
                   help="optionally turn to this heading before landing (default keep = route's last heading)")
    p.add_argument("--spawn-pose", action="store_true", help="print gz spawn pose for run_sitl.sh and exit")
    p.add_argument("--plan-json", help="write expected stages here")
    p.add_argument("--csv", default="auto", help="flight script --csv value")
    p.add_argument("--sim-clock", type=int, default=0, metavar="UDP_PORT",
                   help="run the mission on the Gazebo sim clock relayed by sim_clock_udp.py (preferred over --sim-speed)")
    p.add_argument("--sim-speed", type=float, default=1.0,
                   help="Gazebo real_time_factor the world is running at; scales the flight script's clock (see install_scaled_loop)")
    a = p.parse_args(argv)
    try:
        if a.survey:
            a.route = survey_route(a.survey)
            print("survey route: %s" % a.route)
            if "--hover" not in (argv or []):
                a.hover = 0.0      # continuous video: no pause between stages (the reach judgement's stable_time still applies)
        if not a.route:
            sys.exit("need --route or --survey")
        compiled = compile_route(a.route.split(), a.alt, a.speed, a.hover, a.yaw_speed, a.margin, a.final_heading)
    except RouteError as e:
        sys.exit("route error: %s" % e)
    if a.spawn_pose:
        print(spawn_pose(compiled))
        return
    print_plan(compiled)
    if a.plan_json:
        with open(a.plan_json, "w") as f:
            json.dump(compiled, f, indent=1)
        print("plan written: %s" % a.plan_json)
    if compiled["problems"]:
        sys.exit("route rejected (see PROBLEMS)")
    if a.check:
        return
    run_flight(compiled, extra, a.csv, a.sim_speed, a.sim_clock)


if __name__ == "__main__":
    main()
