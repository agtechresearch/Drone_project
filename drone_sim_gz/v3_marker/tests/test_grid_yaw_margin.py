#!/usr/bin/env python3
"""Requested margin on every heading change in place (2026-11-01, codex automatic-yaw-margin). Route compiler only, no flight.
At the start cell's own yaw clearance c0: margin c0 + 1 mm must reject the tag scan, the first heading after takeoff, the
start-tag look, the final heading AND an explicit route turn at the same cell; margin c0 - 1 mm must accept all of them.
Every yaw stage records the clearance and the margin it was checked against.
Run on the host or in starling-flight:  python3 tests/test_grid_yaw_margin.py      exit = failures"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import grid_mission as GM  # noqa: E402
F = []


def check(name, cond, info=""):
    print("%s  %s%s" % ("PASS" if cond else "FAIL", name, ("  (" + info + ")") if info else ""))
    if not cond:
        F.append(name)


def comp(route, margin, scan=False, final="keep"):
    c = GM.compile_route(route.split(), 0.4, 0.45, 2.0, 30.0, margin, final, scan=scan)
    return c, [p for p in c["problems"] if "yaw clearance" in p]


c0 = GM.parse_token("R1C9")[0]["yaw_clearance"]
print("      start cell R1C9 yaw clearance %.3f m" % c0)
cases = (("explicit turn at the start cell", "R1C9:N R1C9:W R1C1", False, "keep", "s01_yaw_W_at_R1C9"),
         ("automatic first heading after takeoff", "R1C9:W R1C1", False, "keep", "s00_yaw_W_after_takeoff"),
         ("automatic tag scan", "R1C9:N R1C1", True, "keep", "scan1_yaw45"),
         ("automatic start-tag look", "R1C9:W R1C1", True, "keep", "s00a_look_start_tag"),
         ("automatic final heading (route ends at R1C9)", "R1C9:N R1C1 R1C9", False, "W", "_yaw_W_before_land"))
for what, route, scan, final, stage in cases:
    _, hi = comp(route, c0 + 0.001, scan, final)
    cl, lo = comp(route, c0 - 0.001, scan, final)
    hit = [p for p in hi if stage in p]
    st = [s for s in cl["stages"] if stage in s["name"]]
    check("%s: margin c0+1mm rejected, c0-1mm accepted" % what, bool(hit) and not lo, "%s | %s" % (hit[:1], lo[:1]))
    check("%s: stage records clearance %.3f and margin" % (what, c0), bool(st) and st[0]["yaw_clearance"] == c0
          and abs(st[0]["margin"] - (c0 - 0.001)) < 1e-12, str(st[:1]))
print("%d failure(s)" % len(F)); sys.exit(len(F))
