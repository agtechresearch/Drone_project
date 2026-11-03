#!/usr/bin/env python3
"""Automatic judgement of a grid flight: planned stages (grid_mission.py --plan-json) vs gz ground truth
(truth_sampler.sh) and the mission log. Ground truth decides, not EKF/MAVSDK.

usage: python3 grid_verify.py <plan.json> <mission.log> <truth.txt> [sitl.log]
Per goto stage : script status, truth cell at [END] (must equal the planned cell), distance to the cell centre,
                 max lateral deviation from the corridor centre line during the leg (all truth samples between
                 [START] and [END]), remaining clearance to the rack/wall (corridor half width - radius - deviation).
Per yaw stage  : truth yaw (NED) at [END] vs planned heading. "At [END]" = first truth sample at/after the [END]
                 time (within the post-stage hover), since a sample just before [END] can still be moving.
Overall        : PASS only if every stage is OK, no end cell mismatch, no contact (clearance left <= 0), landing
                 back at the end cell, and the SITL log has no failsafe / attitude failure / preflight fail.
"""
import json
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import qvix_grid as G  # noqa: E402


def secs(hms):
    h, m, s = hms.split(":")
    return int(h) * 3600 + int(m) * 60 + float(s)


def load_truth(path):
    T = []
    for line in open(path, errors="replace"):
        m = re.match(r"(\d+:\d+:\d+\.\d)\s+\[([-\d.e ]+)\]\s*\[([-\d.e ]+)\]", line)
        if m:
            p = [float(v) for v in m.group(2).split()]
            r = [float(v) for v in m.group(3).split()]
            T.append((secs(m.group(1)), p, r))
    return T


def load_stages(path):
    """name -> {start, end, status, fb}"""
    st = {}
    for line in open(path, errors="replace"):
        m = re.match(r"\[(\d+:\d+:\d+\.\d+)\] \[(goto|yaw) (\S+)\] \[(START|END)\]\s*(\S*)(.*)", line)
        if not m:
            continue
        t, kind, name, tag, status, rest = m.groups()
        d = st.setdefault(name, {"kind": kind})
        if tag == "START":
            d["start"] = secs(t)
        else:
            d["end"] = secs(t)
            d["status"] = status
            d["rest"] = rest.strip()
    return st


def yaw_ned_deg(rpy):
    return (90.0 - math.degrees(rpy[2]) + 180.0) % 360.0 - 180.0


def ang_err(target, actual):
    return (actual - target + 180.0) % 360.0 - 180.0


def nearest(T, t):
    return min(T, key=lambda s: abs(s[0] - t))


def settled(T, t_end, window=2.0, min_after=0.0):
    """Truth sample for a stage [END]: the sample closest to [END] within +-0.6 s (the sampler is now an odometry
    subscription stamped at the right time, and with hover 0 the next stage - a climb, a leg or the landing - starts
    right after [END], so a later sample would already be moving). Fallbacks for sampler gaps: first sample within
    `window` s after [END] (at least `min_after` s after it), then the next one up to 5 s late, then the nearest."""
    near = [s for s in T if abs(s[0] - t_end) <= 0.6]
    if near:
        return min(near, key=lambda s: abs(s[0] - t_end))
    after = [s for s in T if t_end + min_after <= s[0] <= t_end + window]
    if after:
        return after[0]
    later = [s for s in T if s[0] > t_end + window]
    if later and later[0][0] <= t_end + 5.0:
        return later[0]
    return nearest(T, t_end)


def main():
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    plan = json.load(open(sys.argv[1]))
    stages = load_stages(sys.argv[2])
    T = load_truth(sys.argv[3])
    sitl_log = sys.argv[4] if len(sys.argv) > 4 else None
    if not T:
        sys.exit("no truth samples")
    z0 = T[0][1][2]
    fails = []
    print("grid flight verdict  (truth samples %d, z0 %.3f, plan spawn %s facing %s, alt %.2f)" % (
        len(T), z0, plan["spawn"]["cell"], plan["spawn"]["heading"], plan["alt"]))
    print()
    print("%-22s %-7s %-7s %-7s %-7s %-9s %-9s %s" % ("stage", "script", "cell", "truth", "d_ctr", "max_dev", "clr_left", "note"))
    for sp in plan["stages"]:
        name = sp["name"]
        s = stages.get(name)
        if s is None or "end" not in s:
            print("%-22s %-7s %s" % (name, "MISSING", "no [END] line in the mission log"))
            fails.append("%s missing" % name)
            continue
        status = s["status"]
        if status != "OK":
            fails.append("%s status %s" % (name, status))
        end_s = settled(T, s["end"])
        if sp["kind"] == "alt":
            alt_u = end_s[1][2] - z0
            err = alt_u - sp["alt"]
            note = "truth U %.3f (planned %.3f, err %+.3f)" % (alt_u, sp["alt"], err)
            if abs(err) > 0.10:
                fails.append("%s altitude err %.3f" % (name, err))
            print("%-22s %-7s %-7s %-7s %-7s %-9s %-9s %s" % (name, status, sp["cell"], "", "", "", "", note))
            continue
        if sp["kind"] == "yaw":
            err = ang_err(sp["yaw_deg"], yaw_ned_deg(end_s[2]))
            note = "truth yaw %+.1f (target %.0f, err %+.1f)" % (yaw_ned_deg(end_s[2]), sp["yaw_deg"], err)
            if abs(err) > 5.0:
                fails.append("%s yaw err %.1f" % (name, err))
            print("%-22s %-7s %-7s %-7s %-7s %-9s %-9s %s" % (name, status, sp["cell"], "", "", "", "", note))
            continue
        # goto stage
        wx, wy = end_s[1][0], end_s[1][1]
        truth_cell = G.locate(wx, wy) or "none"
        cx, cy = sp["world_xy"]
        d_ctr = math.hypot(wx - cx, wy - cy)
        corr = sp["corridor"]
        legT = [x for x in T if s.get("start", s["end"]) - 0.5 <= x[0] <= s["end"] + 0.5]
        if corr["axis"] == "y":
            devs = [abs(x[1][0] - corr["centre"]) for x in legT]
        else:
            devs = [abs(x[1][1] - corr["centre"]) for x in legT]
        max_dev = max(devs) if devs else float("nan")
        clr_left = corr["half_width"] - corr["radius"] - max_dev
        # leg altitude from the last sample at/before [END]: with hover 0 the next stage (often a climb at the aisle end)
        # starts at [END], so a sample just after it already climbed (2026-10-28: false "altitude err" 0.12-0.13 m)
        before = [x for x in T if s["end"] - 1.5 <= x[0] <= s["end"]]
        alt_s = before[-1] if before else end_s
        alt_u = alt_s[1][2] - z0
        note = "U %.3f, %d samples" % (alt_u, len(legT))
        if sp.get("alt") is not None and abs(alt_u - sp["alt"]) > 0.10:
            fails.append("%s altitude err %+.3f" % (name, alt_u - sp["alt"]))
            note += " ALT"
        yaw_errs = [abs(ang_err(G.COMPASS_YAW[sp["heading"]], yaw_ned_deg(x[2]))) for x in legT]
        if yaw_errs:
            note += ", yaw hold max err %.1f" % max(yaw_errs)
            if max(yaw_errs) > 10.0:
                fails.append("%s camera heading drifted %.1f deg during the leg" % (name, max(yaw_errs)))
        if truth_cell != sp["cell"]:
            fails.append("%s ended in %s not %s" % (name, truth_cell, sp["cell"]))
            note += " CELL MISMATCH"
        if clr_left <= 0:
            fails.append("%s contact: clearance left %.3f" % (name, clr_left))
            note += " CONTACT"
        print("%-22s %-7s %-7s %-7s %-7.3f %-9.3f %-9.3f %s" % (name, status, sp["cell"], truth_cell, d_ctr, max_dev, clr_left, note))
    # landing
    last = T[-1]
    ec = G.cell(plan["end"]["cell"])
    land_cell = G.locate(last[1][0], last[1][1]) or "none"
    d_land = math.hypot(last[1][0] - ec["world_xy"][0], last[1][1] - ec["world_xy"][1])
    print()
    print("landing: truth E%+.4f N%+.4f U%+.4f -> cell %s (planned %s), %.3f m from its centre" % (
        last[1][0], last[1][1], last[1][2] - z0, land_cell, ec["name"], d_land))
    if land_cell != ec["name"]:
        fails.append("landed in %s not %s" % (land_cell, ec["name"]))
    land_yaw = yaw_ned_deg(last[2]); yaw_err = ang_err(G.COMPASS_YAW[plan["end"]["heading"]], land_yaw)
    print("landing yaw: truth %+.1f deg (planned %s = %.0f, err %+.1f)" % (land_yaw, plan["end"]["heading"], G.COMPASS_YAW[plan["end"]["heading"]], yaw_err))
    if abs(yaw_err) > 10.0:
        fails.append("landed facing %+.1f deg, planned %s (touchdown spin? see worklog 2026-10-27)" % (land_yaw, plan["end"]["heading"]))
    if last[1][2] - z0 > 0.05:
        fails.append("not on the ground at the last truth sample (U %.3f)" % (last[1][2] - z0))
    rolls = [abs(math.degrees(x[2][0])) for x in T]
    pitches = [abs(math.degrees(x[2][1])) for x in T]
    print("attitude: max |roll| %.1f deg, max |pitch| %.1f deg; max U %.3f m" % (
        max(rolls), max(pitches), max(x[1][2] for x in T) - z0))
    for line in open(sys.argv[2], errors="replace"):
        if "Landing drift" in line or "EMERGENCY" in line or "Traceback" in line:
            print("mission log: " + line.strip()[:160])
    if sitl_log and os.path.exists(sitl_log):
        bad = []
        for line in open(sitl_log, errors="replace"):
            if re.search(r"failsafe|Attitude failure|Preflight Fail|Failure detected|CRITICAL", line, re.I):
                bad.append(line.strip()[:140])
        print("sitl log warnings: %d" % len(bad))
        for b in bad[:8]:
            print("   " + b)
        if bad:
            fails.append("%d sitl warnings" % len(bad))
    print()
    if fails:
        print("VERDICT: FAIL")
        for f in fails:
            print("  - " + f)
        sys.exit(1)
    print("VERDICT: PASS")


if __name__ == "__main__":
    main()
