#!/usr/bin/env python3
"""Ground-truth flight accuracy for the route "R1C9:W@0.68 R1C1 R1C1@1.265 R1C9" (v3_marker, 2026-10-30; rewritten 2026-10-31
for the codex review: collision-coverage, estimator-comparison, ground-truth-timestamps).
  QVIX_YAW_DEG=<world yaw> [LOC_TRUTH=<survey truth.json>] python3 loc_compare.py <run_dir>...
Geometry (2026-11-01, codex report-truth-geometry reply-8): every run is checked with run_geometry.check() before the clearance
- a survey world loads ITS truth (gz/survey/<sv>_truth.json, also without LOC_TRUTH), the geometry hashes and the world yaw must
match the ones the run's manifest recorded. "geometry ..." line per run; status missing / mismatch -> "clearance INCOMPLETE:
geometry not verified", unverified (no manifest: older runs) the same unless LOC_GEOMETRY_UNVERIFIED=accept (labelled).
Truth: gt.csv (gt_logger.py, every odometry message with its own sim stamp) when the run has it, else tags.csv (true pose of the
moment an image was processed, stamped with the IMAGE time: older runs). The source is printed.
Per run:
  lateral / heading on the legs s01, s03; arrival at R1C1 / R1C9 and touchdown (along / across) - as before
  clearance  every truth sample in the air (no exclusion): the four rotor discs (hubs +-0.066 / +-0.088 m, r 0.06) against
             every R1 rack unit rectangle (a moved line from LOC_TRUTH rack_shift, turned and shifted) and the room walls (door
             gap left open). 2-D only (no roll / pitch / height). Verdict PASS (min > 0, coverage ok) | FAIL (contact) |
             INCOMPLETE (a truth gap > MAX_GAP s or non-finite samples in the air)
  estimator  loc.jsonl against truth at the estimate's own time, per mission stage (fixed by the plan, not by when the
             correction was applied): n, first / last time, median, max; "legs" = s01 + s03"""
import csv, json, math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import qvix_grid as G  # noqa: E402
import run_geometry as RG  # noqa: E402

MAX_GAP = 0.5            # s between truth samples in the air before the clearance verdict is INCOMPLETE
HUBS = [(0.066, 0.088), (0.066, -0.088), (-0.066, 0.088), (-0.066, -0.088)]; ROTOR_R = 0.06
LT = json.load(open(os.environ["LOC_TRUTH"])) if os.environ.get("LOC_TRUTH") else {}
SHIFT = LT.get("rack_shift")        # main() sets it per run from run_geometry.check(); kept for the geometry-only tests


def unwrap_finite(deg):
    """continuous heading (deg) from compass samples; non-finite samples stay NaN (2026-11-01 codex ground-truth-timestamps
    reply-4: nan_to_num turned a missing yaw into 0 deg, which then passed as a valid heading)"""
    deg = np.asarray(deg, float); out = np.full(len(deg), np.nan); m = np.isfinite(deg)
    if m.any():
        out[m] = np.degrees(np.unwrap(np.radians(deg[m])))
    return out


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def load_truth(d):
    """-> source, t, x, y, z (model frame; None for tags.csv), compass (deg), dropped {reason: rows}.
    2026-10-31 (codex collision-coverage reply-2): rows are no longer filtered by their x before anything is counted. Only a
    row without a finite time is dropped (it cannot be placed), and counted; rows with a non-finite x / y / yaw / z stay in,
    so clearance() counts them as non-finite samples in the air."""
    # source: gt.csv when it has rows, else tags.csv, else none (2026-11-01 codex ground-truth-timestamps reply-6: a missing / empty
    # / unreadable file is a named outcome, never an exception) - the reason is part of the source text
    gp, tp = os.path.join(d, "gt.csv"), os.path.join(d, "tags.csv")
    R, src, why = [], None, []
    for path, name in ((gp, "gt.csv (odometry stamps)"), (tp, "tags.csv (image stamps, pose = latest odometry)")):
        if not os.path.exists(path):
            why.append("%s missing" % os.path.basename(path)); continue
        try:
            rows = list(csv.DictReader(open(path)))
        except (OSError, csv.Error, UnicodeDecodeError) as e:
            why.append("%s unreadable: %s" % (os.path.basename(path), e)); continue
        if not rows:
            why.append("%s empty" % os.path.basename(path)); continue
        R, src = rows, name; break
    if src is None:
        src = "none (%s)" % ", ".join(why)
    elif why:
        src += " (%s)" % ", ".join(why)
    if src.startswith("gt.csv"):
        t = np.array([_f(r.get("t_sim")) for r in R]); raw = [(_f(r.get("x")), _f(r.get("y")), _f(r.get("z"))) for r in R]
        yaw = np.array([_f(r.get("yaw")) for r in R])
    else:
        t = np.array([_f(r.get("t_sim")) for r in R]); raw = [(_f(r.get("true_x")), _f(r.get("true_y")), None) for r in R]
        yaw = np.array([_f(r.get("true_yaw")) for r in R])
    keep = np.isfinite(t) if len(t) else np.zeros(0, bool); dropped = {"time not finite": int((~keep).sum())}
    W = [G.world_to_model(x, y, z) if z is not None else G.world_to_model(x, y) for (x, y, z), k in zip(raw, keep) if k]
    t = t[keep]; yaw = yaw[keep]
    order = np.argsort(t, kind="stable"); t = t[order]
    X = np.array([w[0] for w in W], float)[order] if W else np.zeros(0); Y = np.array([w[1] for w in W], float)[order] if W else np.zeros(0)
    Z = (np.array([w[2] for w in W], float)[order] if W else np.zeros(0)) if src.startswith("gt.csv") else None
    if not src.startswith("gt.csv") and not len(t):
        Z = None
    C = np.array([(90.0 - (math.degrees(a) - G.MODEL_YAW_DEG)) % 360.0 for a in yaw], float)[order] if len(yaw) else np.zeros(0)
    return src, t, X, Y, Z, C, dropped


def rack_rects():
    """R1 rack unit rectangles: (x0, x1, y0, y1, moved?, name)"""
    out = []
    for li, (x0, x1) in enumerate(G.RACKS_X):
        for y0, y1 in G.RACK_UNITS_Y["R1"]:
            out.append((x0, x1, y0, y1, bool(SHIFT) and li == SHIFT["line"], "rack line %d y %.1f-%.1f" % (li, y0, y1)))
    return out


def to_rack_frame(px, py):
    """undo the moved line's transform (make_survey_world.shift_xy) so the rectangle test runs in design coordinates"""
    c, s = math.cos(math.radians(SHIFT["dyaw_deg"])), math.sin(math.radians(SHIFT["dyaw_deg"]))
    cx, cy = SHIFT["centre"]; qx, qy = px - SHIFT["dx"] - cx, py - cy
    return cx + c * qx + s * qy, cy - s * qx + c * qy


def rect_signed(px, py, x0, x1, y0, y1):
    """distance from a point to a rectangle, negative inside"""
    dx = max(x0 - px, 0.0, px - x1); dy = max(y0 - py, 0.0, py - y1)
    if dx == 0.0 and dy == 0.0:
        return -min(px - x0, x1 - px, py - y0, y1 - py)
    return math.hypot(dx, dy)


def wall_clear(px, py):
    """west / east / south walls as lines; the north partition as its two finite pieces left and right of the door (0.1 m
    thick), so a rotor disc reaching over a door jamb's end counts (2026-10-31 codex collision-coverage reply-2: the whole
    north wall used to be skipped while the hub was inside the door width)"""
    y0, y1 = G.ROOMS["R1"]["y"]; d0, d1 = G.PARTITION_DOOR_X
    d = [px - G.WALL, G.X_MAX - G.WALL - px, py - y0,
         rect_signed(px, py, 0.0, d0, y1, y1 + 0.1), rect_signed(px, py, d1, G.X_MAX, y1, y1 + 0.1)]
    return min(d), "wall"


def clearance(t, X, Y, Z, C, t_air, dropped_t=0, require_end=True):
    """-> verdict, worst (min, t, what), n evaluated, non-finite x/y/heading in the air, max gap, info
    info: non-finite heights in the air (evaluated 2-D), rows dropped for a non-finite time, end check. The end check
    (2026-11-01 codex collision-coverage reply-4): the truth must reach the ground after the last airborne sample (the
    interval ends at the first sample at <= 0.08 m), else the landing tail may be cut -> INCOMPLETE. Without heights
    (tags.csv) it cannot be checked -> INCOMPLETE unless require_end=False (geometry-only tests)."""
    rects = rack_rects(); worst = (None, None, None); n = bad = 0
    air = (t >= t_air[0]) & (t <= t_air[1])
    if Z is not None:
        # only a FINITE height at / below 0.08 m is "on the ground"; NaN, +inf and -inf count as in the air (2-D still evaluated) and
        # as non-finite heights below (2026-11-01 codex reply-6: ~(Z <= 0.08) let -inf pass as ground)
        air &= ~(np.isfinite(Z) & (Z <= 0.08))
    idx = np.where(air)[0]
    for i in idx:
        if not (math.isfinite(X[i]) and math.isfinite(Y[i]) and math.isfinite(C[i])):
            bad += 1; continue
        n += 1
        th = math.radians(90.0 - C[i]); c, s = math.cos(th), math.sin(th)
        for hx, hy in HUBS:
            px = X[i] + c * hx - s * hy; py = Y[i] + s * hx + c * hy
            for x0, x1, y0, y1, moved, name in rects:
                qx, qy = to_rack_frame(px, py) if moved else (px, py)
                v = rect_signed(qx, qy, x0, x1, y0, y1) - ROTOR_R
                if worst[0] is None or v < worst[0]:
                    worst = (v, t[i], name)
            v, name = wall_clear(px, py); v -= ROTOR_R
            if v < worst[0]:
                worst = (v, t[i], name)
    # coverage over the WHOLE interval: its start and end count as edges, so missing truth at either end is a gap too
    # (every truth sample in the interval counts for coverage, also the ones on the ground before take-off / after touchdown)
    cov = np.where((t >= t_air[0]) & (t <= t_air[1]))[0]
    edges = np.concatenate(([t_air[0]], t[cov], [t_air[1]]))
    gap = float(np.diff(edges).max()) if len(edges) > 1 else float("inf")
    bad_z = int((~np.isfinite(Z[idx])).sum()) if Z is not None and len(idx) else 0
    if Z is None:
        end = "unverified (no height)"
    elif len(cov) and math.isfinite(Z[cov[-1]]) and Z[cov[-1]] <= 0.08:
        end = "on the ground"
    else:
        end = "truth ends in the air"
    info = {"nonfinite_height": bad_z, "dropped_time": int(dropped_t), "end": end}
    if worst[0] is not None and worst[0] < 0:
        verdict = "FAIL"
    else:
        verdict = "INCOMPLETE" if (gap > MAX_GAP or bad or n == 0 or dropped_t or bad_z
                                   or (require_end and end != "on the ground")) else "PASS"
    return verdict, worst, n, bad, gap, info


STAGE_GROUPS = (("scan", lambda s: s.startswith("scan") or s == "takeoff"), ("s00 turns", lambda s: s.startswith("s00")),
                ("leg s01", lambda s: s.startswith("s01_")), ("alt / turns", lambda s: s.startswith("s02_")),
                ("leg s03", lambda s: s.startswith("s03_")), ("land", lambda s: "preland" in s))


def interval_air(tt, TZ, span):
    """evaluation interval: from the first stage start (or the first truth sample above 0.08 m, whichever is earlier) to the
    last stage end (or the last truth sample above 0.08 m, whichever is later): the landing after the last stage is in it"""
    a = [v[0] for v in span.values()]; b = [v[1] for v in span.values()]
    if TZ is not None and len(TZ):
        up = np.where(TZ > 0.08)[0]
        if len(up):
            # end at the first truth sample after the last airborne one (the touchdown), when there is one
            a.append(tt[up[0]]); b.append(tt[min(up[-1] + 1, len(tt) - 1)])
    if not a and len(tt):
        a, b = [tt[0]], [tt[-1]]
    return (min(a), max(b)) if a else None


def truth_at(tt, V, t):
    """V interpolated at the times t, NaN where the two truth samples around t are more than MAX_GAP apart, not finite, or
    t is outside the truth (2026-10-31 codex ground-truth-timestamps reply-2: no interpolation across truth gaps)"""
    j = np.searchsorted(tt, t)
    out = np.full(len(t), np.nan)
    ok = (j > 0) & (j < len(tt))
    jj = np.clip(j, 1, max(1, len(tt) - 1))
    if len(tt) >= 2:
        ok &= (tt[jj] - tt[jj - 1] <= MAX_GAP) & np.isfinite(V[jj]) & np.isfinite(V[jj - 1])
        out[ok] = np.interp(t[ok], tt, V)
    return out


def main(runs):
    global SHIFT
    for d in runs:
        geo = RG.check(d, os.environ.get("LOC_TRUTH") or None)
        SHIFT = geo["truth"].get("rack_shift")
        src, tt, TX, TY, TZ, TC, dropped = load_truth(d)
        plan = json.load(open(os.path.join(d, "plan.json")))
        xc = plan["spawn"]["model_xy"][0]
        K = list(csv.DictReader(open(os.path.join(d, "wk.csv"))))
        span = {}
        for r in K:
            if r["stage"]:
                a = span.setdefault(r["stage"], [float(r["t_sim"]), float(r["t_sim"])]); a[1] = float(r["t_sim"])
        out = [os.path.basename(os.path.normpath(d)), RG.line(geo), "truth: %s, rows %d, dropped %s" % (src, len(tt), dropped)]
        tcu = unwrap_finite(TC)
        for st in ("s01_to_R1C1", "s03_to_R1C9"):
            m = (tt >= span[st][0]) & (tt <= span[st][1]) if st in span else None
            if m is None or not m.any():
                out.append("%s %s" % (st, "missing" if st not in span else "no truth samples")); continue
            lat = np.abs(TX[m] - xc); hd = np.array([wrap(c - 270.0) for c in TC[m]])
            out.append("%s lateral max %.4f m, heading median %+.2f max |%.2f| deg" % (st[:3], np.nanmax(lat), np.nanmedian(hd), np.nanmax(np.abs(hd))))
        # 2026-11-02: the R1C1 target was looked up before the "st in span" test, so a route without s01_to_R1C1 (survey
        # routes) stopped the whole evaluation with ValueError (clearance never judged, status truth_verdict error)
        names = [s["name"] for s in plan["stages"]]
        arrivals = [("s03_to_R1C9", tuple(plan["spawn"]["model_xy"]), "R1C9")]
        if "s01_to_R1C1" in names:
            arrivals.insert(0, ("s01_to_R1C1", (xc, plan["stages"][names.index("s01_to_R1C1")]["n_rel"] + plan["spawn"]["model_xy"][1]), "R1C1"))
        for st, tgt, lab in arrivals:
            if st in span:
                m = (tt >= span[st][1] - 0.5) & (tt <= span[st][1])
                out.append("arrive %s along %+.4f across %+.4f m" % (lab, np.mean(TY[m]) - tgt[1], np.mean(TX[m]) - tgt[0]) if m.any()
                           else "arrive %s: no truth samples in the last 0.5 s" % lab)
        if len(tt):
            m = tt >= tt[-1] - 1.0
            out.append("touchdown along %+.4f across %+.4f m" % (np.mean(TY[m]) - plan["spawn"]["model_xy"][1], np.mean(TX[m]) - xc))
        t_air = interval_air(tt, TZ, span)
        if not geo["evaluate"]:
            out.append("clearance INCOMPLETE: geometry not verified (%s) - not judged; %s" % (geo["status"], "; ".join(geo["reasons"])))
        elif t_air is None:
            out.append("clearance INCOMPLETE: no truth samples and no stages (min none); 2-D only")
        else:
            verdict, worst, n, bad, gap, info = clearance(tt, TX, TY, TZ, TC, t_air, dropped["time not finite"])
            out.append("clearance %s: min %s at t %s (%s); interval %.1f-%.1f s, end %s; truth samples in the air %d, non-finite x/y/heading %d, non-finite height %d (2-D still evaluated), rows without time %d, max gap %.3f s incl. the interval ends (limit %.1f)%s; 2-D only"
                       % (verdict, "%.4f m" % worst[0] if worst[0] is not None else "none", "%.1f" % worst[1] if worst[1] is not None else "-",
                          worst[2], t_air[0], t_air[1], info["end"], n, bad, info["nonfinite_height"], info["dropped_time"], gap, MAX_GAP,
                          (", moved rack from %s" % os.path.basename(geo["used"]["truth"]["path"])) if SHIFT else "")
                       + ("; geometry UNVERIFIED (accepted)" if geo["accepted_unverified"] else ""))
        lp = os.path.join(d, "loc.jsonl")
        if os.path.exists(lp) and len(tt) >= 2:
            L = [json.loads(s) for s in open(lp) if s.strip()]
            L = [r for r in L if tt[0] <= r["t"] <= tt[-1]]
            if L:
                t = np.array([r["t"] for r in L])
                gx, gy, gc = truth_at(tt, TX, t), truth_at(tt, TY, t), truth_at(tt, tcu, t)
                use = np.isfinite(gx) & np.isfinite(gy) & np.isfinite(gc)
                pe = np.hypot(np.array([r["x"] for r in L]) - gx, np.array([r["y"] for r in L]) - gy)
                he = np.array([wrap(r["comp"] - c) for r, c in zip(L, gc)])
                ready = np.array([r["ready"] for r in L]); stg = [r.get("stage", "") for r in L]
                out.append("estimator (records %d, without bracketing truth <= %.1f s: %d excluded, applied from t %s) pos err median %.4f max %.4f m, heading median %+.2f max |%.2f| deg" % (
                    len(L), MAX_GAP, int((~use).sum()), "%.1f" % t[ready][0] if ready.any() else "-",
                    np.median(pe[use]), pe[use].max(), np.median(he[use]), np.abs(he[use]).max()) if use.any() else
                    "estimator: no record with bracketing truth (%d records)" % len(L))
                for name, f in STAGE_GROUPS + (("legs s01+s03", lambda s: s.startswith("s01_") or s.startswith("s03_")),):
                    m = np.array([f(s) for s in stg]) & use
                    if m.any():
                        out.append("  estimator %-13s n %4d  t %6.1f..%6.1f  pos err median %.4f max %.4f m  heading median %+.2f max |%.2f| deg" % (
                            name, m.sum(), t[m][0], t[m][-1], np.median(pe[m]), pe[m].max(), np.median(he[m]), np.abs(he[m]).max()))
        print("\n  ".join(out))


if __name__ == "__main__":
    main(sys.argv[1:])
