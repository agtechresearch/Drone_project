#!/usr/bin/env python3
"""Wall-keep run report + comparison videos (v2_wallkeep, 2026-10-29). Run on the HOST with the cad venv
(the system matplotlib is broken against numpy 2):
  ~/starling_sim/cad/venv/bin/python wk_report.py <out_dir> <run_dir> [<run_dir> ...] [--label "text" ...] [--no-video]
  ~/starling_sim/cad/venv/bin/python wk_report.py <out_dir> --faces logs/faces_<ts>.txt [--no-video]

Per run (logs/<TS>_<tag>/ from wk_fly.sh):
  metrics.json  - survey-leg distance to the rack face (TRUTH): min / max / max |error| vs the aisle-centre distance,
                  rotor clearance front/back; sensor accuracy (measured - truth); final correction; grid_verify verdict
  run.mp4       - composite: front camera (with compass labels at the left/right edge) | ToF depth with fitted plane |
                  strip chart of true / measured distance over time with the legs shaded and labelled
                  ("① row 9→1 북→남 0.68 m"), the start marked, and a moving cursor (sim-time aligned, 15 fps)
Together (<out_dir>/):
  run list mode : compare.mp4 (2-column grid), compare_distance.png, summary.json, summary.md
  --faces mode  : face_<F>.png per face (distance along the aisle, every condition), faces_grid.png (8 small panels),
                  summary.json / summary.md with face, error, wall-keep columns (no grid video)
Distance-along-aisle figures use the row number on the x axis (south = row 1 on the left, north on the right), mark the
start point with a large dot, and put a direction arrow + leg number on every leg.
"""
import csv, json, math, os, re, subprocess, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import qvix_grid as G  # noqa: E402
import run_geometry as RG  # noqa: E402

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_B = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False
ROTOR = G.ROTOR_RADIUS_FWD
FPS = 15
W_CAM, H_CAM = 640, 480
W_TOF, H_TOF = 624, 480
STRIP_H = 300
HEAD_H = 44
W_ALL = W_CAM + W_TOF
CIRC = "①②③④⑤⑥⑦⑧⑨"
ERR_NAME = {"yaw15": "VIO 헤딩 1.5°", "drift04": "드리프트 0.4 cm/s", "none": "오차 없음"}



def model_xy(wx, wy):
    """world -> QVIX model (map) x, y for arrays (NaN kept)"""
    out = np.array([G.world_to_model(a, b)[:2] if np.isfinite(a) and np.isfinite(b) else (np.nan, np.nan) for a, b in zip(wx, wy)])
    return (out[:, 0], out[:, 1]) if len(out) else (np.array([]), np.array([]))

def font(sz, bold=False):
    for f in ((FONT_B, FONT) if bold else (FONT,)):
        try:
            return ImageFont.truetype(f, sz)
        except OSError:
            pass
    return ImageFont.load_default()


def fnum(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return float("nan")


def row_of(cell):
    return int(re.match(r"R\d[A-E](\d+)", cell).group(1))


# what metrics() counts, in words that match the mask in load_run() exactly (2026-11-01 codex report-truth-geometry reply-4:
# the earlier text said "survey legs only", the mask has no stage / aisle condition)
SCOPE_SAMPLES = ("frames whose sensor row names a mapped face, true height > take-off height + 0.3 m, finite true distance; "
                 "NOT limited to a stage or an aisle: every phase that meets these conditions is included - turns, climbs and the "
                 "parts of the take-off / landing above that height facing a mapped face too; only samples at or below take-off + "
                 "0.3 m (and those without a face / distance) are left out")


def load_run(d, label=None):
    cond = open(os.path.join(d, "condition.txt")).read().strip()
    # 2026-10-31 (codex report-truth-geometry): the "true" distances below come from the DESIGN map (qvix_grid). In a survey
    # world whose rack line was moved (make_survey_world.py --rack-shift) they are not the truth -> refuse; use
    # loc_compare.py with LOC_TRUTH=<truth.json> (moved rack, all samples) for those runs.
    # 2026-11-01 (codex report-truth-geometry reply-8): the design map used below must be the run's - same qvix_grid.py and world
    # yaw as the run's manifest recorded; missing / mismatch refused, unverified (no manifest) refused unless
    # LOC_GEOMETRY_UNVERIFIED=accept (then labelled in metrics.json scope)
    geo = RG.check(d)
    print("wk_report: %s %s" % (os.path.basename(os.path.normpath(d)), RG.line(geo)))
    if not geo["evaluate"]:
        sys.exit("wk_report: run %s not evaluated - geometry %s: %s" % (os.path.basename(d), geo["status"], "; ".join(geo["reasons"])))
    wm = [w.split("=", 1)[1] for w in cond.split() if w.startswith("world=")]
    if wm and wm[0].startswith("qvix_sv"):
        tp = os.path.join(HERE, "..", "gz", "survey", wm[0][len("qvix_"):] + "_truth.json")
        if not os.path.exists(tp) or json.load(open(tp)).get("rack_shift"):
            sys.exit("wk_report: run %s is in world %s with a moved rack line (or unknown truth): the design-map distances here "
                     "are not the truth. Use loc_compare.py with LOC_TRUTH=%s" % (os.path.basename(d), wm[0], tp))
    parts = cond.split()
    info = {"dir": d, "tag": parts[0], "condition": cond, "geometry": geo,
            "wk": parts[1].split("=")[1], "rot": float(parts[2].split("=")[1]), "drift": float(parts[3].split("=")[1])}
    info["label"] = label or "%s  | wall-keep %s, yaw err %.1f°, drift %.1f cm/s" % (
        info["tag"], info["wk"].upper(), info["rot"], info["drift"] * 100)
    rows = list(csv.DictReader(open(os.path.join(d, "sense.csv"))))
    S = {k: np.array([fnum(r[k]) for r in rows]) for k in
         ("t_sim", "d_meas", "alpha_meas_deg", "conf", "x_true", "y_true", "z_true", "d_true", "alpha_true_deg", "frame")}
    S["face"] = np.array([r["face"] for r in rows])
    # v3: rotation-aware (QVIX_YAW_DEG, turned world qvix_rot); model x across the aisles, model y along the aisle
    S["mx"], S["my"] = model_xy(S["x_true"], S["y_true"])
    # true distance recomputed from the true position with the CURRENT map (qvix_grid faces re-measured from the mesh
    # 2026-10-29), so runs logged with the old 5 cm map are judged the same way as new ones
    mx = S["mx"]
    S["d_true"] = np.array([(m - G.AISLES[f[0]][0]) if f[1:] == "W" else (G.AISLES[f[0]][1] - m) if f[1:] == "E" else np.nan
                            for m, f in zip(mx, S["face"])])
    info["S"] = S
    wk = os.path.join(d, "wk.csv")
    info["K"] = list(csv.DictReader(open(wk))) if os.path.exists(wk) else []
    cam = os.path.join(d, "cam", "index.txt")
    info["cam_t"] = np.array([float(l.split()[1]) for l in open(cam)]) if os.path.exists(cam) else np.array([])
    ch = os.path.join(d, "chase", "index.txt")
    info["chase_t"] = np.array([float(l.split()[1]) for l in open(ch)]) if os.path.exists(ch) else np.array([])
    zz = S["z_true"][np.isfinite(S["z_true"])]
    z0 = np.min(zz[:30]) if len(zz) else 0
    air = np.where(S["z_true"] > z0 + 0.15)[0]
    info["t0"] = S["t_sim"][air[0]] if len(air) else S["t_sim"][0]
    info["t1"] = S["t_sim"][air[-1]] + 2.0 if len(air) else S["t_sim"][-1]
    sv = (S["face"] != "") & (S["z_true"] > z0 + 0.3) & np.isfinite(S["d_true"])
    info["sv"] = sv
    info["plan"] = json.load(open(os.path.join(d, "plan.json")))
    info["heading"] = info["plan"]["spawn"]["heading"]
    info["legs"] = legs_of(info)
    # distance to THIS run's face (spawn cell's aisle, first heading) from the true position, for every sample, so the
    # takeoff / turn / landing parts are drawn too (d_true only exists while the vehicle faces a rack at survey height)
    col, side = info["plan"]["spawn"]["cell"][2], info["heading"]
    x0, x1 = G.AISLES[col]
    S["d_face"] = (mx - x0) if side == "W" else (x1 - mx)
    # arm command time (mission log "[6/7] Arming..." wall clock -> sim time through sense.csv's wall column):
    # the video starts 1 s before it; the hollow ring = position on the floor before arming, the filled dot appears at arming
    info["t_takeoff"] = info["t0"]
    info["t_arm"] = arm_time(d, rows) or info["t0"]
    info["t0"] = info["t_arm"] - 1.0
    return info


def arm_time(d, rows):
    import datetime
    ml = os.path.join(d, "mission.log")
    if not os.path.exists(ml):
        return None
    m = None
    for line in open(ml, errors="ignore"):
        m = re.match(r"\[(\d\d):(\d\d):(\d\d\.\d+)\] \[6/7\] Arming", line)
        if m:
            break
    if not m:
        return None
    day = re.match(r"(\d{8})_", os.path.basename(d.rstrip("/")))
    base = datetime.datetime.strptime(day.group(1), "%Y%m%d")
    wall = (base + datetime.timedelta(hours=int(m.group(1)), minutes=int(m.group(2)), seconds=float(m.group(3)))).timestamp()
    w = np.array([fnum(r["wall"]) for r in rows]); t = np.array([fnum(r["t_sim"]) for r in rows])
    ok = np.isfinite(w) & np.isfinite(t)
    if ok.sum() < 2 or wall < w[ok][0] or wall > w[ok][-1]:
        return None
    return float(np.interp(wall, w[ok], t[ok]))


def hfmt(h):
    return ("%.3f" % h).rstrip("0").rstrip(".")


def legs_of(info):
    """plan stages -> legs with direction words and their sim-time spans (from wk.csv stage column)"""
    span = {}
    for r in info["K"]:
        st = r["stage"]
        if not st:
            continue
        t = fnum(r["t_sim"])
        a, b = span.get(st, (t, t))
        span[st] = (min(a, t), max(b, t))
    legs, n = [], 0
    for st in info["plan"]["stages"]:
        if st["kind"] == "goto":
            n += 1
            ra, rb = row_of(st["from"]), row_of(st["cell"])
            dirw = "북→남" if rb < ra else "남→북"
            h = st["alt"] + G.FLOOR_TO_BODY_AT_REST
            legs.append({"name": st["name"], "kind": "goto", "n": n, "from_row": ra, "to_row": rb, "dir": dirw,
                         "h": h, "span": span.get(st["name"]),
                         "label": "%s row %d→%d %s · %s m" % (CIRC[n - 1], ra, rb, dirw, hfmt(h))})
        elif st["kind"] == "alt" and "preland" not in st["name"]:
            legs.append({"name": st["name"], "kind": "alt", "span": span.get(st["name"]),
                         "label": "상승 %s m" % hfmt(st["height_above_floor"])})
    return legs


def start_point(info):
    """takeoff position: the vehicle on the floor before arming (last true sample before the arm command)"""
    S = info["S"]
    ok = np.where(np.isfinite(S["d_face"]) & (S["t_sim"] <= info["t_arm"]))[0]
    if len(ok) == 0:
        ok = np.where(np.isfinite(S["d_face"]))[0][:1]
    if len(ok) == 0:
        return None
    i = ok[-1]
    return {"t": info["t_arm"], "my": S["my"][i], "d": S["d_face"][i], "mx": S["mx"][i],
            "row": row_of(info["plan"]["spawn"]["cell"])}


def flight_idx(info):
    """samples from arming to the end of the flight (whole movement: takeoff, turn, legs, landing)"""
    S = info["S"]
    return np.where(np.isfinite(S["d_face"]) & (S["t_sim"] >= info["t_arm"]) & (S["t_sim"] <= info["t1"]))[0]


def d_ref_for(info):
    faces = [f for f in np.unique(info["S"]["face"][info["sv"]]) if f]
    out = {}
    for f in faces:
        x0, x1 = G.AISLES[f[0]]
        out[f] = 0.5 * (x1 - x0)
    return out


def verdict(d):
    rep = os.path.join(d, "report.txt")
    if not os.path.exists(rep):
        return "?", ""
    txt = open(rep).read()
    m = re.search(r"VERDICT: (\w+)", txt)
    why = [l.strip(" -") for l in txt.splitlines() if "contact" in l or "TIMEOUT" in l]
    return (m.group(1) if m else "?"), "; ".join(why[:2])


def _leg1_t(info):
    g = [l for l in info["legs"] if l["kind"] == "goto" and l["span"]]
    return g[0]["span"][0] if g else info["t0"]


def metrics(info):
    S, sv = info["S"], info["sv"]
    ref = d_ref_for(info)
    m = {"label": info["label"], "tag": info["tag"], "dir": info["dir"], "wallkeep": info["wk"], "rot_deg": info["rot"],
         "drift_e_m_s": info["drift"]}
    m["grid_verify"], m["grid_verify_why"] = verdict(info["dir"])
    # 2026-10-31 (codex report-truth-geometry reply-2): what these numbers cover, so a margin here is not read as "no contact
    # in the whole flight". The whole-flight, every-rack / wall check is loc_compare.py's clearance (status.json truth_verdict).
    m["scope"] = {"geometry": "design (qvix_grid), unchanged worlds only (qvix_sv* with moved racks refused)",
                  "geometry_check": {k: info["geometry"][k] for k in ("status", "accepted_unverified", "world", "reasons", "used")},
                  "samples": SCOPE_SAMPLES + " (%d of %d sensor frames)" % (int(sv.sum()), len(sv)),
                  "rotor_clear": "front / back rotor to the two faces of this run's aisle, 1-D distance (no rotor discs, no rack ends, "
                                 "no other racks / walls)",
                  "whole_flight_check": "loc_compare.py clearance (status.json truth_verdict)"}
    if sv.sum() == 0:
        m["note"] = "no survey samples"; return m
    dref = np.array([ref.get(f, np.nan) for f in S["face"][sv]])
    dt = S["d_true"][sv]
    err = dt - dref
    m.update({"faces": sorted(ref), "d_ref": {k: round(v, 4) for k, v in ref.items()},
              "n_frames": int(sv.sum()),
              "d_true_min": round(float(np.nanmin(dt)), 4), "d_true_max": round(float(np.nanmax(dt)), 4),
              "err_max_abs": round(float(np.nanmax(np.abs(err))), 4),
              "err_rms": round(float(np.sqrt(np.nanmean(err ** 2))), 4),
              "err_at_end": round(float(err[-1]), 4),
              # same, but only from 5 s after the first leg starts: excludes the takeoff/turn transient, when wall-keep
              # does not update yet (below MIN_ALT, heading not W/E) while an injected drift already runs
              "err_max_abs_legs": round(float(np.nanmax(np.abs(err[S["t_sim"][sv] > _leg1_t(info) + 5.0]))), 4)
              if np.any(S["t_sim"][sv] > _leg1_t(info) + 5.0) else None,
              "rotor_clear_min_front": round(float(np.nanmin(dt) - ROTOR), 4),
              "rotor_clear_min_back": round(float(np.nanmin(2 * dref - dt) - ROTOR), 4),
              "alpha_true_max_abs_deg": round(float(np.nanmax(np.abs(S["alpha_true_deg"][sv]))), 3)})
    ok = sv & np.isfinite(S["d_meas"])
    if ok.sum():
        e = S["d_meas"][ok] - S["d_true"][ok]
        ea = S["alpha_meas_deg"][ok] - S["alpha_true_deg"][ok]
        m.update({"sensor_frames": int(ok.sum()), "sensor_valid_frac": round(float(ok.sum()) / sv.sum(), 3),
                  "sensor_d_err_mean": round(float(np.mean(e)), 4), "sensor_d_err_std": round(float(np.std(e)), 4),
                  "sensor_d_err_p95_abs": round(float(np.percentile(np.abs(e), 95)), 4),
                  "sensor_a_err_mean_deg": round(float(np.mean(ea)), 3), "sensor_a_err_std_deg": round(float(np.std(ea)), 3),
                  "sensor_a_err_p95_abs_deg": round(float(np.percentile(np.abs(ea), 95)), 3)})
    # leg-1 start transient (alignment-hold comparison) and flight time
    t1s = _leg1_t(info)
    l1 = sv & (S["t_sim"] >= t1s) & (S["t_sim"] <= t1s + 5.0)
    if l1.any():
        e1 = np.abs(S["d_true"][l1] - np.array([ref.get(f, np.nan) for f in S["face"][l1]]))
        m["err_leg1_start"] = round(float(e1[0]), 4)
        m["err_leg1_first5s_max"] = round(float(np.nanmax(e1)), 4)
    m["leg1_start_after_arm_s"] = round(float(t1s - info["t_arm"]), 2)
    m["flight_s"] = round(float(info["t1"] - info["t_arm"]), 1)
    ml = os.path.join(info["dir"], "mission.log")
    if os.path.exists(ml):
        mm = re.search(r"\[align\] (\w+) after ([\d.]+) s", open(ml, errors="ignore").read())
        if mm:
            m["align"] = {"result": mm.group(1), "seconds": float(mm.group(2))}
    if info["K"]:
        last = info["K"][-1]
        m["final_c"] = {"cN": fnum(last["cN"]), "cE": fnum(last["cE"]), "cY": fnum(last["cY"])}
        states = [r["state"] for r in info["K"]]
        m["wk_updates"] = sum(1 for s in states if s.startswith("update"))
        m["guard_events"] = sum(1 for s in states if "GUARD" in s)
    return m


# ---------------------------------------------------------------- time strip (video)
LEG_SHADE = ["#dbe8f5", "#f5e6d3", "#e0f0e0", "#efe0f0"]


def strip_png(info, path, tmax):
    S = info["S"]; t = S["t_sim"] - info["t0"]
    ref = d_ref_for(info)
    fig, ax = plt.subplots(figsize=(W_ALL / 100, STRIP_H / 100), dpi=100)
    fig.subplots_adjust(left=0.06, right=0.99, top=0.80, bottom=0.17)
    for k, lg in enumerate(info["legs"]):
        if not lg["span"]:
            continue
        a, b = lg["span"][0] - info["t0"], lg["span"][1] - info["t0"]
        if lg["kind"] == "goto":
            ax.axvspan(a, b, color=LEG_SHADE[(lg["n"] - 1) % len(LEG_SHADE)], zorder=0)
            ax.text(0.5 * (a + b), 66, lg["label"], ha="center", va="top", fontsize=9.5, fontweight="bold", color="#27323a")
        else:
            ax.axvspan(a, b, color="#eeeeee", zorder=0)
            ax.text(0.5 * (a + b), 66, "상승", ha="center", va="top", fontsize=8.5, color="#555")
    sv = info["sv"]
    g = [l for l in info["legs"] if l["kind"] == "goto" and l["span"]]
    if g:
        ax.axvspan(info["t_arm"] - info["t0"], g[0]["span"][0] - info["t0"], color="#f3f3f3", zorder=0)
        ax.text(0.5 * (info["t_arm"] + g[0]["span"][0]) - info["t0"], 66, "시동·이륙·회전", ha="center", va="top", fontsize=8.5, color="#555")
    fi = flight_idx(info)
    tf = np.full(len(t), np.nan); tf[fi] = S["d_face"][fi]
    ax.plot(t, tf * 100, color="#1f77b4", lw=2.0, label="실제 거리(진실값)")
    ax.plot(t, np.where(sv, S["d_meas"], np.nan) * 100, ".", color="#ff7f0e", ms=2, label="ToF 측정")
    for f, v in list(ref.items())[:1]:
        ax.axhline(v * 100, color="#2ca02c", ls="--", lw=1, label="통로 중앙 %.1f cm" % (v * 100))
        ax.axhline(ROTOR * 100, color="#d62728", lw=1.2, label="앞 프로펠러 접촉 %.1f cm" % (ROTOR * 100))
        ax.axhline((2 * v - ROTOR) * 100, color="#d62728", lw=1.2, ls=":", label="뒤 재배단 접촉")
    sp = start_point(info)
    if sp:
        ax.plot(sp["t"] - info["t0"], sp["d"] * 100, "o", ms=11, mfc="none", mec="#111", mew=2.2, zorder=5)
        ax.annotate("시동 · 이륙 위치 row %d" % sp["row"], (sp["t"] - info["t0"], sp["d"] * 100), xytext=(8, -24),
                    textcoords="offset points", ha="left", fontsize=9, fontweight="bold")
    ax.set_xlim(0, tmax); ax.set_ylim(5, 68)
    ax.set_xlabel("시동 후 시간 (s)"); ax.set_ylabel("재배단 면까지 (cm)")
    off = info["t_arm"] - info["t0"]                 # video time 0 = 1 s before arming -> label the axis from arming
    ax.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: "%d" % round(v - off)))
    ax.legend(loc="lower right", ncol=5, fontsize=8, frameon=False, bbox_to_anchor=(1.0, 1.0))
    ax.grid(alpha=0.3)
    fig.savefig(path)
    x0 = ax.transData.transform((0, 0))[0]; x1 = ax.transData.transform((tmax, 0))[0]
    plt.close(fig)
    return x0, x1


def nearest(arr, v):
    if len(arr) == 0:
        return None
    i = int(np.searchsorted(arr, v))
    if i >= len(arr):
        i = len(arr) - 1
    elif i > 0 and abs(arr[i - 1] - v) < abs(arr[i] - v):
        i -= 1
    return i


def compass_lr(heading):
    """what lies at the left / right edge of the camera image"""
    return {"W": ("남", "북"), "E": ("북", "남"), "N": ("서", "동"), "S": ("동", "서")}[heading]


def current_leg(info, tt):
    for lg in info["legs"]:
        if lg["span"] and lg["span"][0] <= tt <= lg["span"][1]:
            return lg
    return None


MID_H = 406                        # middle band: omniscient chase view | front camera (+ ToF inset)
W_CH, W_FC = 722, W_ALL - 722      # 722x406 = 16:9 chase, 542x406 = 4:3 front camera
W_TI, H_TI = 216, 166              # ToF inset (bottom-right of the front camera panel)
FRAME = (W_ALL, HEAD_H + MID_H + STRIP_H)


def run_frames(info, tmax, with_strip_only=False):
    """composite frames; with_strip_only=True yields just the strip chart with the moving cursor (strip_anim.mp4)"""
    d = info["dir"]
    sp = os.path.join(d, "strip%s.png" % os.environ.get("WK_VIDSUFFIX", ""))
    x0, x1 = strip_png(info, sp, tmax)
    strip = Image.open(sp).convert("RGB").resize((W_ALL, STRIP_H))
    n = int(tmax * FPS)
    if with_strip_only:
        for k in range(n):
            fr = strip.copy(); dr = ImageDraw.Draw(fr)
            cx = x0 + (x1 - x0) * (k / FPS) / tmax
            dr.line([(cx, 60), (cx, STRIP_H - 50)], fill=(0, 0, 0), width=2)
            yield fr
        return
    S = info["S"]; ts = S["t_sim"]
    K = info["K"]; kt = np.array([fnum(r["t_sim"]) for r in K]) if K else np.array([])
    f_head = font(22, True); f_txt = font(17); f_cmp = font(18, True); f_tag = font(15, True)
    lw, rw = compass_lr(info["heading"])
    blank = lambda w, h, txt: _blank(w, h, txt)                 # noqa: E731
    for k in range(n):
        tt = info["t0"] + k / FPS
        fr = Image.new("RGB", FRAME, (255, 255, 255))
        dr = ImageDraw.Draw(fr)
        dr.rectangle([0, 0, W_ALL, HEAD_H], fill=(20, 30, 50))
        dr.text((10, 8), info["label"], font=f_head, fill=(255, 255, 255))
        lg = current_leg(info, tt)
        if lg:
            wtx = dr.textlength(lg["label"], font=f_head)
            dr.text((W_ALL - wtx - 12, 8), lg["label"], font=f_head, fill=(255, 214, 120))
        # omniscient chase view (sim sensor, chase_cam.py)
        ci = nearest(info["chase_t"], tt)
        if ci is not None and tt <= info["chase_t"][-1] + 0.2:
            p = os.path.join(d, "chase", "f%06d.jpg" % ci)
            fr.paste(_img(p, (W_CH, MID_H)) or blank(W_CH, MID_H, ""), (0, HEAD_H))
        else:
            fr.paste(blank(W_CH, MID_H, "전지적 시점 없음"), (0, HEAD_H))
        dr.text((8, HEAD_H + 6), "전지적 시점 (통로 북쪽에서)", font=f_tag, fill=(255, 255, 255), stroke_width=3, stroke_fill=(0, 0, 0))
        # front camera + compass labels
        ci = nearest(info["cam_t"], tt)
        if ci is not None and tt <= info["cam_t"][-1] + 0.2:
            p = os.path.join(d, "cam", "f%06d.jpg" % ci)
            fr.paste(_img(p, (W_FC, MID_H)) or blank(W_FC, MID_H, ""), (W_CH, HEAD_H))
        else:
            fr.paste(blank(W_FC, MID_H, ""), (W_CH, HEAD_H))
        dr.text((W_CH + W_FC // 2, HEAD_H + 6), "정면 카메라", font=f_tag, fill=(255, 255, 255), anchor="ma", stroke_width=3, stroke_fill=(0, 0, 0))
        for txt, x, anchor in (("← " + lw, W_CH + 8, "la"), (rw + " →", W_ALL - 8, "ra")):
            dr.text((x, HEAD_H + 30), txt, font=f_cmp, fill=(255, 255, 255), anchor=anchor, stroke_width=3, stroke_fill=(0, 0, 0))
        si = nearest(ts, tt)
        if si is not None:
            p = os.path.join(d, "tof", "f%06d.jpg" % int(S["frame"][si]))
            ti = _img(p, (W_TI, H_TI))
            if ti is not None:
                tx, ty = W_ALL - W_TI - 6, HEAD_H + MID_H - H_TI - 6
                fr.paste(ti, (tx, ty))
                dr.rectangle([tx - 1, ty - 1, tx + W_TI, ty + H_TI], outline=(255, 255, 255))
            dtv, dmv = S["d_true"][si], S["d_meas"][si]
            if not np.isfinite(dtv) and S["t_sim"][si] >= info["t_arm"]:
                dtv = S["d_face"][si]                 # takeoff / turn: distance from the position (not facing the rack yet)
            lines = []
            if np.isfinite(dtv):
                lines.append("실제 %.1f cm  (프로펠러 여유 %.1f cm)" % (dtv * 100, (dtv - ROTOR) * 100))
            if np.isfinite(dmv):
                lines.append("ToF 측정 %.1f cm  각도 %+.1f°" % (dmv * 100, S["alpha_meas_deg"][si]))
            ki = nearest(kt, tt)
            if ki is not None:
                r = K[ki]
                lines.append("보정 E %+.1f cm  yaw %+.1f°  [%s]" % (fnum(r["cE"]) * 100, fnum(r["cY"]), r["state"]))
            y = HEAD_H + MID_H - 24 * len(lines) - 8
            dr.rectangle([0, y - 4, 400, HEAD_H + MID_H], fill=(0, 0, 0))
            for jj, line in enumerate(lines):
                col = (255, 255, 255)
                if jj == 0 and np.isfinite(dtv) and dtv - ROTOR < 0.08:
                    col = (255, 90, 90)
                dr.text((8, y + 24 * jj), line, font=f_txt, fill=col)
        fr.paste(strip, (0, HEAD_H + MID_H))
        cx = x0 + (x1 - x0) * (k / FPS) / tmax
        dr.line([(cx, HEAD_H + MID_H + 60), (cx, HEAD_H + MID_H + STRIP_H - 50)], fill=(0, 0, 0), width=2)
        yield fr


def _img(p, size):
    """open + resize a frame; None if missing or broken (the last JPEG can be half-written when the recorder stops)"""
    try:
        return Image.open(p).convert("RGB").resize(size)
    except (OSError, ValueError):
        return None


def _blank(w, h, txt):
    im = Image.new("RGB", (w, h), (30, 30, 30))
    if txt:
        ImageDraw.Draw(im).text((w // 2, h // 2), txt, font=font(18), fill=(200, 200, 200), anchor="mm")
    return im


def encode(frames, out, size):
    p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "%dx%d" % size,
                          "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
                          "-pix_fmt", "yuv420p", out], stdin=subprocess.PIPE)
    for f in frames:
        p.stdin.write(f.tobytes())
    p.stdin.close(); p.wait()


# ---------------------------------------------------------------- top-down synced trajectories (video)
TD_W, TD_H = 1264, 660
TD_HEAD = 96            # header band: title, time, legend, room inset
TD_MAPL, TD_MAPR = 70, 30


def hex2rgb(h):
    h = h.lstrip("#"); return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


def topdown_frames(runs, tmax, title):
    """runs: list of (info, label, hex colour, dashed). One frame per 1/FPS s since each run's takeoff.
    Map: the flown aisle seen from above, rotated so the aisle runs left->right (south left, north right, west up),
    cross-aisle scale exaggerated (stated on the image) so cm-level drift is visible; room inset top right."""
    info0 = runs[0][0]
    col = info0["plan"]["spawn"]["cell"][2]
    room = info0["plan"]["spawn"]["cell"][:2]
    ax0, ax1 = G.AISLES[col]
    ry0, ry1 = G.ROOMS[room]["racks_y"]; y0r, y1r = G.ROOMS[room]["y"]
    xlo, xhi = ax0 - 0.22, ax1 + 0.22                          # aisle + the edge of each neighbouring rack
    sx = (TD_W - TD_MAPL - TD_MAPR) / (y1r - y0r)
    sy = (TD_H - TD_HEAD - 70) / (xhi - xlo)
    X = lambda my: TD_MAPL + (my - y0r) * sx                    # noqa: E731
    Y = lambda mx: TD_HEAD + 10 + (mx - xlo) * sy               # noqa: E731
    bg = Image.new("RGB", (TD_W, TD_H), (250, 251, 249))
    d = ImageDraw.Draw(bg)
    f_s, f_m, f_b = font(14), font(16), font(20, True)
    # racks / walls inside the window
    for rx0, rx1 in G.RACKS_X:
        a, b = max(rx0, xlo), min(rx1, xhi)
        if a < b:                                # two 4 m units per rack line, 0.3 m open gap, posts at unit ends
            for ua, ub in G.RACK_UNITS_Y[room]:
                d.rectangle([X(ua), Y(a), X(ub), Y(b)], fill=(186, 214, 186), outline=(110, 150, 110))
                for px in (ua, ub):
                    d.rectangle([X(px) - 2, Y(a), X(px) + 2, Y(b)], fill=(110, 150, 110))
    for wx0, wx1 in ((0.0, G.WALL), (G.X_MAX - G.WALL, G.X_MAX)):
        a, b = max(wx0, xlo), min(wx1, xhi)
        if a < b:
            d.rectangle([X(y0r), Y(a), X(y1r), Y(b)], fill=(120, 120, 120))
    # rotor-contact lines (body centre positions where a rotor tip touches the rack face) + centre line
    for fx, sgn in ((ax0, 1), (ax1, -1)):
        yy = Y(fx + sgn * ROTOR)
        for xs in range(int(X(ry0)), int(X(ry1)), 14):
            d.line([xs, yy, xs + 8, yy], fill=(214, 60, 50), width=2)
    yc = Y(0.5 * (ax0 + ax1))
    for xs in range(int(X(y0r)), int(X(y1r)), 18):
        d.line([xs, yc, xs + 9, yc], fill=(60, 160, 80), width=1)
    for r in range(1, 11):
        xm = X(y0r + r - 0.5)
        d.text((xm, TD_H - 24), "row %d" % r, font=f_s, fill=(90, 90, 90), anchor="ma")
    d.text((8, Y(0.5 * (ax0 + ax1))), "%s통로" % col, font=f_m, fill=(60, 60, 60), anchor="lm")
    d.text((X(y0r), TD_H - 50), "← 남", font=f_m, fill=(40, 40, 40), anchor="la")
    d.text((X(y1r), TD_H - 50), "북 →", font=f_m, fill=(40, 40, 40), anchor="ra")
    d.text((TD_W / 2, TD_H - 50), "위쪽 = 서, 아래쪽 = 동 · 통로 폭 방향을 길이 방향보다 %.1f배 확대 · 굵은 선 = 최근 10 s" % (sy / sx),
           font=f_s, fill=(90, 90, 90), anchor="ma")
    d.text((X(y1r) - 4, yc - 3), "통로 중앙", font=f_s, fill=(60, 160, 80), anchor="rd")
    d.text((X(ry1) + 6, Y(ax0 + ROTOR) - 2), "프로펠러 접촉선", font=f_s, fill=(214, 60, 50), anchor="ld")
    # header: title + room inset (whole room, same orientation)
    d.rectangle([0, 0, TD_W, TD_HEAD], fill=(20, 30, 50))
    d.text((12, 10), title, font=f_b, fill=(255, 255, 255))
    iw = 150; ih = iw * G.X_MAX / 10.0; ix = TD_W - iw - 14; iy = 6
    IX = lambda my: ix + (my - y0r) / 10.0 * iw                 # noqa: E731
    IY = lambda mx: iy + mx / G.X_MAX * ih                      # noqa: E731
    d.rectangle([ix, iy, ix + iw, iy + ih], fill=(235, 235, 235))
    for rx0, rx1 in G.RACKS_X:
        d.rectangle([IX(ry0), IY(rx0), IX(ry1), IY(rx1)], fill=(140, 180, 140))
    d.rectangle([IX(y0r), IY(ax0), IX(y1r), IY(ax1)], fill=(255, 200, 80))
    d.text((ix + 3, iy + 1), "%s 전체" % room, font=font(11), fill=(40, 40, 40), anchor="la")
    # per-run model-frame tracks, time since takeoff
    tracks = []
    for info, lab, colr, dashed in runs:
        S = info["S"]
        ok = np.isfinite(S["x_true"]) & np.isfinite(S["y_true"]) & (S["t_sim"] >= info["t_arm"])   # dot appears at arming
        t = S["t_sim"][ok] - info["t0"]
        mx = S["mx"][ok]; my = S["my"][ok]
        dt = S["d_true"][ok]
        tracks.append((t, X(my), Y(mx), dt, lab, hex2rgb(colr), dashed))
    # legend
    lx, ly = 12, 44
    for t, px, py, dt, lab, c, dashed in tracks:
        w = d.textlength(lab, font=f_s) + 46
        if lx + w > ix - 320:
            lx = 12; ly += 22
        d.line([lx, ly + 9, lx + 28, ly + 9], fill=c, width=4 if not dashed else 2)
        d.text((lx + 34, ly), lab, font=f_s, fill=(235, 235, 235))
        lx += w
    sp = start_point(info0)
    if sp:                                    # hollow ring = vehicle on the floor before arming
        d.ellipse([X(sp["my"]) - 10, Y(sp["mx"]) - 10, X(sp["my"]) + 10, Y(sp["mx"]) + 10], outline=(0, 0, 0), width=3)
        d.text((X(sp["my"]), Y(sp["mx"]) + 16), "이륙 위치 row %d (바닥)" % sp["row"], font=f_s, fill=(0, 0, 0), anchor="ma")
    n = int(tmax * FPS)
    for k in range(n):
        tt = k / FPS
        fr = bg.copy(); dr = ImageDraw.Draw(fr)
        dr.text((TD_W - iw - 300, 12), "시동 후 %5.1f s" % (tt - (info0["t_arm"] - info0["t0"])), font=f_b, fill=(255, 214, 120))
        lg = current_leg(info0, tt + info0["t0"])
        if lg:
            dr.text((TD_W - iw - 300, 42), lg["label"], font=f_m, fill=(255, 214, 120))
        for t, px, py, dt, lab, c, dashed in tracks:
            j = int(np.searchsorted(t, tt))
            if j < 2:
                continue
            pts = list(zip(px[:j], py[:j]))
            jr = int(np.searchsorted(t, tt - 10.0))                  # last 10 s bold, older part faint
            faint = tuple(int(0.35 * v + 0.65 * 250) for v in c)
            for seg, colr, wid in ((pts[:max(jr, 1) + 1], faint, 2), (pts[jr:], c, 3 if not dashed else 2)):
                if len(seg) < 2:
                    continue
                if dashed:
                    for a in range(0, len(seg) - 1, 6):
                        dr.line(seg[a:a + 4], fill=colr, width=wid)
                else:
                    dr.line(seg, fill=colr, width=wid)
            cx, cy = pts[-1]
            touch = np.isfinite(dt[j - 1]) and dt[j - 1] - ROTOR < 0.0
            r = 7
            dr.ellipse([cx - r, cy - r, cx + r, cy + r], fill=c, outline=(255, 0, 0) if touch else (20, 20, 20), width=4 if touch else 1)
            if touch:
                dr.text((cx + 10, cy + 6), "접촉", font=f_s, fill=(214, 30, 30))
        yield fr


# ---------------------------------------------------------------- distance along the aisle
def leg_segments(info):
    """survey samples split per goto leg (by time span) -> list of (leg, idx array)"""
    S, sv = info["S"], info["sv"]
    out = []
    for lg in info["legs"]:
        if lg["kind"] != "goto" or not lg["span"]:
            continue
        idx = np.where(sv & (S["t_sim"] >= lg["span"][0]) & (S["t_sim"] <= lg["span"][1]))[0]
        if len(idx) > 5:
            out.append((lg, idx))
    return out


def draw_along(ax, info, color, ls="-", lw=1.8, label=None, arrows=True, small=False):
    S = info["S"]
    fi = flight_idx(info)                     # whole movement (takeoff, turn, landing) as a thin line under the legs
    ax.plot(S["my"][fi], S["d_face"][fi] * 100, color=color, ls=ls, lw=max(0.8, lw * 0.55), alpha=0.7)
    first = True
    for lg, idx in leg_segments(info):
        x, y = S["my"][idx], S["d_true"][idx] * 100
        ax.plot(x, y, color=color, ls=ls, lw=lw, label=label if first else None)
        first = False
        if arrows:
            # arrow at 60 % of the leg, pointing in the direction of motion
            j = int((0.35 if lg["n"] % 2 else 0.65) * (len(idx) - 1))     # stagger arrows/labels of the two legs
            j2 = min(len(idx) - 1, j + max(3, len(idx) // 15))
            ax.annotate("", xy=(x[j2], y[j2]), xytext=(x[j], y[j]),
                        arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, mutation_scale=12 if small else 16))
            if not small:
                ax.annotate(CIRC[lg["n"] - 1], (x[j], y[j]), xytext=(0, 9 if lg["n"] % 2 else -16), textcoords="offset points",
                            ha="center", fontsize=10, color=color, fontweight="bold")
    sp = start_point(info)
    if sp:
        ax.plot(sp["my"], sp["d"] * 100, "o", ms=8 if small else 11, mfc="none", mec="black", mew=1.8, zorder=6)


def along_axes(ax, ref_d, small=False, room="R1"):
    y0 = G.ROOMS[room]["y"][0]
    ry0, ry1 = G.ROOMS[room]["racks_y"]
    ax.axvspan(y0, ry0, color="#f2f2f2", zorder=0); ax.axvspan(ry1, y0 + 10, color="#f2f2f2", zorder=0)
    ax.axhline(ref_d * 100, color="#2ca02c", ls="--", lw=1, label=None if small else "통로 중앙 (목표)")
    ax.axhline(ROTOR * 100, color="#d62728", lw=1.3, label=None if small else "앞 재배단 프로펠러 접촉")
    ax.axhline((2 * ref_d - ROTOR) * 100, color="#d62728", lw=1.3, ls=":", label=None if small else "뒤 재배단 프로펠러 접촉")
    ticks = [y0 + r - 0.5 for r in range(1, 11)]
    ax.set_xticks(ticks); ax.set_xticklabels([str(r) for r in range(1, 11)] if small else ["row %d" % r for r in range(1, 11)],
                                             fontsize=8 if small else 9)
    ax.set_xlim(y0, y0 + 10)
    ax.set_ylim(5, max(70, (2 * ref_d - ROTOR) * 100 + 5))
    ax.grid(alpha=0.3)
    if not small:
        ax.set_xlabel("통로 위치 (row)   남쪽 ←   → 북쪽      (회색 = 재배단이 없는 통로 끝)")
        ax.set_ylabel("재배단 면까지 실제 거리 (cm)")


def compare_png(infos, path, title="조건별 재배단 거리 (진실값)"):
    fig, ax = plt.subplots(figsize=(12, 5.8), dpi=110)
    cols = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    refs = {}
    for i, info in enumerate(infos):
        refs.update(d_ref_for(info))
        draw_along(ax, info, cols[i % len(cols)], label=info["label"])
    ref_d = list(refs.values())[0] if refs else 0.375
    along_axes(ax, ref_d)
    sp = start_point(infos[0])
    if sp:
        ax.annotate("○ 이륙 위치 (row %d, 바닥)\n① 0.68 m로 남쪽 → 상승 → ② 1.265 m로 북쪽" % sp["row"],
                    (sp["my"], sp["d"] * 100), xytext=(-12, 60), textcoords="offset points", ha="right", fontsize=9,
                    arrowprops=dict(arrowstyle="->", color="#333"), bbox=dict(fc="white", ec="#999", alpha=0.9))
    ax.set_title(title)
    ax.legend(fontsize=8, loc="lower left", ncol=2)
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


AL_W, AL_H = 1264, 540


def along_anim_frames(runs, tmax, ref_d, title):
    """animated distance-along-the-aisle chart: runs = [(info, label, hex colour, dashed)], lines grow with time since
    takeoff (same clock as run.mp4 / topdown), current position = dot, start point = black ring"""
    fig, ax = plt.subplots(figsize=(AL_W / 100, AL_H / 100), dpi=100)
    fig.subplots_adjust(left=0.06, right=0.99, top=0.84, bottom=0.12)
    along_axes(ax, ref_d)
    ax.set_title(title, fontsize=12)
    handles = [plt.Line2D([], [], color=c, ls="--" if dsh else "-", lw=2, label=lab) for _, lab, c, dsh in runs]
    ax.legend(handles=handles, loc="lower left", ncol=2, fontsize=8.5)
    fig.canvas.draw()
    bg = Image.frombuffer("RGBA", fig.canvas.get_width_height(), fig.canvas.buffer_rgba()).convert("RGB")
    tr = ax.transData.transform
    H = fig.canvas.get_width_height()[1]
    plt.close(fig)
    tracks = []
    for info, lab, colr, dashed in runs:
        S, sv = info["S"], info["sv"]
        idx = flight_idx(info)               # from arming: takeoff, turn, legs, landing
        xy = tr(np.column_stack([S["my"][idx], S["d_face"][idx] * 100]))
        tracks.append((S["t_sim"][idx] - info["t0"], xy[:, 0], H - xy[:, 1], hex2rgb(colr), dashed))
    sp = start_point(runs[0][0])
    f_b = font(20, True)
    for k in range(int(tmax * FPS)):
        tt = k / FPS
        fr = bg.copy(); dr = ImageDraw.Draw(fr)
        if sp:
            px, py = tr((sp["my"], sp["d"] * 100)); py = H - py
            dr.ellipse([px - 9, py - 9, px + 9, py + 9], outline=(0, 0, 0), width=3)
        for t, xs, ys, c, dashed in tracks:
            j = int(np.searchsorted(t, tt))
            if j < 2:
                continue
            # break the polyline where samples jump (between legs / gaps)
            pts = list(zip(xs[:j], ys[:j]))
            seg = [pts[0]]
            for a, b in zip(pts[:-1], pts[1:]):
                if abs(b[0] - a[0]) > 25 or abs(b[1] - a[1]) > 25:
                    if len(seg) > 1:
                        dr.line(seg, fill=c, width=2 if dashed else 3)
                    seg = [b]
                else:
                    seg.append(b)
            if len(seg) > 1:
                dr.line(seg, fill=c, width=2 if dashed else 3)
            cx, cy = pts[-1]
            dr.ellipse([cx - 6, cy - 6, cx + 6, cy + 6], fill=c, outline=(20, 20, 20))
        dr.text((AL_W - 14, 10), "시동 후 %5.1f s" % (tt - (runs[0][0]["t_arm"] - runs[0][0]["t0"])), font=f_b, fill=(30, 30, 30), anchor="ra")
        yield fr

# ---------------------------------------------------------------- faces mode
COND_STYLE = {("yaw15", "off"): ("#c0392b", "--"), ("yaw15", "on"): ("#e67e22", "-"),
              ("drift04", "off"): ("#6c3483", "--"), ("drift04", "on"): ("#2e86c1", "-"),
              ("none", "off"): ("#7f8c8d", "--"), ("none", "on"): ("#27ae60", "-")}
FACE_NAME = lambda f: "%s통로 %s쪽 면" % (f[0], "서" if f[1] == "W" else "동")   # noqa: E731


def faces_figures(out, entries):
    faces = sorted({e["face"] for e in entries}, key=lambda f: ("ABCDE".index(f[0]), f[1]))
    for f in faces:
        es = [e for e in entries if e["face"] == f]
        fig, ax = plt.subplots(figsize=(12, 5.4), dpi=110)
        ref_d = 0.5 * (G.AISLES[f[0]][1] - G.AISLES[f[0]][0])
        for e in es:
            c, ls = COND_STYLE[(e["cond"], e["wk"])]
            draw_along(ax, e["info"], c, ls=ls, label="%s · 보정 %s" % (ERR_NAME[e["cond"]], "켬" if e["wk"] == "on" else "끔"))
        along_axes(ax, ref_d)
        sp = start_point(es[0]["info"])
        if sp:
            ax.annotate("○ 이륙 위치 row %d" % sp["row"], (sp["my"], sp["d"] * 100), xytext=(-14, 48), textcoords="offset points",
                        ha="right", fontsize=9, arrowprops=dict(arrowstyle="->", color="#333"),
                        bbox=dict(fc="white", ec="#999", alpha=0.9))
        ax.set_title("R1 %s (%s) — 진실값 거리, ① 0.68 m 북→남, ② 1.265 m 남→북" % (FACE_NAME(f), f))
        ax.legend(fontsize=8.5, loc="lower left", ncol=2)
        fig.tight_layout(); fig.savefig(os.path.join(out, "face_%s.png" % f)); plt.close(fig)
    # small multiples
    n = len(faces); cols = 4; rows = (n + cols - 1) // cols
    fig, axs = plt.subplots(rows, cols, figsize=(16, 3.6 * rows), dpi=100, squeeze=False)
    for k, f in enumerate(faces):
        ax = axs[k // cols][k % cols]
        ref_d = 0.5 * (G.AISLES[f[0]][1] - G.AISLES[f[0]][0])
        for e in [e for e in entries if e["face"] == f]:
            c, ls = COND_STYLE[(e["cond"], e["wk"])]
            draw_along(ax, e["info"], c, ls=ls, lw=1.4, small=True)
        along_axes(ax, ref_d, small=True)
        ax.set_title("%s  %s" % (f, FACE_NAME(f)), fontsize=11)
        if k % cols == 0:
            ax.set_ylabel("면까지 cm")
        if k // cols == rows - 1:
            ax.set_xlabel("row (남 → 북)")
    for k in range(n, rows * cols):
        axs[k // cols][k % cols].axis("off")
    handles = [plt.Line2D([], [], color=c, ls=ls, lw=2, label="%s · 보정 %s" % (ERR_NAME[cd], "켬" if w == "on" else "끔"))
               for (cd, w), (c, ls) in COND_STYLE.items() if any(e["cond"] == cd and e["wk"] == w for e in entries)]
    handles.append(plt.Line2D([], [], marker="o", color="w", mfc="none", mec="black", mew=1.8, ms=10, label="이륙 위치 (row 9, 바닥)"))
    fig.legend(handles=handles, loc="upper center", ncol=len(handles), fontsize=10, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.94)); fig.savefig(os.path.join(out, "faces_grid.png")); plt.close(fig)


def _video_job(arg):
    d, label, tmax = arg
    inf = load_run(d, label)
    # WK_VIDSUFFIX: a second report over the same run dirs (e.g. the C-aisle set) must not overwrite run.mp4 of the first
    sfx = os.environ.get("WK_VIDSUFFIX", "")
    encode(run_frames(inf, tmax), os.path.join(d, "run%s.mp4" % sfx), FRAME)
    encode(run_frames(inf, tmax, with_strip_only=True), os.path.join(d, "strip_anim%s.mp4" % sfx), (W_ALL, STRIP_H))
    return os.path.join(d, "run%s.mp4" % sfx)


def main():
    args = sys.argv[1:]
    out = args[0]; runs = []; labels = []; faces_list = None
    i = 1
    while i < len(args):
        if args[i] in ("--no-video", "--topdown-only"):
            i += 1
        elif args[i] == "--label":
            labels.append(args[i + 1]); i += 2
        elif args[i] == "--faces":
            faces_list = args[i + 1]; i += 2
        else:
            runs.append(args[i]); i += 1
    video = "--no-video" not in args and "--topdown-only" not in args
    topdown = "--no-video" not in args
    os.makedirs(out, exist_ok=True)
    entries = []
    if faces_list:
        for line in open(faces_list):
            p = line.split()
            if len(p) < 4 or not os.path.exists(os.path.join(p[3], "sense.csv")):
                continue
            lab = "%s %s · %s · 보정 %s" % (p[0], FACE_NAME(p[0]), ERR_NAME[p[1]], "켬" if p[2] == "on" else "끔")
            entries.append({"face": p[0], "cond": p[1], "wk": p[2], "dir": p[3], "info": load_run(p[3], lab)})
        infos = [e["info"] for e in entries]
    else:
        infos = [load_run(r, labels[k] if k < len(labels) else None) for k, r in enumerate(runs)]
    # common video length = the longest flight, capped (WK_TMAX, default 105 s; videos start 1 s before arming): a run that hit the rack with wall-keep off
    # keeps the mission waiting for minutes (2026-10-29: 218 s) and would stretch every video and time axis
    tmax = min(max(inf["t1"] - inf["t0"] for inf in infos), float(os.environ.get("WK_TMAX", "105")))
    summ = []
    for k, inf in enumerate(infos):
        m = metrics(inf)
        if entries:
            m.update(face=entries[k]["face"], cond=entries[k]["cond"])
        summ.append(m)
        json.dump(m, open(os.path.join(inf["dir"], "metrics.json"), "w"), indent=1, ensure_ascii=False)
    if video:                       # run videos in parallel worker processes (serial took ~1.5 min per run)
        from multiprocessing import Pool
        with Pool(int(os.environ.get("WK_JOBS", "8"))) as pool:
            only = [f for f in os.environ.get("WK_VIDEO_FACES", "").split(",") if f]     # e.g. CW: re-encode one face only
            jobs = [(inf["dir"], inf["label"], tmax) for k, inf in enumerate(infos)
                    if not only or (entries and entries[k]["face"] in only)]
            for r in pool.imap_unordered(_video_job, jobs):
                print("run video", r, flush=True)
    json.dump(summ, open(os.path.join(out, "summary.json"), "w"), indent=1, ensure_ascii=False)
    with open(os.path.join(out, "summary.md"), "w") as f:
        f.write("평가 범위: 설계 기하(qvix_grid). 표본 = %s. 프로펠러 여유 = 이 실행 통로의 두 면까지 1-D 거리(원판·랙 끝·다른 랙·벽 없음). "
                "비행 전체 충돌 판정은 loc_compare.py clearance(status.json truth_verdict).\n\n" % SCOPE_SAMPLES)
        f.write("| 조건 | 최소 거리 cm | 최대 거리 cm | 최대 오차 cm | RMS cm | 앞 여유 최소 cm | 뒤 여유 최소 cm | 센서 거리오차 cm | 센서 각도오차 ° | 판정 |\n|---|---|---|---|---|---|---|---|---|---|\n")
        for m in summ:
            if "d_true_min" not in m:
                f.write("| %s | - | - | - | - | - | - | - | - | %s |\n" % (m["label"], m["grid_verify"])); continue
            f.write("| %s | %.1f | %.1f | %.1f | %.1f | %.1f | %.1f | %s | %s | %s |\n" % (
                m["label"], m["d_true_min"] * 100, m["d_true_max"] * 100, m["err_max_abs"] * 100, m["err_rms"] * 100,
                m["rotor_clear_min_front"] * 100, m["rotor_clear_min_back"] * 100,
                "%.2f±%.2f" % (m["sensor_d_err_mean"] * 100, m["sensor_d_err_std"] * 100) if "sensor_d_err_mean" in m else "-",
                "%.2f±%.2f" % (m["sensor_a_err_mean_deg"], m["sensor_a_err_std_deg"]) if "sensor_a_err_mean_deg" in m else "-",
                m["grid_verify"]))
    print(open(os.path.join(out, "summary.md")).read())
    if entries:
        faces_figures(out, entries)
        print("face figures", out)
        if topdown:
            only = [f for f in os.environ.get("WK_VIDEO_FACES", "").split(",") if f]
            for f in sorted({e["face"] for e in entries}):
                if only and f not in only:
                    continue
                es = sorted([e for e in entries if e["face"] == f], key=lambda e: (e["cond"], e["wk"]))
                runs = [(e["info"], "%s · 보정 %s" % (ERR_NAME[e["cond"]], "켬" if e["wk"] == "on" else "끔"),
                         COND_STYLE[(e["cond"], e["wk"])][0], e["wk"] == "off") for e in es]
                encode(topdown_frames(runs, tmax, "위에서 본 궤적 · R1 %s (%s)" % (FACE_NAME(f), f)),
                       os.path.join(out, "topdown_%s.mp4" % f), (TD_W, TD_H))
                ref_d = 0.5 * (G.AISLES[f[0]][1] - G.AISLES[f[0]][0])
                encode(along_anim_frames(runs, tmax, ref_d, "R1 %s (%s) — 통로를 따라 본 거리, 진실값 (○ 이륙 위치, 점 = 지금 위치)" % (FACE_NAME(f), f)),
                       os.path.join(out, "along_anim_%s.mp4" % f), (AL_W, AL_H))
                print("topdown", f, flush=True)
        return
    compare_png(infos, os.path.join(out, "compare_distance.png"))
    if topdown:
        cols_ = plt.rcParams["axes.prop_cycle"].by_key()["color"]
        runs = [(inf, inf["label"], cols_[k % len(cols_)], inf["wk"] == "off") for k, inf in enumerate(infos)]
        encode(topdown_frames(runs, tmax, "위에서 본 궤적 (이륙 시각 기준 동기화)"), os.path.join(out, "topdown.mp4"), (TD_W, TD_H))
        ref_d = list(d_ref_for(infos[0]).values())[0] if d_ref_for(infos[0]) else 0.375
        encode(along_anim_frames(runs, tmax, ref_d, "통로를 따라 본 거리, 진실값 (○ 이륙 위치, 점 = 지금 위치)"), os.path.join(out, "along_anim.mp4"), (AL_W, AL_H))
        print("topdown video", os.path.join(out, "topdown.mp4"))
    if len(infos) > 1 and video:
        cols = 2; rows_n = (len(infos) + 1) // 2
        tw, th = W_ALL // 2, FRAME[1] // 2 // 2 * 2     # even (libx264 yuv420p)
        gens = [run_frames(inf, tmax) for inf in infos]

        def grid():
            for frs in zip(*gens):
                g = Image.new("RGB", (tw * cols, th * rows_n), (255, 255, 255))
                for j, fr in enumerate(frs):
                    g.paste(fr.resize((tw, th)), ((j % cols) * tw, (j // cols) * th))
                yield g
        encode(grid(), os.path.join(out, "compare.mp4"), (tw * cols, th * rows_n))
        print("compare video", os.path.join(out, "compare.mp4"))


if __name__ == "__main__":
    main()
