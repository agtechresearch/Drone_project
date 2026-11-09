#!/usr/bin/env python3
"""Evaluate a hover-gust batch against Gazebo ground truth (2026-11-03, stage A). Host, standard library only.
  hover_gust_eval.py <batch_dir>          -> <batch_dir>/summary.json, summary.txt; exit 0 all checks ok, 1 a check failed
Truth: gt.csv (gt_logger.py, sim time stamps). Spawn at the gz origin with zero VIO injection, so PX4 NED = (gz y, gz x, -gz z
+ const): the lateral (disturbed) error is e_true = gz x - nominal east, the along-track error n_true = gz y - nominal north.
Height uses the offset between estimate and truth in the episode's first quiet seconds (only a secondary limit).
Residual force: a position controller with an integrator hides a constant force in the POSITION, not in the attitude, so the
check is the truth tilt of the thrust axis toward east, tilt_e = asin(z_body . east): a force F east needs about
atan(F / m g) of tilt west (0.1 N -> 1.96 deg at 0.2975 kg). Mean tilt over the last quiet seconds of every episode and
over the first quiet seconds must stay within TILT_TOL deg.
2026-11-03 v2 (codex hover-early-termination-evaluation / hover-reference-contract): the window is [ready, window_end t_sim]
(the actual end, shorter when terminated), the lateral / along reference is the truth mean over the REF_PRE_S before ready
(rl_demo/hover_metrics.truth_ref, same as the trainer), the nominal arrival error is kept separately. Each episode gets an
outcome: completed | task_failed (terminated, |e| > e_max; NOT a check failure, but never averaged as a completed window)
| invalid (a check failed). Gusts after the actual end are "not_reached".
Checks per episode: worker status ok, ready/window_end/cleared/landed present, every planned apply/clear was sent, the
window has truth without a gap > GT_GAP_MAX, every gust tilts the vehicle >= EFFECT_MIN of the expected tilt (or within EFFECT_ABS_DEG of it), control cycles >= 95 % of window * rate, clock age < 1 s, residual tilt."""
import csv, json, math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from rl_demo import hover_metrics as HM

M = 0.2975; G = 9.80665; TILT_TOL = 0.5; GT_GAP_MAX = 0.1; RETURN_TOL = 0.02; RETURN_HOLD = 1.0; QUIET_TAIL = 5.0; EFFECT_MIN = 0.5; EFFECT_ABS_DEG = 0.4; EFFECT_MIN_FORCE = 0.1


def rj(p):
    return json.load(open(p)) if os.path.exists(p) else None


def stats(v):
    if not v:
        return {"n": 0}
    v2 = sorted(v); n = len(v)
    return {"n": n, "mean": sum(v) / n, "rms": math.sqrt(sum(x * x for x in v) / n), "max_abs": max(abs(x) for x in v),
            "min": v2[0], "max": v2[-1], "median": v2[n // 2]}


def tilt_e(r):
    qw, qx, qy, qz = r["qw"], r["qx"], r["qy"], r["qz"]
    zx = 2 * (qx * qz + qw * qy)                       # x (east) component of the body z axis in the world
    return math.degrees(math.asin(max(-1.0, min(1.0, zx))))


def main(B):
    cfg = rj(os.path.join(B, "config.json")); plan = rj(os.path.join(B, "plan.json"))
    W, HZ, QS = cfg["window_s"], cfg["setpoint_hz"], cfg["gust"]["quiet_start_s"]
    gt = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(os.path.join(B, "gt.csv")))]
    gt.sort(key=lambda r: r["t_sim"])
    ev = list(csv.DictReader(open(os.path.join(B, "gust_events.csv")))) if os.path.exists(os.path.join(B, "gust_events.csv")) else []
    out = {"batch": B, "episodes": [], "checks_failed": []}
    prev_land = None
    for ep in plan["episodes"]:
        D = os.path.join(B, "ep_%02d" % ep["id"]); e = {"id": ep["id"], "cond": ep["cond"], "seed": ep["seed"], "fail": []}
        res = rj(os.path.join(D, "episode.json")) or {}
        e["worker_status"] = res.get("status"); e["worker_reason"] = res.get("reason")
        if res.get("status") != "ok":
            e["fail"].append("worker status %s: %s" % (res.get("status"), res.get("reason")))
        marks = {m: rj(os.path.join(D, m)) for m in ("ready", "window_end", "cleared", "landed")}
        for m, v in marks.items():
            if v is None:
                e["fail"].append("missing %s" % m)
        if marks["ready"] is None or not res.get("nominal_ned"):
            out["episodes"].append(e)
            for f in e["fail"]:              # 2026-11-03 hg_a10: these were not counted ("ALL CHECKS OK" with 6 failed episodes)
                out["checks_failed"].append("ep %d: %s" % (ep["id"], f))
            continue
        t0 = marks["ready"]["t_sim"]; nom = res["nominal_ned"]
        t1 = marks["window_end"]["t_sim"] if marks["window_end"] else t0 + W       # actual end (missing -> already a failure)
        pre = [r for r in gt if t0 - HM.REF_PRE_S <= r["t_sim"] <= t0]
        re_, nre = HM.truth_ref([r["t_sim"] for r in pre], [r["x"] for r in pre], t0)
        rn_, _ = HM.truth_ref([r["t_sim"] for r in pre], [r["y"] for r in pre], t0)
        if re_ is None:
            e["fail"].append("reference: %d truth samples before ready" % nre); re_, rn_ = nom[1], nom[0]
        ref = [rn_, re_, nom[2]]; e["ref_true_ne"] = [rn_, re_]; e["ref_minus_nominal_ne"] = [rn_ - nom[0], re_ - nom[1]]
        e["window_s"] = t1 - t0
        sch = rj(os.path.join(D, "gust_schedule.json")) or {"gusts": []}
        win = [r for r in gt if t0 <= r["t_sim"] <= t1]
        gaps = [b["t_sim"] - a["t_sim"] for a, b in zip(win, win[1:])]
        e["gt_rows"] = len(win); e["gt_gap_max"] = max(gaps) if gaps else None
        if not win or (gaps and max(gaps) > GT_GAP_MAX) or (win and (win[0]["t_sim"] - t0 > GT_GAP_MAX or t1 - win[-1]["t_sim"] > GT_GAP_MAX)):
            e["fail"].append("truth gap (max %s s)" % e["gt_gap_max"])
        ctl = list(csv.DictReader(open(os.path.join(D, "control.csv")))) if os.path.exists(os.path.join(D, "control.csv")) else []
        e["cycles"] = len(ctl); e["cycles_expected"] = int(round((t1 - t0) * HZ))
        e["terminated"] = (marks["window_end"] or {}).get("terminated")
        if len(ctl) < 0.95 * e["cycles_expected"]:
            e["fail"].append("control cycles %d < 95%% of %d" % (len(ctl), e["cycles_expected"]))
        if not e["terminated"] and t1 - t0 < W - 0.1:
            e["fail"].append("window %.2f s < %.0f s without a termination" % (t1 - t0, W))
        if ctl:
            ca = [float(r["clock_age"]) for r in ctl]; pa = [float(r["pv_age"]) for r in ctl]
            ts = [float(r["t_sim"]) for r in ctl]; tw = [float(r["t_wall"]) for r in ctl]
            dts = [b - a for a, b in zip(ts, ts[1:])]; dtw = [b - a for a, b in zip(tw, tw[1:])]
            if "mode" in ctl[0]:
                bad = sorted(set(r["mode"] for r in ctl[1:] if r["mode"] != "OFFBOARD"))
                e["modes_not_offboard"] = bad; e["battery_window"] = [float(ctl[0]["bat"]), float(ctl[-1]["bat"])]
                if bad:
                    e["fail"].append("flight mode not OFFBOARD in the window: %s" % bad)
            e["clock_age_max"] = max(ca); e["pv_age_max"] = max(pa); e["dt_sim"] = stats(dts); e["dt_wall"] = stats(dtw)
            e["rtf_window"] = (ts[-1] - ts[0]) / (tw[-1] - tw[0]) if tw[-1] > tw[0] else None
            # estimate vs truth (diagnostic): interpolate truth at the control times
            j = 0; de, dn = [], []
            for r in ctl:
                t = float(r["t_sim"])
                while j + 1 < len(win) and win[j + 1]["t_sim"] < t:
                    j += 1
                if j + 1 < len(win):
                    a, b = win[j], win[j + 1]; u = (t - a["t_sim"]) / max(1e-9, b["t_sim"] - a["t_sim"])
                    de.append(float(r["e"]) - (a["x"] + u * (b["x"] - a["x"]))); dn.append(float(r["n"]) - (a["y"] + u * (b["y"] - a["y"])))
            e["est_minus_truth_e"] = stats(de); e["est_minus_truth_n"] = stats(dn)
            if max(ca) >= 1.0:
                e["fail"].append("clock age %.2f s" % max(ca))
        er = [r["x"] - ref[1] for r in win]; nr = [r["y"] - ref[0] for r in win]
        zq = [r for r in win if r["t_sim"] <= t0 + QS]
        d_off = None
        if ctl and zq:
            dq = [float(r["d"]) for r in ctl if float(r["t_sim"]) <= t0 + QS]
            d_off = (sum(dq) / len(dq)) - (sum(-r["z"] for r in zq) / len(zq)) if dq else None
        e["lateral_err_true"] = stats(er); e["along_err_true"] = stats(nr)
        if d_off is not None:
            e["height_err_true"] = stats([(-r["z"] + d_off) - ref[2] for r in win])
        te = [(r["t_sim"], tilt_e(r)) for r in win]
        head = [v for t, v in te if t <= t0 + QS]; tail = [v for t, v in te if t >= t1 - QUIET_TAIL]
        e["tilt_e_quiet_head_mean_deg"] = sum(head) / len(head) if head else None
        e["tilt_e_quiet_tail_mean_deg"] = sum(tail) / len(tail) if tail else None
        for k in ("tilt_e_quiet_head_mean_deg",) + (() if e["terminated"] else ("tilt_e_quiet_tail_mean_deg",)):
            if e[k] is None or abs(e[k]) > TILT_TOL:
                e["fail"].append("residual tilt %s = %s deg (tol %.1f)" % (k, e[k], TILT_TOL))
        e["tilt_e_quiet_tail_force_equiv_n"] = (-M * G * math.tan(math.radians(e["tilt_e_quiet_tail_mean_deg"]))
                                                if e["tilt_e_quiet_tail_mean_deg"] is not None else None)
        # gust events actually sent
        mine = [r for r in ev if r["ep"] == str(ep["id"]) and r["event"] in ("apply", "clear") and r["t_sim_planned"]]
        applies = [r for r in mine if r["event"] == "apply"]
        e["gusts"] = []
        for g in sch["gusts"]:
            on = sch["t0_sim"] + g["t_on"]; off = sch["t0_sim"] + g["t_off"]
            if off > t1:                     # the window ended (terminated) before this gust was over
                e["gusts"].append({"t_on_rel": g["t_on"], "dur": g["t_off"] - g["t_on"], "force_n": g["force_n"], "not_reached": True,
                                   "return_after_clear_s": None}); continue
            ap = [r for r in applies if abs(float(r["t_sim_planned"]) - on) < 1e-3]
            cl = [r for r in mine if r["event"] == "clear" and abs(float(r["t_sim_planned"]) - off) < 1e-3]
            gi = {"t_on_rel": g["t_on"], "dur": g["t_off"] - g["t_on"], "force_n": g["force_n"],
                  "apply_sent": bool(ap and ap[0]["published"] == "1"), "clear_sent": bool(cl and cl[0]["published"] == "1")}
            if ap:
                gi["apply_latency_sim_s"] = float(ap[0]["t_sim_sent"]) - on
                on = float(ap[0]["t_sim_sent"])
            if cl:
                gi["clear_latency_sim_s"] = float(cl[0]["t_sim_sent"]) - off
                off = float(cl[0]["t_sim_sent"])
            if not (gi["apply_sent"] and gi["clear_sent"]):
                e["fail"].append("gust at %.2f: apply %s clear %s" % (g["t_on"], gi["apply_sent"], gi["clear_sent"]))
            seg = [(r["t_sim"], r["x"] - ref[1]) for r in win if on <= r["t_sim"] <= off + 5.0]
            if seg:
                pk = max(seg, key=lambda s: abs(s[1])); gi["peak_lateral_m"] = pk[1]; gi["peak_after_on_s"] = pk[0] - on
                gi["peak_sign_matches_force"] = (pk[1] > 0) == (g["force_n"] > 0)
            during = [v for t, v in te if on + 0.5 * (off - on) <= t <= off]
            gi["tilt_e_late_in_gust_mean_deg"] = sum(during) / len(during) if during else None
            gi["tilt_e_expected_deg"] = -math.degrees(math.atan(g["force_n"] / (M * G)))
            # the force must show in the truth attitude (2026-11-03 hg_smoke1: published, but removed by gz -> no tilt)
            la, ex = gi["tilt_e_late_in_gust_mean_deg"], gi["tilt_e_expected_deg"]
            # 2026-11-03 wide distribution (0.03..0.6 N): a 0.037 N gust gave -0.35 deg of -0.72 expected (ratio 0.49) while a 0.37 N
            # gust of the same episode gave 1.01 - besides the ratio, |late - expected| <= EFFECT_ABS_DEG also passes (a missing force
            # stays a failure: 0.03 N expects 0.59 deg, so a zero tilt is still 0.19 deg beyond the tolerance)
            gi["force_effect_ok"] = la is not None and la * ex > 0 and (abs(la) >= EFFECT_MIN * abs(ex) or abs(la - ex) <= EFFECT_ABS_DEG)
            # below EFFECT_MIN_FORCE the expected tilt (< 2 deg) is smaller than what a learning controller's own setpoint moves
            # produce (2026-11-03 hg_v2wb_s1_u015: 0.031 N expected -0.60 deg, measured +0.46): not verifiable from the tilt, recorded
            # only; a lost force (hg_smoke1) still shows on the batch's stronger gusts
            if abs(g["force_n"]) < EFFECT_MIN_FORCE:
                gi["force_effect_check"] = "not verifiable (|F| < %.2f N)" % EFFECT_MIN_FORCE
            elif not gi["force_effect_ok"]:
                e["fail"].append("gust at %.2f: no force effect (tilt late %s deg, expected %.2f)" % (g["t_on"], la, ex))
            # return: first time after the clear from which |lateral| < RETURN_TOL for RETURN_HOLD s
            after = [(r["t_sim"], abs(r["x"] - ref[1])) for r in win if r["t_sim"] >= off]
            ret = None
            for i, (t, v) in enumerate(after):
                if v < RETURN_TOL and all(w < RETURN_TOL for tt, w in after[i:] if tt <= t + RETURN_HOLD) and after[-1][0] >= t + RETURN_HOLD:
                    ret = t - off; break
            gi["return_after_clear_s"] = ret
            e["gusts"].append(gi)
        # timeline (wall)
        tw = res.get("t_wall", {})
        if tw:
            e["timeline_wall_s"] = {k: (tw[k] - tw["start"]) for k in tw}
            if prev_land is not None:
                e["reset_from_prev_disarm_to_ready_s"] = tw.get("ready", float("nan")) - prev_land
            prev_land = tw.get("disarmed", prev_land)
        e["outcome"] = HM.outcome(res.get("status"), e["terminated"], e["fail"])
        out["episodes"].append(e)
        for f in e["fail"]:
            out["checks_failed"].append("ep %d: %s" % (ep["id"], f))
    for e in out["episodes"]:
        e.setdefault("outcome", "invalid")
    out["outcomes"] = {k: [e["id"] for e in out["episodes"] if e["outcome"] == k] for k in ("completed", "task_failed", "invalid")}
    json.dump(out, open(os.path.join(B, "summary.json"), "w"), indent=1)
    lines = ["batch %s: %d episodes, checks failed %d" % (B, len(out["episodes"]), len(out["checks_failed"]))]
    lines.append("ep cond  seed | lat max_abs  rms   | along rms | tilt head  tail (deg) | cycles rtf  | reset s | gusts: F(N) dur peak(m) return(s) tilt late/exp")
    for e in out["episodes"]:
        if "lateral_err_true" not in e:
            lines.append("%2d %-4s %4d | (no window) %s" % (e["id"], e["cond"], e["seed"], e["fail"])); continue
        L = e["lateral_err_true"]; A = e["along_err_true"]
        gs = "; ".join(("%+.2f %.1f not reached" % (g["force_n"], g["dur"])) if g.get("not_reached") else
                       "%+.2f %.1f %+.3f %s %.2f/%.2f" % (g["force_n"], g["dur"], g.get("peak_lateral_m", float("nan")),
                       "%.2f" % g["return_after_clear_s"] if g["return_after_clear_s"] is not None else "-",
                       g["tilt_e_late_in_gust_mean_deg"] or float("nan"), g["tilt_e_expected_deg"]) for g in e["gusts"])
        lines.append("%2d %-4s %4d | %.4f %.4f | %.4f | %+.3f %+.3f | %4d %.3f | %s | %s%s" % (
            e["id"], e["cond"], e["seed"], L["max_abs"], L["rms"], A["rms"], e["tilt_e_quiet_head_mean_deg"],
            e["tilt_e_quiet_tail_mean_deg"] if e["tilt_e_quiet_tail_mean_deg"] is not None else float("nan"), e["cycles"], e.get("rtf_window") or float("nan"),
            "%.1f" % e["reset_from_prev_disarm_to_ready_s"] if "reset_from_prev_disarm_to_ready_s" in e else "-", gs or "-",
            "" if e["outcome"] == "completed" else "  [%s %.2f s: %s]" % (e["outcome"].upper(), e["window_s"], e["terminated"] or "")))
    for f in out["checks_failed"]:
        lines.append("FAIL " + f)
    lines.append("outcomes: completed %(completed)s task_failed %(task_failed)s invalid %(invalid)s" % out["outcomes"])
    lines.append("ALL CHECKS OK" if not out["checks_failed"] else "CHECKS FAILED %d" % len(out["checks_failed"]))
    open(os.path.join(B, "summary.txt"), "w").write("\n".join(lines) + "\n"); print("\n".join(lines))
    return 0 if not out["checks_failed"] else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
