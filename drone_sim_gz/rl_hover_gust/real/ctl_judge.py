#!/usr/bin/env python3
"""Pre-registered judgement of the real PPO vs PX4-alone fan flights (2026-11-04, real/EVAL_METRIC.md "사전 등록").
Every rule here was fixed BEFORE the flights; do not change it after seeing results.

Per flight (a ctl_flight.sh directory: ctl/ctl.json + ctl.csv, rec/, flight/):
  hover t = loop clock since the post_takeoff hover start (ctl.json t0_mono = VOXL CLOCK_MONOTONIC, same clock as rec/ and
  fan_mark.py). Fan ON / OFF = the first "on" in the hover and the first "off" after it, from a fan_mark.py csv.
  lateral = tag translation rotated by the PX4 heading change (method A of fan_two_methods.py), zero = mean from the end of the
  settle phase to fan ON. PRIMARY = max |1 s moving mean (5 tag samples)| from fan ON to OFF + 5 s, in cm.
Validity (decided beforehand; an invalid flight is re-flown, never analysed as a result):
  launcher exit in {0, 2} (2 = v13 take-off judgement, the hover still runs in full); no "[ctl] off" / abort; fan ON and OFF
  recorded with ON >= settle + 3 s (baseline) and OFF + 5 s <= hover end - end_s (before the ramp back); tag coverage in
  [settle, OFF + 5 s]: >= 3 samples/s on average and no gap > 1.5 s (was 1.0 s until 2026-11-05, changed before any comparison flight).
Test: one-sided exact permutation test on the rank sum (Mann-Whitney), H1 = PPO smaller than PX4 alone. Two looks:
  look 1 at 4 + 4 valid flights, alpha 0.025; if not significant, 2 more of each and look 2 at 6 + 6, alpha 0.025 (overall <= 0.05).
Mechanism evidence (PPO flights): correlation of delta with the estimate's east error in the active phase (expected < 0),
  delta during the fan vs before.
  python3 ctl_judge.py <flight dirs ...> [--events-dir DIR] [--tag-size 0.100] [--out judge.json]"""
import argparse, glob, itertools, json, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import hover_analyze as H, tag_ref as TR  # noqa: E402

EXTR = os.path.join(HERE, "collect_20261103", "20261103_071629", "conf", "etc_modalai", "extrinsics.conf")
LOOKS = {1: (4, 0.025), 2: (6, 0.025)}
MIN_RATE, MAX_GAP, POST_S, BASE_MIN_S = 3.0, 1.5, 5.0, 3.0   # MAX_GAP 1.0 -> 1.5 s on 2026-11-05, before any comparison flight


def group_of(info):
    if info["ctrl"] == "zero":
        return "px4"
    p = os.path.basename(info["options"].get("policy") or "")
    for key, g in (("bcrand", "ppo"), ("pd_imit", "pd"), ("gz_wide", "gz"), ("sur_scratch", "scratch"), ("sur_bc_", "bc")):
        if key in p:      # technique ladder groups added 2026-11-05: gz < scratch < bc < ppo(bcrand)
            return g
    return "other:" + p


def fan_times(t0, dur, dirs):
    for c in dirs:
        for f in sorted(glob.glob(os.path.join(c, "fan_*.csv"))) if os.path.isdir(c) else [c]:
            if not os.path.exists(f):
                continue
            ev = [l.strip().split(",") for l in open(f).read().splitlines()[1:] if l.strip()]
            ev = [(float(e[0]) - t0, e[3]) for e in ev]
            ons = [t for t, k in ev if k == "on" and 0 <= t <= dur]
            if ons:
                offs = [t for t, k in ev if k == "off" and t > ons[0]]
                return ons[0], (offs[0] if offs else None), f
    return None, None, None


def flight(D, a):
    r = {"dir": D, "valid": False, "why": []}
    CD = os.path.join(D, "ctl") if os.path.isdir(os.path.join(D, "ctl")) else os.path.join(D, "flight", "ctl")   # ctl_flight.sh writes flight/ctl/
    try:
        info = json.load(open(os.path.join(CD, "ctl.json")))
    except Exception as e:  # noqa: BLE001
        r["why"].append("no ctl/ctl.json (%s)" % e); return r
    r["group"] = group_of(info); o = info["options"]; dur = info.get("duration_s"); t0 = info.get("t0_mono")
    if t0 is None or dur is None:
        r["why"].append("controller hover never started"); return r
    settle, end_s = o["settle_s"], o["end_s"]
    try:
        r["exit"] = json.load(open(os.path.join(D, "flight", "mission_result.json")))["exit"]
    except Exception:  # noqa: BLE001
        r["exit"] = None
    if r["exit"] not in (0, 2):
        r["why"].append("launcher exit %s" % r["exit"])
    kinds = [e["kind"] for e in info.get("events", [])]
    if info.get("off") or "off" in kinds or "aborted" in kinds:
        r["why"].append("correction off / aborted: %s" % (info.get("off") or "aborted"))
    rec = json.load(open(os.path.join(D, "rec", "record.json")))
    on, off, src = fan_times(t0, dur, ([a.events_dir] if a.events_dir else []) + [os.path.join(D, "fan_events.csv"),
                                                                                os.path.join(os.path.dirname(os.path.abspath(D)), "fan_events")])
    r.update(fan_on=on, fan_off=off, fan_src=src)
    if on is None or off is None:
        r["why"].append("fan ON/OFF not recorded"); return r
    if on < settle + BASE_MIN_S:
        r["why"].append("fan ON %.1f s < settle + %.0f s (no baseline)" % (on, BASE_MIN_S))
    if off + POST_S > dur - end_s:
        r["why"].append("fan OFF + %.0f s = %.1f s runs into the ramp (%.1f s)" % (POST_S, off + POST_S, dur - end_s))
    # tag lateral, method A
    Tbc = TR.body_from_cam(TR.load_extrinsics(EXTR), "tracking_front")
    P, _ = H.poses(os.path.join(D, "rec", "px4_vehicle_local_position.bin")); py = np.unwrap(P[:, 4])
    R = []
    for line in open(os.path.join(D, "rec", "tag_detections.jsonl")):
        q = json.loads(line); d = [x for x in q["dets"] if x["cam"] == "tracking_front" and x["id"] == 0]
        if not d:
            continue
        d = d[0]; sc = a.tag_size / d["size_m"]; pb = TR.Tf(d["R"], [v * sc for v in d["t_cam"]])
        pb = np.array((Tbc @ pb).t); j = int(np.argmin(np.abs(P[:, 0] - q["t"]))); R.append((q["t"] - t0, pb[0], pb[1], py[j]))
    A = np.array(R) if R else np.zeros((0, 4))
    A = A[(A[:, 0] >= settle) & (A[:, 0] <= off + POST_S)] if len(A) else A
    span = off + POST_S - settle
    if len(A) < 2 or len(A) / span < MIN_RATE or np.max(np.diff(np.concatenate([[settle], A[:, 0], [off + POST_S]]))) > MAX_GAP:
        r["why"].append("tag coverage %d samples in %.1f s (need >= %.0f/s, no gap > %.1f s)" % (len(A), span, MIN_RATE, MAX_GAP))
        r["tag_n"] = int(len(A)); return r
    t = A[:, 0]; ps = A[:, 3] - np.median(A[t < on, 3])
    lat = -(np.sin(ps) * A[:, 1] + np.cos(ps) * A[:, 2]); lat = lat - lat[t < on].mean()
    mm = np.convolve(lat, np.ones(5) / 5, mode="same"); w = (t >= on) & (t <= off + POST_S)
    i = np.argmax(np.abs(mm[w]))
    r.update(tag_n=int(len(A)), peak_cm=float(100 * abs(mm[w][i])), peak_signed_cm=float(100 * mm[w][i]), peak_t=float(t[w][i]),
             heading_change_deg=float(np.degrees(np.max(np.abs(ps)))), pre_sd_cm=float(100 * lat[t < on].std()),
             wall_start=rec["start"][0])
    # mechanism evidence from ctl.csv
    import csv
    c = [x for x in csv.DictReader(open(os.path.join(CD, "ctl.csv"))) if x["phase"] == "active"]
    if c:
        e = np.array([float(x["e_err"]) for x in c]); dl = np.array([float(x["delta"]) for x in c]); tt = np.array([float(x["t"]) for x in c])
        r["delta_max_cm"] = float(100 * np.max(np.abs(dl)))
        r["corr_delta_eerr"] = float(np.corrcoef(dl, e)[0, 1]) if np.std(dl) > 1e-9 else None
        r["delta_mean_fan_cm"] = float(100 * dl[(tt >= on) & (tt <= off)].mean()); r["delta_mean_pre_cm"] = float(100 * dl[tt < on].mean())
    r["valid"] = not r["why"]
    return r


def perm_p(x, y):
    """one-sided exact p of the rank sum of x being this small (x = PPO, y = PX4), ties by mid-ranks"""
    z = np.concatenate([x, y]); s = np.sort(z); ranks = {}
    for v in np.unique(s):
        idx = np.flatnonzero(s == v); ranks[v] = idx.mean() + 1
    rk = np.array([ranks[v] for v in z]); obs = rk[:len(x)].sum(); n = 0; k = 0
    for comb in itertools.combinations(range(len(z)), len(x)):
        n += 1; k += rk[list(comb)].sum() <= obs + 1e-9
    return k / n


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("dirs", nargs="+"); ap.add_argument("--events-dir")
    ap.add_argument("--tag-size", type=float, default=0.100); ap.add_argument("--out", default="ctl_judge.json")
    ap.add_argument("--pairs", default="ppo<px4", help="comma list of A<B: H1 = group A pushed less than group B (one-sided)")
    a = ap.parse_args()
    F = sorted([flight(d, a) for d in a.dirs], key=lambda r: r.get("wall_start") or 0)
    print("flight                                  group exit valid peak(cm) pre-sd heading  max|delta| corr(delta,e)  why")
    for r in F:
        print("%-40s %-5s %-4s %-5s %8s %6s %7s %10s %13s  %s" % (
            os.path.basename(os.path.normpath(r["dir"])), r.get("group", "?"), r.get("exit"), "yes" if r["valid"] else "NO",
            "%.2f" % r["peak_cm"] if "peak_cm" in r else "-", "%.2f" % r["pre_sd_cm"] if "pre_sd_cm" in r else "-",
            "%.1f" % r["heading_change_deg"] if "heading_change_deg" in r else "-", "%.1f" % r["delta_max_cm"] if "delta_max_cm" in r else "-",
            "%+.2f" % r["corr_delta_eerr"] if r.get("corr_delta_eerr") is not None else "-", "; ".join(r["why"])))
    V = [r for r in F if r["valid"]]
    out = {"flights": F, "rule": {"looks": LOOKS, "primary": "max |1 s moving mean| fan ON .. OFF+5 s, cm, tag + PX4 heading"}, "results": {}}
    print("\nvalid per group: %s" % {g: sum(1 for r in V if r["group"] == g) for g in sorted(set(r["group"] for r in V))})
    for pair in a.pairs.split(","):
        A, B = pair.split("<"); X = [r["peak_cm"] for r in V if r["group"] == A]; Y = [r["peak_cm"] for r in V if r["group"] == B]
        look = 2 if min(len(X), len(Y)) >= LOOKS[2][0] else (1 if min(len(X), len(Y)) >= LOOKS[1][0] else None)
        if look is None:
            print("%s: not enough valid flights for look 1 (need %d of each, have %d / %d)" % (pair, LOOKS[1][0], len(X), len(Y))); continue
        n, alpha = LOOKS[look]; x, y = np.array(X[:n]), np.array(Y[:n])     # the first n valid flights of each, in flight order
        p = perm_p(x, y); ratio = float(np.median(x) / np.median(y))
        verdict = ("SIGNIFICANT (%s < %s)" % (A, B)) if p < alpha else ("not significant -> fly 2 more of each, look 2" if look == 1 else "NOT significant (final)")
        print("%s look %d (%d + %d): %s median %.2f cm, %s median %.2f cm, ratio %.0f %%, one-sided exact p = %.4f vs alpha %.3f -> %s" % (
            pair, look, n, n, A, np.median(x), B, np.median(y), 100 * ratio, p, alpha, verdict))
        out["results"][pair] = {"look": look, "n": n, "alpha": alpha, "p": p, A + "_cm": x.tolist(), B + "_cm": y.tolist(),
                                "median_ratio": ratio, "significant": bool(p < alpha), "verdict": verdict}
        if pair == "ppo<px4":
            out["result"] = out["results"][pair]
    json.dump(out, open(a.out, "w"), indent=1, default=float); print("-> %s" % a.out)


if __name__ == "__main__":
    main()
