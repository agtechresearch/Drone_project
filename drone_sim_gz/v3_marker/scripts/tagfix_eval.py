#!/usr/bin/env python3
"""Tag-scan initial fix accuracy vs ground truth (v3_marker, 2026-10-29).  QVIX_YAW_DEG=7 python3 tagfix_eval.py <run_dir>...
Compares the corrected estimate logged by wk_mission.py (wk.csv est_wx/est_wy = map x/y, yaw_est = map NED yaw) with the
TRUE model pose (tags.csv true_x/y/yaw from /model/<m>/odometry, world -> map with qvix_grid.world_to_model) from the
moment the tag fix is applied (first nonzero cY) to the end of the flight."""
import csv, json, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import qvix_grid as G  # noqa: E402

for d in sys.argv[1:]:
    T = [r for r in csv.DictReader(open(os.path.join(d, "tags.csv"))) if r["true_x"] not in ("nan", "")]; tt = np.array([float(r["t_sim"]) for r in T])
    TX = np.array([float(r["true_x"]) for r in T]); TY = np.array([float(r["true_y"]) for r in T])
    TW = np.unwrap([float(r["true_yaw"]) for r in T])          # interpolated truth: the scan/turns run at 30 deg/s
    cond = open(os.path.join(d, "condition.txt")).read().split()
    rot = float([c for c in cond if c.startswith("rot=")][0][4:])
    # truth of the frame rotation: VIO boots aligned with the world plus the injected rot, the map is the world turned by
    # MODEL_YAW_DEG -> PX4 yaw = map NED yaw - (MODEL_YAW_DEG + rot)
    dpsi_true = -(G.MODEL_YAW_DEG + rot)
    K = list(csv.DictReader(open(os.path.join(d, "wk.csv"))))
    fixj = os.path.join(d, "wk_tagfix.json"); fx = json.load(open(fixj)) if os.path.exists(fixj) else {}
    t_fix = None; rows = []
    for r in K:
        if t_fix is None and float(r["cY"]) != 0.0:
            t_fix = float(r["t_sim"])
        if t_fix is None:
            continue
        t = float(r["t_sim"])
        if t > tt[-1]:
            continue
        x, y, _ = G.world_to_model(np.interp(t, tt, TX), np.interp(t, tt, TY))
        yn = (90.0 - (math.degrees(np.interp(t, tt, TW)) - G.MODEL_YAW_DEG)) % 360.0
        rows.append((t - t_fix, float(r["est_wx"]) - x, float(r["est_wy"]) - y, ((float(r["yaw_est"]) - yn + 180) % 360) - 180,
                     float(r["cY"]), r["stage"]))
    if not rows:
        print(d, "no fix"); continue
    stg = [r[5] for r in rows]; a = np.array([r[:5] for r in rows]); first = a[a[:, 0] <= 1.0]
    print("%s\n  fix: tags %s used %s/%s  dpsi %+.2f deg (true %+.2f, error %+.2f deg)" % (os.path.basename(d), fx.get("ids"), fx.get("n_used"),
          fx.get("n_samples"), fx.get("dpsi_deg", float("nan")), dpsi_true, fx.get("dpsi_deg", float("nan")) - dpsi_true))
    print("  first 1 s after fix: |pos err| max %.4f m, heading err mean %+.3f deg" % (np.hypot(first[:, 1], first[:, 2]).max(), first[:, 3].mean()))
    print("  whole flight after fix: |pos err| max %.4f m, heading err max |%.3f| deg, cY %+.2f..%+.2f" % (
        np.hypot(a[:, 1], a[:, 2]).max(), np.abs(a[:, 3]).max(), a[:, 4].min(), a[:, 4].max()))
    # per detection (wk_tagfix.json "samples", 2026-10-29 late): tag-only error = map pose from the tag vs TRUE body pose at
    # the image time; the fix is judged against the ACTUAL PX4-vs-map rotation at those times (PX4's yaw itself is not exactly
    # truth + injected rot: EKF heading error), not against -(MODEL_YAW_DEG + rot)
    sm = [x for x in fx.get("samples", []) if x["used"]]
    if sm:
        eh, ep, act = [], [], []
        for x in sm:
            th = math.degrees(np.interp(x["t"], tt, TW)) - G.MODEL_YAW_DEG               # true map heading (CCW from +x)
            bx, by, _ = G.world_to_model(np.interp(x["t"], tt, TX), np.interp(x["t"], tt, TY))
            eh.append(((math.degrees(x["map_pose"][2]) - th + 180) % 360) - 180)
            ep.append(math.hypot(x["map_pose"][0] - bx, x["map_pose"][1] - by))
            act.append(((x["px4_yaw_deg"] - (90.0 - th) + 180) % 360) - 180)          # actual PX4 <- map rotation
        act_m = math.degrees(math.atan2(np.mean(np.sin(np.radians(act))), np.mean(np.cos(np.radians(act)))))
        print("  per detection (%d used): tag heading err mean %+.3f sd %.3f deg, tag pos err mean %.4f max %.4f m" % (
            len(sm), np.mean(eh), np.std(eh), np.mean(ep), np.max(ep)))
        print("  actual PX4<-map rotation at the detections %+.3f deg -> fix error %+.3f deg (vs assumed rot: %+.3f)" % (
            act_m, fx["dpsi_deg"] - act_m, fx["dpsi_deg"] - dpsi_true))
    for name in sorted(set(stg), key=stg.index):
        m = np.array([s_ == name for s_ in stg]); b = a[m]
        print("    %-26s n %4d  heading err median %+.2f max |%.2f| deg   pos err max %.4f m  (map x across %+.4f..%+.4f, y along %+.4f..%+.4f)" % (
            name or "(scan end)", len(b), np.median(b[:, 3]), np.abs(b[:, 3]).max(), np.hypot(b[:, 1], b[:, 2]).max(),
            b[:, 1].min(), b[:, 1].max(), b[:, 2].min(), b[:, 2].max()))
    rcj = os.path.join(d, "wk_recheck.json")
    if os.path.exists(rcj):
        for ev in json.load(open(rcj)):
            i = int(np.argmin(np.abs(a[:, 0] + t_fix - ev["t"])))
            print("    re-check t %.1f tag %d n %d innov x %+.4f y %+.4f h %+.2f %s | est err then x %+.4f y %+.4f" % (
                ev["t"], ev["id"], ev["n"], ev["dx"], ev["dy"], ev["dh"], "applied" if ev["applied"] else "rejected", a[i, 1], a[i, 2]))
