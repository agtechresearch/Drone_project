#!/usr/bin/env python3
"""Real hover analysis (2026-11-03): marker (tag) / VIO / PX4 position during the hold of one hover_flight.sh run.
  python3 hover_analyze.py <hover_dir> [--tag-size 0.100] [--extrinsics <extrinsics.conf>] [--trim 2.0]
Hold window = from "[goto takeoff] [END] ..." + trim to "CUSTOM MISSION COMPLETE" - trim (launcher.log wall times, mapped to
CLOCK_MONOTONIC with rec/record.json's (wall, monotonic) pairs; the VOXL clock is UTC, the run date from record.json).
Sources: tag (tag_detections, body pose in the tag frame, tag_ref.py; x = right of the tag centre seen facing it, forward =
toward the tag), VIO (vvhub_body_wrt_local, pose_vel_6dof_t 84 B: magic, ts ns, T[3], R[3x3], v[3], w[3]), PX4 estimate
(px4_vehicle_local_position, same layout - checked: magic and 84 B spacing). VIO / PX4 horizontal positions are rotated into
the vehicle's heading at the hold start (forward, right) so all three use the same axes. Per source: samples, rate, and for
right / forward / height the mean, RMS about the mean, max |dev| and peak-to-peak. Writes <hover_dir>/analysis.json and
hold_series.csv (one row per tag sample with the nearest VIO / PX4 sample)."""
import argparse, csv, datetime, json, math, os, re, struct, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import tag_ref as TR, tag_step1 as T1

POSE = struct.Struct("<Iq3f9f3f3f"); POSE_MAGIC = 0x706F7365


def poses(path):
    b = open(path, "rb").read(); mb = struct.pack("<I", POSE_MAGIC); o = 0; out = []; skipped = 0
    while o + POSE.size <= len(b):
        if b[o:o + 4] != mb:
            n = b.find(mb, o + 1)
            if n < 0:
                break
            skipped += n - o; o = n; continue
        v = POSE.unpack_from(b, o); o += POSE.size
        R = np.array(v[5:14]).reshape(3, 3); yaw = math.atan2(R[1, 0], R[0, 0])
        out.append((v[1] / 1e9, v[2], v[3], v[4], yaw))
    return np.array(out), skipped


def wall_of(line, date):
    m = re.match(r"\[(\d\d):(\d\d):(\d\d\.\d+)\]", line)
    if not m:
        return None
    h, mi, se = int(m.group(1)), int(m.group(2)), float(m.group(3))
    return datetime.datetime(date.year, date.month, date.day, h, mi, tzinfo=datetime.timezone.utc).timestamp() + se


def stats(v):
    v = np.asarray(v)
    if len(v) == 0:
        return None
    m = float(v.mean()); d = v - m
    return {"n": int(len(v)), "mean": m, "rms_about_mean": float(np.sqrt(np.mean(d ** 2))), "max_abs_dev": float(np.max(np.abs(d))),
            "p2p": float(v.max() - v.min())}


def main(a):
    D = a.hover_dir; rec = json.load(open(os.path.join(D, "rec", "record.json")))
    off = rec["start"][0] - rec["start"][1]                 # wall - monotonic
    date = datetime.datetime.fromtimestamp(rec["start"][0], datetime.timezone.utc)
    L = open(os.path.join(D, "flight", "launcher.log"), errors="replace").read().splitlines()
    t_hold0 = next(wall_of(l, date) for l in L if "[goto takeoff] [END]" in l) + a.trim
    t_hold1 = next(wall_of(l, date) for l in L if "CUSTOM MISSION COMPLETE" in l) - a.trim
    m0, m1 = t_hold0 - off, t_hold1 - off
    # tag
    T1.decode(os.path.join(D, "rec", "tag_detections.bin"), os.path.join(D, "rec", "tag_detections.jsonl"))
    E = TR.load_extrinsics(a.extrinsics); Tbc = TR.body_from_cam(E, "tracking_front"); tag = []
    for line in open(os.path.join(D, "rec", "tag_detections.jsonl")):
        r = json.loads(line); d = r["dets"][0]
        if d["cam"] == "tracking_front" and d["id"] == 0:
            p = TR.pose_fields(TR.tag_from_body(Tbc, d["R"], d["t_cam"], a.tag_size / d["size_m"]))
            tag.append((r["t"], p["x_m"], -p["dist_m"], p["height_above_tag_m"], p["yaw_deg"]))
    tag = np.array(tag); th = tag[(tag[:, 0] >= m0) & (tag[:, 0] <= m1)]
    res = {"hold_wall_utc": [datetime.datetime.fromtimestamp(t, datetime.timezone.utc).isoformat() for t in (t_hold0, t_hold1)],
           "hold_s": t_hold1 - t_hold0, "trim_s": a.trim, "tag_size_m": a.tag_size}
    res["tag"] = {"n": int(len(th)), "rate_hz": (len(th) - 1) / (th[-1, 0] - th[0, 0]) if len(th) > 1 else None,
                  "max_gap_s": float(np.max(np.diff(th[:, 0]))) if len(th) > 1 else None,
                  "right_m": stats(th[:, 1]), "forward_m": stats(th[:, 2]), "height_above_tag_m": stats(th[:, 3]), "yaw_deg": stats(th[:, 4])}
    series = {"tag": th}
    for name, fn in (("vio", "vvhub_body_wrt_local.bin"), ("px4", "px4_vehicle_local_position.bin")):
        P, sk = poses(os.path.join(D, "rec", fn)); h = P[(P[:, 0] >= m0) & (P[:, 0] <= m1)]
        yaw0 = float(np.median(h[:20, 4])); c, s = math.cos(yaw0), math.sin(yaw0)
        dx, dy = h[:, 1] - h[0, 1], h[:, 2] - h[0, 2]
        fwd = c * dx + s * dy; rgt = -s * dx + c * dy; up = -(h[:, 3])
        res[name] = {"n": int(len(h)), "rate_hz": (len(h) - 1) / (h[-1, 0] - h[0, 0]), "skipped_bytes": sk, "yaw0_deg": math.degrees(yaw0),
                     "right_m": stats(rgt), "forward_m": stats(fwd), "height_m": stats(up),
                     "yaw_drift_deg": float(math.degrees(np.unwrap(h[:, 4])[-1] - np.unwrap(h[:, 4])[0]))}
        series[name] = np.column_stack([h[:, 0], rgt, fwd, up])
    with open(os.path.join(D, "hold_series.csv"), "w") as f:
        f.write("t_mono,tag_right,tag_forward,tag_height,vio_right,vio_forward,vio_up,px4_right,px4_forward,px4_up\n")
        for r in th:
            row = [r[0], r[1], r[2], r[3]]
            for k in ("vio", "px4"):
                S = series[k]; i = int(np.argmin(np.abs(S[:, 0] - r[0]))); row += list(S[i, 1:])
            f.write(",".join("%.5f" % x for x in row) + "\n")
    # tag vs VIO consistency: both demeaned right / forward at the tag times
    A = np.loadtxt(os.path.join(D, "hold_series.csv"), delimiter=",", skiprows=1)
    if len(A) > 3:
        for k, (ti, vi) in (("right", (1, 4)), ("forward", (2, 5))):
            d = (A[:, ti] - A[:, ti].mean()) - (A[:, vi] - A[:, vi].mean())
            res.setdefault("tag_minus_vio", {})[k] = {"rms_m": float(np.sqrt(np.mean(d ** 2))), "max_abs_m": float(np.max(np.abs(d)))}
    json.dump(res, open(os.path.join(D, "analysis.json"), "w"), indent=1)
    print("hold %.1f s (%s .. %s, trim %.1f s)" % (res["hold_s"], res["hold_wall_utc"][0][11:23], res["hold_wall_utc"][1][11:23], a.trim))
    f = lambda s: "mean %+.4f  RMS %.4f  max|dev| %.4f  p2p %.4f" % (s["mean"], s["rms_about_mean"], s["max_abs_dev"], s["p2p"]) if s else "-"
    print("tag: %d samples, %.2f Hz, largest gap %.2f s" % (res["tag"]["n"], res["tag"]["rate_hz"] or 0, res["tag"]["max_gap_s"] or 0))
    for k in ("right_m", "forward_m", "height_above_tag_m", "yaw_deg"):
        print("   %-20s %s" % (k, f(res["tag"][k])))
    for name in ("vio", "px4"):
        r = res[name]; print("%s: %d samples, %.1f Hz, yaw drift %.2f deg" % (name, r["n"], r["rate_hz"], r["yaw_drift_deg"]))
        for k in ("right_m", "forward_m", "height_m"):
            print("   %-20s %s" % (k, f(r[k])))
    for k, v in res.get("tag_minus_vio", {}).items():
        print("tag - VIO (demeaned) %-8s RMS %.4f  max %.4f m" % (k, v["rms_m"], v["max_abs_m"]))
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("hover_dir"); ap.add_argument("--tag-size", type=float, default=0.100)
    ap.add_argument("--extrinsics", default=os.path.join(HERE, "collect_20261103", "20261103_071629", "conf", "etc_modalai", "extrinsics.conf"))
    ap.add_argument("--trim", type=float, default=2.0)
    sys.exit(main(ap.parse_args()))
