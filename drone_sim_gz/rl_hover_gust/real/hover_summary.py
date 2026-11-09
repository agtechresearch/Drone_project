#!/usr/bin/env python3
"""Quick on-vehicle summary of a hover recording (2026-11-03). Python 3.6 stdlib.
  python3 hover_summary.py <rec_dir> [--tag-size 0.100]
Decodes <rec_dir>/tag_detections.bin (tag_step1.decode), computes the BODY pose in the tag frame (tag_ref.py, extrinsics
/etc/modalai/extrinsics.conf, front camera, id 0) and prints detections, rate, the largest gap, and median / sd / min / max of
the body position relative to the tag: x (right of the tag centre, vehicle facing the tag = PX4 east), distance, height above
the tag centre, yaw. Writes <rec_dir>/tag_ref.csv and summary.json. The full analysis (hold window, VIO / PX4 comparison)
is done on the server."""
import json, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import tag_ref as TR, tag_step1 as T1


def stats(v):
    if not v:
        return None
    s = sorted(v); n = len(s); m = sum(s) / n
    return {"n": n, "median": s[n // 2], "sd": math.sqrt(sum((x - m) ** 2 for x in s) / max(1, n - 1)), "min": s[0], "max": s[-1]}


a = sys.argv[1:]; D = a[0]; size = float(a[a.index("--tag-size") + 1]) if "--tag-size" in a else 0.100
ext = os.environ.get("TAG_STEP1_EXTRINSICS", "/etc/modalai/extrinsics.conf")
binp = os.path.join(D, "tag_detections.bin")
if not os.path.exists(binp):
    sys.exit("no %s" % binp)
dec = T1.decode(binp, os.path.join(D, "tag_detections.jsonl"))
E = TR.load_extrinsics(ext); Tbc = TR.body_from_cam(E, "tracking_front"); rows = []; other = {}
with open(os.path.join(D, "tag_ref.csv"), "w") as f:
    f.write("t,x_m,dist_m,height_above_tag_m,yaw_deg,roll_deg,pitch_deg\n")
    for line in open(os.path.join(D, "tag_detections.jsonl")):
        r = json.loads(line); d = r["dets"][0]
        if d["cam"] != "tracking_front" or d["id"] != 0:
            k = "%s/id%d" % (d["cam"], d["id"]); other[k] = other.get(k, 0) + 1; continue
        p = TR.pose_fields(TR.tag_from_body(Tbc, d["R"], d["t_cam"], size / d["size_m"]))
        f.write("%.6f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f\n" % (r["t"], p["x_m"], p["dist_m"], p["height_above_tag_m"], p["yaw_deg"], p["roll_deg"], p["pitch_deg"]))
        rows.append((r["t"], p))
ts = [t for t, _ in rows]
sm = {"decode": dec, "front_id0": len(rows), "other": other,
      "span_s": (ts[-1] - ts[0]) if len(ts) > 1 else 0.0, "max_gap_s": max((b - x for x, b in zip(ts, ts[1:])), default=None)}
sm["rate_hz"] = (len(ts) - 1) / sm["span_s"] if sm["span_s"] > 0 else None
for k in ("x_m", "dist_m", "height_above_tag_m", "yaw_deg"):
    sm[k] = stats([p[k] for _, p in rows])
json.dump(sm, open(os.path.join(D, "summary.json"), "w"), indent=1)
print("tag: %d front id-0 detections over %.1f s (%.1f Hz, largest gap %s s), decode failed %d, other %s" % (
    len(rows), sm["span_s"], sm["rate_hz"] or 0, "%.2f" % sm["max_gap_s"] if sm["max_gap_s"] is not None else "-", dec["failed"], other or "-"))
for k, lab in (("x_m", "left-right x (m)"), ("dist_m", "distance (m)"), ("height_above_tag_m", "height above tag (m)"), ("yaw_deg", "yaw (deg)")):
    s = sm[k]
    print("  %-22s %s" % (lab, "median %+.3f  sd %.4f  min %+.3f  max %+.3f" % (s["median"], s["sd"], s["min"], s["max"]) if s else "-"))
