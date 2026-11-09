#!/usr/bin/env python3
"""Real-vehicle step 1 (2026-11-03, plans/2026-11-03_tag_bench_step1.md): AprilTag detection -> inverse transform -> stored
reference. Run ON THE VOXL by the operator. Python 3.6, standard library only. Props off, no arm. Writes only under OUT.
  python3 tag_step1.py pre                       state before: service enabled/active, sha256 of the configs
  (operator)  systemctl start voxl-tag-detector  (start only, never enable)
  python3 tag_step1.py capture <pose> [--secs 15] [--tape-dist M] [--tape-left M] [--tape-up M]
                                                 records tag_detections + vvhub_body_wrt_local together, then decodes the tag
                                                 records, computes the body pose in the tag frame (tag_ref.py) and prints it
  (operator)  systemctl stop voxl-tag-detector
  python3 tag_step1.py post                      state after, compared with 'pre' (must be identical)
  python3 tag_step1.py report                    the table of every pose so far (also OUT/report.txt)
Tag size: the real tag is 0.100 m; tag_locations.conf says 0.40. The scale 0.100 / size_m is applied to the camera
translation (tag_ref.py). --tape-*: the operator's tape values, measured from the FRONT CAMERA LENS (distance to the tag face,
lens left of the tag centre, lens above the tag centre); the report compares them with the marker values of the lens point.
Output per pose in OUT/<pose>/: raw .bin/.idx/.info.json/.status.json of both pipes, tag_detections.jsonl (decoded),
tag_ref.csv (one row per detection: the stored reference), summary.json. Decoding: tag_detection_t from
/usr/include/pipe_interfaces/tag_detection_t.h (packed, 252 bytes, magic 0x564F584C, cam char[64] on Linux)."""
import argparse, hashlib, json, math, os, struct, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import tag_ref as TR

OUT = os.environ.get("TAG_STEP1_OUT", "/data/claude_collect/tag_20261103")
TOOLS = os.environ.get("TAG_STEP1_TOOLS", HERE)
CONFS = ["/etc/modalai/voxl-tag-detector.conf", "/etc/modalai/tag_locations.conf", "/etc/modalai/extrinsics.conf"]
EXTR = os.environ.get("TAG_STEP1_EXTRINSICS", "/etc/modalai/extrinsics.conf")
TAG = struct.Struct("<Iifq64si3f9f3f9f64si"); MAGIC = 0x564F584C; TAG_SIZE = 0.100; SERVICE = "voxl-tag-detector"
LENS_X = None            # body -> tracking_front translation, from the extrinsics (lens point = body + R t)


def sh(cmd):
    p = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return p.stdout.decode(errors="replace").strip()


def state():
    s = {"t": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "enabled": sh("systemctl is-enabled %s" % SERVICE),
         "active": sh("systemctl is-active %s" % SERVICE), "sha256": {}}
    for c in CONFS:
        s["sha256"][c] = hashlib.sha256(open(c, "rb").read()).hexdigest() if os.path.exists(c) else None
    s["tag_pipe"] = os.path.isdir("/run/mpa/tag_detections")
    return s


def decode(binpath, jsonl):
    """stdlib decoder (same layout and checks as mpa_parse.py) -> tag_ref.py input"""
    buf = open(binpath, "rb").read(); mb = struct.pack("<I", MAGIC); o = 0; n = bad = 0; skipped = 0; t_prev = None; reasons = {}
    with open(jsonl, "w") as f:
        while o + TAG.size <= len(buf):
            if buf[o:o + 4] != mb:
                nxt = buf.find(mb, o + 1)
                if nxt < 0:
                    skipped += len(buf) - o; o = len(buf); break
                skipped += nxt - o; o = nxt; continue
            v = TAG.unpack_from(buf, o); o += TAG.size
            R = [list(v[9 + 3 * i:12 + 3 * i]) for i in range(3)]; t = list(v[6:9]); why = []
            if v[3] <= 0 or (t_prev is not None and v[3] < t_prev):
                why.append("timestamp")
            if not all(math.isfinite(x) for x in t + sum(R, [])):
                why.append("not finite")
            else:
                RtR = TR.mat_mul(TR.transpose(R), R)
                if max(abs(RtR[i][j] - (1.0 if i == j else 0.0)) for i in range(3) for j in range(3)) > 0.02:
                    why.append("R not orthonormal")
            if not (v[2] > 0):
                why.append("size_m")
            cam = v[30].split(b"\0")[0].decode(errors="replace")
            if why:
                bad += 1
                for w in why:
                    reasons[w] = reasons.get(w, 0) + 1
                continue
            t_prev = v[3]; n += 1
            f.write(json.dumps({"t": v[3] / 1e9, "dets": [{"id": v[1], "t_cam": t, "R": R, "size_m": v[2], "cam": cam}]}) + "\n")
    return {"bytes": len(buf), "valid": n, "failed": bad, "reasons": reasons, "unaligned_bytes": skipped,
            "trailing_bytes": len(buf) - o if o < len(buf) else 0, "record_bytes": TAG.size}


def med(v):
    v = sorted(v); n = len(v)
    return None if not n else (v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2]))


def sd(v):
    if len(v) < 2:
        return None
    m = sum(v) / len(v); return math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1))


def capture(a):
    s = state()
    if (s["active"] != "active" or not s["tag_pipe"]) and os.environ.get("TAG_STEP1_TEST") != "1":   # test harness only
        print("!! %s is %s, tag pipe %s: start it first (systemctl start %s)" % (SERVICE, s["active"], s["tag_pipe"], SERVICE)); return 2
    D = os.path.join(OUT, a.pose)
    if os.path.exists(D):
        print("!! %s exists (choose another pose name)" % D); return 2
    os.makedirs(D); os.chdir(D); cap = os.path.join(TOOLS, "mpa_capture.py")
    print("recording %.0f s: tag_detections + vvhub_body_wrt_local -> %s" % (a.secs, D), flush=True)
    ps = [subprocess.Popen([sys.executable, cap, p, str(a.secs), a.pose]) for p in ("tag_detections", "vvhub_body_wrt_local")]
    rcs = [p.wait() for p in ps]
    sm = {"pose": a.pose, "secs": a.secs, "capture_rc": {"tag_detections": rcs[0], "vvhub_body_wrt_local": rcs[1]},
          "tape": {"dist_m": a.tape_dist, "left_m": a.tape_left, "up_m": a.tape_up}, "tag_size_m": TAG_SIZE, "state": s}
    binp = "%s_tag_detections.bin" % a.pose
    if os.path.exists(binp):
        sm["decode"] = decode(binp, "tag_detections.jsonl")
        E = TR.load_extrinsics(EXTR); Tbc = TR.body_from_cam(E, "tracking_front")
        rows = []; other = {}
        with open("tag_ref.csv", "w") as f:
            f.write("t,cam,id,x_m,y_m,z_m,dist_m,height_above_tag_m,yaw_deg,roll_deg,pitch_deg,scale,lens_x_m,lens_dist_m,lens_up_m\n")
            for line in open("tag_detections.jsonl"):
                r = json.loads(line); d = r["dets"][0]
                if d["cam"] != "tracking_front" or d["id"] != 0:
                    k = "%s/id%d" % (d["cam"], d["id"]); other[k] = other.get(k, 0) + 1; continue
                T = TR.tag_from_body(Tbc, d["R"], d["t_cam"], TAG_SIZE / d["size_m"]); p = TR.pose_fields(T)
                lens = T.apply(Tbc.t)                       # the lens point (camera origin) in the tag frame
                f.write("%.6f,%s,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.4f,%.5f,%.5f,%.5f\n" % (r["t"], d["cam"], d["id"], p["x_m"], p["y_m"],
                        p["z_m"], p["dist_m"], p["height_above_tag_m"], p["yaw_deg"], p["roll_deg"], p["pitch_deg"], TAG_SIZE / d["size_m"],
                        lens[0], -lens[2], -lens[1]))
                rows.append((r["t"], p, lens))
        sm["other_detections"] = other; sm["n_front_id0"] = len(rows)
        if rows:
            ts = [r[0] for r in rows]; span = ts[-1] - ts[0]
            g = lambda f: [f(r) for r in rows]
            sm["rate_hz"] = (len(rows) - 1) / span if span > 0 else None
            sm["max_gap_s"] = max(b - a_ for a_, b in zip(ts, ts[1:])) if len(ts) > 1 else None
            for k, f in (("body_x_m", lambda r: r[1]["x_m"]), ("body_dist_m", lambda r: r[1]["dist_m"]),
                         ("body_height_above_tag_m", lambda r: r[1]["height_above_tag_m"]), ("yaw_deg", lambda r: r[1]["yaw_deg"]),
                         ("roll_deg", lambda r: r[1]["roll_deg"]), ("pitch_deg", lambda r: r[1]["pitch_deg"]),
                         ("lens_left_m", lambda r: -r[2][0]), ("lens_dist_m", lambda r: -r[2][2]), ("lens_up_m", lambda r: -r[2][1])):
                v = g(f); sm[k] = {"median": med(v), "sd": sd(v)}
            for k, tk in (("lens_dist_m", "dist_m"), ("lens_left_m", "left_m"), ("lens_up_m", "up_m")):
                if sm["tape"][tk] is not None:
                    sm[k]["minus_tape_m"] = sm[k]["median"] - sm["tape"][tk]
    json.dump(sm, open("summary.json", "w"), indent=1)
    print(line_of(sm)); report_write(); return 0


def line_of(sm):
    if not sm.get("n_front_id0"):
        dec = sm.get("decode", {})
        return "%-12s | NO front id-0 detections (capture rc %s, decoded %s, failed %s, other %s)" % (
            sm["pose"], sm["capture_rc"], dec.get("valid"), dec.get("failed"), sm.get("other_detections"))
    f = lambda k: "%.3f" % sm[k]["median"]
    t = lambda k: (" (%+.3f)" % sm[k]["minus_tape_m"]) if "minus_tape_m" in sm[k] else ""
    return "%-12s | n %4d %5.1f Hz gap %.2f s | lens dist %s%s left %s%s up %s%s | yaw %s roll %s pitch %s | sd dist %.4f left %.4f | fail %d other %s" % (
        sm["pose"], sm["n_front_id0"], sm["rate_hz"] or 0, sm["max_gap_s"] or 0, f("lens_dist_m"), t("lens_dist_m"), f("lens_left_m"), t("lens_left_m"),
        f("lens_up_m"), t("lens_up_m"), f("yaw_deg"), f("roll_deg"), f("pitch_deg"), sm["lens_dist_m"]["sd"] or 0, sm["lens_left_m"]["sd"] or 0,
        sm["decode"]["failed"], sm["other_detections"] or "-")


def report_write():
    L = ["pose         | detections, rate, max gap | marker (lens point; in brackets: marker - tape) m | angles deg | sd m | decode"]
    for p in sorted(os.listdir(OUT)):
        sp = os.path.join(OUT, p, "summary.json")
        if os.path.exists(sp):
            L.append(line_of(json.load(open(sp))))
    for k in ("pre", "post"):
        sp = os.path.join(OUT, "state_%s.json" % k)
        if os.path.exists(sp):
            s = json.load(open(sp)); L.append("state %-4s %s enabled=%s active=%s" % (k, s["t"], s["enabled"], s["active"]))
    open(os.path.join(OUT, "report.txt"), "w").write("\n".join(L) + "\n")
    return L


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("cmd", choices=["pre", "capture", "post", "report"]); ap.add_argument("pose", nargs="?")
    ap.add_argument("--secs", type=float, default=15.0); ap.add_argument("--tape-dist", type=float); ap.add_argument("--tape-left", type=float)
    ap.add_argument("--tape-up", type=float)
    a = ap.parse_args(); os.makedirs(OUT, exist_ok=True)
    if a.cmd == "pre":
        s = state(); p = os.path.join(OUT, "state_pre.json")
        if os.path.exists(p):
            print("!! %s exists" % p); return 2
        json.dump(s, open(p, "w"), indent=1); print(json.dumps(s, indent=1)); return 0
    if a.cmd == "capture":
        if not a.pose:
            print("!! pose name needed"); return 2
        return capture(a)
    if a.cmd == "post":
        s = state(); json.dump(s, open(os.path.join(OUT, "state_post.json"), "w"), indent=1)
        pre = json.load(open(os.path.join(OUT, "state_pre.json")))
        same = s["enabled"] == pre["enabled"] and s["active"] == pre["active"] and s["sha256"] == pre["sha256"]
        print(json.dumps(s, indent=1)); print("SAME AS BEFORE: %s" % ("YES" if same else "NO  <-- check (pre %s/%s)" % (pre["enabled"], pre["active"])))
        report_write(); return 0 if same else 1
    print("\n".join(report_write())); return 0


if __name__ == "__main__":
    sys.exit(main())
