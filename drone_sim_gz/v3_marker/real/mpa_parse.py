#!/usr/bin/env python3
"""Decode mpa_capture.py recordings on the PC (2026-10-31). NOT YET CHECKED AGAINST REAL DATA: the record layouts below are
written from the libmodal_pipe headers as remembered (modal_pipe_interfaces.h) and every decode checks itself.
  python3 mpa_parse.py C1_rack_0p37m_tof_pc.bin          -> .npz (t_ns, xyz per frame) + a summary
  python3 mpa_parse.py E2_tag_0p5m_tag_detections.bin    -> .jsonl in the flight side's format {"t", "dets": [...]}
Self-checks (2026-11-01 codex real-tag-validation): NOTHING HERE IS VERIFIED AGAINST A REAL VOXL RECORDING YET - every
summary says "layout_verified": false until a real header / fixture has been compared.
  record start word (magic): every record must start with it. The value is NOT taken from the headers (not at hand): it is
    --magic HEX when given, else the first record's first 4 bytes, and the summary says which
  tags: timestamp > 0 and not going back, id >= 0, size_m > 0, camera name printable, t_cam finite, R finite with
    R^T R = I (max |.| <= 0.02) and det(R) = +1 (+-0.02). A failed record goes to <base>.bad.jsonl with its reasons, never
    into the flight-format .jsonl. A start word missing in the middle is a corrupt region (resynchronised at the next one);
    bytes after the last whole record are a partial record. Both are counted.
  point clouds: format known, the whole frame present; a short last frame is a partial frame.
  summary <base>.summary.json; exit 0 = every record valid and no corrupt / partial bytes, 2 = some records failed or
    corrupt / partial bytes (output written, NOT a clean decode), 3 = nothing decoded.
Tag quality: tag_detection_t has no decision margin / hamming -> written as null with "quality": "unavailable" (the
consumers' contract is scripts/tag_quality.py: rejected unless WK_TAG_QUALITY_UNAVAILABLE=accept)"""
import json, os, struct, sys
import numpy as np

# point_cloud_metadata_t (packed): magic u32, timestamp_ns i64, n_points u32, format u32, id u32, server_name char[32], reserved u32
PC_HDR = struct.Struct("<IqIII32sI")
PC_FMT_BYTES = {0: 12, 1: 16, 2: 15, 3: 19, 4: 8, 5: 12}     # XYZ, XYZC, XYZRGB, XYZCRGB, XY, XYC (float32 + uint8 rgb)
# tag_detection_t (packed): magic u32, id i32, size_m f32, timestamp_ns i64, name char[64], loc_type i32,
#   T_tag_wrt_cam f32[3], R_tag_to_cam f32[9], T_tag_wrt_fixed f32[3], R_tag_to_fixed f32[9], cam char[64], reserved i32
TAG = struct.Struct("<Iifq64si3f9f3f9f64si")


LAYOUT_VERIFIED = False      # set True only after a real VOXL header / fixture comparison (see the docstring)


def summary(out, d):
    d.update(layout_verified=LAYOUT_VERIFIED)
    json.dump(d, open(os.path.splitext(out)[0] + ".summary.json", "w"), indent=1)
    print("summary: %s" % json.dumps(d))
    if not LAYOUT_VERIFIED:
        print("  NOTE: record layout not yet checked against real VOXL data - not a validated decode")


def spacing(buf, magic):
    pos = []; i = buf.find(magic)
    while i >= 0 and len(pos) < 200:
        pos.append(i); i = buf.find(magic, i + 1)
    return np.diff(pos) if len(pos) > 1 else np.array([])


def point_clouds(buf, out, magic=None):
    src = "argument" if magic else "first 4 bytes"; magic = magic or buf[:4]; frames = []; o = 0; partial = 0
    while o + PC_HDR.size <= len(buf):
        mg, ts, n, fmt, pid, srv, _ = PC_HDR.unpack_from(buf, o)
        if buf[o:o + 4] != magic or fmt not in PC_FMT_BYTES:
            sp = spacing(buf, magic)
            summary(out, {"kind": "point_cloud", "status": "failed", "frames": len(frames), "lost_framing_at": o, "magic": magic.hex(),
                          "magic_source": src})
            print("lost framing at byte %d (fmt %s, n %s); magic spacings %s - fix PC_HDR / PC_FMT_BYTES" % (o, fmt, n, sp[:10]))
            return 3
        size = n * PC_FMT_BYTES[fmt]; data = buf[o + PC_HDR.size:o + PC_HDR.size + size]
        if len(data) < size:
            partial = len(buf) - o
            break
        if fmt in (0, 1, 5):
            k = {0: 3, 1: 4, 5: 3}[fmt]
            arr = np.frombuffer(data, np.float32).reshape(n, k)
        else:
            arr = None
        frames.append((ts, fmt, arr, srv.split(b"\0")[0].decode(errors="ignore")))
        o += PC_HDR.size + size
    if o < len(buf) and not partial and len(buf) - o < PC_HDR.size:
        partial = len(buf) - o
    if not frames:
        summary(out, {"kind": "point_cloud", "status": "failed", "frames": 0, "partial_bytes": partial, "magic": magic.hex(), "magic_source": src})
        print("no frame decoded; first bytes %s" % buf[:64].hex())
        return 3
    ts = np.array([f[0] for f in frames]); xyz = [f[2] for f in frames]
    np.savez_compressed(out, t_ns=ts, **{"f%05d" % i: a for i, a in enumerate(xyz) if a is not None})
    dt = np.diff(ts) / 1e9
    print("point clouds: %d frames, format %d (%s), %.2f Hz, points/frame median %d, server %s -> %s" % (
        len(frames), frames[0][1], {0: "xyz", 1: "xyz+conf", 5: "xy+conf"}.get(frames[0][1], "?"), 1 / np.median(dt) if len(dt) else 0,
        int(np.median([len(a) for a in xyz if a is not None])), frames[0][3], out))
    a = xyz[len(xyz) // 2]
    if a is not None:
        fin = np.isfinite(a[:, 0]) & (np.abs(a[:, :3]).sum(1) > 0)
        print("  middle frame: valid %.2f, x %.2f..%.2f, y %.2f..%.2f, z %.2f..%.2f m (axes as published by the server)" % (
            fin.mean(), *np.percentile(a[fin, 0], [2, 98]), *np.percentile(a[fin, 1], [2, 98]), *np.percentile(a[fin, 2], [2, 98])))
    summary(out, {"kind": "point_cloud", "status": "ok" if not partial else "partial", "frames": len(frames), "partial_bytes": partial,
                  "magic": magic.hex(), "magic_source": src})
    return 0 if not partial else 2


def check_tag(v, t_prev):
    """reasons a decoded tag_detection_t is not plausible ([] = valid)"""
    why = []
    R = np.array(v[9:18], float).reshape(3, 3)
    if v[3] <= 0:
        why.append("timestamp <= 0")
    elif t_prev is not None and v[3] < t_prev:
        why.append("timestamp goes back")
    if v[1] < 0:
        why.append("id < 0")
    if not (np.isfinite(v[2]) and v[2] > 0):
        why.append("size_m not > 0")
    cam = v[30].split(b"\0")[0]
    if not cam or not all(32 <= c < 127 for c in cam):
        why.append("camera name not printable")
    if not np.all(np.isfinite(v[6:9])):
        why.append("t_cam not finite")
    if not np.all(np.isfinite(R)):
        why.append("R not finite")
    else:
        if np.max(np.abs(R.T.dot(R) - np.eye(3))) > 0.02:
            why.append("R not orthonormal (max |R^T R - I| %.3g)" % np.max(np.abs(R.T.dot(R) - np.eye(3))))
        if abs(np.linalg.det(R) - 1.0) > 0.02:
            why.append("det(R) %.3g != +1" % np.linalg.det(R))
    return why


def tags(buf, out, magic=None):
    src = "argument" if magic else "first 4 bytes"; magic = magic or buf[:4]; sp = spacing(buf, magic)
    if len(sp) and not np.all(sp % TAG.size == 0):
        print("WARNING: magic spacing %s is not a multiple of the assumed record size %d" % (sorted(set(sp.tolist()))[:6], TAG.size))
    n = bad = 0; corrupt = []; o = 0; t_prev = None; reasons = {}
    badf = open(os.path.splitext(out)[0] + ".bad.jsonl", "w")
    with open(out, "w") as f:
        while o + TAG.size <= len(buf):
            if buf[o:o + 4] != magic:                       # corrupt region: resynchronise at the next start word
                nxt = buf.find(magic, o + 1)
                corrupt.append([o, (nxt if nxt >= 0 else len(buf)) - o])
                if nxt < 0:
                    o = len(buf); break
                o = nxt; continue
            v = TAG.unpack_from(buf, o)
            R = np.array(v[9:18], float).reshape(3, 3); why = check_tag(v, t_prev)
            rec = {"t": v[3] / 1e9, "dets": [{"id": v[1], "t_cam": list(v[6:9]), "R": R.tolist(), "size_m": v[2],
                                              "cam": v[30].split(b"\0")[0].decode(errors="ignore"), "margin": None, "hamming": None,
                                              "quality": "unavailable"}]}
            if why:
                bad += 1; rec["offset"] = o; rec["bad"] = why
                for w_ in why:
                    k = w_.split(" (")[0]; reasons[k] = reasons.get(k, 0) + 1
                badf.write(json.dumps(rec, allow_nan=True) + "\n")
            else:
                f.write(json.dumps(rec) + "\n"); n += 1; t_prev = v[3]
            o += TAG.size
    badf.close()
    partial = len(buf) - o if o < len(buf) else 0
    status = "failed" if n == 0 else "ok" if not (bad or corrupt or partial) else "partial"
    print("tag detections: %d valid records, %d failed (%s), corrupt regions %d, trailing partial bytes %d (record %d bytes) -> %s"
          % (n, bad, reasons or "-", len(corrupt), partial, TAG.size, out))
    summary(out, {"kind": "tag", "status": status, "valid": n, "failed": bad, "reasons": reasons, "corrupt": corrupt,
                  "partial_bytes": partial, "record_bytes": TAG.size, "magic": magic.hex(), "magic_source": src})
    if bad:
        print("  -> failed records in %s; if most fail, the layout of tag_detection_t differs: fix TAG (first bytes %s)"
              % (os.path.splitext(out)[0] + ".bad.jsonl", buf[:96].hex()))
    return 3 if n == 0 else 0 if status == "ok" else 2


if __name__ == "__main__":
    args = [x for x in sys.argv[1:] if not x.startswith("--magic=")]
    mg = [bytes.fromhex(x.split("=", 1)[1]) for x in sys.argv[1:] if x.startswith("--magic=")]
    p = args[0]; buf = open(p, "rb").read(); base = os.path.splitext(p)[0]
    if "tag" in os.path.basename(p):
        sys.exit(tags(buf, base + ".jsonl", mg[0] if mg else None))
    else:
        sys.exit(point_clouds(buf, base + ".npz", mg[0] if mg else None))
