#!/usr/bin/env python3
"""Real-vehicle tag / point-cloud decoder self-checks (2026-11-01, codex real-tag-validation / real-tag-quality-contract).
Synthetic recordings in the ASSUMED layout (real/mpa_parse.py TAG / PC_HDR) - this checks that the decoder separates good
records from bad ones; it does NOT check the layout against a real VOXL recording (summary layout_verified false).
Cases: clean, det=1 shear R, NaN R, start word destroyed in the middle, trailing partial record, timestamp going back,
point clouds clean / short last frame. Output: each run's exit, summary, .jsonl / .bad.jsonl line counts.
Run on the host:  python3 tests/test_mpa_parse.py      exit = failures"""
import json, math, os, sys, tempfile
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..", "real"))
import mpa_parse as MP  # noqa: E402
F = []; MAGIC = 0x5441474D


def check(name, cond, info=""):
    print("%s  %s%s" % ("PASS" if cond else "FAIL", name, ("  (" + info + ")") if info else ""))
    if not cond:
        F.append(name)


def rec(i, R=None, ts=None):
    a = math.radians(10 * i); c, s = math.cos(a), math.sin(a)
    R = R if R is not None else [c, 0, s, 0, 1, 0, -s, 0, c]
    return MP.TAG.pack(MAGIC, 30 + i, 0.05, ts if ts is not None else 1000000000 + i * 100000000, b"tag", 0,
                       0.01 * i, 0.02, 0.5, *R, 0, 0, 0, *[1, 0, 0, 0, 1, 0, 0, 0, 1], b"tracking_front", 0)


def run(name, buf):
    d = tempfile.mkdtemp(); out = os.path.join(d, name + ".jsonl")
    rc = MP.tags(buf, out)
    sm = json.load(open(os.path.join(d, name + ".summary.json")))
    good = [json.loads(x) for x in open(out)]; bad = [json.loads(x) for x in open(os.path.join(d, name + ".bad.jsonl"))]
    print("      %-10s rc %d, status %s, valid %d, failed %d %s, corrupt %s, partial %d, layout_verified %s" % (
        name, rc, sm["status"], sm["valid"], sm["failed"], sm["reasons"], sm["corrupt"], sm["partial_bytes"], sm["layout_verified"]))
    return rc, sm, good, bad


recs = [rec(i) for i in range(6)]
rc, sm, good, bad = run("clean", b"".join(recs))
check("clean: exit 0, 6 valid, status ok, not marked layout-verified", rc == 0 and sm["valid"] == 6 and sm["status"] == "ok"
      and sm["layout_verified"] is False)
check("clean: margin / hamming null, quality unavailable (no made-up margin)",
      all(g["dets"][0]["margin"] is None and g["dets"][0]["hamming"] is None and g["dets"][0]["quality"] == "unavailable" for g in good))
shear = [1, 0.3, 0, 0, 1, 0, 0, 0, 1]                      # det = 1, not orthonormal
rc, sm, good, bad = run("shear", b"".join(recs[:2] + [rec(2, shear)] + recs[3:]))
check("det=1 shear R: failed, kept out of the flight .jsonl, exit 2", rc == 2 and sm["failed"] == 1 and len(good) == 5
      and any("orthonormal" in w for w in bad[0]["bad"]), str(bad[0]["bad"] if bad else None))
nanR = [float("nan")] + [0.0] * 8
rc, sm, good, bad = run("nanR", b"".join(recs[:3] + [rec(3, nanR)] + recs[4:]))
check("NaN R: failed ('R not finite'), exit 2", rc == 2 and sm["failed"] == 1 and "R not finite" in bad[0]["bad"], str(bad[0]["bad"] if bad else None))
mid = bytearray(b"".join(recs)); o = 3 * MP.TAG.size; mid[o:o + 4] = b"\0\0\0\0"
rc, sm, good, bad = run("corrupt", bytes(mid))
check("start word destroyed in record 4: one corrupt region at its offset, resync, 5 valid, exit 2",
      rc == 2 and sm["corrupt"] == [[o, MP.TAG.size]] and sm["valid"] == 5 and sm["status"] == "partial", str(sm["corrupt"]))
rc, sm, good, bad = run("partial", b"".join(recs) + recs[0][:40])
check("trailing 40 bytes: partial_bytes 40, the 6 whole records valid, exit 2", rc == 2 and sm["partial_bytes"] == 40 and sm["valid"] == 6)
rc, sm, good, bad = run("tsback", b"".join(recs[:4] + [rec(4, ts=1000)] + recs[5:]))
check("timestamp going back: failed record, exit 2", rc == 2 and sm["failed"] == 1 and "timestamp goes back" in bad[0]["bad"])
rc, sm, good, bad = run("empty", b"\x01\x02\x03")
check("nothing decodable: exit 3, status failed", rc == 3 and sm["status"] == "failed")


def pc(n, ts):
    return MP.PC_HDR.pack(MAGIC, ts, n, 0, 1, b"tof", 0) + np.random.default_rng(1).random((n, 3)).astype(np.float32).tobytes()


d = tempfile.mkdtemp()
rc = MP.point_clouds(b"".join(pc(50, 10 ** 9 + k * 10 ** 8) for k in range(4)), os.path.join(d, "pc.npz"))
sm = json.load(open(os.path.join(d, "pc.summary.json")))
check("point clouds clean: exit 0, 4 frames", rc == 0 and sm["frames"] == 4 and sm["status"] == "ok", str(sm))
full = b"".join(pc(50, 10 ** 9 + k * 10 ** 8) for k in range(4))
rc = MP.point_clouds(full + pc(50, 2 * 10 ** 9)[:300], os.path.join(d, "pc2.npz"))
sm = json.load(open(os.path.join(d, "pc2.summary.json")))
check("point clouds short last frame: exit 2, partial 300 bytes reported", rc == 2 and sm["partial_bytes"] == 300 and sm["frames"] == 4, str(sm))
print("%d failure(s)" % len(F)); sys.exit(len(F))
