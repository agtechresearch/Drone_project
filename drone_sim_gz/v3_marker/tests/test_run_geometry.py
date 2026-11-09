#!/usr/bin/env python3
"""run_geometry.check() and its use in loc_compare.py / wk_report.py (2026-11-01, codex report-truth-geometry reply-8).
Synthetic run dirs for every branch, then the real run 20261101_143400_i2_b1929p14d_r3_th1 (qvix_rot, manifest, clearance PASS
0.1665 m) end to end: the right yaw reproduces the stored clearance line; a wrong yaw / a changed qvix_grid hash / no manifest
give "clearance INCOMPLETE: geometry not verified"; wk_report refuses the same cases. exit code = number of failures."""
import hashlib, json, os, shutil, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__)); V3 = os.path.dirname(HERE); SC = os.path.join(V3, "scripts")
sys.path.insert(0, SC)
import run_geometry as RG  # noqa: E402

F = 0


def chk(ok, txt):
    global F
    print("%s  %s" % ("PASS" if ok else "FAIL", txt)); F += (not ok)


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


GRID = sha(os.path.join(SC, "qvix_grid.py"))
T = {k: os.path.join(V3, "gz", "survey", "%s_truth.json" % k) for k in ("sv1", "sv3", "sv5")}
tmp = tempfile.mkdtemp()


def run(name, world, yaw="7", manifest=True, grid=GRID, truth_sha="same", cond=True):
    d = os.path.join(tmp, name); os.makedirs(d)
    if cond:
        open(os.path.join(d, "condition.txt"), "w").write("t on rot=0 drift=0\nworld=%s yaw=%s tags=0\n" % (world, yaw))
    if manifest:
        m = {"world": world, "env": {"QVIX_YAW_DEG": yaw}, "code_sha256": {"scripts/qvix_grid.py": grid}}
        if world.startswith("qvix_sv"):
            tp = os.path.join(V3, "gz", "survey", world[5:] + "_truth.json")
            m["survey_truth"] = {"path": tp, "sha256": (sha(tp) if os.path.exists(tp) else None) if truth_sha == "same" else truth_sha}
        json.dump(m, open(os.path.join(d, "run_manifest.json"), "w"))
    return d


def g(d, yaw="7", lt=None, accept=False):
    os.environ["QVIX_YAW_DEG"] = yaw
    os.environ.pop("LOC_GEOMETRY_UNVERIFIED", None)
    if accept:
        os.environ["LOC_GEOMETRY_UNVERIFIED"] = "accept"
    r = RG.check(d, lt)
    return r


r = g(run("design_ok", "qvix_rot")); chk(r["status"] == "ok" and r["evaluate"] and r["truth"] == {}, "design world, same yaw + qvix_grid: ok (%s)" % RG.line(r))
r = g(run("design_yaw", "qvix_rot"), yaw="0"); chk(r["status"] == "mismatch" and not r["evaluate"], "design world, QVIX_YAW_DEG 0 vs run 7: mismatch (%s)" % r["reasons"])
r = g(run("design_grid", "qvix_rot", grid="0" * 64)); chk(r["status"] == "mismatch" and not r["evaluate"], "qvix_grid.py hash differs: mismatch (%s)" % r["reasons"])
r = g(run("design_nomf", "qvix_rot", manifest=False)); chk(r["status"] == "unverified" and not r["evaluate"], "no manifest: unverified, not evaluated (%s)" % r["reasons"])
r = g(os.path.join(tmp, "design_nomf"), accept=True); chk(r["status"] == "unverified" and r["evaluate"] and r["accepted_unverified"], "no manifest + LOC_GEOMETRY_UNVERIFIED=accept: evaluated, labelled")
r = g(run("nocond", "qvix_rot", manifest=False, cond=False)); chk(r["status"] == "unverified" and "world unknown" in r["reasons"], "no manifest, no condition: world unknown, unverified")
r = g(run("design_lt5", "qvix_rot"), lt=T["sv5"]); chk(r["status"] == "mismatch", "design world with a moved-rack LOC_TRUTH (sv5): mismatch (%s)" % r["reasons"])
r = g(run("sv5_ok", "qvix_sv5")); chk(r["status"] == "ok" and r["truth"].get("rack_shift"), "survey sv5 without LOC_TRUTH: loads its own truth (rack_shift), ok")
r = g(run("sv5_lt1", "qvix_sv5"), lt=T["sv1"]); chk(r["status"] == "mismatch", "survey sv5 with LOC_TRUTH = sv1's: mismatch (%s)" % r["reasons"])
r = g(run("sv5_hash", "qvix_sv5", truth_sha="f" * 64)); chk(r["status"] == "mismatch" and not r["evaluate"], "survey truth changed since the run: mismatch (%s)" % r["reasons"])
r = g(run("sv5_nohash", "qvix_sv5", truth_sha=None)); chk(r["status"] == "unverified", "survey truth hash not in the manifest: unverified")
r = g(run("sv9_missing", "qvix_sv9")); chk(r["status"] == "missing" and not r["evaluate"], "survey world without its truth file (sv9): missing (%s)" % r["reasons"])
r = g(os.path.join(tmp, "sv9_missing"), accept=True); chk(not r["evaluate"], "missing is never accepted by LOC_GEOMETRY_UNVERIFIED=accept")

# --- end to end on a real run ---------------------------------------------------------------------------------------------
REAL = os.path.realpath(os.path.join(V3, "logs", "20261101_143400_i2_b1929p14d_r3_th1"))
stored = [l.strip() for l in open(os.path.join(REAL, "loc_compare.txt")) if l.strip().startswith("clearance")][0]


def copy_run(name, manifest_edit=None, drop_manifest=False):
    d = os.path.join(tmp, name, os.path.basename(REAL)); os.makedirs(d)   # the real name: wk_report reads the date from it
    for f in os.listdir(REAL):
        if f == "run_manifest.json" and (manifest_edit or drop_manifest):
            continue
        os.symlink(os.path.join(REAL, f), os.path.join(d, f))
    if manifest_edit:
        m = json.load(open(os.path.join(REAL, "run_manifest.json"))); manifest_edit(m)
        json.dump(m, open(os.path.join(d, "run_manifest.json"), "w"))
    return d


def lc(d, yaw="7", extra=None):
    env = dict(os.environ, QVIX_YAW_DEG=yaw); env.pop("LOC_TRUTH", None); env.pop("LOC_GEOMETRY_UNVERIFIED", None); env.update(extra or {})
    p = subprocess.run([sys.executable, os.path.join(SC, "loc_compare.py"), d], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       universal_newlines=True, env=env)
    cl = [l.strip() for l in p.stdout.splitlines() if l.strip().startswith("clearance")]
    ge = [l.strip() for l in p.stdout.splitlines() if l.strip().startswith("geometry")]
    return p.returncode, (cl[0] if cl else ""), (ge[0] if ge else "")


rc, cl, ge = lc(copy_run("real_ok"))
chk(rc == 0 and cl == stored and ge.startswith("geometry ok"), "real run, yaw 7: geometry ok, clearance line identical to the stored one\n      %s\n      %s" % (ge[:150], cl[:110]))
rc, cl, ge = lc(os.path.join(tmp, "real_ok", os.path.basename(REAL)), yaw="0")
chk(rc == 0 and cl.startswith("clearance INCOMPLETE: geometry not verified (mismatch)"), "real run, yaw 0: %s" % cl[:150])
rc, cl, ge = lc(copy_run("real_grid", manifest_edit=lambda m: m["code_sha256"].__setitem__("scripts/qvix_grid.py", "0" * 64)))
chk(cl.startswith("clearance INCOMPLETE: geometry not verified (mismatch)"), "real run, manifest qvix_grid hash changed: %s" % cl[:150])
rc, cl, ge = lc(copy_run("real_nomf", drop_manifest=True))
chk(cl.startswith("clearance INCOMPLETE: geometry not verified (unverified)"), "real run, no manifest: %s" % cl[:150])
rc, cl, ge = lc(os.path.join(tmp, "real_nomf", os.path.basename(REAL)), extra={"LOC_GEOMETRY_UNVERIFIED": "accept"})
chk(cl.startswith("clearance PASS: min 0.1665 m") and cl.endswith("2-D only; geometry UNVERIFIED (accepted)"), "real run, no manifest + accept: %s" % cl[-60:])
rc, cl, ge = lc(os.path.join(tmp, "real_ok", os.path.basename(REAL)), extra={"LOC_TRUTH": T["sv5"]})
chk(cl.startswith("clearance INCOMPLETE: geometry not verified (mismatch)"), "real run (qvix_rot) with LOC_TRUTH sv5: %s" % cl[:150])

VENV = "/home/dronesim/starling_sim/cad/venv/bin/python"   # wk_report needs matplotlib (the host python3 numpy/matplotlib do not import)
# wk_report: refuses the same runs (it exits before plotting); the ok run gets past the geometry check
def wr(d, yaw="7"):
    env = dict(os.environ, QVIX_YAW_DEG=yaw, MPLBACKEND="Agg"); env.pop("LOC_GEOMETRY_UNVERIFIED", None)
    sys.path.insert(0, SC)
    code = ("import sys; sys.path.insert(0, %r); import wk_report as W\ntry:\n W.load_run(%r)\n print('LOADED')\nexcept SystemExit as e:\n print('EXIT', e)\n"
            % (SC, d))
    p = subprocess.run([VENV, "-c", code], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, env=env)
    return p.stdout


o = wr(os.path.join(tmp, "real_ok", os.path.basename(REAL))); chk("LOADED" in o and "geometry ok" in o, "wk_report.load_run real run yaw 7: loaded")
o = wr(os.path.join(tmp, "real_ok", os.path.basename(REAL)), yaw="0"); chk("not evaluated - geometry mismatch" in o, "wk_report yaw 0: refused (%s)" % o.strip().splitlines()[-1][:120])
o = wr(os.path.join(tmp, "real_nomf", os.path.basename(REAL))); chk("not evaluated - geometry unverified" in o, "wk_report no manifest: refused")
shutil.rmtree(tmp)
print("%d failure(s)" % F)
sys.exit(F)
