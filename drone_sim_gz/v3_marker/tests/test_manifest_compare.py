#!/usr/bin/env python3
"""run_manifest.py compare / finish and the wk_fly.sh order (2026-11-01, codex experiment-provenance reply-10).
  compare: per run VERIFIED only with complete True and every required field; a field missing on BOTH sides and complete False on
           both are reported (the code before this change printed nothing for them - shown with the backup as the negative control)
  finish:  an evaluator without a start hash makes the manifest incomplete ("has no start hash")
  wk_fly:  run_manifest finish comes after loc_compare and before run_status (static order of the lines)
Base manifest: the real run 20261101_143400_i2_b1929p14d_r3_th1 (complete True). exit code = number of failures."""
import copy, hashlib, json, os, re, shutil, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__)); V3 = os.path.dirname(HERE); SC = os.path.join(V3, "scripts")
OLD = os.path.join(V3, "logs", "bak_review6_20261101", "run_manifest.py")          # the code before this change
REAL = os.path.realpath(os.path.join(V3, "logs", "20261101_143400_i2_b1929p14d_r3_th1"))
BASE = json.load(open(os.path.join(REAL, "run_manifest.json")))
F = 0; tmp = tempfile.mkdtemp()


def chk(ok, txt):
    global F
    print("%s  %s" % ("PASS" if ok else "FAIL", txt)); F += (not ok)


def mk(name, edit=None, none=False):
    d = os.path.join(tmp, name); os.makedirs(d)
    if not none:
        m = copy.deepcopy(BASE)
        if edit:
            edit(m)
        json.dump(m, open(os.path.join(d, "run_manifest.json"), "w"))
    return d


def cmp(script, *runs):
    p = subprocess.run([sys.executable, script, "compare"] + list(runs), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       universal_newlines=True)
    return p.returncode, p.stdout


NEW = os.path.join(SC, "run_manifest.py")
a, b = mk("a"), mk("b")
rc, o = cmp(NEW, a, b)
chk(rc == 0 and "run a: VERIFIED" in o and "run b: VERIFIED" in o and "UNVERIFIED" not in o, "two complete copies of the real manifest: both VERIFIED, rc %d" % rc)
x, y = mk("nop1", lambda m: m.pop("plan_sha256")), mk("nop2", lambda m: m.pop("plan_sha256"))
rc, o = cmp(NEW, x, y)
chk(rc == 3 and o.count("UNVERIFIED - missing 1: plan_sha256") == 2, "plan_sha256 missing on BOTH: both UNVERIFIED, rc %d" % rc)
rc0, o0 = cmp(OLD, x, y)
chk("plan_sha256" not in o0, "negative control, code before the change: nothing printed for it (rc %d, %d lines)" % (rc0, len(o0.splitlines())))
x, y = mk("inc1", lambda m: m.update(complete=False)), mk("inc2", lambda m: m.update(complete=False))
rc, o = cmp(NEW, x, y)
chk(rc == 3 and o.count("UNVERIFIED - complete False") == 2, "complete False on both: both UNVERIFIED, rc %d" % rc)
rc0, o0 = cmp(OLD, x, y)
chk("complete" not in o0, "negative control, code before: complete False on both not printed")
x = mk("nullpx4", lambda m: m["px4_params"].update(EKF2_EV_CTRL=None))
rc, o = cmp(NEW, a, x)
chk(rc == 3 and "px4_params.EKF2_EV_CTRL" in o and "run nullpx4: UNVERIFIED" in o and "run a: VERIFIED" in o,
    "a px4 param not read on one run: that run UNVERIFIED, the other VERIFIED")
x = mk("nocli", lambda m: m["cli"].pop("route"))
rc, o = cmp(NEW, x); chk(rc == 3 and "cli.route" in o, "cli.route missing: UNVERIFIED")
x = mk("noeval", lambda m: m["evaluator_sha256"].pop("scripts/loc_compare.py"))
rc, o = cmp(NEW, x); chk(rc == 3 and "evaluator_sha256.scripts/loc_compare.py" in o, "an evaluator hash missing: UNVERIFIED")
x = mk("nullcode", lambda m: m["code_sha256"].update({"scripts/wk_mission.py": None}))
rc, o = cmp(NEW, x); chk(rc == 3 and "code_sha256.scripts/wk_mission.py" in o, "a code hash recorded as null: UNVERIFIED")
x = mk("none", none=True)
rc, o = cmp(NEW, a, x); chk(rc == 3 and "run none: UNVERIFIED - no manifest" in o, "a run without a manifest: UNVERIFIED")
x = mk("sv_notruth", lambda m: m.update(world="qvix_sv5", survey_truth=None))
rc, o = cmp(NEW, x); chk(rc == 3 and "survey_truth.sha256" in o, "survey world without the survey truth hash: UNVERIFIED")
chk("scripts/run_geometry.py" not in BASE["code_sha256"], "the base manifest predates run_geometry.py / tag_quality.py in CODE and still verifies (above)")

# finish(): evaluator without a start hash
sys.path.insert(0, SC)
import run_manifest as RM  # noqa: E402


def fin(name, edit):
    d = os.path.join(tmp, name); os.makedirs(d)
    shutil.copy(os.path.join(REAL, "plan.json"), d)
    m = {"complete": True, "collection": [], "code_sha256": {c: hashlib.sha256(open(os.path.join(V3, c), "rb").read()).hexdigest() for c in RM.EVAL}}
    edit(m); json.dump(m, open(os.path.join(d, "run_manifest.json"), "w"))
    del RM.PROBLEMS[:]
    ok = RM.finish(d)
    return ok, json.load(open(os.path.join(d, "run_manifest.json")))


ok, m = fin("fin_ok", lambda m: None)
chk(ok and m["complete"], "finish, every evaluator hashed at the start and unchanged: complete")
ok, m = fin("fin_nostart", lambda m: m["code_sha256"].pop("scripts/run_geometry.py"))
chk(not ok and "scripts/run_geometry.py has no start hash" in m["collection"], "finish, an evaluator without a start hash: incomplete (%s)" % m["collection"])
ok, m = fin("fin_changed", lambda m: m["code_sha256"].update({"scripts/loc_compare.py": "0" * 64}))
chk(not ok and any("loc_compare.py changed" in c for c in m["collection"]), "finish, an evaluator changed since the start: incomplete")

# wk_fly.sh order
L = open(os.path.join(SC, "wk_fly.sh")).read().splitlines()
pos = lambda pat: [i + 1 for i, l in enumerate(L) if re.search(pat, l) and not l.lstrip().startswith("#")]
lc, fi, rs = pos(r'python3 "\$SC/loc_compare.py"'), pos(r'run_manifest.py" finish'), pos(r'^"\$SC/run_status.sh"')
chk(len(fi) == 1 and lc and rs and lc[-1] < fi[0] < rs[0], "wk_fly.sh: loc_compare line %s < manifest finish %s < run_status %s" % (lc, fi, rs))
shutil.rmtree(tmp)
print("%d failure(s)" % F)
sys.exit(F)
