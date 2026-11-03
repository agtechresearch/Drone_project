#!/usr/bin/env python3
"""Expected outcome of every run of a batch, written BEFORE the batch, compared AFTER it (2026-11-01, codex runner-exit-status
reply-4 / follow-up review: tell the intended safety landing of a fault-injection run apart from an unexpected failure, and
make the top-level script fail when they differ).
  plan  <batch> <spec.json>   -> logs/<batch>_plan.json: the runs (lane, seq, tag, injection, expected exit codes, log lines
                                 that must appear) + sha256 of the code / config the batch runs with
  check <batch>               -> table + logs/<batch>_check.json; exit = number of mismatches (missing run, other exit code,
                                 missing log line, run_watch problem, code changed during the batch)
spec.json: {"runs": [{"lane": 2, "seq": 1, "tag": "b<batch>_r3_th1", "inject": "none", "expect_exit": [0],
                      "expect_log": ["Disarmed."], "forbid_log": ["Kill command sent"],
                      "expect_truth": ["PASS"], "expect_archive": ["ok"], "expect_provenance": ["ok"]}, ...],   (last three: defaults)
           "children": ["batch", "watch", "inject_i3"]}   (the logs/<batch>_children.txt rows that MUST be there; default batch, watch)
A run dir <time>_i<lane>_<tag> belongs to a planned run only when lane AND the whole tag are equal (2026-11-01 codex
batch-watch-run-lifecycle reply-4: "..._th1" was a substring of "..._th10")."""
import glob, hashlib, json, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__)); V3 = os.path.dirname(HERE); SIM = os.path.dirname(V3)
LOGS = os.environ.get("BATCH_EXPECT_LOGS") or os.path.join(V3, "logs")
CODE = (sorted(glob.glob(os.path.join(V3, "scripts", "*.py")) + glob.glob(os.path.join(V3, "scripts", "*.sh"))
               + glob.glob(os.path.join(V3, "config", "*.json"))) + [os.path.join(V3, "gz", "tags_map.json"),
        os.path.join(SIM, "repo_export", "scripts", "px4_model_swap.sh"), os.path.join(SIM, "path_flight_phase1_v13.py")])


STATUS_KEYS = ("exit", "truth_verdict", "archive", "provenance")


def run_key(path):
    """(lane, tag) of a run dir "<YYYYmmdd_HHMMSS>_i<lane>_<tag>" (first "_i<digits>_"), or None"""
    m = re.match(r"^.*?_i(\d+)_(.+)$", os.path.basename(os.path.normpath(path)))
    return (m.group(1), m.group(2)) if m else None


def hashes():
    out = {}
    for p in CODE:
        try:
            out[os.path.relpath(p, SIM)] = hashlib.sha256(open(p, "rb").read()).hexdigest()
        except OSError:
            out[os.path.relpath(p, SIM)] = None
    return out


def plan(b, spec):
    d = json.load(open(spec)); d["batch"] = b; d["code_sha256"] = hashes()
    for r in d["runs"]:
        r.setdefault("expect_log", []); r.setdefault("forbid_log", [])
    json.dump(d, open(os.path.join(LOGS, "%s_plan.json" % b), "w"), indent=1)
    print("plan %s: %d runs, %d code files hashed" % (b, len(d["runs"]), len(d["code_sha256"])))


def check(b):
    """2026-11-01 codex runner-exit-status reply-6: besides the exit code, the truth verdict, the archive and the provenance are
    compared on their own (an expected exit 13 must not hide a clearance FAIL); every planned run must match exactly one row
    (lane + tag), a second row or a row of no planned run is a mismatch; the watcher must point at the same runs; the
    batch / watcher / injector exit codes (logs/<batch>_children.txt "name rc") are compared too."""
    P = json.load(open(os.path.join(LOGS, "%s_plan.json" % b)))
    rows = []
    try:
        for line in open(os.path.join(LOGS, "tofh_%s_status.txt" % b)):
            p = line.split()
            if len(p) >= 2:
                rows.append((p[0], p[1]))
    except OSError:
        pass
    res = []; bad = 0; used = set()
    for r in P["runs"]:
        hit = [x for x in rows if run_key(x[0]) == (str(r["lane"]), r["tag"]) and "eval_failed" not in x[1]]
        why = []; got = None; d = None; st = {}
        if not hit:
            why.append("run missing from tofh_%s_status.txt" % b)
        else:
            if len(hit) > 1:
                why.append("%d rows for this planned run (duplicate)" % len(hit))
            d, got = hit[0]; used.update(x[0] for x in hit)
            try:
                got = int(got)
            except ValueError:
                pass
            if got not in r["expect_exit"]:
                why.append("exit %s, expected %s" % (got, r["expect_exit"]))
            try:
                st = json.load(open(os.path.join(d, "status.json")))
                if not isinstance(st, dict):
                    why.append("status.json is not an object"); st = {}
            except (OSError, ValueError) as e:
                why.append("status.json unreadable: %s" % e)
            # every key must be there: an empty / partial status.json is a mismatch, not a skipped check (codex reply-8)
            miss = [k for k in STATUS_KEYS if k not in st]
            if miss and not any(w.startswith("status.json") for w in why):
                why.append("status.json lacks %s" % ", ".join(miss))
            if st.get("exit") != got:
                why.append("status.json exit %s != lane exit %s" % (st.get("exit"), got))
            if st.get("result_file") in ("not_written", "missing", "run_id_mismatch") and st.get("result_file") not in r.get("expect_result_file", []):
                why.append("mission_result.json %s (status result_file)" % st.get("result_file"))
            for key, exp in (("truth_verdict", r.get("expect_truth", ["PASS"])), ("archive", r.get("expect_archive", ["ok"])),
                             ("provenance", r.get("expect_provenance", ["ok"]))):
                if st.get(key) not in exp:
                    why.append("%s %s, expected %s" % (key, st.get(key), exp))
            try:
                log = open(os.path.join(d, "mission.log"), errors="replace").read()
            except OSError:
                log = ""; why.append("no mission.log")
            for s_ in r["expect_log"]:
                if not re.search(s_, log):
                    why.append("log line missing: %s" % s_)
            for s_ in r["forbid_log"]:
                if re.search(s_, log):
                    why.append("log line that must not appear: %s" % s_)
            ev = [x[1] for x in rows if x[0] == d and "eval_failed" in x[1]]
            if ev:
                why.append("evaluator failed: %s" % ev)
        bad += bool(why)
        res.append({"lane": r["lane"], "seq": r["seq"], "tag": r["tag"], "inject": r.get("inject"), "expect_exit": r["expect_exit"],
                    "run": d, "exit": got, "truth": st.get("truth_verdict"), "archive": st.get("archive"), "ok": not why, "why": why})
        print("%-4s i%s#%s %-24s inject %-38s exit %-4s expected %-8s %s" % ("OK" if not why else "BAD", r["lane"], r["seq"], r["tag"],
              r.get("inject", "")[:38], got, r["expect_exit"], "; ".join(why)))
    extra = sorted(set(x[0] for x in rows if "eval_failed" not in x[1]) - used)
    for x in extra:
        bad += 1; print("BAD  row of no planned run: %s" % x)
    now = hashes(); changed = [k for k in P["code_sha256"] if P["code_sha256"][k] != now.get(k)]
    if changed:
        bad += 1; print("BAD  code changed during the batch: %s" % ", ".join(changed))
    w = os.path.join(LOGS, "%s_watch.json" % b)
    try:
        W = json.load(open(w)); wbad = [x for x in W["runs"] if x["failed_at_s"]] + W.get("missing", [])
        if W["end"] != "finished" or wbad or W.get("unexpected_runs"):
            bad += 1; print("BAD  run_watch: end %s, runs with a missed / stalled step or missing %d, unexpected %d"
                            % (W["end"], len(wbad), len(W.get("unexpected_runs", []))))
        wruns = {(str(x["lane"]), x["seq"]): x["run"] for x in W.get("expected_runs", [])}
        for x in res:
            wr = wruns.get((str(x["lane"]), x["seq"]))
            if x["run"] and (wr is None or os.path.realpath(wr) != os.path.realpath(x["run"])):
                bad += 1; print("BAD  run_watch i%s#%s watched %s, the status row is %s" % (x["lane"], x["seq"], wr, x["run"]))
    except (OSError, ValueError, KeyError) as e:
        bad += 1; print("BAD  run_watch result unreadable: %r" % e)
    # child exit codes: the file, every required child exactly once, numbers only (codex runner-exit-status reply-8: a missing
    # file or row passed before)
    ch = {}
    try:
        for i, line in enumerate(open(os.path.join(LOGS, "%s_children.txt" % b))):
            p = line.split()
            if not p:
                continue
            try:
                k, v = p[0], int(p[1])
                if len(p) != 2:
                    raise ValueError
            except (ValueError, IndexError):
                bad += 1; print("BAD  children line %d unreadable: %r" % (i + 1, line.rstrip())); continue
            if k in ch:
                bad += 1; print("BAD  child %s listed twice" % k)
            ch[k] = v
    except OSError as e:
        bad += 1; print("BAD  %s_children.txt unreadable: %s" % (b, e))
    for k in P.get("children", ["batch", "watch"]):
        if k not in ch:
            bad += 1; print("BAD  child %s has no exit code in %s_children.txt" % (k, b))
    want_batch = sum(1 for r in P["runs"] if 0 not in r["expect_exit"])
    for k, v in sorted(ch.items()):
        exp = want_batch if k == "batch" else 0
        if v != exp:
            bad += 1; print("BAD  %s exited %s, expected %s" % (k, v, exp))
    json.dump({"batch": b, "mismatches": bad, "runs": res, "unplanned_rows": extra, "code_changed": changed, "children": ch},
              open(os.path.join(LOGS, "%s_check.json" % b), "w"), indent=1)
    print("%s: %d mismatch(es)" % (b, bad))
    return bad


if __name__ == "__main__":
    if sys.argv[1] == "plan":
        plan(sys.argv[2], sys.argv[3])
    else:
        sys.exit(min(check(sys.argv[2]), 100))
