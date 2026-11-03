#!/usr/bin/env python3
"""Watch a running wk_tofh_batch.sh batch and report, line by line, every checklist step of every RUN that passes, misses its
deadline, or stalls (2026-10-31: phase 9 lane i5 hung 80 min in land_and_disarm and nobody noticed). Run on the HOST next to
the batch; each stdout line is one event (meant for a Monitor).
  python3 run_watch.py <batch_tag> [lanes "2 3 4 5"] [stagger_s 45] [runs_per_lane 1]
  The expected runs come from logs/<batch>_plan.json when it exists (batch_expect.py writes it), else lanes x runs_per_lane.
2026-11-01 (codex batch-watch-run-lifecycle / batch-watch-observer-failure):
  - the unit is the RUN, not the lane: every run directory a lane names gets its own checklist and deadlines, runs that are
    replaced before their status is read are still finished from their own files, and the watcher ends only when every
    EXPECTED run is finished or failed (a missing run is a failure, never a normal end)
  - observer failures are recorded, not fatal: status.json read errors are retried (READ_RETRIES cycles), `docker ps` has a
    time limit and its errors are told apart from a stopped container, container down / up events stay in the result, the
    state is written to logs/<batch>_watch.json every cycle, and an exception in the watcher itself ends with
    "end": "observer_error" and exit code 3 (never a normal completion)
Checklist per run (wall seconds):
  run_dir    the lane's run k appears                 lane run 1: <= stagger * lane_index + RUN_DIR_S after start;
                                                       run k > 1: <= NEXT_RUN_S after run k-1 finished
  sitl_up    "New NED origin" in <run>/sitl.log        <= SITL_S after the run dir appeared
  mission    "asyncio clock" in <run>/mission.log      <= MISSION_S after sitl_up
  progress   mission.log written within STALL_S        (reported once per stall, and when it resumes)
  finished   <run>/status.json readable                <= FINISH_S after the run dir appeared; the lane recording the run in
                                                       tofh_<batch>_status.txt without a status.json = failed at once
Exit code: number of runs with a missed / stalled step or missing, 3 more if the watcher itself failed."""
import glob, json, os, subprocess, sys, time, traceback

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from batch_expect import run_key  # noqa: E402  (one definition of "which planned run is this dir")
LOGS = os.environ.get("RUN_WATCH_LOGS") or os.path.join(HERE, "..", "logs")
K = float(os.environ.get("RUN_WATCH_SCALE", "1"))          # tests/test_run_watch.sh shrinks every deadline
RUN_DIR_S, NEXT_RUN_S, SITL_S, MISSION_S, STALL_S, FINISH_S = (v * K for v in (120, 120, 240, 120, 120, 1800))
# landing step -> result line (2026-11-01, 1921p14 i4: the program end hung on a swallowed cancel while land retries kept the
# log growing, so the silence check never fired): after "[7/7] Landing" the safety layer's "[result] EXIT" line must follow
# (touchdown deadline 40 s sim + summary; sim time runs up to ~3.5 x slower than wall time under load)
LAND_END_S = 420 * K
POLL = float(os.environ.get("RUN_WATCH_POLL", "5")); READ_RETRIES, DOCKER_TIMEOUT = 6, 10 * K

B = sys.argv[1]; LANES = (sys.argv[2] if len(sys.argv) > 2 else "2 3 4 5").split()
STAG = (float(sys.argv[3]) if len(sys.argv) > 3 else 45.0) * K; PER = int(sys.argv[4]) if len(sys.argv) > 4 else 1
T0 = time.time(); OUT = os.path.join(LOGS, "%s_watch.json" % B)
EXPECT = {n: [{}] * PER for n in LANES}
LANE = {n: {"k": k, "runs": []} for k, n in enumerate(LANES)}


def load_plan():
    """expected runs per lane from logs/<batch>_plan.json (inside the guarded section: a broken plan is an observer error)"""
    p = os.path.join(LOGS, "%s_plan.json" % B)
    if os.path.exists(p):
        plan = json.load(open(p))["runs"]
        for n in LANES:
            EXPECT[n] = sorted([r for r in plan if str(r["lane"]) == n], key=lambda r: r["seq"])
            # input contract (2026-11-01 codex batch-watch-run-lifecycle reply-4): the seq of a lane are exactly 1..N (the
            # watcher's "run k" IS seq k) and no two planned runs of a lane carry the same tag; anything else -> observer error
            seqs = [r["seq"] for r in EXPECT[n]]; tags = [r.get("tag") for r in EXPECT[n] if r.get("tag")]
            if seqs != list(range(1, len(seqs) + 1)):
                raise ValueError("plan lane %s: seq %s is not 1..%d" % (n, seqs, len(seqs)))
            if len(set(tags)) != len(tags):
                raise ValueError("plan lane %s: a tag is planned twice %s" % (n, tags))
RUNS = {}                  # run dir -> checklist state
OBS = {"events": [], "read_errors": 0, "docker_errors": 0, "containers_down": []}
cont_ok = True


def say(who, msg):
    """never raises: a closed stdout (the Monitor gone) is counted, the event still goes to the JSON (2026-11-01 codex reply-2)"""
    line = "[watch %s +%4.0fs] %s%s" % (time.strftime("%H:%M:%S"), time.time() - T0, (who + " ") if who else "", msg)
    OBS["events"].append(line)
    try:
        print(line, flush=True)
    except Exception:  # noqa: BLE001
        OBS["stdout_failed"] = OBS.get("stdout_failed", 0) + 1
        try:                                   # later prints and the interpreter's final flush must not fail again (exit 120)
            sys.stdout = open(os.devnull, "w")
        except Exception:  # noqa: BLE001
            pass


def grep(path, s):
    try:
        with open(path, "rb") as f:
            return s.encode() in f.read()
    except OSError:
        return False


def last_line(path):
    try:
        with open(path, "rb") as f:
            f.seek(max(0, os.path.getsize(path) - 400)); return f.read().decode("utf-8", "replace").strip().splitlines()[-1][:140]
    except (OSError, IndexError):
        return ""


def step(r, name, ok, deadline, info=""):
    if name in r["done"] or name in r["fail"]:
        return name in r["done"]
    if ok:
        r["done"][name] = round(time.time() - T0, 1); say(r["who"], "OK   %-9s %s" % (name, info)); return True
    if deadline is not None and time.time() > deadline:
        r["fail"][name] = round(time.time() - T0, 1); say(r["who"], "MISS %-9s deadline passed %s" % (name, info))
    return False


def read_status(r):
    p = os.path.join(r["dir"], "status.json")
    if not os.path.exists(p):
        return None
    try:
        return json.load(open(p))
    except (OSError, ValueError) as e:                 # half written / moving / permissions: retry next cycle
        r["read_err"] += 1; OBS["read_errors"] += 1
        if r["read_err"] == READ_RETRIES:
            say(r["who"], "WARN status.json unreadable %d times: %r" % (r["read_err"], e))
        return None


def check_run(r):
    if r["end"]:
        return
    d = r["dir"]; sl, ml = os.path.join(d, "sitl.log"), os.path.join(d, "mission.log")
    st = read_status(r)
    if st is not None:
        step(r, "finished", True, None, "exit %s (mission %s, verify %s, truth %s)" % (st.get("exit"), st.get("mission"),
                                                                                     st.get("verify"), st.get("truth_verdict")))
        r["end"] = "finished"; r["exit"] = st.get("exit"); return
    if grep(os.path.join(LOGS, "tofh_%s_status.txt" % B), d + " "):
        if r["read_err"] >= READ_RETRIES or not os.path.exists(os.path.join(d, "status.json")):
            step(r, "finished", False, 0, "lane ended without a readable status.json: %s" % last_line(os.path.join(LOGS, ".lane_i%s.log" % r["lane"])))
            r["end"] = "failed"; return
    if not step(r, "sitl_up", grep(sl, "New NED origin"), r["t_run"] + SITL_S):
        if "sitl_up" in r["fail"]:
            say(r["who"], "     sitl.log: %s" % last_line(sl)); r["end"] = "failed"
        return
    if not step(r, "mission", grep(ml, "asyncio clock"), T0 + r["done"]["sitl_up"] + MISSION_S):
        if "mission" in r["fail"]:
            say(r["who"], "     mission.log: %s" % last_line(ml)); r["end"] = "failed"
        return
    if grep(ml, "[result] first mission event"):          # runs with the result-reporting safety layer only
        if step(r, "landing", grep(ml, "[7/7] Landing"), None):
            got = grep(ml, "[result] EXIT")          # the miss text only on a miss (1927p14c printed it on OK too)
            if not step(r, "result", got, T0 + r["done"]["landing"] + LAND_END_S,
                        "" if got else "(no '[result] EXIT' line %d s after '[7/7] Landing': landing / program end stuck?)" % LAND_END_S):
                if "result" in r["fail"] and not r.get("said_result"):
                    r["said_result"] = True; say(r["who"], "     mission.log: %s" % last_line(ml))
    age = time.time() - os.path.getmtime(ml) if os.path.exists(ml) else 0.0
    if age > STALL_S and not r["stalled"]:
        r["stalled"] = True; r["fail"].setdefault("progress", round(time.time() - T0, 1))
        say(r["who"], "STALL mission.log silent %.0f s, last: %s" % (age, last_line(ml)))
    elif age <= STALL_S and r["stalled"]:
        r["stalled"] = False; say(r["who"], "     mission.log moving again")
    if time.time() > r["t_run"] + FINISH_S:
        step(r, "finished", False, r["t_run"] + FINISH_S, "(no status.json %d s after the run dir)" % FINISH_S); r["end"] = "failed"


def snapshot(end):
    out = {"batch": B, "end": end, "elapsed_s": round(time.time() - T0, 1), "observer": {k: v for k, v in OBS.items() if k != "events"},
           "expected": {n: len(EXPECT[n]) for n in LANES},
           "expected_runs": [{"lane": n, "seq": k + 1, "tag": e.get("tag"),
                              "run": next((r["dir"] for r in LANE[n]["runs"] if r["seq"] == k + 1 and not r.get("unexpected")), None)}
                             for n in LANES for k, e in enumerate(EXPECT[n])],
           "unexpected_runs": [r["dir"] for r in RUNS.values() if r.get("unexpected")],
           "runs": [{"lane": r["lane"], "seq": r["seq"], "run": r["dir"], "end": r["end"], "exit": r.get("exit"),
                     "done_at_s": r["done"], "failed_at_s": r["fail"]} for r in RUNS.values()],
           "missing": [{"lane": n, "seq": k + 1} for n in LANES for k in range(len(EXPECT[n]))
                       if LANE[n].get("gave_up") and not any(r["seq"] == k + 1 and not r.get("unexpected") for r in LANE[n]["runs"])],
           "events_tail": OBS["events"][-50:]}
    tmp = OUT + ".tmp"
    with open(tmp, "w") as f:
        json.dump(out, f, indent=1)
    os.replace(tmp, OUT)
    return out


def cycle():
    global cont_ok
    try:
        p = subprocess.run(["docker", "ps", "--format", "{{.Names}}"], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=DOCKER_TIMEOUT)
        if p.returncode != 0:
            OBS["docker_errors"] += 1; say(None, "WARN docker ps rc %d: %s" % (p.returncode, p.stderr.strip()[:100]))
        else:
            bad = [c for c in ("starling-gz", "starling-flight") if c not in p.stdout.split()]
            if bad and cont_ok:
                say(None, "FAIL container not running: %s" % " ".join(bad)); OBS["containers_down"].append([round(time.time() - T0, 1), bad])
            elif not bad and not cont_ok:
                say(None, "     containers running again"); OBS["containers_down"].append([round(time.time() - T0, 1), "up"])
            cont_ok = not bad
    except subprocess.TimeoutExpired:
        OBS["docker_errors"] += 1; say(None, "WARN docker ps did not answer within %d s (query failure, not a stopped container)" % DOCKER_TIMEOUT)
    for n, ln in LANE.items():
        # every run dir of this lane and batch, not only the one .last_run_i<N> names now: a run that came and went between two
        # polls is still found (directory names start with the start time, so sorted = started order)
        try:
            cur = open(os.path.join(LOGS, ".last_run_i%s" % n)).read().strip()
        except OSError:
            cur = ""
        # one key per directory: normalised, symlinks NOT resolved (the archive step turns the dir into a link mid-run). The glob
        # gave ".../scripts/../logs/x" while .last_run_i<N> holds ".../logs/x": two strings for one run (batch 1916p12)
        found = set(os.path.abspath(g) for g in glob.glob(os.path.join(LOGS, "*_i%s_b%s_*" % (n, B))))
        if ("_b%s_" % B) in cur:
            found.add(os.path.abspath(cur))
        found = sorted(found, key=os.path.basename)
        for d in found:
            if d in RUNS or not os.path.isdir(d):
                continue
            # exact plan key: the next expected (lane, seq) whose tag this dir carries; anything else is an unexpected run
            # lane AND whole tag equal (dir "<time>_i<lane>_<tag>"): "..._th1" is no longer found in "..._th10" (codex reply-4)
            matched = [k for k, e in enumerate(EXPECT[n]) if (not e.get("tag") or run_key(d) == (str(n), e["tag"]))
                       and not any(r["seq"] == k + 1 and not r.get("unexpected") for r in ln["runs"])]
            r = {"lane": n, "who": "i%s" % n, "dir": d, "t_run": time.time(), "done": {}, "fail": {}, "stalled": False, "end": None,
                 "read_err": 0}
            if matched:
                r["seq"] = matched[0] + 1; r["who"] = "i%s#%d" % (n, r["seq"])
                say(r["who"], "OK   run_dir   %s" % os.path.basename(d)); r["done"]["run_dir"] = round(time.time() - T0, 1)
            else:
                r["seq"] = None; r["unexpected"] = True; r["fail"]["unexpected"] = round(time.time() - T0, 1); r["end"] = "unexpected"
                say(r["who"], "FAIL unexpected run %s (no planned run of lane %s left with a matching tag)" % (os.path.basename(d), n))
            RUNS[d] = r; ln["runs"].append(r); ln["t_prev_end"] = None
        for r in ln["runs"]:
            if not r.get("unexpected"):
                check_run(r)
        # the next expected run of this lane must appear in time: run 1 after the lane's start slot, run k > 1 after run k-1 ended
        expected_runs = [r for r in ln["runs"] if not r.get("unexpected")]
        k = len(expected_runs)
        if k < len(EXPECT[n]) and not ln.get("gave_up"):
            if k == 0:
                deadline = T0 + STAG * ln["k"] + RUN_DIR_S
            elif expected_runs[-1]["end"] is None:
                deadline = None
            else:
                ln["t_prev_end"] = ln.get("t_prev_end") or time.time()
                deadline = ln["t_prev_end"] + NEXT_RUN_S
            if deadline is not None and time.time() > deadline:
                ln["gave_up"] = True
                say("i%s#%d" % (n, k + 1), "MISS run_dir   expected run %d of %d never appeared" % (k + 1, len(EXPECT[n])))


try:
    load_plan()
    while True:
        cycle()
        done = all((len([r for r in LANE[n]["runs"] if not r.get("unexpected")]) >= len(EXPECT[n]) or LANE[n].get("gave_up"))
                   and all(r["end"] for r in LANE[n]["runs"])
                   for n in LANES)
        snapshot("running")
        if done:
            break
        time.sleep(POLL)
except Exception as e:  # noqa: BLE001
    say(None, "FAIL watcher itself crashed: %r" % e)
    try:
        snapshot("observer_error: %s" % traceback.format_exc().strip().splitlines()[-1])
    except Exception as e2:  # noqa: BLE001
        say(None, "FAIL could not even write %s: %r" % (OUT, e2))
    sys.exit(3)
try:
    res = snapshot("finished")
except Exception as e:  # noqa: BLE001  (the result file itself cannot be written: only the exit code is left)
    try:
        sys.stderr.write("[watch] FAIL final %s not written: %r\n" % (OUT, e))
    except Exception:  # noqa: BLE001
        pass
    sys.exit(4)
nbad = sum(1 for r in RUNS.values() if r["fail"]) + len(res["missing"])
say(None, "DONE expected %d runs, seen %d (unexpected %d), with a missed / stalled step, unexpected or missing %d -> logs/%s_watch.json"
    % (sum(len(v) for v in EXPECT.values()), len(RUNS), len(res["unexpected_runs"]), nbad, B))
sys.exit(nbad)
