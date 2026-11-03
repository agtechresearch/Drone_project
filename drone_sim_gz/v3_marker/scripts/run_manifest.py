#!/usr/bin/env python3
"""Run manifest: what a flight actually ran with (v3_marker, 2026-10-31, codex experiment-provenance).
  write:   python3 run_manifest.py write <run_dir> <instance> <world>       (wk_fly.sh, before the mission starts)
  finish:  python3 run_manifest.py finish <run_dir>                          (wk_fly.sh, after the mission: plan.json, evaluators)
  compare: python3 run_manifest.py compare <run_dir>...                     prints the fields that differ between runs
           and, per run, VERIFIED / UNVERIFIED: complete must be True and every REQUIRED field present (2026-11-01 codex
           experiment-provenance reply-10: a field missing on BOTH sides and complete=False on both used to pass silently);
           exit 0 = every run verified, 3 = at least one not (the differences are printed either way)
2026-10-31 (codex experiment-provenance reply-2): also the wk_fly.sh CLI values (MF_* variables: WK on/off, ROT, DRIFT, ROUTE,
EXTRA, the computed POSE) and the effective defaults it passes on, the full condition.txt; every hash / PX4 query records
whether it worked ("collection" list, "complete"); compare shows a value that is missing or failed on either side as
UNVERIFIED, never as equal.
Recorded: the effective switches / values of every WK_* / QVIX_* / NO_VIDEO / CHASE / SPAWN_* variable as wk_fly.sh normalised
them, sha256 of the flight and sensor code, the configs, the map the flight uses (WK_MAP or the design tags_map.json), the world
and model SDF actually installed in the PX4 tree, the PX4 EKF2 EV parameters of the instance (read through px4-param), and the
ToF model seed. Older runs have no manifest: compare reports them as "unknown", never as equal."""
import hashlib, json, os, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__)); V3 = os.path.dirname(HERE); SIM = os.path.dirname(V3)
PX4 = os.path.join(SIM, "PX4-Autopilot")
CODE = ["scripts/wk_mission.py", "scripts/safety_patch.py", "scripts/loc_compare.py", "scripts/grid_verify.py", "scripts/tagfix_eval.py", "scripts/loc2d.py", "scripts/grid_mission.py", "scripts/qvix_grid.py", "scripts/wall_sense.py",
        "scripts/wall_sense_fit.py", "scripts/tof_sim_model.py", "scripts/tag_detect.py", "scripts/vio_relay.py", "scripts/gt_logger.py",
        "scripts/wk_fly.sh", "scripts/run_status.sh", "scripts/run_wk_instance.sh", "scripts/install_tof.sh", "wk3/localize.py",
        "scripts/run_geometry.py", "scripts/tag_quality.py",
        "config/tof_sim_model.json", "config/survey_starling2.json", "gz/tags_map.json"]
PARAMS = ["EKF2_EV_CTRL", "EKF2_EV_DELAY", "EKF2_EVP_NOISE", "EKF2_EVA_NOISE", "EKF2_EVV_NOISE", "EKF2_HGT_REF"]


PROBLEMS = []


def sha(p):
    try:
        return hashlib.sha256(open(p, "rb").read()).hexdigest()
    except OSError as e:
        PROBLEMS.append("hash %s: %s" % (p, e.strerror))
        return None


def px4_params(n):
    out = {}
    for p in PARAMS:
        try:
            r = subprocess.run(["docker", "exec", "starling-gz", "bash", "-c",
                                "cd /home/dronesim/PX4-Autopilot/build/px4_sitl_default/bin && ./px4-param --instance %s show %s" % (n, p)],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, timeout=10)
            line = [l for l in r.stdout.splitlines() if p in l and ":" in l]
            out[p] = line[0].split(":")[-1].strip() if (r.returncode == 0 and line) else None
            if out[p] is None:
                PROBLEMS.append("px4 param %s: rc %d, %s" % (p, r.returncode, r.stdout.strip()[-80:]))
        except Exception as e:  # noqa: BLE001
            out[p] = None; PROBLEMS.append("px4 param %s: %r" % (p, e))
    return out


def write(run, inst, world):
    env = {k: v for k, v in os.environ.items() if k.startswith(("WK_", "QVIX_", "SPAWN_")) or k in ("NO_VIDEO", "CHASE", "ARCHIVE", "MISSION_TIMEOUT")}
    wmap = os.environ.get("WK_MAP") or "gz/tags_map.json"
    tofm = os.environ.get("WK_TOF_MODEL")
    m = {"run": os.path.basename(os.path.normpath(run)), "instance": int(inst), "world": world, "env": env,
         "code_sha256": {c: sha(os.path.join(V3, c)) for c in CODE},
         "v13_sha256": sha(os.path.join(SIM, "path_flight_phase1_v13.py")),
         "map": {"path": wmap, "sha256": sha(os.path.join(V3, wmap))},
         "tof_model": {"path": tofm, "sha256": sha(tofm) if tofm else None, "seed": json.load(open(tofm)).get("seed") if tofm else None},
         "world_sdf_sha256": sha(os.path.join(PX4, "Tools/simulation/gz/worlds", world + ".sdf")),
         # the model this run's instance spawned: the WK_MODEL_DIR variant when set (run_wk_instance.sh), else the installed one
         "model_sdf_sha256": sha(os.path.join(os.environ["WK_MODEL_DIR"], "starling2_tof/model.sdf") if os.environ.get("WK_MODEL_DIR")
                                 else os.path.join(PX4, "Tools/simulation/gz/models/starling2_tof/model.sdf")),
         "model_dir": os.environ.get("WK_MODEL_DIR") or "installed",
         "px4_params": px4_params(inst),
         "cli": {k[3:].lower(): v for k, v in os.environ.items() if k.startswith("MF_")}}
    cp = os.path.join(run, "condition.txt")
    m["condition_txt"] = open(cp).read() if os.path.exists(cp) else None
    lt = os.path.join(V3, "gz", "survey", world[5:] + "_truth.json") if world.startswith("qvix_sv") else None
    m["survey_truth"] = {"path": lt, "sha256": sha(lt)} if lt else None
    # required, item by item (2026-11-01 codex experiment-provenance reply-4: "cli not empty" let half a record pass)
    for k in REQUIRED_CLI:
        if not m["cli"].get(k):
            PROBLEMS.append("cli.%s missing (MF_%s)" % (k, k.upper()))
    for k in ("condition_txt", "world_sdf_sha256", "model_sdf_sha256"):
        if not m.get(k):
            PROBLEMS.append("%s missing" % k)
    for k, v in m["px4_params"].items():
        if v is None:
            PROBLEMS.append("px4 param %s not read" % k)
    m["collection"] = list(PROBLEMS); m["complete"] = not PROBLEMS
    json.dump(m, open(os.path.join(run, "run_manifest.json"), "w"), indent=1, sort_keys=True)
    print("run_manifest.json: %d code files, map %s, px4 %s, complete %s%s" % (len(CODE), wmap, m["px4_params"], m["complete"],
          "" if m["complete"] else " " + "; ".join(PROBLEMS)))
    return m["complete"]


REQUIRED_CLI = ("wk", "rot", "drift", "route", "pose", "effective")   # MF_EXTRA / MF_SURVEY may be empty
EVAL = ["scripts/loc_compare.py", "scripts/grid_verify.py", "scripts/run_status.sh", "scripts/tagfix_eval.py", "scripts/qvix_grid.py",
        "scripts/run_geometry.py"]


def finish(run):
    """after the evaluation (wk_fly.sh calls it after loc_compare, 2026-11-01 codex reply-10): the plan the flight actually used and
    the evaluators' versions, which must equal the hashes taken before the mission"""
    p = os.path.join(run, "run_manifest.json")
    m = json.load(open(p)) if os.path.exists(p) else {"complete": False, "collection": ["no manifest written before the mission"]}
    m["plan_sha256"] = sha(os.path.join(run, "plan.json"))
    m["evaluator_sha256"] = {c: sha(os.path.join(V3, c)) for c in EVAL}
    if not m["plan_sha256"]:
        PROBLEMS.append("plan.json missing")
    # the evaluator code must be the code the batch started with: compare with the hashes taken before the mission
    for c in EVAL:
        before = (m.get("code_sha256") or {}).get(c)
        if not before:      # no start hash = the evaluator's version at the start is unknown, not "unchanged" (codex reply-10)
            PROBLEMS.append("%s has no start hash" % c)
        elif before != m["evaluator_sha256"][c]:
            PROBLEMS.append("%s changed between the start and the end of the run" % c)
    m["collection"] = m.get("collection", []) + PROBLEMS; m["complete"] = bool(m.get("complete")) and not PROBLEMS
    json.dump(m, open(p, "w"), indent=1, sort_keys=True)
    print("run_manifest.json finished: plan %s, complete %s" % ((m["plan_sha256"] or "MISSING")[:12], m["complete"]))
    return m["complete"]


def flat(d, pre=""):
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(flat(v, pre + k + "."))
        else:
            out[pre + k] = v
    return out


NEW_2026_11_01 = ("scripts/run_geometry.py", "scripts/tag_quality.py")   # added to CODE / EVAL today: older manifests lack them


def required(m):
    """flat keys a complete manifest must have (not None / not empty): the fixed fields, every code / evaluator file of the lists
    (files added on 2026-11-01 only when the manifest recorded them), and every hash key the manifest itself recorded"""
    core = lambda L: [c for c in L if c not in NEW_2026_11_01]
    return sorted(set(["v13_sha256", "map.sha256", "world_sdf_sha256", "model_sdf_sha256", "condition_txt", "plan_sha256"]
                      + ["code_sha256." + c for c in core(CODE)] + ["px4_params." + p for p in PARAMS]
                      + ["cli." + k for k in REQUIRED_CLI] + ["evaluator_sha256." + c for c in core(EVAL)]
                      + [k for k in m if k.startswith(("code_sha256.", "evaluator_sha256."))]
                      + (["survey_truth.sha256"] if str(m.get("world", "")).startswith("qvix_sv") else [])))


def verdict(m):
    """-> (verified?, reasons) for one flattened manifest (None = no manifest)"""
    if m is None:
        return False, ["no manifest"]
    why = []
    if m.get("complete") is not True:
        why.append("complete %s" % m.get("complete"))
    miss = [k for k in required(m) if m.get(k) in (None, "")]
    if miss:
        why.append("missing %d: %s" % (len(miss), ", ".join(miss[:8]) + (" ..." if len(miss) > 8 else "")))
    return not why, why


def compare(runs):
    M = {}
    for r in runs:
        p = os.path.join(r, "run_manifest.json")
        M[os.path.basename(os.path.normpath(r))] = flat(json.load(open(p))) if os.path.exists(p) else None
    keys = sorted(set(k for m in M.values() if m for k in m) - {"run", "collection"})
    print("runs: %s" % ", ".join("%s%s" % (k, "" if v else " (no manifest: unknown)") for k, v in M.items()))
    for k in keys:
        vals = {r: (m.get(k) if m else None) for r, m in M.items()}
        # survey_truth is null by design outside the survey worlds (qvix_sv*: required, checked in verdict())
        if k == "survey_truth" and all(m and m.get(k) is None and not str(m.get("world", "")).startswith("qvix_sv") for m in M.values()):
            continue
        if any(v is None for v in vals.values()):          # missing / failed on one side: not evidence of equality
            print("  UNVERIFIED %-34s %s" % (k, "  |  ".join("%s=%s" % (r[-24:], "unknown" if v is None else str(v)[:16]) for r, v in vals.items())))
        elif len(set(map(str, vals.values()))) > 1:
            print("  differs    %-34s %s" % (k, "  |  ".join("%s=%s" % (r[-24:], str(v)[:16]) for r, v in vals.items())))
    ok = True
    for r, m in M.items():
        v, why = verdict(m)
        ok &= v
        print("run %s: %s%s" % (r, "VERIFIED" if v else "UNVERIFIED", "" if v else " - " + "; ".join(why)))
    print("compare: %d run(s), %s" % (len(M), "all verified" if ok else "NOT all verified - differences above are not evidence"))
    return ok


if __name__ == "__main__":
    if sys.argv[1] == "write":
        sys.exit(0 if write(*sys.argv[2:5]) else 3)
    elif sys.argv[1] == "finish":
        sys.exit(0 if finish(sys.argv[2]) else 3)
    else:
        sys.exit(0 if compare(sys.argv[2:]) else 3)
