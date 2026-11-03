"""Which geometry a run must be judged against, and whether the input at hand is it (2026-11-01, codex report-truth-geometry
reply-8: a standalone loc_compare.py / wk_report.py call used LOC_TRUTH or the design racks without checking the run).
  check(run_dir, loc_truth=None) -> dict
    world        from run_manifest.json "world", else condition.txt "world=", else None
    yaw          QVIX_YAW_DEG of this process (qvix_grid's world turn) against the run's (manifest env, else condition "yaw=")
    design       qvix_grid.py sha256 now against the manifest's code_sha256 at the start of the run
    truth        survey world qvix_sv<k>: gz/survey/sv<k>_truth.json is REQUIRED (moved racks); its sha256 against the manifest's
                 survey_truth.sha256. A LOC_TRUTH given by the caller must have the same content. Design worlds: a LOC_TRUTH with
                 a rack_shift is refused (a moved rack is not this run's geometry)
    status       "ok" | "unverified" (something could not be compared: no manifest / no hash / world unknown) |
                 "mismatch" (an input differs from the run's) | "missing" (the required truth file does not exist)
    evaluate     True only for ok; for unverified also when LOC_GEOMETRY_UNVERIFIED=accept (an explicit operator choice for old
                 runs without a manifest - the result is then labelled); never for mismatch / missing
    reasons, used (path + sha256 of every geometry input actually used), truth (the loaded truth dict, {} for design worlds)
  line(g) -> one text line for the reports."""
import hashlib, json, os

HERE = os.path.dirname(os.path.abspath(__file__)); V3 = os.path.dirname(HERE)
GRID = os.path.join(HERE, "qvix_grid.py")


def _sha(p):
    try:
        return hashlib.sha256(open(p, "rb").read()).hexdigest()
    except OSError:
        return None


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def check(run, loc_truth=None):
    reasons, bad = [], set()
    mp = os.path.join(run, "run_manifest.json")
    try:
        M = json.load(open(mp)) if os.path.exists(mp) else None
    except (OSError, ValueError) as e:
        M = None; reasons.append("run_manifest.json unreadable: %s" % e); bad.add("unverified")
    cond = ""
    try:
        cond = open(os.path.join(run, "condition.txt")).read()
    except OSError:
        pass
    ctok = dict(w.split("=", 1) for w in cond.split() if "=" in w)
    if M is None and "unverified" not in bad:
        reasons.append("no run_manifest.json (hashes at the start of the run unknown)"); bad.add("unverified")
    world = (M or {}).get("world") or ctok.get("world")
    if not world:
        reasons.append("world unknown"); bad.add("unverified")
    # world turn
    yaw_now = _f(os.environ.get("QVIX_YAW_DEG", "0"))
    yaw_run = _f(((M or {}).get("env") or {}).get("QVIX_YAW_DEG", ctok.get("yaw")))
    if yaw_run is None:
        reasons.append("world yaw of the run not recorded"); bad.add("unverified")
    elif yaw_now is None or abs(yaw_now - yaw_run) > 1e-9:
        reasons.append("QVIX_YAW_DEG %s here, %s in the run" % (os.environ.get("QVIX_YAW_DEG", "0"), yaw_run)); bad.add("mismatch")
    # design geometry (qvix_grid: racks, aisles, walls)
    g_now = _sha(GRID); g_run = ((M or {}).get("code_sha256") or {}).get("scripts/qvix_grid.py")
    if not g_run:
        if M is not None:
            reasons.append("qvix_grid.py hash not in the manifest"); bad.add("unverified")
    elif g_run != g_now:
        reasons.append("qvix_grid.py changed since the run (%s -> %s)" % (g_run[:12], (g_now or "none")[:12])); bad.add("mismatch")
    used = {"qvix_grid.py": {"path": GRID, "sha256": g_now}, "yaw_deg": yaw_now}
    # survey truth
    truth = {}
    req = os.path.join(V3, "gz", "survey", world[len("qvix_"):] + "_truth.json") if world and world.startswith("qvix_sv") else None
    if req:
        path = loc_truth or req
        if not os.path.exists(req):
            reasons.append("required truth %s missing" % os.path.relpath(req, V3)); bad.add("missing")
        if loc_truth and os.path.exists(req) and _sha(loc_truth) != _sha(req):
            reasons.append("LOC_TRUTH %s is not this run's truth %s" % (loc_truth, os.path.relpath(req, V3))); bad.add("mismatch")
        t_now = _sha(path); t_run = ((M or {}).get("survey_truth") or {}).get("sha256")
        if t_now and not t_run:
            if M is not None:
                reasons.append("survey truth hash not in the manifest"); bad.add("unverified")
        elif t_now and t_run != t_now:
            reasons.append("survey truth changed since the run (%s -> %s)" % (t_run[:12], t_now[:12])); bad.add("mismatch")
        if t_now:
            try:
                truth = json.load(open(path))
            except (OSError, ValueError) as e:
                reasons.append("truth unreadable: %s" % e); bad.add("missing")
        used["truth"] = {"path": path, "sha256": t_now}
    elif loc_truth:
        try:
            lt = json.load(open(loc_truth))
        except (OSError, ValueError) as e:
            lt = {}; reasons.append("LOC_TRUTH unreadable: %s" % e); bad.add("mismatch")
        if lt.get("rack_shift"):
            reasons.append("LOC_TRUTH %s moves a rack but the run's world is %s" % (loc_truth, world)); bad.add("mismatch")
        used["truth"] = {"path": loc_truth, "sha256": _sha(loc_truth), "ignored": True}
    status = "missing" if "missing" in bad else "mismatch" if "mismatch" in bad else "unverified" if "unverified" in bad else "ok"
    accept = os.environ.get("LOC_GEOMETRY_UNVERIFIED", "") == "accept"
    return {"status": status, "evaluate": status == "ok" or (status == "unverified" and accept), "accepted_unverified":
            status == "unverified" and accept, "world": world, "reasons": reasons, "used": used, "truth": truth}


def line(g):
    u = g["used"]; t = u.get("truth")
    return "geometry %s%s: world %s, yaw %s, qvix_grid %s%s%s" % (
        g["status"], " (accepted: LOC_GEOMETRY_UNVERIFIED=accept)" if g["accepted_unverified"] else "", g["world"], u["yaw_deg"],
        (u["qvix_grid.py"]["sha256"] or "none")[:12],
        ", truth %s %s" % (os.path.basename(t["path"]), (t["sha256"] or "none")[:12]) if t and not t.get("ignored") else "",
        "; " + "; ".join(g["reasons"]) if g["reasons"] else "")
