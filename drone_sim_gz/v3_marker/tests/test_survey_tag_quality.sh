#!/bin/bash
# survey_map.py uses the shared tag quality rule (2026-11-01, codex real-tag-quality-contract; fork B dependency): a detection
# without a margin is rejected unless WK_TAG_QUALITY_UNAVAILABLE=accept, numbers are judged as before. Offline, on the real
# survey3 sv1 run (logs/20261031_144956_i2_survey3_sv1_r3) with the flight's ToF position (as logs/1930_survey_rerun/run3.sh).
#   A numeric margins (the recorded file)         -> tags identical to logs/1930_survey_rerun/sv1_final/survey_map.json
#   B every passing detection's margin + hamming -> null, failing ones removed, default policy -> no tag used
#   C the same file, WK_TAG_QUALITY_UNAVAILABLE=accept                -> tags identical to A, counted as unavailable_accepted
# exit code = number of failures
cd /home/dronesim/starling_sim/v3_marker; PY=/home/dronesim/starling_sim/cad/venv/bin/python
RUN=logs/20261031_144956_i2_survey3_sv1_r3; S=$(mktemp -d); F=0
SET=(--set 'tof.pos_body_flu=[0.066,-0.009,0.012]' --no-plots)
mkdir -p $S/runB; for f in $RUN/*; do [ "$(basename $f)" = tags_raw.jsonl ] || ln -s "$(readlink -f $f)" $S/runB/; done
$PY - "$RUN/tags_raw.jsonl" "$S/runB/tags_raw.jsonl" <<'PY'
import json, sys
n = k = 0
with open(sys.argv[2], "w") as o:
    for s in open(sys.argv[1]):
        m = json.loads(s); keep = []
        for d in m["dets"]:
            n += 1
            if d["margin"] >= 30 and d["hamming"] == 0:
                d["margin"] = None; d["hamming"] = None; keep.append(d); k += 1
        m["dets"] = keep; o.write(json.dumps(m) + "\n")
print("detections %d, kept with null quality %d" % (n, k))
PY
SURVEY_TRUTH=gz/survey/sv1_truth.json $PY scripts/survey_map.py $RUN --out $S/A "${SET[@]}" > $S/A.log 2>&1; ra=$?
SURVEY_TRUTH=gz/survey/sv1_truth.json $PY scripts/survey_map.py $S/runB --out $S/B "${SET[@]}" > $S/B.log 2>&1; rb=$?
WK_TAG_QUALITY_UNAVAILABLE=accept SURVEY_TRUTH=gz/survey/sv1_truth.json $PY scripts/survey_map.py $S/runB --out $S/C "${SET[@]}" > $S/C.log 2>&1; rc=$?
echo "rc A $ra, B $rb, C $rc"; grep -h "tags corrected" $S/A.log $S/B.log $S/C.log | sed 's/^/  /'; tail -2 $S/B.log | sed 's/^/  B: /'
$PY - $S logs/1930_survey_rerun/sv1_final/survey_map.json $ra $rb $rc <<'PY'
import json, os, sys
S, ref, ra, rb, rc = sys.argv[1], sys.argv[2], *map(int, sys.argv[3:6])
F = 0
def chk(ok, txt):
    global F
    print("%s  %s" % ("PASS" if ok else "FAIL", txt)); F += (not ok)
def load(p):
    return json.load(open(p)) if os.path.exists(p) else None
A, B, C, R = load(S + "/A/survey_map.json"), load(S + "/B/survey_map.json"), load(S + "/C/survey_map.json"), load(ref)
obs = lambda m: sorted(k for k, t in m["tags"].items() if t.get("observed")) if m else None
chk(ra == 0 and A is not None, "A ran (rc %d)" % ra)
if A and R:
    same = all(A["tags"][k].get(f) == R["tags"][k].get(f) for k in R["tags"] for f in ("x", "y", "observed"))
    chk(same and A["tags"].keys() == R["tags"].keys(), "A tags identical to sv1_final (%d observed)" % len(obs(A)))
    print("      A decisions %s" % A["tag_quality"]["decisions"])
if B is not None:
    chk(len(obs(B)) == 0 and B["tag_quality"]["decisions"]["unavailable"] > 0 and B["tag_quality"]["decisions"]["ok"] == 0,
        "B null quality, default reject: observed %d, decisions %s" % (len(obs(B)), B["tag_quality"]["decisions"]))
else:
    chk(rb != 0, "B null quality, default reject: no map written, rc %d (no tag observation left)" % rb)
if C and A:
    same = all(C["tags"][k].get(f) == A["tags"][k].get(f) for k in A["tags"] for f in ("x", "y", "observed"))
    d = C["tag_quality"]["decisions"]
    chk(rc == 0 and same and d["unavailable_accepted"] == d["unavailable"] > 0,
        "C accept: tags identical to A (%d observed), decisions %s" % (len(obs(C)), d))
else:
    chk(False, "C accept ran (rc %d)" % rc)
sys.exit(F)
PY
F=$?; rm -rf $S; echo "$F failure(s)"; exit $F
