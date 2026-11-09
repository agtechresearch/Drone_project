#!/bin/bash
# Outcome of one wk_fly.sh run (2026-10-31, codex runner-exit-status). Called by wk_fly.sh after the archive step:
#   run_status.sh <run_dir> <mission_rc> <grid_verify_rc> <world> <archive: ok|failed|skipped> [truth: PASS|FAIL|INCOMPLETE|error|none] [provenance: ok|incomplete]
# Writes <run_dir>/status.json and exits with the run's code:
#   0 ok | 10 mission failed (rc != 0, Traceback, stop_on_fail, NO_YAW_FEEDBACK, YAW_TIMEOUT, stage TIMEOUT; anywhere: touchdown
#   not confirmed, land never sent, motor kill)
#   11 grid_verify failed (original qvix world only) | 12 archive failed | 13 safety landing (EMERGENCY LAND; may be intended in
#   a fault-injection test, but never a normal success)
#   14 verdict incomplete (turned world: loc_compare clearance INCOMPLETE / error / missing) | 15 status.json not written
#   16 the safety layer's mission_result.json not written / missing / of another run (status "result_file": not_written | missing |
#   run_id_mismatch; when nothing else failed - with 10 / 13 the code stays, the field carries it, batch_expect counts a mismatch)
# In turned / survey worlds grid_verify is not rotation-aware (2026-10-30): those runs are judged by the truth clearance of
# loc_compare.py (wk_fly.sh passes its verdict). Without that verdict the run is NOT counted as passed (codex reply-2).
OUT=$1; MRC=$2; VRC=$3; WORLD=$4; ASTAT=$5; TRUTHV=${6:-none}; PROV=${7:-unknown}   # provenance (run_manifest complete?) is recorded, it does not change the exit code
LOG=$OUT/mission.log
MSTAT=ok
[ "$MRC" != "0" ] && MSTAT=failed
# failures BEFORE an emergency landing count; stages that the landing itself aborted (YAW_TIMEOUT ... reason=ABORTED) do not
EM=$(grep -a -n -m1 "EMERGENCY LAND TRIGGERED" "$LOG" 2>/dev/null | cut -d: -f1); EM=${EM:-999999999}
head -n $((EM - 1)) "$LOG" 2>/dev/null | grep -a -v "reason=ABORTED" | \
  grep -a -q -e "Traceback" -e "stop_on_fail:" -e "NO_YAW_FEEDBACK" -e "YAW_TIMEOUT" -e "\] \[END\] TIMEOUT" && MSTAT=failed
[ -f "$LOG" ] || MSTAT=failed
# a safety landing that did not finish is a failure, wherever it is in the log (2026-11-01 codex graceful-stop-kill reply-4)
grep -a -q -e "TOUCHDOWN NOT CONFIRMED" -e "land NOT sent after" -e "Kill command sent" "$LOG" 2>/dev/null && MSTAT=failed
grep -a -q "EMERGENCY LAND TRIGGERED" "$LOG" 2>/dev/null && [ "$MSTAT" = ok ] && MSTAT=safety_landing
if [ "$WORLD" = "qvix" ]; then VBY=grid_verify; [ "$VRC" = 0 ] && VSTAT=ok || VSTAT=failed
else VBY="loc_compare.py clearance (truth)"
  case "$TRUTHV" in PASS) VSTAT=ok;; FAIL) VSTAT=failed;; *) VSTAT=incomplete;; esac; fi
# the safety layer's result file, checked on its own (codex decisions reply-9): when the log names a run id ("[result] run id <id>"),
# mission_result.json must exist and carry that id; else missing / run_id_mismatch. Runs without the layer: n/a
RID=$(grep -a -o -m1 "\[result\] run id [^ ,]*" "$LOG" 2>/dev/null | awk '{print $4}')
if [ -z "$RID" ]; then RFILE=n/a
elif [ ! -f "$OUT/mission_result.json" ]; then RFILE=missing
else JID=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1])).get('run_id'))" "$OUT/mission_result.json" 2>/dev/null)
  [ "$JID" = "$RID" ] && RFILE=ok || RFILE=run_id_mismatch; fi
grep -a -q "\[result\] !!! RESULT FILE NOT WRITTEN" "$LOG" 2>/dev/null && RFILE=not_written
RC=0
[ "$MSTAT" = failed ] && RC=10
[ "$MSTAT" = safety_landing ] && RC=13
[ "$VSTAT" = failed ] && [ $RC = 0 ] && RC=11
[ "$VSTAT" = incomplete ] && [ $RC = 0 ] && RC=14
[ "$ASTAT" = failed ] && [ $RC = 0 ] && RC=12
case $RFILE in ok|n/a) ;; *) [ $RC = 0 ] && RC=16;; esac
printf '{"run": "%s", "mission": "%s", "mission_rc": %s, "verify": "%s", "verify_rc": %s, "judged_by": "%s", "truth_verdict": "%s", "provenance": "%s", "archive": "%s", "result_file": "%s", "exit": %s}\n' \
  "$(basename "$OUT")" "$MSTAT" "$MRC" "$VSTAT" "$VRC" "$VBY" "$TRUTHV" "$PROV" "$ASTAT" "$RFILE" "$RC" > "$OUT/.status.json.tmp" \
  && mv -f "$OUT/.status.json.tmp" "$OUT/status.json" \
  || { echo "!! status.json not written in $OUT"; [ $RC = 0 ] && RC=15; }   # complete file or none (readers never see half of it)
echo "status: mission $MSTAT, verify $VSTAT ($VBY), provenance $PROV, archive $ASTAT -> exit $RC"
exit $RC
