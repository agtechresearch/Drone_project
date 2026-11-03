#!/bin/bash
# Tag-heading vs ToF-heading scan fix, paired per condition, instances 2-5 in parallel (run on the HOST, 2026-10-30).
#   [THS="1"] [LOCB=1: th1 = --wk-loc instead of the ToF heading] wk_tofh_batch.sh [batch_tag]     -> logs/tofh_<batch_tag>_runs.txt (one run dir per line), then tagfix_eval on all
# Each lane flies the same condition twice (WK_TOF_HEADING off, then on); lanes start 45 s apart (gz spawn timeouts).
# NO_VIDEO=1: no frame images (disk).
set -u
SC=${TOFH_SC:-$HOME/starling_sim/v3_marker/scripts}; LOGS=${TOFH_LOGS:-$HOME/starling_sim/v3_marker/logs}   # TOFH_*: tests/test_tofh_batch.sh
B=${1:-$(date +%H%M)}; LIST=$LOGS/tofh_${B}_runs.txt; : > "$LIST"
STAT=$LOGS/tofh_${B}_status.txt; : > "$STAT"     # 2026-10-31 (codex runner-exit-status): per-run exit codes, counted at the end
ROUTE="R1C9:W@0.68 R1C1 R1C1@1.265 R1C9"
# lane: instance rot drift_e drift_n spawn_offset
LANES=${TOFH_LANES:-"2 3 0.004 0.004 0.04,-0.03
3 -4 0.008 0.0 0.04,-0.03
4 1.5 0.004 0.004 -0.04,-0.03
5 -2 0.004 0.004 0.10,-0.03"}
lane(){
  local n=$1 rot=$2 de=$3 dn=$4 off=$5
  for th in ${THS:-0 1}; do
    local tag=b${B}_r${rot}_th${th}
    rm -f "$LOGS/.last_run_i$n"        # a run that dies before naming its dir must not inherit the previous run's dir
    # WK_TOF_HEADING empty = off (wk_fly.sh tests ${WK_TOF_HEADING:+...})
    INSTANCE=$n WK_WORLD=qvix_rot QVIX_YAW_DEG=7 WK_TAGS=1 WK_RECHECK=1 NO_VIDEO=1 WK_DRIFT_N=$dn SPAWN_OFFSET=$off \
      WK_TOF_HEADING=$( [ $th = 1 ] && [ -z "${LOCB:-}" ] && echo 1 ) WK_LOC=$( [ $th = 1 ] && [ -n "${LOCB:-}" ] && echo 1 ) "$SC/wk_fly.sh" "$tag" on "$rot" "$de" "$ROUTE" > "$LOGS/.lane_i$n.log" 2>&1 < /dev/null
    local rc=$?
    local run; run=$(cat "$LOGS/.last_run_i$n" 2>/dev/null) || run="NO_RUN_DIR_i${n}_$tag"
    [ -d "$run" ] && echo "$run" >> "$LIST"
    # run dir + wk_fly exit code: 0 ok, 10 mission failed, 11 verify/clearance failed, 12 archive failed, 13 safety landing,
    # 14 verdict incomplete, 15 status.json not written, 1/2 not started (also: instance stop failed)
    echo "$run $rc" >> "$STAT"
  done
}
PIDS=""
while read -r n rot de dn off; do
  lane "$n" "$rot" "$de" "$dn" "$off" < /dev/null &      # the flights' docker exec -i would eat the lane list
  PIDS="$PIDS $!:$n"; sleep ${TOFH_STAGGER:-45}
done <<< "$LANES"
# 2026-10-31 (codex runner-exit-status reply-2): every lane's own exit, the expected number of rows, the evaluation's result
LANEFAIL=0
for pl in $PIDS; do wait "${pl%%:*}" || { echo "!! lane i${pl#*:} exited $?"; LANEFAIL=$((LANEFAIL+1)); }; done
NEXP=$(( $(grep -c . <<< "$LANES") * $(wc -w <<< "${THS:-0 1}") ))
echo "runs:"; cat "$LIST"
for d in $(cat "$LIST"); do
  echo "== $(basename "$d")  $(grep -a -h 'tof-heading' "$d/mission.log" | sed 's/.*\[tof-heading\]//' | cut -c1-120)"
  QVIX_YAW_DEG=7 python3 "$SC/tagfix_eval.py" "$d" > "$LOGS/.tagfix_eval.out" 2>&1; erc=$?
  grep -E 'fix:|first 1 s|s00_yaw|s01_to|s03_to|Traceback|Error' "$LOGS/.tagfix_eval.out"
  [ $erc = 0 ] || { echo "$d eval_failed_$erc" >> "$STAT"; echo "!! tagfix_eval exited $erc"; }
done
NFAIL=$(awk '$2 != 0' "$STAT" | wc -l); NRUN=$(grep -vc eval_failed "$STAT")
[ "$NRUN" = "$NEXP" ] || { echo "!! expected $NEXP run rows, got $NRUN"; NFAIL=$((NFAIL + (NRUN > NEXP ? 1 : NEXP - NRUN))); }
NFAIL=$((NFAIL + LANEFAIL))
echo "runs $NRUN of $NEXP, lanes failed $LANEFAIL, not ok $NFAIL:"; awk '$2 != 0 {print "  " $0}' "$STAT"
echo EXIT=$NFAIL
exit $NFAIL
