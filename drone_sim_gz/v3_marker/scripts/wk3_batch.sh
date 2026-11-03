#!/bin/bash
# Gazebo validation of wk3 lateral controllers (v3_marker, 2026-10-29), C face route, parallel lanes on instances 2..(1+LANES).
#   JOBS_FILE=<file> LANES=4 CHASE=0 wk3_batch.sh     job line: <tag> <rot_deg> <drift_m_s> <step_m> <policy> <params json (v3-relative)>
V3=$HOME/starling_sim/v3_marker; SC=$V3/scripts; LANES=${LANES:-4}
ROUTE="R1C9:W@0.68 R1C1 R1C1@1.265 R1C9"
mapfile -t JOBS < <(grep -v '^#' "${JOBS_FILE:?}" | grep -v '^$')
LIST=$V3/logs/wk3_batch_$(date +%Y%m%d_%H%M%S).txt; : > "$LIST"
lane(){ local k=$1 N=$((2 + $1)) i
  for ((i = k; i < ${#JOBS[@]}; i += LANES)); do
    set -- ${JOBS[$i]}
    WK_STEP_E=$4 WK_STEP_AT=20 CHASE=${CHASE:-0} INSTANCE=$N "$SC/wk_fly.sh" "$1" on "$2" "$3" "$ROUTE" --wk-policy "$5" --wk-params "/home/dronesim/workspace/v3_marker/$6" > "$V3/logs/.lane3_i$N.log" 2>&1
    echo "$1 $(cat "$V3/logs/.last_run_i$N")" >> "$LIST"
    echo "[lane $N] $1 -> $(grep -a VERDICT "$(cat "$V3/logs/.last_run_i$N")/report.txt" 2>/dev/null || echo no-verdict)"
  done
  "$SC/stop_sitl_instance.sh" $N > /dev/null; }
for ((k = 0; k < LANES; k++)); do lane $k & sleep 45; done; wait
echo "list: $LIST"
