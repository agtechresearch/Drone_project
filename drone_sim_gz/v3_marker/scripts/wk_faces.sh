#!/bin/bash
# All 8 rack faces of R1 x error conditions, flown in parallel on instances 2..(1+LANES) (run on the HOST). 2026-10-29.
#   wk_faces.sh                 # 8 faces x {yaw15, drift04} x {off, on} = 32 flights, 4 lanes
#   FACES="CW CE" CONDS="yaw15" LANES=2 wk_faces.sh
# Per face <aisle><side> the route is the same shape: start at row 9 facing the face, 0.68 m (gap 1) north -> south to row 1,
# climb to 1.265 m (gap 2), south -> north back to row 9:   R1<a>9:<s>@0.68 R1<a>1 R1<a>1@1.265 R1<a>9
# Conditions: yaw15 = VIO heading rotated 1.5 deg, drift04 = east drift 0.4 cm/s, none = no error; each flown off and on.
# Output: logs/faces_<ts>.txt with one line per flight: <face> <cond> <off|on> <run dir>
V2=$HOME/starling_sim/v3_marker; SC=$V2/scripts
FACES=${FACES:-"AE BW BE CW CE DW DE EW"}; CONDS=${CONDS:-"yaw15 drift04"}; LANES=${LANES:-4}
LIST=$V2/logs/faces_$(date +%Y%m%d_%H%M%S).txt; : > "$LIST"
JOBS=()
WKS=${WKS:-"off on"}                 # e.g. WKS=on to re-fly only the wall-keep runs
if [ -n "${JOBS_FILE:-}" ]; then   # explicit job list, one "<face> <cond> <off|on>" per line (e.g. the missing ones)
  while read -r f c w; do [ -n "$f" ] && JOBS+=("$f $c $w"); done < "$JOBS_FILE"
else
  for f in $FACES; do for c in $CONDS; do for w in $WKS; do JOBS+=("$f $c $w"); done; done; done
fi
lane(){   # lane index -> instance 2+idx, takes every LANES-th job
  local k=$1 N=$((2 + $1)) i
  for ((i = k; i < ${#JOBS[@]}; i += LANES)); do
    set -- ${JOBS[$i]}; local f=$1 c=$2 w=$3 a=${1:0:1} s=${1:1:1} rot=0 dr=0
    st=""                                  # cond names: yaw<NN> = NN/10 deg, drift<NN> = NN/1000 m/s, step<NN> = NN cm jump
    case $c in none) ;; yaw*) rot=$(awk "BEGIN{print ${c#yaw}/10}");; drift*) dr=$(awk "BEGIN{print ${c#drift}/1000}");;
      step*) st=$(awk "BEGIN{print ${c#step}/100}");; esac
    WK_STEP_E=$st WK_STEP_AT=${WK_STEP_AT:-20} INSTANCE=$N "$SC/wk_fly.sh" "${f}_${c}_${w}" "$w" "$rot" "$dr" "R1${a}9:${s}@0.68 R1${a}1 R1${a}1@1.265 R1${a}9" > "$V2/logs/.lane_i$N.log" 2>&1
    echo "$f $c $w $(cat "$V2/logs/.last_run_i$N")" >> "$LIST"
    echo "[lane $N] $f $c $w -> $(grep -a "VERDICT" "$(cat "$V2/logs/.last_run_i$N")/report.txt" 2>/dev/null || echo "no verdict (see wk_faces_fill.sh)")"
  done
  "$SC/stop_sitl_instance.sh" $N > /dev/null
}
for ((k = 0; k < LANES; k++)); do lane $k & sleep 45; done      # stagger the starts (gz/PX4 boot load; 20 s let spawns time out)
wait
echo "list: $LIST ($(wc -l < "$LIST") flights)"
