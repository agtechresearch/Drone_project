#!/bin/bash
# Complete an 8-face matrix (run on the HOST): for every face x cond x off/on take the newest run dir after SINCE that has
# a grid_verify VERDICT, re-fly the missing ones one at a time on instance ${INSTANCE:-2}, write logs/faces_final_<ts>.txt.
# Why: with 4 lanes booting together the gz spawn service can time out ("gz_bridge Service call timed out", 2026-10-29
# lanes 4/5), and that job is then skipped by wk_faces.sh.
#   SINCE=20261029_1038 wk_faces_fill.sh
V2=$HOME/starling_sim/v3_marker; SC=$V2/scripts; L=$V2/logs
SINCE=${SINCE:?SINCE=YYYYmmdd_HHMM}; N=${INSTANCE:-2}
FACES=${FACES:-"AE BW BE CW CE DW DE EW"}; CONDS=${CONDS:-"yaw15 drift04"}
pick(){ for d in $(ls -d "$L"/*_"$1"_"$2"_"$3" 2>/dev/null | sort -r); do
          b=$(basename "$d"); [[ "$b" < "$SINCE" ]] && continue
          grep -q "VERDICT" "$d/report.txt" 2>/dev/null && { echo "$d"; return; }
        done; }
OUT=$L/faces_final_$(date +%Y%m%d_%H%M%S).txt; : > "$OUT"
for f in $FACES; do for c in $CONDS; do for w in off on; do
  d=$(pick "$f" "$c" "$w")
  if [ -z "$d" ]; then
    a=${f:0:1}; s=${f:1:1}; rot=0; dr=0
    case $c in yaw15) rot=1.5;; drift04) dr=0.004;; esac
    for try in 1 2; do
      echo "[fill] $f $c $w (try $try) on instance $N"
      INSTANCE=$N "$SC/wk_fly.sh" "${f}_${c}_${w}" "$w" "$rot" "$dr" "R1${a}9:${s}@0.68 R1${a}1 R1${a}1@1.265 R1${a}9" > "$L/.fill_i$N.log" 2>&1
      d=$(pick "$f" "$c" "$w"); [ -n "$d" ] && break
    done
  fi
  echo "$f $c $w ${d:-MISSING}" >> "$OUT"
done; done; done
"$SC/stop_sitl_instance.sh" $N > /dev/null
echo "final list: $OUT"; grep -c MISSING "$OUT"
