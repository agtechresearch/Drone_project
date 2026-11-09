#!/bin/bash
# 2026-10-31 (codex review): LEGACY entry point, not part of the current pipeline (see scripts/ENTRYPOINTS.md). Known problem: runs the v2_wallkeep mission and writes v2 logs although it sits in v3, codex legacy-workspace-entrypoints
[ "${LEGACY_OK:-0}" = "1" ] || { echo "!! legacy entry point $(basename "$0"): runs v2_wallkeep, not v3. Set LEGACY_OK=1 to run anyway (see scripts/ENTRYPOINTS.md)."; exit 3; }
# Condition matrix for the wall-keep comparison (2026-10-29): A/B no error, C/D VIO heading 1.5 deg, E/F east drift 0.4 cm/s;
# off = sensor observe only, on = correction applied. Same route for all. Run dirs are listed in logs/batch_<ts>.txt.
V2=$HOME/starling_sim/v2_wallkeep; ROUTE=${ROUTE:-"R1C9:W@0.68 R1C1 R1C1@1.265 R1C9"}
LIST=$V2/logs/batch_$(date +%Y%m%d_%H%M%S).txt
CONDS=${CONDS:-"A_none_off off 0 0|B_none_on on 0 0|C_yaw15_off off 1.5 0|D_yaw15_on on 1.5 0|E_drift04_off off 0 0.004|F_drift04_on on 0 0.004"}
IFS="|" read -r -a CL <<< "$CONDS"
for c in "${CL[@]}"; do
  [ -z "$c" ] && continue
  set -- $c
  echo "##### $1"
  "$V2/scripts/wk_fly.sh" "$1" "$2" "$3" "$4" "$ROUTE" 2>&1 | tail -4
  cat "$V2/logs/.last_run" >> "$LIST"
done
"$V2/scripts/stop_sitl_instance.sh" 2
echo "list: $LIST"
