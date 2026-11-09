#!/bin/bash
# Release runner for the Gazebo same-wind comparison (2026-11-09; general form of logs/run_evsur.sh without its wait on the
# 2026-11-04 training driver). Run on the HOST in the development v3_marker tree, after experiments/split_eval_plan.py:
#   bash scripts/run_eval_batches.sh <plan_dir>        (reads <plan_dir>/batches.txt and <plan_dir>/plan_<name>.json)
# Each batch = one SITL boot on instance 2 (scripts/hover_gust_run.sh) -> logs/hg_<name>/. Batches with a summary are skipped.
# Continues on batch rc 0/1/4/5 (episode-level problems, judged later); stops on restore (8), signal (32), setup (64), other.
# CHECK_ONLY=1 only checks the paths (no SITL). Nothing else may use SITL instance 2 meanwhile. Afterwards: surrogate/agg_evsur.py logs/hg_<prefix>_* (+ agg_evsur_common.py).
cd "$(dirname "$0")/.." || exit 2; PD=$(cd "${1:?plan dir}" && pwd) || exit 2; ts(){ date +%H:%M:%S; }
[ -f "$PD/batches.txt" ] || { echo "no $PD/batches.txt"; exit 2; }
# hover_gust_run.sh changes to its own fixed project root and writes logs/hg_<name>/ there: the skip check below and the
# aggregation afterwards read logs/ of THIS root, so both must be the same directory (run from the development v3_marker tree).
HR=$(sed -n 's/^cd \([^;]*\);.*/\1/p' scripts/hover_gust_run.sh | head -1)
[ "$(cd "$HR" 2>/dev/null && pwd -P)" = "$(pwd -P)" ] || { echo "!! hover_gust_run.sh writes to '$HR', this runner is in '$(pwd -P)': run from that tree"; exit 2; }
[ "${CHECK_ONLY:-0}" = 1 ] && { echo "paths OK: batches write, skip check and aggregation all use $(pwd -P)/logs"; exit 0; }
CFG=${CFG:-$PWD/config/hover_gust_rl_wide.json}
while IFS='|' read -r name xp; do
  [ -z "$name" ] && continue
  [ -f logs/hg_$name/summary.json ] && { echo "$(ts) $name already done, skip"; continue; }
  PLAN=$PD/plan_$name.json CFG=$CFG scripts/hover_gust_run.sh "$name" < /dev/null > "$PD/run_$name.out" 2>&1; r=$?
  echo "$(ts) $name rc $r"
  case $r in 0|1|4|5) ;; *) echo "$(ts) STOP at $name rc $r"; echo EXIT=1; exit 1;; esac
done < "$PD/batches.txt"
echo "$(ts) all batches done"; echo EXIT=0
