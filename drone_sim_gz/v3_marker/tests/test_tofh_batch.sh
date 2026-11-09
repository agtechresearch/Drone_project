#!/bin/bash
# Offline check of wk_tofh_batch.sh's final count (2026-10-31, codex runner-exit-status reply-2): fake wk_fly.sh / tagfix_eval.py
# per case, no flight. exit code = number of failures.
REAL=$(cd "$(dirname "$0")/../scripts" && pwd); F=0
t(){ # name expect_exit lanes wk_fly_body [eval_rc]   (every case starts with a stale .last_run_i2/3 from an older run)
  local T; T=$(mktemp -d); mkdir -p $T/sc $T/logs $T/logs/old_run; echo $T/logs/old_run > $T/logs/.last_run_i2; echo $T/logs/old_run > $T/logs/.last_run_i3
  printf '#!/bin/bash\nL=%s; N=$INSTANCE; tag=$1\n%s\n' "$T/logs" "$4" > $T/sc/wk_fly.sh; chmod +x $T/sc/wk_fly.sh
  printf 'import sys; sys.exit(%s)\n' "${5:-0}" > $T/sc/tagfix_eval.py
  TOFH_SC=$T/sc TOFH_LOGS=$T/logs TOFH_LANES="$3" TOFH_STAGGER=0 THS=1 bash "$REAL/wk_tofh_batch.sh" t > $T/out 2>&1; rc=$?
  grep -q old_run $T/logs/tofh_t_status.txt && { echo "FAIL  $1: stale run dir reported"; F=$((F+1)); }
  if [ $rc = "$2" ]; then echo "PASS  $1 -> $rc  ($(tail -2 $T/out | head -1))"; else echo "FAIL  $1 -> $rc (expected $2)"; cat $T/out; F=$((F+1)); fi
  rm -rf $T; }
OK='d=$L/run_${N}_$tag; mkdir -p $d; echo $d > $L/.last_run_i$N; exit 0'
TWO="2 0 0 0 0,0
3 0 0 0 0,0"
t "two lanes ok"                       0 "$TWO" "$OK"
t "one run safety landing (13)"        1 "$TWO" 'd=$L/run_$N; mkdir -p $d; echo $d > $L/.last_run_i$N; [ $N = 3 ] && exit 13; exit 0'
t "dies before naming its run dir (stale old_run not reused)" 1 "$TWO" '[ $N = 3 ] && exit 2; d=$L/run_$N; mkdir -p $d; echo $d > $L/.last_run_i$N; exit 0'
t "lane itself killed (row missing)"   2 "$TWO" '[ $N = 3 ] && kill -9 $PPID; d=$L/run_$N; mkdir -p $d; echo $d > $L/.last_run_i$N; exit 0'
t "evaluation fails on every run"      2 "$TWO" "$OK" 3
t "truth verdict incomplete (14)"      1 "$TWO" 'd=$L/run_$N; mkdir -p $d; echo $d > $L/.last_run_i$N; [ $N = 2 ] && exit 14; exit 0'
echo "$F failure(s)"; exit $F
