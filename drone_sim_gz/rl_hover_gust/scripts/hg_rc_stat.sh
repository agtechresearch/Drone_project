# Exit-code composition of hover_gust_run.sh (2026-11-03 v2), sourced by it and by tests/test_status_propagation.py. (2026-11-03 19:0x: renamed from run_status.sh, which is the wk_fly.sh status script and had been overwritten by mistake)
# compose_rc <entry> <worker> <eval> <restore> <trainer>: worker rc (0/1/2) + eval (0/4) + restore (0/8) + 16 trainer rc != 0
# + 32 entered by a signal (entry >= 128); a non-zero entry with every component 0 (setup "exit 2" after the worker ran) -> 64.
compose_rc(){ local entry=$1 w=$2 e=$3 p=$4 t=$5 rc
  rc=$((w + e + p)); [ "$t" != 0 ] && rc=$((rc + 16)); [ "$entry" -ge 128 ] && rc=$((rc + 32))
  [ "$entry" != 0 ] && [ $rc = 0 ] && rc=64
  echo $rc; }
