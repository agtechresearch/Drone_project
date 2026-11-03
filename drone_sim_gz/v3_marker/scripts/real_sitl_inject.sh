#!/bin/bash
# Fault injection of the real-mode SITL harnesses (logs/run_real_voxl_sitl.sh, logs/run_real_sitl.sh), every step with its own
# exit code (scripts/inject_lib.sh) - 2026-11-01 codex runner-exit-status reply-10: a wait that ran out, an empty PID, a failed
# PAUSE / OFFSET write or a failed SIGINT went unnoticed and the case could still pass on its mission result.
#   real_sitl_inject.sh <inject> <instance> <run dir>     inject: none | voxlstop | offset | preflight | sigint | failprobe
#   failprobe = a NEGATIVE CONTROL: waits for the first stage like voxlstop, then a step that must fail - the harness has to mark
#   that case NOT OK and end nonzero although the flight itself is normal
# Waits for its trigger line in <run dir>/mission.log (at most INJ_WAIT_S, default 300 s), acts, prints the steps; exit = number
# of failed steps (0 = the injection really happened). The harness fails the case on a nonzero exit.
INJ=$1; N=$2; D=$3; W=${INJ_WAIT_S:-300}
. "$(dirname "$0")/inject_lib.sh"
wait_log(){ local pat=$1 t0=$SECONDS; while [ $((SECONDS - t0)) -lt $W ]; do grep -a -q -- "$pat" "$D/mission.log" 2>/dev/null && return 0; sleep 0.2; done
            echo "[inject] '$pat' not in mission.log within $W s"; return 1; }
pypid(){ docker exec starling-flight bash -c "for q in \$(pgrep -x python3); do tr '\\0' ' ' < /proc/\$q/cmdline | grep -q real_flight_safe.py && echo \$q; done" | head -1; }
case $INJ in
  none) ;;
  voxlstop)  step "first STAGE in the log" wait_log ">>> STAGE" || inj_end; sleep 10
             step "VOXL relay paused" docker exec starling-gz touch /tmp/voxl_relay_$N.PAUSE;;
  offset)    step "take-off END in the log" wait_log "\[goto takeoff\] \[END\]" || inj_end
             step "VOXL position offset 0.3 0.3 m" docker exec starling-gz bash -c "echo '0.3 0.3' > /tmp/voxl_relay_$N.OFFSET";;
  preflight|sigint)
             # the python process itself (not `timeout` / `bash -c` in front of it: they do not pass SIGINT on; 1915 try 2)
             if [ $INJ = preflight ]; then step "[4/7] in the log" wait_log "\[4/7\]" || inj_end
             else step "first STAGE in the log" wait_log ">>> STAGE" || inj_end; sleep 8; fi
             p=$(pypid); step "launcher pid found ($p)" test -n "$p" || inj_end
             step "SIGINT $p" docker exec starling-flight kill -INT $p;;
  failprobe) step "first STAGE in the log" wait_log ">>> STAGE" || inj_end
             step "deliberately failing step (negative control)" false;;
  *) step "known injection '$INJ'" false;;
esac
inj_end
