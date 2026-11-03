#!/bin/bash
# Fault-injection steps of a batch script with their own exit codes (2026-11-01, codex runner-exit-status reply-8: the phase 12/13
# injectors went on after a failed wait / kill / SIGSTOP / SIGCONT and ended with the rc 0 of a final echo or grep).
# Source it inside the injector's subshell:
#   ( . $SC/inject_lib.sh
#     step "wait for the apply"  after_apply 3            || inj_end     # a failed precondition ends the injector here
#     p=$(...); step "pid found" test -n "$p"             || inj_end
#     step "SIGSTOP $p" docker exec starling-flight kill -STOP $p         # a failed action is counted, the rest goes on
#     inj_end ) > logs/${B}_inject_i4.log 2>&1 & I4=$!
#   ... wait $I4; echo "inject_i4 $?" >> logs/${B}_children.txt          (batch_expect.py: a nonzero injector = mismatch)
# Every step prints "[inject] <time> step '<what>' rc <rc>"; the subshell exits with the number of failed steps (max 100).
INJ_FAIL=0
step(){ local what=$1; shift; "$@"; local r=$?
        echo "[inject] $(date +%T.%N | cut -c1-12) step '$what' rc $r"; [ $r = 0 ] || INJ_FAIL=$((INJ_FAIL + 1)); return $r; }
inj_end(){ echo "[inject] $(date +%T.%N | cut -c1-12) end: $INJ_FAIL failed step(s)"; [ $INJ_FAIL -gt 100 ] && INJ_FAIL=100; exit $INJ_FAIL; }
