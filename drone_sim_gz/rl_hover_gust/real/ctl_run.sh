#!/bin/bash
# Hover flight WITH the east correction (2026-11-04) = step2_run.sh with real_ctl_safe.py in place of real_flight_safe.py.
# Same hover-only v13 arguments, same pre-check and plan check, same own mavsdk_server. The correction runs only inside v13's
# take-off hover (ctl_patch.py); everything else (take-off, 1 cm step, landing, safety) is unchanged.
#   bash ctl_run.sh <px4|pd|ppo> [ALT=1.0] [HOLD_S=40]
CTRL=$1; ALT=${2:-1.0}; HOLD=${3:-40}; T=/data/claude_collect/tools_safe
case $CTRL in
  px4) CA="--ctrl zero";;
  pd)  CA="--ctrl ppo --policy $T/policies/pd_imit.json";;
  ppo) CA="--ctrl ppo --policy $T/policies/sur_bcrand_s2.json";;
  *) echo "!! controller must be px4, pd or ppo (got '$CTRL') - nothing started"; exit 3;;
esac
ARGS="--ignore-mission-plan --takeoff-hover-time $HOLD --directions forward --distance 0.01 --hover-time 1 --no-return-between --altitude $ALT --stop-on-fail --skip-preland"
bash $T/step2_check.sh ctl_$(date -u +%H%M%S) || { echo "!! pre-check NOT READY - nothing started"; exit 3; }
python3 $T/hover_plan_check.py /home/root -- $ARGS || { echo "!! plan check failed - nothing started"; exit 3; }
D=${STEP2_DIR:-/data/claude_collect/ctl_${CTRL}_$(date -u +%Y%m%d_%H%M%S)}; mkdir -p $D; cd $D
echo "controller $CTRL ($CA), log + result in $D"
python3 $T/real_ctl_safe.py $CA --log-dir $D/ctl --v13 /home/root --own-server udp://:14551 --server-bin /home/mavsdk_server \
  --result $D/mission_result.json -- $ARGS 2>&1 | tee $D/launcher.log
rc=${PIPESTATUS[0]}; echo "LAUNCHER_EXIT=$rc" | tee -a $D/launcher.log
echo "### mavsdk_server after the end"; pgrep -a -x mavsdk_server || echo "none (the own server ended with the launcher)"
exit $rc
