#!/bin/bash
# Hover-gust batch, stage A (2026-11-03; review/CODEX_PPO_HOVER_GUST_PLAN_2026-11-03.md §6 A). Run on the HOST.
#   hover_gust_run.sh <batch name> [episodes "none:0,gust:1,..."]   (default 10: N G G N G G N G G N)
# Instance 2 only (0 / 1 protected), empty world "default" (ApplyLinkWrench declared in PX4 default.sdf), starling2_tof (m3),
# zero VIO injection, EKF2_EV_CTRL 7 (real setting) through px4_param_guard.sh, restored on every exit. ONE boot for the whole
# batch: every episode takes off, hovers, gets its gusts, lands and disarms on the same instance (the reset under test).
# Processes: mavsdk_server (starling-flight, 50053), sim_clock_udp.py (14602), gt_logger.py, gust_scheduler.py (starling-gz),
# hover_gust_worker.py (starling-flight). Output logs/hg_<name>/ ; judgement hover_gust_eval.py -> summary.txt; exit code: see finish().
set -u
cd /home/dronesim/starling_sim/v3_marker; SC=scripts; N=2; G=$SC/px4_param_guard.sh
# 2026-11-03 hg_a10: PX4's simulated battery drains to SIM_BAT_MIN_PCT 50 % within SIM_BAT_DRAIN 60 s of every armed flight
# (reset to 100 % on disarm); after 4 flights arming was denied "Battery unhealthy". Sim-only parameter, nothing of the real
# vehicle: SIM_BAT_DRAIN 86400 for the batch through a SECOND guard record (the guard holds one parameter per record) in
# logs/.guard_hover_bat, restored on every exit like EKF2_EV_CTRL; a record left by a killed batch is recovered at the next start.
GB=$PWD/logs/.guard_hover_bat; mkdir -p "$GB"
NAME=${1:?batch name}; EPS=${2:-none:0,gust:1,gust:2,none:0,gust:3,gust:4,none:0,gust:5,gust:6,none:0}
# stage B/C (2026-11-03): CFG=<config> (default config/hover_gust.json), PLAN=<plan.json> (episodes with "ctrl"; replaces the
# episode spec), TRAINER=1 runs experiments/train_hover_gust.py (host venv) next to the worker on this batch (TRAIN_ARGS extra args)
CFG=${CFG:-$PWD/config/hover_gust.json}; D=$PWD/logs/hg_$NAME; FD=/home/dronesim/workspace/v3_marker/logs/hg_$NAME
FCFG=${CFG/#\/home\/dronesim\/starling_sim/\/home\/dronesim\/workspace}
[ -e "$D" ] && { echo "!! $D exists"; exit 2; }
mkdir -p "$D"; cp "$CFG" "$D/config.json"
WORLD=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['world'])" "$CFG")
POSE=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['spawn_pose'])" "$CFG")
python3 -c "
import sys, json; sys.path.insert(0, '.')
from rl_demo.contracts import load_config, parse_episodes, gust_schedule
c = load_config(sys.argv[1])
plan = json.load(open(sys.argv[4])) if sys.argv[4] else {'episodes': parse_episodes(sys.argv[2])}
for i, e in enumerate(plan['episodes']):
    assert e['id'] == i, 'episode ids must be 0..n-1'
    gust_schedule(c, e['cond'], e['seed'])      # every schedule must fit before anything starts
json.dump(plan, open(sys.argv[3], 'w'), indent=1)" "$CFG" "$EPS" "$D/plan.json" "${PLAN:-}" || { echo "!! plan rejected"; exit 2; }
say(){ echo "$(date +%H:%M:%S) $*" | tee -a "$D/run.log"; }
( cd /home/dronesim/starling_sim && sha256sum v3_marker/scripts/hover_gust_run.sh v3_marker/scripts/hover_gust_worker.py \
    v3_marker/scripts/gust_scheduler.py v3_marker/scripts/hover_gust_eval.py v3_marker/rl_demo/contracts.py \
    v3_marker/rl_demo/policy_runtime.py v3_marker/rl_demo/hover_metrics.py v3_marker/scripts/hg_rc_stat.sh v3_marker/experiments/train_hover_gust.py ${CFG#/home/dronesim/starling_sim/} \
    v3_marker/config/hover_gust.json v3_marker/scripts/run_wk_instance.sh v3_marker/scripts/gt_logger.py \
    v3_marker/scripts/sim_clock_udp.py v3_marker/gz/models/starling2_tof/model.sdf \
    PX4-Autopilot/Tools/simulation/gz/worlds/$WORLD.sdf ) > "$D/code_sha256.txt"
GZE="-e GZ_PARTITION=starling_$N"; MAVSDK_BIN=/usr/local/lib/python3.6/dist-packages/mavsdk/bin/mavsdk_server; CLOCK=$((14600 + N))
srv_kill(){ docker exec -i starling-flight bash -c "for q in \$(pgrep -x mavsdk_server); do tr '\0' ' ' < /proc/\$q/cmdline | grep -q -- '-p $((50051 + N)) ' && kill \$q; done; true"; }
DONE=0; WRC=2; ERC=0; TPID=""; . $PWD/$SC/hg_rc_stat.sh
# 2026-11-03 v2 (codex hover-training-completion-status): exit code = worker rc (0/1/2) + 4 eval failed + 8 restore failed
# + 16 trainer failed (TRAINER=1) + 32 entered by a signal; any other non-zero entry (a setup "exit 2") with everything else 0 -> 64.
# run_status.json keeps every component, so a successful cleanup never hides the original failure.
finish(){ [ $DONE = 1 ] && exit $1; DONE=1; local entry=$1 trc=0 tst=""
  if [ -n "$TPID" ]; then for i in $(seq 1 120); do kill -0 $TPID 2>/dev/null || break; sleep 1; done; kill $TPID 2>/dev/null; wait $TPID; trc=$?
    tst=$(ls "$D" | grep -o 'trainer_[a-z]*' | head -1); say "trainer rc $trc ($tst)"; fi
  touch "$D/gust.STOP" "$D/gt.csv.STOP"; docker exec starling-gz touch /tmp/sim_clock_$CLOCK.STOP
  sleep 1; docker exec $GZE starling-gz timeout 20 python3 $PWD/$SC/gust_scheduler.py "$CFG" --clear-only >> "$D/final_clear.log" 2>&1 \
    && say "final clear ok" || say "!! final clear failed (instance $N is stopped below anyway)"
  local prc=0; $G restore >> "$D/param_guard.log" 2>&1 || prc=8
  PX4_GUARD_LOGS=$GB $G restore >> "$D/param_guard.log" 2>&1 || prc=8
  say "param restore rc $prc"
  # the instance is left in the empty world without a mission; a stop removes any persistent force for certain
  $SC/stop_sitl_instance.sh $N >> "$D/stop_after.log" 2>&1 && say "instance $N stopped" || say "!! instance $N stop rc $?"
  local cl=0; srv_kill || cl=1
  local rc=$(compose_rc "$entry" "$WRC" "$ERC" "$prc" "$trc")
  printf '{"entry": %s, "worker": %s, "eval": %s, "restore": %s, "trainer_rc": %s, "trainer_state": "%s", "server_kill": %s, "exit": %s}\n' \
    "$entry" "$WRC" "$ERC" "$prc" "$trc" "$tst" "$cl" "$rc" > "$D/run_status.json"
  say "EXIT=$rc"; echo "EXIT=$rc" >> "$D/run.log"; exit $rc
}
trap 'finish 130' INT TERM; trap 'finish $?' EXIT

boot(){ local tag=$1 ok=""
  for k in 1 2 3; do $SC/stop_sitl_instance.sh $N >> "$D/stop_before.log" 2>&1 && { ok=1; break; }; sleep 3; done
  [ -n "$ok" ] || { say "!! instance $N not stopped"; return 1; }; srv_kill
  docker exec starling-gz bash -c "echo '{\"rot_deg\": 0, \"drift_n\": 0, \"drift_e\": 0, \"step_e\": 0, \"step_at\": 1e9}' > /tmp/wk_inject_$N.json"
  local t0=$(date +%s.%N)
  setsid docker exec starling-gz bash -c "$PWD/$SC/run_wk_instance.sh $N $WORLD headless $POSE" > "$D/sitl_$tag.log" 2>&1 < /dev/null &
  for i in $(seq 1 150); do grep -a -q "New NED origin\|EXIT=" "$D/sitl_$tag.log" && break; sleep 1; done
  grep -a -q "New NED origin" "$D/sitl_$tag.log" || { say "!! no boot ($tag)"; return 1; }
  for i in $(seq 1 30); do docker exec starling-gz pgrep -f "[s]et_global_origin.py udpin:0.0.0.0:$((14540 + N))" >/dev/null || break; sleep 1; done
  say "booted ($tag) in $(python3 -c "import sys,time; print('%.1f' % (time.time()-float(sys.argv[1])))" $t0) s"
}

say "batch $NAME: instance $N world $WORLD pose $POSE episodes $EPS"
$G recover >> "$D/param_guard.log" 2>&1 || { say "!! previous parameter restore pending/failed"; exit 2; }
PX4_GUARD_LOGS=$GB $G recover >> "$D/param_guard.log" 2>&1 || { say "!! previous SIM_BAT_DRAIN restore pending/failed"; exit 2; }
boot pre || exit 2
$G set EKF2_EV_CTRL 7 "$N" >> "$D/param_guard.log" 2>&1 || { say "!! EKF2_EV_CTRL not set"; exit 2; }
PX4_GUARD_LOGS=$GB $G set SIM_BAT_DRAIN 86400.0000 "$N" >> "$D/param_guard.log" 2>&1 || { say "!! SIM_BAT_DRAIN not set"; exit 2; }
boot main || exit 2                      # parameters are read at boot
docker exec starling-gz bash -c "cd ~/PX4-Autopilot/build/px4_sitl_default/bin; for p in EKF2_EV_CTRL SIM_BAT_DRAIN SIM_BAT_MIN_PCT EKF2_HGT_REF MPC_XY_P MPC_XY_VEL_P_ACC MPC_XY_VEL_I_ACC MPC_THR_HOVER COM_DISARM_LAND; do ./px4-param --instance $N show \$p | grep -a \"\$p \"; done" > "$D/params.txt" 2>&1
grep -q "EKF2_EV_CTRL.*: 7" "$D/params.txt" || { say "!! EKF2_EV_CTRL is not 7 after boot: $(grep EV_CTRL "$D/params.txt")"; exit 2; }
grep -q "SIM_BAT_DRAIN.*: 86400" "$D/params.txt" || { say "!! SIM_BAT_DRAIN is not 86400 after boot: $(grep SIM_BAT_DRAIN "$D/params.txt")"; exit 2; }
docker exec -d starling-flight bash -c "$MAVSDK_BIN -p $((50051 + N)) udp://:$((14540 + N)) > ~/mavsdk_server_i$N.log 2>&1"
for i in $(seq 1 10); do docker exec starling-flight python3 -c "import socket,sys; s=socket.socket(); s.settimeout(0.5); sys.exit(0 if s.connect_ex(('127.0.0.1', $((50051 + N)))) == 0 else 1)" && break; sleep 1; done
docker exec -d $GZE starling-gz bash -c "python3 $PWD/$SC/sim_clock_udp.py $WORLD $CLOCK 50 > $D/sim_clock.log 2>&1"
docker exec -d $GZE starling-gz bash -c "python3 $PWD/$SC/gt_logger.py starling2_tof_$N $D/gt.csv > $D/gt_logger.log 2>&1"
docker exec -d $GZE starling-gz bash -c "python3 $PWD/$SC/gust_scheduler.py $CFG $D > $D/gust_scheduler.log 2>&1"
sleep 4
grep -q "!!" "$D/gust_scheduler.log" && { say "!! scheduler: $(cat $D/gust_scheduler.log)"; exit 2; }
if [ "${TRAINER:-0}" = 1 ]; then
  $PWD/venv/bin/python $PWD/experiments/train_hover_gust.py --batch "$D" --config "$CFG" ${TRAIN_ARGS:-} > "$D/trainer.log" 2>&1 & TPID=$!
  say "trainer pid $TPID"
fi
say "worker start"
docker exec -i starling-flight bash -c "timeout ${WORKER_TIMEOUT:-1800} python3 /home/dronesim/workspace/v3_marker/scripts/hover_gust_worker.py --port $((50051 + N)) --clock-port $CLOCK --config $FCFG --batch $FD" > "$D/worker.log" 2>&1
WRC=$?; say "worker rc $WRC: $(grep -c ' disarmed' "$D/worker.log") episodes disarmed"
sleep 2; touch "$D/gust.STOP" "$D/gt.csv.STOP"; sleep 2
python3 $SC/hover_gust_eval.py "$D" > "$D/eval.log" 2>&1 || ERC=4
tail -n +1 "$D/summary.txt" 2>/dev/null | tee -a "$D/run.log" || cat "$D/eval.log"
exit 0
