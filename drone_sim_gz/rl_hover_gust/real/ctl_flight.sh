#!/bin/bash
# Comparison hover flight (2026-11-04) = hover_flight.sh with the controller: px4 (PX4 alone, correction 0), pd (PD-imitation
# net, as in Gazebo), ppo (surrogate PPO bcrand). Run on the VOXL in /data/claude_collect/tools_safe:
#   bash ctl_flight.sh <px4|pd|ppo> [ALT=1.0] [HOLD_S=40]
# Fan times: in a SECOND ssh window, before take-off:  python3 /data/claude_collect/tools_safe/fan_mark.py   (Enter = fan ON/OFF)
# Steps as hover_flight.sh: pre-check READY -> tag-detector start only -> record tag / VIO / PX4 pipes -> ctl_run.sh -> stop
# recording, tag-detector back, configs compared -> tag summary. Everything in /data/claude_collect/ctl_<ctrl>_<UTC time>/.
CTRL=${1:-}; ALT=${2:-1.0}; HOLD=${3:-40}; T=/data/claude_collect/tools_safe
case $CTRL in px4|pd|ppo) ;; *) echo "usage: bash ctl_flight.sh <px4|pd|ppo> [ALT] [HOLD_S] - nothing started"; exit 3;; esac
D=/data/claude_collect/ctl_${CTRL}_$(date -u +%Y%m%d_%H%M%S)
mkdir -p $D/rec; echo "controller $CTRL, hover $ALT m, $HOLD s -> $D"
sha256sum $T/ctl_flight.sh $T/ctl_run.sh $T/real_ctl_safe.py $T/ctl_patch.py $T/policy_runtime.py $T/real_flight_safe.py \
  $T/safety_patch.py $T/policies/*.json /home/root/path_flight_phase1_v13.py > $D/code_sha256.txt 2>&1
state(){ { systemctl is-enabled voxl-tag-detector; systemctl is-active voxl-tag-detector
           sha256sum /etc/modalai/voxl-tag-detector.conf /etc/modalai/tag_locations.conf /etc/modalai/extrinsics.conf; } > $D/state_$1.txt 2>&1; }
RP=""; WAS_ACTIVE=$(systemctl is-active voxl-tag-detector)
cleanup(){ [ -n "$RP" ] && { touch $D/rec/STOP; wait $RP 2>/dev/null; RP=""; }
  [ "$WAS_ACTIVE" != active ] && systemctl stop voxl-tag-detector
  state post; diff <(tail -3 $D/state_pre.txt) <(tail -3 $D/state_post.txt) > /dev/null && echo "configs unchanged" || echo "!! configs CHANGED - check $D/state_*.txt"
  echo "tag-detector now: $(systemctl is-active voxl-tag-detector) / $(systemctl is-enabled voxl-tag-detector) (before: $WAS_ACTIVE)"; }
trap 'cleanup; exit 130' INT TERM
bash $T/step2_check.sh ctl_pre || { echo "!! pre-check NOT READY - nothing started"; exit 3; }
state pre
systemctl start voxl-tag-detector
for i in $(seq 1 20); do [ -d /run/mpa/tag_detections ] && break; sleep 0.5; done
[ -d /run/mpa/tag_detections ] || { echo "!! tag_detections pipe did not appear - nothing flown"; cleanup; exit 3; }
PIPES=tag_detections,vvhub_body_wrt_local,px4_vehicle_local_position; [ "${NO_VIDEO:-1}" = 1 ] || PIPES=$PIPES,tracking_front_tag_overlay
python3 $T/hover_record.py $D/rec --pipes $PIPES --max-s 300 > $D/rec/record.log 2>&1 & RP=$!
sleep 2; head -1 $D/rec/record.log
STEP2_DIR=$D/flight bash $T/ctl_run.sh $CTRL $ALT $HOLD; FRC=$?
sleep 3; cleanup; trap - INT TERM
tail -1 $D/rec/record.log
python3 $T/hover_summary.py $D/rec
grep -a "\[ctl\] \(active\|off\|end\)" $D/flight/launcher.log
echo "FLIGHT_EXIT=$FRC  (0 complete, 2 a stage failed, 10 operator take-over / hand-over, 3 not started) - files in $D"
