#!/bin/bash
# Hover flight WITH PROPS + simultaneous recording (2026-11-03). Run on the VOXL in /data/claude_collect/tools_safe:
#   bash hover_flight.sh [ALT=1.0] [HOLD_S=30]
# 1. pre-check (step2_check.sh, must be READY)   2. voxl-tag-detector: start only (never enable; stopped again at the end,
# also on Ctrl+C), the tag pipe must appear   3. hover_record.py records tag_detections + vvhub_body_wrt_local +
# px4_vehicle_local_position   4. the hover flight: step2_run.sh ALT HOLD (v13 unchanged, hover-only plan check, own server,
# operator take-over detection)   5. recording stopped 3 s after the flight, tag-detector stopped, configs checked unchanged,
# hover_summary.py prints the tag numbers. Everything in /data/claude_collect/hover_<UTC time>/ (flight/, rec/, state_*.txt).
ALT=${1:-1.0}; HOLD=${2:-30}; T=/data/claude_collect/tools_safe; D=/data/claude_collect/hover_$(date -u +%Y%m%d_%H%M%S)
mkdir -p $D/rec; echo "hover $ALT m, $HOLD s -> $D"
state(){ { systemctl is-enabled voxl-tag-detector; systemctl is-active voxl-tag-detector
           sha256sum /etc/modalai/voxl-tag-detector.conf /etc/modalai/tag_locations.conf /etc/modalai/extrinsics.conf; } > $D/state_$1.txt 2>&1; }
RP=""; WAS_ACTIVE=$(systemctl is-active voxl-tag-detector)
cleanup(){ [ -n "$RP" ] && { touch $D/rec/STOP; wait $RP 2>/dev/null; RP=""; }
  [ "$WAS_ACTIVE" != active ] && systemctl stop voxl-tag-detector
  state post; diff <(tail -3 $D/state_pre.txt) <(tail -3 $D/state_post.txt) > /dev/null && echo "configs unchanged" || echo "!! configs CHANGED - check $D/state_*.txt"
  echo "tag-detector now: $(systemctl is-active voxl-tag-detector) / $(systemctl is-enabled voxl-tag-detector) (before: $WAS_ACTIVE)"; }
trap 'cleanup; exit 130' INT TERM
bash $T/step2_check.sh hover_pre || { echo "!! pre-check NOT READY - nothing started"; exit 3; }
state pre
systemctl start voxl-tag-detector
for i in $(seq 1 20); do [ -d /run/mpa/tag_detections ] && break; sleep 0.5; done
[ -d /run/mpa/tag_detections ] || { echo "!! tag_detections pipe did not appear - nothing flown"; cleanup; exit 3; }
# tag overlay video (tracking_front_tag_overlay, ~1 MB per frame at ~5 fps, ~0.5 GB per flight); NO_VIDEO=1 leaves it out
PIPES=tag_detections,vvhub_body_wrt_local,px4_vehicle_local_position; [ "${NO_VIDEO:-0}" = 1 ] || PIPES=$PIPES,tracking_front_tag_overlay
python3 $T/hover_record.py $D/rec --pipes $PIPES --max-s 300 > $D/rec/record.log 2>&1 & RP=$!
sleep 2; head -1 $D/rec/record.log
STEP2_DIR=$D/flight bash $T/step2_run.sh $ALT $HOLD; FRC=$?
sleep 3; cleanup; trap - INT TERM
tail -1 $D/rec/record.log
python3 $T/hover_summary.py $D/rec
echo "FLIGHT_EXIT=$FRC  (0 complete, 2 a stage failed, 10 operator take-over / hand-over) - files in $D"
