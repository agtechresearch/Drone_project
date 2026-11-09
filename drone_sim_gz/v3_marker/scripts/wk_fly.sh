#!/bin/bash
# Wall-keep test flight on instance N (INSTANCE, default 2; run on the HOST). v3_marker, 2026-10-29. Instances 0/1 are not touched.
#   wk_fly.sh <tag> <on|off> <rot_deg> <drift_e_m_s> --survey "R1C W 10 1 0.68,1.265" [extra grid_mission args]
#   wk_fly.sh <tag> <on|off> <rot_deg> <drift_e_m_s> "<route>"                         [extra grid_mission args]
# Always restarts instance 2 (starling2_tof, QVIX, headless, spawn at the route's first cell facing N) with the VIO
# error injection in /tmp/wk_inject_2.json (rot_deg = VIO frame rotation, drift_e = east drift rate once airborne), then
# flies wk_mission.py on the Gazebo sim clock (sim_clock_udp.py) and records:
#   logs/<TS>_<tag>/  sitl.log mission.log truth.txt plan.json report.txt  wk.csv (correction layer, per depth frame)
#                     sense.csv (measured vs TRUE distance/angle to the rack face, per depth frame)
#                     cam/ (front camera JPEG, 15 fps sim time)  tof/ (false-colour depth + fit)  -> cam.mp4 tof.mp4
# Then wk_report.py builds the comparison figures/videos from one or more run dirs.
set -u
N=${INSTANCE:-2}             # instance number (2..); several wk_fly.sh may run in parallel with different INSTANCE
# 2026-10-31 (codex mission-process-cleanup): instances 0 and 1 are the protected ones (run_sitl.sh / workbench)
case "$N" in 2|3|4|5) ;; *) echo "!! INSTANCE=$N not allowed (2..5)"; exit 2;; esac
# 2026-10-31 (codex boolean-env-flags): the options were tested with ${VAR:+...}, so WK_LOC=0 switched the feature ON. Every
# switch is normalised here: unset / "" / 0 / false / no / off -> off (""), 1 / true / yes / on -> on ("1"), anything else stops.
for v in WK_TAGS WK_RECHECK WK_LOC WK_TOF_HEADING WK_TOF_HOLD WK_ALIGN WK_SPAWN_ROT WK_SPAWN_ALIGN NO_VIDEO WK_TOF_RAW; do
  case "${!v:-}" in ""|0|false|no|off) printf -v "$v" '%s' "";; 1|true|yes|on) printf -v "$v" '%s' "1";;
    *) echo "!! $v=${!v} is not a switch (use 0/1)"; exit 2;; esac
done
for v in CHASE WK_LOC_DRIFTV WK_LOC_EARLY; do   # default-ON switches kept as 0/1 (DRIFTV, EARLY go to the flight process)
  case "${!v:-1}" in 0|false|no|off) printf -v "$v" '%s' 0;; 1|true|yes|on) printf -v "$v" '%s' 1;;
    *) echo "!! $v=${!v} is not a switch (use 0/1)"; exit 2;; esac
done
TAG=${1:?tag}; WK=${2:?on|off}; ROT=${3:?rot_deg}; DRIFT=${4:?drift_e}; shift 4
SIM=$HOME/starling_sim; V2=$SIM/v3_marker; SC=$V2/scripts
if [ "${1:-}" = "--survey" ]; then
  ROUTE=$(python3 "$SC/grid_mission.py" --survey "$2" --check 2>/dev/null | sed -n 's/^survey route: //p'); SURVEY=1; shift 2
else
  ROUTE=${1:?route}; SURVEY=""; shift
fi
[ -n "$ROUTE" ] || { echo "!! route/survey rejected"; exit 1; }
EXTRA="$*"; [ -n "$SURVEY" ] && ! echo "$EXTRA" | grep -q -- "--hover" && EXTRA="$EXTRA --hover 0"
TS=$(date +%Y%m%d_%H%M%S)_i$N; OUT=$V2/logs/${TS}_${TAG}; mkdir -p "$OUT"
MODEL=starling2_tof_$N; MAVSDK_PORT=$((50051 + N)); CLOCK=$((14600 + N)); SENSE_PORT=$((14650 + N))
WORLD=${WK_WORLD:-qvix}; export QVIX_YAW_DEG=${QVIX_YAW_DEG:-0}   # v3: turned world qvix_rot (QVIX_YAW_DEG must match make_tag_world.py)
TAG_PORT=$((14660 + N))
GZE="-e GZ_PARTITION=starling_$N -e QVIX_YAW_DEG=$QVIX_YAW_DEG"
[ "${WK_TAGS:-0}" = "1" ] && EXTRA="$EXTRA --scan"
MAVSDK_BIN=/usr/local/lib/python3.6/dist-packages/mavsdk/bin/mavsdk_server
step(){ echo; echo "=== [$(date +%H:%M:%S)] $*"; }
echo "$TAG wallkeep=$WK rot=$ROT drift_e=$DRIFT route: $ROUTE${WK_ALIGN:+ align=1}${WK_STEP_E:+ step_e=$WK_STEP_E step_at=${WK_STEP_AT:-1e9}}" > "$OUT/condition.txt"
echo "$OUT" > "$V2/logs/.last_run_i$N"     # written at the start too, so a failed run is never reported under the previous run's dir

# disk guard (2026-10-29: a 35 GB idle-SITL ulog filled the shared disk and 27 of 34 runs failed while writing frames)
FREE_GB=$(df -BG --output=avail "$V2/logs" | tail -1 | tr -dc 0-9)
[ "${FREE_GB:-0}" -ge "${MIN_FREE_GB:-5}" ] || { echo "!! 디스크 여유 ${FREE_GB} GB < ${MIN_FREE_GB:-5} GB — 비행하지 않음"; exit 2; }
step "경로 검사"
python3 "$SC/grid_mission.py" --check --route "$ROUTE" --plan-json "$OUT/plan.json" $EXTRA > "$OUT/check.txt" || { cat "$OUT/check.txt"; exit 1; }
POSE=$(python3 "$SC/grid_mission.py" --spawn-pose --route "$ROUTE")
if [ -n "${SPAWN_OFFSET:-}" ]; then   # "dx,dy" world metres: the vehicle is NOT placed exactly on the planned start cell
  POSE=$(python3 -c "import sys; p=[float(v) for v in sys.argv[1].split(',')]; d=[float(v) for v in sys.argv[2].split(',')]; p[0]+=d[0]; p[1]+=d[1]; print(','.join('%g' % v for v in p))" "$POSE" "$SPAWN_OFFSET")
fi
# WK_SPAWN_ROT=1 (2026-10-30): the start heading error is the vehicle itself turned by rot (clockwise, NED) at spawn, as when the
# real vehicle is set down off-axis; vio_relay's rotation about the spawn then makes the VIO start at yaw 0 = the vehicle's facing,
# like the real VIO, so it agrees with an EKF that does not fuse VIO yaw (EKF2_EV_CTRL 7, the real setting). Without it the
# vehicle faces exactly north and only the VIO data is turned (equivalent only while the EKF follows the VIO yaw, EV_CTRL 15).
# WK_SPAWN_ALIGN=1 (2026-10-31, survey worlds): the vehicle is set down facing along the AISLE (map N) +- rot, as a person
# would, whatever the module's yaw in the world. The spawn turn and the VIO frame rotation both become rot - QVIX_YAW_DEG.
ROT_SET=$ROT
if [ "${WK_SPAWN_ALIGN:-0}" = "1" ]; then
  ROT=$(python3 -c "import sys; print('%g' % (float(sys.argv[1]) - float(sys.argv[2])))" "$ROT" "$QVIX_YAW_DEG")
fi
if [ "${WK_SPAWN_ROT:-0}" = "1" ]; then
  POSE=$(python3 -c "import sys, math; p=[float(v) for v in sys.argv[1].split(',')]; p[5]-=math.radians(float(sys.argv[2])); print(','.join('%g' % v for v in p))" "$POSE" "$ROT")
fi
echo "loc=${WK_LOC:-0} tof_heading=${WK_TOF_HEADING:-0} tof_hold=${WK_TOF_HOLD:-0} spawn_rot=${WK_SPAWN_ROT:-0} spawn_align=${WK_SPAWN_ALIGN:-0} rot_set=$ROT_SET rot_eff=$ROT tof_raw=${WK_TOF_RAW:-0} tof_model=${WK_TOF_MODEL:+1} map=${WK_MAP:-design} driftv=$WK_LOC_DRIFTV early=$WK_LOC_EARLY sensors=${WK_SENSORS:-sim} tag_hz=${WK_TAG_HZ:-} tof_hz=${WK_TOF_HZ:-} recheck=${WK_RECHECK:-0} drift_n=${WK_DRIFT_N:-0.0}" >> "$OUT/condition.txt"
echo "world=$WORLD yaw=$QVIX_YAW_DEG tags=${WK_TAGS:-0} spawn=$POSE offset=${SPAWN_OFFSET:-0,0}" >> "$OUT/condition.txt"
echo "model_dir=${WK_MODEL_DIR:-installed}" >> "$OUT/condition.txt"   # 2026-11-01: model variant (WK_MODEL_DIR, run_wk_instance.sh)

step "인스턴스 $N 재시작 (starling2_tof, 주입 rot=$ROT deg drift_e=$DRIFT m/s)"
# 2026-10-31 (codex mission-process-cleanup reply-2): no restart unless the stop confirmed the instance is empty
"$SC/stop_sitl_instance.sh" $N || { echo "!! 인스턴스 $N 정리 실패 (rc $?) - 재기동하지 않음, logs/stop_sitl_$N.log 확인"; exit 2; }
docker exec starling-gz bash -c "echo '{\"rot_deg\": $ROT, \"drift_n\": ${WK_DRIFT_N:-0.0}, \"drift_e\": $DRIFT, \"step_e\": ${WK_STEP_E:-0.0}, \"step_at\": ${WK_STEP_AT:-1e9}}' > /tmp/wk_inject_$N.json"
# up to 3 boots: with several instances booting at once the gz spawn service can time out
# ("gz_bridge Service call timed out", 2026-10-29 lanes 4/5) -> stop the instance and boot again
for boot in 1 2 3; do
  setsid docker exec starling-gz bash -c "${WK_MODEL_DIR:+WK_MODEL_DIR=$WK_MODEL_DIR }$SC/run_wk_instance.sh $N $WORLD headless $POSE" > "$OUT/sitl.log" 2>&1 < /dev/null &
  for i in $(seq 1 150); do grep -a -q "New NED origin" "$OUT/sitl.log" && break; grep -a -q "EXIT=" "$OUT/sitl.log" && break; sleep 1; done
  grep -a -q "New NED origin" "$OUT/sitl.log" && break
  echo "!! SITL 기동 실패 (시도 $boot): $(grep -a ERROR "$OUT/sitl.log" | head -1)"; cp "$OUT/sitl.log" "$OUT/sitl_fail$boot.log"
  "$SC/stop_sitl_instance.sh" $N || { echo "!! 인스턴스 $N 정리 실패 (rc $?) - 재기동하지 않음"; exit 2; }; sleep $((10 * boot))
done
grep -a -q "New NED origin" "$OUT/sitl.log" || { echo "!! SITL 기동 3회 실패"; exit 1; }
for i in $(seq 1 30); do docker exec starling-gz pgrep -f "[s]et_global_origin.py udpin:0.0.0.0:$((14540 + N))" >/dev/null || break; sleep 1; done

step "mavsdk_server, 시뮬 시계, 진실값, 센서, 카메라 시작"
docker exec starling-flight pkill -f "[m]avsdk_server -p $MAVSDK_PORT " 2>/dev/null   # stale server from a previous run would hold the port
# 2026-10-31 survey3 sv4: the old server was still releasing the port 1 s after pkill, the new one failed to bind ("Address already
# in use") and the mission waited for it until the 25 min timeout -> wait for a free port, then check the new server listens
for i in $(seq 1 20); do docker exec starling-flight bash -c "python3 -c \"import socket,sys; s=socket.socket(); s.settimeout(0.5); sys.exit(0 if s.connect_ex(('127.0.0.1', $MAVSDK_PORT)) == 0 else 1)\"" || break; sleep 1; done
for try in 1 2 3; do
  docker exec -d starling-flight bash -c "$MAVSDK_BIN -p $MAVSDK_PORT udp://:$((14540 + N)) > ~/mavsdk_server_i$N.log 2>&1"
  for i in $(seq 1 10); do docker exec starling-flight bash -c "python3 -c \"import socket,sys; s=socket.socket(); s.settimeout(0.5); sys.exit(0 if s.connect_ex(('127.0.0.1', $MAVSDK_PORT)) == 0 else 1)\"" && break; sleep 1; done
  docker exec starling-flight bash -c "python3 -c \"import socket,sys; s=socket.socket(); s.settimeout(0.5); sys.exit(0 if s.connect_ex(('127.0.0.1', $MAVSDK_PORT)) == 0 else 1)\"" && break
  echo "!! mavsdk_server not listening on $MAVSDK_PORT (try $try): $(docker exec starling-flight tail -1 ~/mavsdk_server_i$N.log | cut -c1-120)"; sleep 5
done
docker exec -d $GZE starling-gz bash -c "python3 $SC/sim_clock_udp.py $WORLD $CLOCK 50 > /tmp/sim_clock_$N.log 2>&1"
docker exec $GZE starling-gz python3 "$SC/truth_sampler.py" "$MODEL" 1.0 > "$OUT/truth.txt" 2>&1 &
# ground truth with the simulator's own timestamps, every odometry message (2026-10-31, codex ground-truth-timestamps)
docker exec -d $GZE starling-gz bash -c "python3 $SC/gt_logger.py $MODEL $OUT/gt.csv > $OUT/gt_logger.log 2>&1"
SAMPLER=$!
# NO_VIDEO=1 (2026-10-30, batch comparisons on a nearly full disk): no depth/camera frame images (82 of 87 MB per run);
# the sensor, tag detection and every CSV/log are unchanged
TOF_VIS="$OUT/tof"; [ "${NO_VIDEO:-}" = "1" ] && TOF_VIS=""
docker exec -d $GZE starling-gz bash -c "WK_SENSORS=${WK_SENSORS:-} WK_TAG_HZ=${WK_TAG_HZ:-} WK_TOF_HZ=${WK_TOF_HZ:-} WK_TOF_RAW=${WK_TOF_RAW:-} WK_TOF_MODEL=${WK_TOF_MODEL:-} python3 $SC/wall_sense.py $MODEL $OUT/sense.csv $SENSE_PORT $TOF_VIS > $OUT/wall_sense.log 2>&1"
[ "${NO_VIDEO:-}" = "1" ] || docker exec -d $GZE starling-gz bash -c "python3 $SC/cam_grab.py $OUT/cam 15 3600 /front_camera > $OUT/cam_grab.log 2>&1"
if [ "$CHASE" != "0" ] && [ "${NO_VIDEO:-}" != "1" ]; then   # omniscient chase camera = sim sensor moved along the aisle (chase_cam.py), frames on sim time
  docker exec -d $GZE starling-gz bash -c "python3 $SC/chase_cam.py $MODEL $OUT/chase_cam.STOP > $OUT/chase_cam.log 2>&1"
  sleep 3
  docker exec -d $GZE starling-gz bash -c "python3 $SC/cam_grab.py $OUT/chase 15 3600 /chase_cam > $OUT/chase_grab.log 2>&1"
fi
if [ "${WK_TAGS:-0}" = "1" ]; then   # simulated voxl-tag-detector: front camera -> tag36h11 poses -> udp TAG_PORT
  docker exec -d $GZE starling-gz bash -c "WK_SENSORS=${WK_SENSORS:-} WK_TAG_HZ=${WK_TAG_HZ:-} WK_TOF_HZ=${WK_TOF_HZ:-} python3 $SC/tag_detect.py $MODEL $OUT/tags.csv $TAG_PORT > $OUT/tag_detect.log 2>&1"
  WKTAGS="--wk-tags --wk-tag-port $TAG_PORT${WK_RECHECK:+ --wk-tag-recheck}${WK_TOF_HEADING:+ --wk-tof-heading}${WK_TOF_HOLD:+ --wk-tof-hold}${WK_LOC:+ --wk-loc}"
else WKTAGS=""; fi
sleep 4

# what this run actually runs with (2026-10-31, codex experiment-provenance): effective switches, code / config / map / world /
# model hashes, PX4 EV parameters of this instance
export WK_TAGS WK_RECHECK WK_LOC WK_TOF_HEADING WK_TOF_HOLD WK_ALIGN WK_SPAWN_ROT WK_SPAWN_ALIGN NO_VIDEO WK_TOF_RAW CHASE WK_LOC_DRIFTV WK_LOC_EARLY
export MF_WK="$WK" MF_ROT="$ROT" MF_DRIFT="$DRIFT" MF_ROUTE="$ROUTE" MF_EXTRA="$EXTRA" MF_POSE="$POSE" MF_SURVEY="${SURVEY:-}" \
  MF_EFFECTIVE="WK_SENSORS=${WK_SENSORS:-sim} WK_TAG_HZ=${WK_TAG_HZ:-$([ "${WK_SENSORS:-}" = real ] && echo 5 || echo 0)} WK_TOF_HZ=${WK_TOF_HZ:-0} WK_TOF_MIN_VALID=${WK_TOF_MIN_VALID:-0.25} WK_MAP=${WK_MAP:-design} MISSION_TIMEOUT=${MISSION_TIMEOUT:-1500} QVIX_YAW_DEG=$QVIX_YAW_DEG WK_ALLOW_KILL=${WK_ALLOW_KILL:-0}"
PROV=ok; python3 "$SC/run_manifest.py" write "$OUT" "$N" "$WORLD" || { echo "!! run_manifest incomplete or failed"; PROV=incomplete; }
step "미션 (wall-keep $WK)"
docker exec -i -e QVIX_YAW_DEG=$QVIX_YAW_DEG -e WK_SENSORS=${WK_SENSORS:-} -e WK_TOF_MIN_VALID=${WK_TOF_MIN_VALID:-0.25} -e WK_MAP=${WK_MAP:-} -e WK_MAP_ALLOW_INELIGIBLE=${WK_MAP_ALLOW_INELIGIBLE:-0} -e WK_LOC_DRIFTV=$WK_LOC_DRIFTV -e WK_LOC_EARLY=$WK_LOC_EARLY starling-flight bash -c "cd ~/workspace && timeout -s INT ${MISSION_TIMEOUT:-1500} python3 v3_marker/scripts/wk_mission.py --wallkeep $WK ${WK_ALIGN:+--wk-align} $WKTAGS --wk-port $SENSE_PORT --wk-csv /home/dronesim/workspace/v3_marker/logs/${TS}_${TAG}/wk.csv --route '$ROUTE' $EXTRA --csv /home/dronesim/workspace/v3_marker/logs/${TS}_${TAG}/reach_sync.csv --sim-clock $CLOCK -- --port $MAVSDK_PORT" 2>&1 \
  | tee "$OUT/mission.log" | grep --line-buffered -E "\[START\]|TIMEOUT|EMERGENCY|Traceback|Disarmed|wall-keep|final correction|Error" | cut -c1-160
MISSION_RC=${PIPESTATUS[0]}
sleep 2
touch "$OUT/sense.csv.STOP" "$OUT/tags.csv.STOP" "$OUT/gt.csv.STOP" "$OUT/cam/STOP" "$OUT/chase_cam.STOP"; [ -d "$OUT/chase" ] && touch "$OUT/chase/STOP"; docker exec starling-gz touch /tmp/sim_clock_${CLOCK}.STOP
kill $SAMPLER 2>/dev/null; docker exec starling-gz pkill -f "[t]ruth_sampler.py $MODEL " >/dev/null 2>&1
sleep 3
docker exec starling-gz cp /tmp/vio_relay_$N.log "$OUT/vio_relay.log" 2>/dev/null   # injected offset history (per instance)
step "영상 인코딩"
for d in cam tof chase; do
  [ -f "$OUT/$d/f000000.jpg" ] && ffmpeg -y -loglevel error -framerate 15 -i "$OUT/$d/f%06d.jpg" -c:v libx264 -pix_fmt yuv420p -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" "$OUT/$d.mp4" && echo "  $d.mp4 ($(ls "$OUT/$d" | grep -c jpg) frames)"
done
step "판정 (grid_verify)"
python3 "$SC/grid_verify.py" "$OUT/plan.json" "$OUT/mission.log" "$OUT/truth.txt" "$OUT/sitl.log" > "$OUT/report.txt" 2>&1; VRC=$?
tail -5 "$OUT/report.txt"
echo "mission rc $MISSION_RC verify rc $VRC -> $OUT"
# turned / survey worlds: grid_verify is not rotation-aware, the run is judged by the truth clearance of loc_compare.py instead
# (2026-10-31 codex runner-exit-status reply-2: "judged_by" alone is no evidence) -> its verdict goes into the status
TRUTHV=none
if [ "$WORLD" != "qvix" ]; then
  LT=""; case "$WORLD" in qvix_sv*) LT="$V2/gz/survey/${WORLD#qvix_}_truth.json";; esac
  if [ -n "$LT" ] && [ ! -f "$LT" ]; then
    # a survey world without its truth geometry (moved racks) must not be judged against the design racks (codex
    # report-truth-geometry reply-2)
    echo "clearance INCOMPLETE: survey world $WORLD without its truth geometry $LT - not judged against the design racks" | tee "$OUT/loc_compare.txt"
    TRUTHV=INCOMPLETE
  else
    QVIX_YAW_DEG=$QVIX_YAW_DEG LOC_TRUTH=$LT python3 "$SC/loc_compare.py" "$OUT" > "$OUT/loc_compare.txt" 2>&1; LCRC=$?
    TRUTHV=$(grep -o -m1 "clearance \(PASS\|FAIL\|INCOMPLETE\)" "$OUT/loc_compare.txt" | cut -d' ' -f2); TRUTHV=${TRUTHV:-error}
    [ $LCRC = 0 ] || { echo "!! loc_compare exited $LCRC - verdict not trusted"; TRUTHV=error; }   # codex runner-exit-status reply-4
    grep "clearance" "$OUT/loc_compare.txt" | head -1
  fi
fi
# manifest finish AFTER the evaluation (2026-11-01 codex experiment-provenance reply-10: finished before loc_compare, it did not
# show the evaluator versions stayed the same until the evaluation was done); run_status.sh below is hashed here, before it runs
python3 "$SC/run_manifest.py" finish "$OUT" || PROV=incomplete
echo "$OUT" > "$V2/logs/.last_run_i$N"; echo "$OUT" > "$V2/logs/.last_run"
# keep the run on the big disk (2026-10-30: / was 100 % full, other users): the containers only mount ~/starling_sim, so the
# run is written here and moved afterwards; a symlink keeps $OUT valid for the batch scripts and tagfix_eval. ARCHIVE= to skip.
ARCH=${ARCHIVE-/990data/dronesim/starling_sim/v3_marker/logs}
if [ -z "$ARCH" ]; then ASTAT=skipped                         # ARCHIVE= given explicitly: keep the run here
elif ! mkdir -p "$ARCH" 2>/dev/null; then echo "!! $ARCH 를 만들 수 없음, $OUT 에 그대로 둠"; ASTAT=failed
elif rsync -a "$OUT" "$ARCH/" && diff -rq "$OUT" "$ARCH/$(basename "$OUT")" >/dev/null; then
  if rm -rf "$OUT" && ln -s "$ARCH/$(basename "$OUT")" "$OUT"; then echo "  -> $ARCH/$(basename "$OUT")"; ASTAT=ok
  else echo "!! 원본을 링크로 바꾸지 못함 ($OUT)"; ASTAT=failed; fi
else echo "!! $ARCH 로 옮기기 실패, $OUT 에 그대로 둠"; ASTAT=failed; fi
# 2026-10-31 (codex runner-exit-status): the run's outcome is written to status.json and returned (run_status.sh)
"$SC/run_status.sh" "$(readlink -f "$OUT")" "$MISSION_RC" "$VRC" "$WORLD" "$ASTAT" "$TRUTHV" "$PROV"
exit $?
