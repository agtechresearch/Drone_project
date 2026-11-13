#!/bin/bash
# 마커 드리프트 실험 비행을 SITL 에서 1회 돌린다 (연구실 서버 호스트에서 실행).
#   run_sim_drift.sh [marker_drift_flight.py 인자...]
#   예) run_sim_drift.sh --condition A --run 1                       # 정렬 포함 본 시퀀스 (태그 0~9, 4.5 m)
#       run_sim_drift.sh --no-align --distance 0.5 --condition T     # 패키지 점검 비행 (지상 arm 시험 대신)
#       NO_TAGS=1 run_sim_drift.sh --distance 1.0                    # 태그 검출기 없이 (--no-align 자동)
# drone_sim_gz(v3_marker) 의 인스턴스 2~5 도구(stop_sitl_instance.sh, run_wk_instance.sh, gt_logger.py)를 그대로 호출하고,
# 비행은 starling-flight 컨테이너(Python 3.6.9 + MAVSDK 0.12 = 실기)에서 marker_drift_flight.py --sim 으로 한다.
# 기록: $MD/logs/<시각>_iN/  sitl.log mission.log gt.csv(진실값) tags.csv(+_raw.jsonl) tag_detect.log + 비행 산출물(md_*.csv/json)
# env: INSTANCE(2) WORLD(drift_banner) SIM(~/starling_sim) MD($SIM/marker_drift) FLIGHT_WS(/home/dronesim/workspace)
#      CAM_TOPIC(/front_camera_real) TAG_HZ(5) TAG_SIZE(0.20) SPAWN(0,0,0.035,0,0,3.1416 = 서향, 현수막 정면) RTF(1) MISSION_TIMEOUT(600 s)
#      KEEP_SITL(0 = 끝나면 인스턴스 정지) NO_TAGS(0)
#      INJECT('{}') vio_relay.py 오차 주입 JSON: rot_deg(VIO 프레임 회전 °), drift_n/drift_e(m/s), step_e/step_at(m, 이륙 후 s),
#                   noise_pos_m/noise_vel_m_s. 산출물 inject.json 에 남는다.
#      CMD_YAW_BIAS=<deg>: 명령 프레임의 우측을 헤딩에서 이만큼 돌림 (v0.2.10 이격 안전 정지 검증; 실기 A1 은 +10 에 해당)
#      VIO_FAIL_AT=<s> [VIO_FAIL_FROM=offboard|start] [VIO_FAIL_JSON=<json>] [VIO_RECOVER_AFTER=<s>]: offboard 시작(기본) s 초 뒤 시뮬 VIO 상태를 상실로 바꿔
#                   비행 코드의 VIO 감시(비상 착륙)를 검증한다 (s2flight.sim.sim_vio_task, 2026-11-10 사고 대책).
set -u
N=${INSTANCE:-2}; case "$N" in 2|3|4|5) ;; *) echo "!! INSTANCE=$N 은 허용되지 않음 (2..5; 0/1 은 다른 작업대)"; exit 2;; esac
SIM=${SIM:-$HOME/starling_sim}; V3=$SIM/v3_marker; SC=$V3/scripts; MD=${MD:-$SIM/marker_drift}
FLIGHT_WS=${FLIGHT_WS:-/home/dronesim/workspace}          # starling-flight 안에서 $SIM 이 보이는 경로 (docker/run_flight.sh)
case "$MD" in "$SIM"/*) ;; *) echo "!! MD=$MD 는 SIM=$SIM 아래여야 비행 컨테이너에서 보인다"; exit 2;; esac
MD_IN_FLIGHT=$FLIGHT_WS/${MD#$SIM/}
WORLD=${WORLD:-drift_banner}; MODEL=starling2_tof_$N
MAVSDK_PORT=$((50051 + N)); TAG_PORT=$((14680 + N)); SPAWN=${SPAWN:-0,0,0.035,0,0,3.1416}   # v0.2.4: 서향 스폰 = 현수막(-x) 정면, 회전 없음. 종전 북향 1.5708 + --turn-deg -90
MAVSDK_BIN=/usr/local/lib/python3.6/dist-packages/mavsdk/bin/mavsdk_server
TS=$(date +%Y%m%d_%H%M%S)_i$N; OUT=$MD/logs/$TS; mkdir -p "$OUT"; OUT_IN_FLIGHT=$FLIGHT_WS/${OUT#$SIM/}
step(){ echo; echo "=== [$(date +%H:%M:%S)] $*"; }

# 0. 전제
for f in "$SC/stop_sitl_instance.sh" "$SC/run_wk_instance.sh" "$MD/flight/marker_drift_flight.py" "$MD/flight/s2flight/vehicle.py" "$MD/sim/sim_tag_detect.py"; do
  [ -e "$f" ] || { echo "!! 없음: $f  (sync_to_server.sh 로 flight/ 와 sim/ 을 올렸는지, drone_sim_gz 가 $SIM 에 있는지)"; exit 2; }
done
for c in starling-gz starling-flight; do docker ps --format '{{.Names}}' | grep -qx "$c" || { echo "!! 컨테이너 $c 가 떠 있지 않다 (drone_sim_gz/docker/run_*.sh)"; exit 2; }; done
docker exec starling-gz bash -c "test -f \$HOME/PX4-Autopilot/Tools/simulation/gz/worlds/$WORLD.sdf" \
  || { echo "!! 월드 $WORLD 미설치: starling-gz 안에서 $MD/sim/install_banner_world.sh (먼저 호스트에서 make_banner_world.py)"; exit 2; }
echo "$*" > "$OUT/args.txt"; echo "instance=$N world=$WORLD spawn=$SPAWN cam=${CAM_TOPIC:-/front_camera_real} tag_hz=${TAG_HZ:-5} tag_size=${TAG_SIZE:-0.20} no_tags=${NO_TAGS:-0}" >> "$OUT/args.txt"

# 1. 인스턴스 재기동 (starling2_tof: front_camera_real 1280x800 = 실기 tracking_front 화각)
step "인스턴스 $N 정지"
"$SC/stop_sitl_instance.sh" $N || { echo "!! 인스턴스 $N 정리 실패 (rc $?) — 재기동하지 않음. $V3/logs/stop_sitl_$N.log 확인"; exit 2; }
step "인스턴스 $N 기동: world=$WORLD spawn=$SPAWN (기본 서향 = 현수막 정면, v0.2.4)"
INJECT=${INJECT:-"{}"}   # vio_relay.py 오차 주입 JSON (기본 {} = 없음). 예: INJECT='{"drift_n":-0.02}' / '{"rot_deg":3}' / '{"step_e":0.3,"step_at":25}'
docker exec -i starling-gz bash -c "cat > /tmp/wk_inject_$N.json" <<<"$INJECT"
echo "inject=$INJECT" >> "$OUT/args.txt"; echo "$INJECT" > "$OUT/inject.json"
for boot in 1 2 3; do
  setsid docker exec starling-gz bash -c "$SC/run_wk_instance.sh $N $WORLD headless $SPAWN ${RTF:-1}" > "$OUT/sitl.log" 2>&1 < /dev/null &
  for i in $(seq 1 150); do grep -a -q "New NED origin" "$OUT/sitl.log" && break; grep -a -q "EXIT=" "$OUT/sitl.log" && break; sleep 1; done
  grep -a -q "New NED origin" "$OUT/sitl.log" && break
  echo "!! SITL 기동 실패 (시도 $boot): $(grep -a -m1 ERROR "$OUT/sitl.log")"; cp "$OUT/sitl.log" "$OUT/sitl_fail$boot.log"
  "$SC/stop_sitl_instance.sh" $N || { echo "!! 인스턴스 $N 정리 실패 — 중단"; exit 2; }; sleep $((10 * boot))
done
grep -a -q "New NED origin" "$OUT/sitl.log" || { echo "!! SITL 기동 3회 실패"; exit 1; }
grep -a -q "starting gazebo with world: .*$WORLD" "$OUT/sitl.log" || echo "?? sitl.log 에 world $WORLD 기동 줄이 없다 — 어떤 월드가 떴는지 확인"
for i in $(seq 1 30); do docker exec starling-gz pgrep -f "[s]et_global_origin.py udpin:0.0.0.0:$((14540 + N))" >/dev/null || break; sleep 1; done

# 2. mavsdk_server (starling-flight)
step "mavsdk_server -p $MAVSDK_PORT udp://:$((14540 + N))"
port_open(){ docker exec starling-flight python3 -c "import socket,sys; s=socket.socket(); s.settimeout(0.5); sys.exit(0 if s.connect_ex(('127.0.0.1', $MAVSDK_PORT)) == 0 else 1)"; }
docker exec starling-flight pkill -f "[m]avsdk_server -p $MAVSDK_PORT " 2>/dev/null
for i in $(seq 1 20); do port_open || break; sleep 1; done
for try in 1 2 3; do
  docker exec -d starling-flight bash -c "$MAVSDK_BIN -p $MAVSDK_PORT udp://:$((14540 + N)) > ~/mavsdk_server_md$N.log 2>&1"
  for i in $(seq 1 10); do port_open && break; sleep 1; done
  port_open && break
  echo "!! mavsdk_server 가 $MAVSDK_PORT 에서 듣지 않음 (시도 $try)"; sleep 5
done
port_open || { echo "!! mavsdk_server 실패"; exit 1; }

# 3. 진실값 + 시뮬 태그 검출기 (starling-gz, 인스턴스 파티션)
GZE="-e GZ_PARTITION=starling_$N"
if [ -f "$SC/gt_logger.py" ]; then
  docker exec -d $GZE starling-gz bash -c "python3 $SC/gt_logger.py $MODEL $OUT/gt.csv > $OUT/gt_logger.log 2>&1"
else echo "?? gt_logger.py 없음 — 진실값 기록 생략"; fi
if [ "${NO_TAGS:-0}" != "1" ]; then
  step "시뮬 태그 검출기: ${CAM_TOPIC:-/front_camera_real} → udp $TAG_PORT (size ${TAG_SIZE:-0.20}, ${TAG_HZ:-5} Hz)"
  docker exec -d $GZE starling-gz bash -c "TAG_HZ=${TAG_HZ:-5} python3 $MD/sim/sim_tag_detect.py $MODEL $OUT/tags.csv $TAG_PORT ${CAM_TOPIC:-/front_camera_real} ${TAG_SIZE:-0.20} > $OUT/tag_detect.log 2>&1"
  TAGARGS="--sim-tag-port $TAG_PORT --tag-size ${TAG_SIZE:-0.20}"
else
  TAGARGS="--no-align"
fi
sleep 4
[ -f "$OUT/tag_detect.log" ] && head -2 "$OUT/tag_detect.log"

# 3b. 시뮬 VIO 상태 파일 (s2flight.sim.sim_vio_task 가 0.5 s 마다 읽음). VIO_FAIL_AT=<s> 면 비행 시작 s 초 뒤 VIO 상실을 주입한다
#     (VIO_FAIL_JSON 기본 {"quality": -1, "state": "FAILED", "features": 0}; VIO_RECOVER_AFTER=<s> 면 그 뒤 복구). 2026-11-10 사고 대책 검증용.
VIO_FILE="$OUT/sim_vio.json"; echo "{}" > "$VIO_FILE"
echo "vio_fail_at=${VIO_FAIL_AT:-}" >> "$OUT/args.txt"
VIO_FAIL_JSON=${VIO_FAIL_JSON:-'{"quality": -1, "state": "FAILED", "features": 0}'}   # 커스텀 JSON 은 변수로 (${v:-{...}} 안의 } 가 하나 더 붙던 버그, 2026-11-12)
if [ -n "${VIO_FAIL_AT:-}" ]; then
  # 기준 시각: VIO_FAIL_FROM=offboard(기본, 'Offboard active!' 가 mission.log 에 찍힌 뒤) | start(비행 스크립트 기동 시점)
  ( if [ "${VIO_FAIL_FROM:-offboard}" = "offboard" ]; then
      for i in $(seq 1 600); do grep -q "Offboard active" "$OUT/mission.log" 2>/dev/null && break; sleep 0.5; done; fi
    sleep "$VIO_FAIL_AT"; echo "$VIO_FAIL_JSON" > "$VIO_FILE"; echo "[sim] VIO FAIL injected at +${VIO_FAIL_AT}s after ${VIO_FAIL_FROM:-offboard}" >> "$OUT/mission.log"
    if [ -n "${VIO_RECOVER_AFTER:-}" ]; then sleep "$VIO_RECOVER_AFTER"; echo "{}" > "$VIO_FILE"; echo "[sim] VIO recovered" >> "$OUT/mission.log"; fi ) &
fi

# 4. 비행 (starling-flight: Python 3.6.9 + MAVSDK 0.12.0)
step "비행: marker_drift_flight.py --sim --port $MAVSDK_PORT $TAGARGS $*"
echo "cmd_yaw_bias=${CMD_YAW_BIAS:-}" >> "$OUT/args.txt"   # v0.2.10: CMD_YAW_BIAS=<deg> 면 명령 프레임 우측을 헤딩에서 이만큼 돌린다 (실기 헤딩-위치 어긋남 재현, 이격 안전 정지 검증)
docker exec -i starling-flight bash -c "cd $MD_IN_FLIGHT/flight && S2_SIM_VIO_FILE=$OUT_IN_FLIGHT/sim_vio.json S2_SIM_CMD_YAW_BIAS_DEG=${CMD_YAW_BIAS:-} timeout -s INT ${MISSION_TIMEOUT:-600} python3 marker_drift_flight.py --sim --port $MAVSDK_PORT $TAGARGS --out-dir $OUT_IN_FLIGHT -y $*" 2>&1 \
  | tee -a "$OUT/mission.log" | grep --line-buffered -E "^\[|\[align\]|\[vio|REACHED|TIMEOUT|EMERGENCY|GEOFENCE|PREFLIGHT|REFUSED|DISAGREE|Traceback|Exception|Landed|Disarmed|OK |FAILED|WARNING" | cut -c1-160
RC=${PIPESTATUS[0]}

# 5. 정리
touch "$OUT/tags.csv.STOP" "$OUT/gt.csv.STOP"; sleep 2
[ "${KEEP_SITL:-0}" = "1" ] || "$SC/stop_sitl_instance.sh" $N >/dev/null 2>&1 || echo "?? 인스턴스 $N 정지 실패 — stop_sitl_instance.sh $N 을 직접"
echo; echo "mission rc $RC -> $OUT"; ls "$OUT"
S=$(ls "$OUT"/md_*_summary.json 2>/dev/null | head -1)
[ -n "$S" ] && python3 - "$S" <<'EOF'
import json, sys
s = json.load(open(sys.argv[1]))
seq = s.get("sequence") or {}
veh = s.get("vehicle") or {}
print("summary:", "completed" if seq.get("completed") else "NOT completed", "| aborted" if seq.get("aborted") else "",
      "| align:", (seq.get("align") or {}).get("state", "-"), "| exception:", s.get("exception", "-"),
      "| preflight_abort:", s.get("preflight_abort", "-"))
print("  vio_gate:", veh.get("vio_gate"), "| abort_reason:", veh.get("abort_reason"), "| vio_bad_episodes:", veh.get("vio_bad_episodes"))
for r in (s.get("vehicle") or {}).get("results", [])[:20]:
    print("  %-16s %-8s %s" % (r.get("name"), r.get("status"), r.get("fb_end_rel", "")))
EOF
exit $RC
