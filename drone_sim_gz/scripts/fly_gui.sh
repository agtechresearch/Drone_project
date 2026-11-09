#!/bin/bash
# 2026-10-31 (codex legacy-demo-isolation reply-2): LEGACY entry point (v3_marker/scripts/ENTRYPOINTS.md). Stops px4/gz/mavsdk_server of the WHOLE container (all instances, the workbench).
[ "${LEGACY_OK:-0}" = "1" ] || { echo "!! legacy entry point $0: kills every instance in the container. Set LEGACY_OK=1 to run anyway (see v3_marker/scripts/ENTRYPOINTS.md)."; exit 3; }
# One-command demo: start PX4 SITL (gz Harmonic, GUI on DISPLAY :1), then fly the real mission script in SITL mode.
# Run on the HOST (not inside a container):
#   ~/starling_sim/repo_export/scripts/fly_gui.sh            # empty world, spawn at origin
#   ~/starling_sim/repo_export/scripts/fly_gui.sh qvix       # inside the QVIX module (spawn 2.05,-0.8,0.18, facing north)
#   HEADLESS=1 ~/starling_sim/repo_export/scripts/fly_gui.sh # no GUI
#   SKIP_MISSION=1 ...                                        # only start SITL (then fly by hand, see bottom)
# Logs: ~/starling_sim/logs/<world>_<timestamp>_{sitl,mission,truth}.{log,txt}. SITL stays running afterwards; stop with stop_sitl.sh.
set -u
WORLD=${1:-default}
SIM=$HOME/starling_sim; LOGS=$SIM/logs; mkdir -p "$LOGS"
TS=$(date +%Y%m%d_%H%M%S); TAG="${WORLD}_${TS}"
SITL_LOG=$LOGS/${TAG}_sitl.log; MISSION_LOG=$LOGS/${TAG}_mission.log; TRUTH=$LOGS/${TAG}_truth.txt
MAVSDK_BIN=/usr/local/lib/python3.6/dist-packages/mavsdk/bin/mavsdk_server
step(){ echo; echo "=== [$(date +%H:%M:%S)] $*"; }

step "컨테이너 확인"
for c in starling-gz starling-flight; do
  if [ "$(docker inspect -f '{{.State.Running}}' $c 2>/dev/null)" != "true" ]; then echo "  $c 시작"; docker start $c >/dev/null || { echo "!! $c 를 시작할 수 없음"; exit 1; }; fi
done

step "이전 SITL / mavsdk_server 정리"
docker exec starling-gz bash -c "pkill -x px4; pkill -x ruby; pkill -x gz; pkill -x tail; true" >/dev/null 2>&1
docker exec starling-gz pkill -f truth_sampler.sh >/dev/null 2>&1
docker exec starling-flight pkill -x mavsdk_server >/dev/null 2>&1
docker exec starling-flight pkill -f path_flight_phase1 >/dev/null 2>&1
sleep 2

step "SITL 기동: world=$WORLD $( [ -n "${HEADLESS:-}" ] && echo headless || echo GUI\(DISPLAY=:1\) )  (로그: $SITL_LOG)"
HL=""; [ -n "${HEADLESS:-}" ] && HL=headless
setsid docker exec starling-gz bash -c "$SIM/repo_export/scripts/run_sitl.sh $WORLD $HL" > "$SITL_LOG" 2>&1 < /dev/null &   # setsid: SITL survives closing this terminal
SITL_PID=$!
for i in $(seq 1 150); do
  grep -a -q "New NED origin" "$SITL_LOG" 2>/dev/null && break
  if grep -a -q -E "EXIT=|ninja: error|FAILED" "$SITL_LOG" 2>/dev/null; then echo "!! SITL 기동 실패. 로그 끝부분:"; tail -20 "$SITL_LOG"; exit 1; fi
  sleep 1; printf "."
done; echo
grep -a -E "starting gazebo with world|spawning|New NED origin" "$SITL_LOG" | head -3 | sed 's/^/  /'
grep -a -q "New NED origin" "$SITL_LOG" || { echo "!! 150초 안에 글로벌 원점 설정이 안 됨. 로그: $SITL_LOG"; exit 1; }
echo "  기체 진실 위치: $(docker exec starling-gz bash -c 'timeout -s KILL 5 gz model -m starling2_0 -p 2>/dev/null' | grep -A1 'Pose \[' | tail -1)"

if [ -n "${SKIP_MISSION:-}" ]; then
  echo; echo "SKIP_MISSION 설정됨. SITL만 실행 중. 수동 비행:"
  echo "  docker exec -d starling-flight bash -c '$MAVSDK_BIN -p 50051 udp://:14540 > ~/mavsdk_server.log 2>&1'"
  echo "  echo | docker exec -i starling-flight bash -c 'cd ~/workspace && python3 path_flight_phase1_v13.py --sim --csv auto'"
  exit 0
fi

# run_sitl.sh sets the origin over udp 14540: wait until that script released the port (else mavsdk_server: bind error)
for i in $(seq 1 30); do docker exec starling-gz pgrep -f "[s]et_global_origin.py udpin:0.0.0.0:14540" >/dev/null || break; sleep 1; done
step "mavsdk_server 시작 (starling-flight, gRPC 50051 ← UDP 14540)"
docker exec -d starling-flight bash -c "$MAVSDK_BIN -p 50051 udp://:14540 > ~/mavsdk_server.log 2>&1"
sleep 4

step "진실값 샘플러 시작 (2.5 s 간격 → $TRUTH)"
docker exec starling-gz "$SIM/repo_export/scripts/truth_sampler.sh" starling2_0 2.5 > "$TRUTH" 2>&1 &
SAMPLER_PID=$!

step "미션 실행: path_flight_phase1_v13.py --sim --csv auto  (로그: $MISSION_LOG)  — Gazebo 창을 보세요"
echo | docker exec -i starling-flight bash -c "cd ~/workspace && python3 path_flight_phase1_v13.py --sim --csv auto" 2>&1 \
  | tee "$MISSION_LOG" | grep --line-buffered -E "\[START\]|\[END\]|TIMEOUT|EMERGENCY|Traceback|Landing|Disarmed|Done" | cut -c1-170
MISSION_RC=${PIPESTATUS[1]}
docker exec starling-gz pkill -f truth_sampler.sh >/dev/null 2>&1; kill $SAMPLER_PID 2>/dev/null
CSV=$(ls -t "$SIM"/px4_reach_sync_custom_v13_*.csv 2>/dev/null | head -1); [ -n "$CSV" ] && mv "$CSV" "$LOGS/${TAG}_$(basename "$CSV")" && echo "  CSV → $LOGS/${TAG}_$(basename "$CSV")"

step "결과: 스크립트 판정 vs gz 진실값"
python3 "$SIM/repo_export/scripts/truth_vs_stages.py" "$MISSION_LOG" "$TRUTH" 2>/dev/null || echo "  (대조 실패: 진실값 샘플 없음?)"
echo
echo "미션 종료 코드 $MISSION_RC. SITL은 계속 실행 중(Gazebo 창 유지). 종료: $SIM/repo_export/scripts/stop_sitl.sh"
