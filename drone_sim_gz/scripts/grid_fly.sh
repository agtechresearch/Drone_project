#!/bin/bash
# Fly a grid route in SITL end to end and judge it with ground truth (run on the HOST).
#   grid_fly.sh "R1C10:S R1C1 R1C10:N R1D10:E R1D1:S R1D10:N R1C10:W R1C10:S" [gui|headless] [extra grid_mission.py args...]
#   RECORD=1 grid_fly.sh "<route>"          # also record the Gazebo window (GUI_DISPLAY, default :1 = VNC; :99 = hidden Xvfb; 25 fps) to logs/grid_<ts>.mp4 with the GUI
#                                          # camera following the vehicle as a dolly shot (cam_follow.py, 20 Hz, 1 s low-pass; FOLLOW_OFFSET="0,-2.5,1.5"
#                                          # = fixed WORLD offset: 2.5 m south of the vehicle, 1.5 m up, looking north) with the QVIX module made
#                                          # semi-transparent (TRANSPARENT=0 to keep it opaque); FOLLOW=0 keeps the fixed room camera, FOLLOW=1 follows without recording
#                                          # GUI mode always restarts the Gazebo GUI with gz/gui_record.config (no Entity tree, 1900x970 window)
#   RECORD_CAM=1 grid_fly.sh "<route>"      # record the vehicle's FRONT CAMERA (gz sensor 640x480 15 Hz, every frame via cam_grab.py) to logs/grid_<ts>_cam.mp4
#                                          # CAM_FPS=5 to save fewer frames (sim-time rate)
#   grid_fly.sh --survey "R1C W" ...        # first arg may be --survey <spec> instead of a route (see grid_mission.py)
#   SIM_SPEED=2 grid_fly.sh "<route>"       # Gazebo at 2x real time (runtime set_physics); the mission script runs on the Gazebo SIM clock relayed
#                                          # over UDP (sim_clock_udp.py, also with SIM_CLOCK=1 at 1x), truth samples every 1 s of sim time, screen video slowed to real time
#   INSTANCE=1 grid_fly.sh "<route>" headless  # parallel instance N>0: own Gazebo world (GZ_PARTITION starling_N), PX4 -i N (udp 14540+N, GCS 18570+N),
#                                          # model starling2_N, mavsdk_server 50051+N, sim clock udp 14600+N, logs grid_<ts>_iN_*; always headless, stopped
#                                          # per instance (does not touch instance 0). Run several in parallel from separate shells, each with its own INSTANCE.
#   REUSE_SITL=1 grid_fly.sh "<route>"      # SITL already running: preflight check first (auto-falls back to a full restart if
#                                          # arming would be refused, e.g. accel-bias warning after a long idle); the route's first token must be
#                                          # the cell+heading where the vehicle currently sits, e.g. where the last route ended)
# Steps: stop old SITL -> start SITL in the QVIX world spawned at the route's first cell/heading -> grid overlay (GUI)
#        -> mavsdk_server -> truth sampler (truth_sampler.py, odometry subscription, 1 s) -> grid_mission.py in starling-flight -> grid_verify.py report.
# Logs: ~/starling_sim/logs/grid_<timestamp>_{sitl,mission}.log, _truth.txt, _plan.json, _report.txt (+ CSV). SITL stays up afterwards.
set -u
N=${INSTANCE:-0}            # parallel instance number (0 = the normal single SITL)
GUI_DISPLAY=${GUI_DISPLAY:-:1}
SURVEY=""
if [ "${1:-}" = "--survey" ]; then
  SURVEY=${2:?survey spec}; shift 2
  ROUTE=$(python3 "$HOME/starling_sim/repo_export/scripts/grid_mission.py" --survey "$SURVEY" --check 2>/dev/null | sed -n 's/^survey route: //p')
  [ -n "$ROUTE" ] || { echo "!! survey spec rejected: $SURVEY"; python3 "$HOME/starling_sim/repo_export/scripts/grid_mission.py" --survey "$SURVEY" --check; exit 1; }
  echo "survey route: $ROUTE"
else
  ROUTE=${1:?route}; shift
fi
MODE=${1:-gui}; [ $# -gt 0 ] && shift; EXTRA="$*"
# SIM_SPEED=2: run Gazebo at real_time_factor 2 (runtime /world/<w>/set_physics, restored to 1 afterwards); the mission script's
# clock is scaled to match (grid_mission.py --sim-speed), the truth sampler keeps 1 s of SIM time, the front-camera video is
# sim-timed anyway, and the screen recording is slowed back to real time with setpts. Only --sim-speed is added to EXTRA here.
SIM_SPEED=${SIM_SPEED:-1}
SIM_CLOCK_PORT=$((14600 + N))
if [ "$SIM_SPEED" != "1" ] || [ -n "${SIM_CLOCK:-}" ]; then EXTRA="$EXTRA --sim-clock $SIM_CLOCK_PORT"; USE_SIM_CLOCK=1; else USE_SIM_CLOCK=""; fi
# survey = continuous video: no hover between stages unless the caller sets --hover (grid_mission.py applies this rule
# only to its own --survey mode, and this script hands it the compiled --route)
if [ -n "${SURVEY:-}" ] && ! echo "$EXTRA" | grep -q -- "--hover"; then EXTRA="$EXTRA --hover 0"; fi
SIM=$HOME/starling_sim; SC=$SIM/repo_export/scripts; LOGS=$SIM/logs; mkdir -p "$LOGS"
TS=$(date +%Y%m%d_%H%M%S); TAG="grid_${TS}"; [ "$N" != "0" ] && TAG="grid_${TS}_i${N}"
MODEL="starling2_${N}"; MAV_UDP=$((14540 + N)); MAVSDK_PORT=$((50051 + N)); GCS="udpout:127.0.0.1:$((18570 + N))"
GZE=""; [ "$N" != "0" ] && GZE="-e GZ_PARTITION=starling_${N}"
# PX4 shell command for this instance: instance 0 via MAVLink shell (GCS 14550, as before), N>0 via the local PX4 client
# PX4 shell command for this instance through the local PX4 client (no MAVLink: with several instances the GCS port
# 14550 receives every vehicle, so the MAVLink shell could talk to the wrong one)
pxsh(){ docker exec starling-gz bash -c "cd ~/PX4-Autopilot/build/px4_sitl_default && timeout 15 ./bin/px4-$1 --instance $N ${*:2} 2>&1"; }
if [ "$N" != "0" ]; then MODE=headless; fi
SITL_LOG=$LOGS/${TAG}_sitl.log; MISSION_LOG=$LOGS/${TAG}_mission.log; TRUTH=$LOGS/${TAG}_truth.txt
PLAN=$LOGS/${TAG}_plan.json; REPORT=$LOGS/${TAG}_report.txt
MAVSDK_BIN=/usr/local/lib/python3.6/dist-packages/mavsdk/bin/mavsdk_server
step(){ echo; echo "=== [$(date +%H:%M:%S)] $*"; }
# (Re)start the Gazebo GUI with the recording config. Match "^gz sim -g": an unanchored pattern also matches this very
# bash -c command line (it contains the launch command) and the shell kills itself. A frozen GUI ignores SIGTERM -> -9.
# (Re)start the Gazebo GUI with the recording config: GPU (vglrun -d egl) when the container can reach the GPU, else llvmpipe.
gui_start(){
  xvfb_up
  docker exec starling-gz bash -c 'for pid in $(pgrep -f "^gz sim -g"); do tr "\0" "\n" < /proc/$pid/environ | grep -q "^GZ_PARTITION=" || kill $pid; done; sleep 3;
    for pid in $(pgrep -f "^gz sim -g"); do tr "\0" "\n" < /proc/$pid/environ | grep -q "^GZ_PARTITION=" || kill -9 $pid; done;
    VGL=""; python3 -c "open(\"/dev/nvidiactl\",\"rb\")" 2>/dev/null && command -v vglrun >/dev/null && VGL="vglrun -d egl";
    export __EGL_VENDOR_LIBRARY_FILENAMES=$HOME/starling_sim/repo_export/gz/glvnd/10_nvidia.json;
    DISPLAY='"$GUI_DISPLAY"' setsid $VGL gz sim -g --gui-config ~/starling_sim/repo_export/gz/gui_record.config > /tmp/gzgui.log 2>&1 < /dev/null &'
}
# GUI_DISPLAY: where the instance-0 GUI and the screen recording live. :1 = the VNC desktop (default); any other display,
# e.g. GUI_DISPLAY=:99, is a hidden Xvfb started on demand inside starling-gz (its socket appears on the host through the
# /tmp/.X11-unix bind mount), so recordings can run while the VNC desktop is used for the workbench (bench.sh).
xvfb_up(){
  [ "$GUI_DISPLAY" = ":1" ] && return
  [ -S "/tmp/.X11-unix/X${GUI_DISPLAY#:}" ] && return
  docker exec -d starling-gz bash -c "Xvfb $GUI_DISPLAY -screen 0 1920x1080x24 -ac +extension GLX > /tmp/xvfb${GUI_DISPLAY#:}.log 2>&1"
  for i in $(seq 1 20); do [ -S "/tmp/.X11-unix/X${GUI_DISPLAY#:}" ] && break; sleep 0.5; done
}
gui_alive(){ docker exec starling-gz bash -c 'for pid in $(pgrep -f "^gz sim -g"); do tr "\0" "\n" < /proc/$pid/environ | grep -q "^GZ_PARTITION=" || exit 0; done; exit 1'; }

step "경로 컴파일 검사"
python3 "$SC/grid_mission.py" --check --route "$ROUTE" --plan-json "$PLAN" $EXTRA || { echo "!! 경로 거부"; exit 1; }
POSE=$(python3 "$SC/grid_mission.py" --spawn-pose --route "$ROUTE")
ROOM=$(echo "$ROUTE" | cut -c1-2)

HL=""; [ "$MODE" = "headless" ] && HL=headless
if [ -n "${REUSE_SITL:-}" ]; then
  step "SITL 재사용 요청 (REUSE_SITL=1): 프리플라이트 검사 (commander check)"
  CHK=$(pxsh commander check | grep -a -o "Preflight check: [A-Z]*" | tail -1)
  echo "  $CHK"
  if ! echo "$CHK" | grep -q "OK"; then
    LAST=$(cat "$LOGS/.last_sitl_log_i$N" 2>/dev/null); REASON=$(grep -a "Preflight Fail" "${LAST:-/dev/null}" 2>/dev/null | tail -1)
    echo "  !! 시동 불가 (${REASON:-이유 미기록}; SITL을 오래 켜 두면 가속도계 바이어스 경고로 시동이 거부됨) → SITL 재시작으로 전환"
    REUSE_SITL=""
  fi
fi
if [ -n "${REUSE_SITL:-}" ]; then
  step "SITL 재사용: 재시작 생략, 현재 진실 위치 확인"
  docker exec starling-flight pkill -f "mavsdk_server -p $MAVSDK_PORT " >/dev/null 2>&1
  SITL_LOG=$(cat "$LOGS/.last_sitl_log_i$N" 2>/dev/null || echo /dev/null)
  TRUTH_NOW=$(docker exec $GZE starling-gz bash -c "timeout -s KILL 5 gz model -m $MODEL -p 2>/dev/null" | grep -A1 'Pose \[' | tail -1)
  echo "  기체 진실 위치: $TRUTH_NOW   (경로 첫 토큰의 스폰 포즈: $POSE)"
  if [ -z "$HL" ] && [ "$N" = "0" ] && ! gui_alive; then
    step "Gazebo GUI가 없음 (종료/크래시) → 재기동"
    gui_start
    for i in $(seq 1 40); do docker exec starling-gz bash -c 'gz service -l 2>/dev/null | grep -q "^/marker/list"' && break; sleep 1; printf "."; done; echo
    rm -f "$LOGS/.gui_transparent"; "$SC/grid_overlay.sh" "$ROOM" camera >/dev/null 2>&1
  fi
else
if [ "$N" = "0" ]; then
  step "이전 SITL / mavsdk_server 정리"
  "$SC/stop_sitl.sh"
  step "SITL 기동: world=qvix pose=$POSE mode=${HL:-GUI}  (로그: $SITL_LOG)"
  setsid docker exec starling-gz bash -c "$SC/run_sitl.sh qvix '$HL' $POSE" > "$SITL_LOG" 2>&1 < /dev/null &
else
  step "인스턴스 $N 정리 (px4 -i $N, partition starling_$N의 gz, mavsdk_server $MAVSDK_PORT)"
  "$SC/stop_sitl_instance.sh" "$N"
  step "인스턴스 $N SITL 기동: partition starling_$N world=qvix pose=$POSE headless  (로그: $SITL_LOG)"
  setsid docker exec starling-gz bash -c "$SC/run_sitl_instance.sh $N qvix headless $POSE" > "$SITL_LOG" 2>&1 < /dev/null &
fi
for i in $(seq 1 150); do
  grep -a -q "New NED origin" "$SITL_LOG" 2>/dev/null && break
  if grep -a -q -E "EXIT=|ninja: error|FAILED" "$SITL_LOG" 2>/dev/null; then echo "!! SITL 기동 실패"; tail -20 "$SITL_LOG"; exit 1; fi
  sleep 1; printf "."
done; echo
grep -a -E "starting gazebo with world|spawning|New NED origin" "$SITL_LOG" | head -3 | sed 's/^/  /'
grep -a -q "New NED origin" "$SITL_LOG" || { echo "!! 150초 안에 글로벌 원점 설정이 안 됨. 로그: $SITL_LOG"; exit 1; }
echo "  기체 진실 위치: $(docker exec $GZE starling-gz bash -c "timeout -s KILL 5 gz model -m $MODEL -p 2>/dev/null" | grep -A1 'Pose \[' | tail -1)"
echo "$SITL_LOG" > "$LOGS/.last_sitl_log_i$N"; [ "$N" = "0" ] && echo "$SITL_LOG" > "$LOGS/.last_sitl_log"
if [ -z "$HL" ]; then
  step "Gazebo GUI를 녹화용 설정으로 재기동 (gz/gui_record.config: Entity tree 없음, 1900x1010)"
  gui_start
  for i in $(seq 1 40); do docker exec starling-gz bash -c 'gz service -l 2>/dev/null | grep -q "^/marker/list"' && break; sleep 1; printf "."; done; echo
  rm -f "$LOGS/.gui_transparent"
  step "격자 오버레이 ($ROOM)"; "$SC/grid_overlay.sh" "$ROOM"
fi
fi

# the instance's origin setter (set_global_origin.py, udpin on the offboard port) must have released the port first,
# otherwise mavsdk_server fails with "bind error: Address already in use" and every mission hangs at "Connecting..."
for i in $(seq 1 30); do docker exec starling-gz pgrep -f "[s]et_global_origin.py udpin:0.0.0.0:$((14540 + N))" >/dev/null || break; sleep 1; done
step "mavsdk_server 시작"
docker exec -d starling-flight bash -c "$MAVSDK_BIN -p $MAVSDK_PORT udp://:$MAV_UDP > ~/mavsdk_server_i$N.log 2>&1"
sleep 4

if [ "$SIM_SPEED" != "1" ]; then
  step "시뮬 배속 x$SIM_SPEED (gz set_physics real_time_factor)"
  docker exec $GZE starling-gz bash -c "gz service -s /world/qvix/set_physics --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 3000 --req 'real_time_factor: $SIM_SPEED'" | tr -d '\n'; echo
fi
if [ -n "$USE_SIM_CLOCK" ]; then
  step "시뮬 시계 UDP 중계 시작 (sim_clock_udp.py → 127.0.0.1:$SIM_CLOCK_PORT, 미션 스크립트가 시뮬 시간으로 동작)"
  docker exec starling-gz pkill -f "sim_clock_udp.py qvix $SIM_CLOCK_PORT" >/dev/null 2>&1
  docker exec -d $GZE starling-gz bash -c "python3 $SC/sim_clock_udp.py qvix $SIM_CLOCK_PORT 50 > /tmp/sim_clock_$N.log 2>&1"
fi
SAMPLE_WALL=$(python3 -c "print(1.0/$SIM_SPEED)")
step "진실값 샘플러 시작 (시뮬 1 s = 벽시계 ${SAMPLE_WALL} s → $TRUTH)"
docker exec $GZE starling-gz python3 "$SC/truth_sampler.py" "$MODEL" "$SAMPLE_WALL" > "$TRUTH" 2>&1 &
SAMPLER_PID=$!

REC_PID=""
FOLLOW_ON=""
if [ -z "$HL" ]; then                                  # GUI mini-maps (rack face + plan, live_map.py)
  docker exec starling-gz pkill -f "[l]ive_map.py $MODEL " >/dev/null 2>&1
  docker exec -d $GZE starling-gz bash -c "python3 $SC/live_map.py $MODEL 5 $ROOM > /tmp/live_map_$N.log 2>&1"
fi
if [ -z "$HL" ] && [ "${FOLLOW:-}" != "0" ] && { [ -n "${RECORD:-}" ] || [ "${FOLLOW:-}" = "1" ]; }; then
  VIEW=${FOLLOW_VIEW:-aisle}
  BOX=$(python3 -c "import sys; sys.path.insert(0,'$SC'); import qvix_grid as G; y0,y1=G.ROOMS['$ROOM']['y']; w=G.model_to_world
print('%.2f,%.2f,%.2f,%.2f,%.2f,%.2f' % (w(0.4,0)[0], w(5.4,0)[0], w(0,y0+0.3)[1], w(0,y1-0.3)[1], w(0,0,0.3)[2], w(0,0,4.8)[2]))")
  step "GUI 전지적 시점 시작 (cam_follow.py view=$VIEW, GUI 추적 FOLLOW_LOOK_AT, 고도 막대·후광·궤적 마커)"
  docker exec starling-gz pkill -f cam_follow.py >/dev/null 2>&1
  docker exec starling-gz bash -c "gz topic -t /gui/track -m gz.msgs.CameraTrack -p 'track_mode: NONE' >/dev/null 2>&1"
  # see-through module (walls + racks semi-transparent) so the camera is never blocked; the service TOGGLES, so the state is
  # remembered in logs/.gui_transparent (cleared when the GUI restarts)
  # default: transparent (the aisle view looks through the room's end wall near the south end; TRANSPARENT=0 to keep it opaque)
  WANT_T=${TRANSPARENT:-1}
  if [ "$WANT_T" = "0" ] && [ -f "$LOGS/.gui_transparent" ]; then
    docker exec starling-gz bash -c "timeout 10 gz service -s /gui/view/transparent --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: \"qvix_r_module\"'" >/dev/null && rm -f "$LOGS/.gui_transparent" && echo "  QVIX 모듈 투시 끔"
  fi
  if [ "$WANT_T" != "0" ] && [ ! -f "$LOGS/.gui_transparent" ]; then
    docker exec starling-gz bash -c "timeout 10 gz service -s /gui/view/transparent --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: \"qvix_r_module\"'" >/dev/null && touch "$LOGS/.gui_transparent" && echo "  QVIX 모듈 투시(반투명) 켬"
  fi
  docker exec -d -e CAM_FOLLOW_BOX=$BOX -e OBL_SIDE -e OBL_BACK -e CAM_Z -e TAU_POS -e TAU_LOOK -e HALO -e TRAIL_S -e CAM_FOLLOW_MARKERS -e CAM_FOLLOW_OFFSET -e FOLLOW_PGAIN -e TRACK_PGAIN -e AISLE_BACK -e AISLE_Z -e AISLE_DIR \
    starling-gz bash -c "CAM_FOLLOW_ROLE=camera python3 $SC/cam_follow.py $VIEW 10 3600 > /tmp/cam_follow.log 2>&1"
  docker exec -d -e TRAIL_S -e CAM_FOLLOW_MARKERS -e HALO starling-gz bash -c "CAM_FOLLOW_ROLE=markers python3 $SC/cam_follow.py $VIEW 10 3600 > /tmp/cam_follow_markers.log 2>&1"
  FOLLOW_ON=1
fi
if [ -n "${RECORD:-}" ] && [ -z "$HL" ]; then
  # record only the Gazebo window (absolute geometry from xwininfo; even width/height for yuv420p), whole display if not found
  read -r RW RH RX RY <<< "$(DISPLAY=$GUI_DISPLAY xwininfo -root -tree 2>/dev/null | grep '"Gazebo Sim"' | head -1 | sed -E 's/.* ([0-9]+)x([0-9]+)\+[0-9]+\+[0-9]+ +\+([0-9]+)\+([0-9]+).*/\1 \2 \3 \4/')"
  [ -n "${RY:-}" ] || { RW=1920; RH=1080; RX=0; RY=0; }
  RW=$((RW / 2 * 2)); RH=$((RH / 2 * 2))
  step "화면 녹화 시작 (창 ${RW}x${RH}+${RX}+${RY}) → $LOGS/${TAG}.mp4"
  SLOW=""; [ "$SIM_SPEED" != "1" ] && SLOW="-vf setpts=${SIM_SPEED}*PTS"      # play back at real (sim) speed
  DISPLAY=$GUI_DISPLAY ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 -framerate 25 -video_size ${RW}x${RH} -i $GUI_DISPLAY+${RX},${RY} $SLOW -c:v libx264 -preset ultrafast -pix_fmt yuv420p "$LOGS/${TAG}.mp4" < /dev/null > /dev/null 2>&1 &
  REC_PID=$!
fi

CAM_DIR=""
if [ -n "${RECORD_CAM:-}" ]; then
  CAM_DIR=$LOGS/${TAG}_cam
  CAM_FPS=${CAM_FPS:-15}
  step "전방 카메라 프레임 저장 시작 ($CAM_FPS fps JPEG → $CAM_DIR, 끝나면 ${TAG}_cam.mp4)"
  docker exec -d $GZE starling-gz bash -c "python3 $SC/cam_grab.py $CAM_DIR $CAM_FPS 3600 /front_camera > $CAM_DIR.log 2>&1"
fi

step "격자 미션 실행 (로그: $MISSION_LOG)"
docker exec -i starling-flight bash -c "cd ~/workspace && python3 repo_export/scripts/grid_mission.py --route '$ROUTE' $EXTRA -- --port $MAVSDK_PORT" 2>&1 \
  | tee "$MISSION_LOG" | grep --line-buffered -E "\[START\]|\[END\]|TIMEOUT|EMERGENCY|Traceback|Landing|Disarmed|Done|Exception" | cut -c1-150
MISSION_RC=${PIPESTATUS[0]}
sleep 3
# After action.land() PX4 stays in Offboard mode with the signal lost, so the next preflight check (REUSE_SITL) would fail.
# Put the landed vehicle into Hold so a follow-up route can arm without a restart.
pxsh commander mode auto:loiter >/dev/null
docker exec starling-gz pkill -f "truth_sampler.py $MODEL " >/dev/null 2>&1; kill $SAMPLER_PID 2>/dev/null
if [ -n "$CAM_DIR" ]; then
  touch "$CAM_DIR/STOP"; sleep 2
  ffmpeg -y -loglevel error -framerate "$CAM_FPS" -i "$CAM_DIR/f%06d.jpg" -c:v libx264 -pix_fmt yuv420p "$LOGS/${TAG}_cam.mp4" \
    && echo "  카메라 영상 → $LOGS/${TAG}_cam.mp4 ($(du -h "$LOGS/${TAG}_cam.mp4" | cut -f1), $(ls "$CAM_DIR"/f*.jpg | wc -l) frames)"
fi
if [ -n "$REC_PID" ]; then kill -INT $REC_PID 2>/dev/null; wait $REC_PID 2>/dev/null; echo "  영상 → $LOGS/${TAG}.mp4 ($(du -h "$LOGS/${TAG}.mp4" 2>/dev/null | cut -f1))"; fi
docker exec starling-gz pkill -f "[l]ive_map.py $MODEL " >/dev/null 2>&1
if [ -n "$FOLLOW_ON" ]; then docker exec starling-gz touch /tmp/cam_follow.STOP; sleep 1; docker exec starling-gz cat /tmp/cam_follow.log | tail -1 | sed 's/^/  /'; fi
CSV=$(ls -t "$SIM"/px4_reach_sync_custom_v13_*.csv 2>/dev/null | head -1); [ -n "$CSV" ] && mv "$CSV" "$LOGS/${TAG}_$(basename "$CSV")" && echo "  CSV → $LOGS/${TAG}_$(basename "$CSV")"

[ -n "$USE_SIM_CLOCK" ] && docker exec starling-gz touch /tmp/sim_clock_${SIM_CLOCK_PORT}.STOP
if [ "$SIM_SPEED" != "1" ]; then
  docker exec $GZE starling-gz bash -c "gz service -s /world/qvix/set_physics --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 3000 --req 'real_time_factor: 1.0'" >/dev/null; echo "  시뮬 배속 원복 (x1)"
fi
step "판정: 계획 vs gz 진실값 (리포트: $REPORT)"
python3 "$SC/grid_verify.py" "$PLAN" "$MISSION_LOG" "$TRUTH" "$SITL_LOG" | tee "$REPORT"
VERDICT_RC=${PIPESTATUS[0]}
echo
echo "미션 종료 코드 $MISSION_RC, 판정 코드 $VERDICT_RC (0=PASS). SITL 계속 실행 중. 종료: $SC/stop_sitl.sh (인스턴스 $N>0: $SC/stop_sitl_instance.sh $N)"
exit $VERDICT_RC
