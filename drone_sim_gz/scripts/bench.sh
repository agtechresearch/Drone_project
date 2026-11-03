#!/bin/bash
# Workbench: a separate, persistent SITL you control from inside the Gazebo window (run on the HOST).
#   bench.sh start [cell]   start instance N (default 1) in the QVIX world at <cell> (default R1A10), Gazebo GUI on the VNC
#                           desktop (DISPLAY :1, GPU), mavsdk_server, sim-clock relay and the controller bench_ctl.py
#   bench.sh stop           stop everything of the workbench (the main SITL / grid_fly.sh runs are not touched)
#   bench.sh status         show what is running
#   bench.sh ui             restart only the GUI and the controller (after editing gui_bench.config / bench_ctl.py)
#   bench.sh cmd "run 2"    send a command from a shell (same as the command panel in the GUI)
# Everything is started detached (setsid), so closing the terminal, Ctrl-C, or the end of a flight script never stops the
# simulator; only "bench.sh stop" (or stop_sitl.sh all) does. stop_sitl.sh without "all" leaves the workbench alone.
# Your own scripts can fly the workbench vehicle too: mavsdk_server for it listens on gRPC 50051+N, e.g.
#   echo | docker exec -i starling-flight bash -c "cd ~/workspace && python3 path_flight_phase1_v13.py --sim --port 50052"
# When such a script ends, send "reset" (or press R in the 3D view) before the next run.
set -u
N=${BENCH_INSTANCE:-1}
SIM=$HOME/starling_sim; SC=$SIM/repo_export/scripts; LOGS=$SIM/logs; mkdir -p "$LOGS"
PART=starling_$N; MAVSDK_BIN=/usr/local/lib/python3.6/dist-packages/mavsdk/bin/mavsdk_server
GUI_DISPLAY=${BENCH_DISPLAY:-:1}
step(){ echo "=== [$(date +%H:%M:%S)] $*"; }
start_gui(){
  step "Gazebo GUI (작업대 화면, DISPLAY $GUI_DISPLAY, GPU)"
  docker exec -e GZ_PARTITION=$PART starling-gz bash -c "VGL=''; python3 -c 'open(\"/dev/nvidiactl\",\"rb\")' 2>/dev/null && command -v vglrun >/dev/null && VGL='vglrun -d egl';
    export __EGL_VENDOR_LIBRARY_FILENAMES=\$HOME/starling_sim/repo_export/gz/glvnd/10_nvidia.json;
    DISPLAY=$GUI_DISPLAY setsid \$VGL gz sim -g --gui-config \$HOME/starling_sim/repo_export/gz/gui_bench.config > /tmp/gzgui_bench$N.log 2>&1 < /dev/null &"
}
bench_camera(){   # aim the GUI camera into R1 from the north end (like grid_overlay.sh camera)
  for i in $(seq 1 30); do docker exec -e GZ_PARTITION=$PART starling-gz bash -c "gz service -l 2>/dev/null | grep -q '^/gui/move_to/pose'" && break; sleep 1; done
  docker exec -e GZ_PARTITION=$PART starling-gz bash -c "gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 5000 --req 'pose: {position: {x: 3.4, y: -1.5, z: 4.7}, orientation: {x: 0.2425, y: 0.2425, z: -0.6642, w: 0.6642}}'" >/dev/null 2>&1
}
start_map(){      # live mini-maps for the GUI panels (rack face + plan), keeps running with the workbench
  docker exec -d -e GZ_PARTITION=$PART starling-gz bash -c "setsid python3 $SC/live_map.py starling2_$N 5 R1 > /tmp/live_map_bench$N.log 2>&1"
}
start_panel(){    # button panel (Tk, host) over the right side of the Gazebo window on the VNC desktop
  pkill -f "[b]ench_panel.py" 2>/dev/null
  DISPLAY=$GUI_DISPLAY BENCH_INSTANCE=$N setsid python3 "$SC/bench_panel.py" > "$LOGS/bench_panel.log" 2>&1 < /dev/null &
}
start_ctl(){
  step "제어기 bench_ctl.py (로그 $LOGS/bench_ctl.log)"
  BENCH_INSTANCE=$N setsid python3 "$SC/bench_ctl.py" > "$LOGS/bench_ctl_stdout.log" 2>&1 < /dev/null &
  echo $! > "$LOGS/.bench_ctl_pid"
}

case "${1:-status}" in
start)
  CELL=${2:-R1A10}
  POSE=$(python3 "$SC/grid_mission.py" --spawn-pose --route "$CELL:N $CELL") || { echo "!! 셀 이름 오류: $CELL"; exit 1; }
  "$0" stop >/dev/null 2>&1
  TS=$(date +%Y%m%d_%H%M%S); SITL_LOG=$LOGS/bench_${TS}_sitl.log; echo "$SITL_LOG" > "$LOGS/.bench_sitl_log"
  step "작업대 SITL 기동 (인스턴스 $N, partition $PART, $CELL, 로그 $SITL_LOG)"
  setsid docker exec starling-gz bash -c "$SC/run_sitl_instance.sh $N qvix headless $POSE" > "$SITL_LOG" 2>&1 < /dev/null &
  for i in $(seq 1 150); do grep -a -q "New NED origin" "$SITL_LOG" 2>/dev/null && break
    grep -a -q -E "EXIT=|ninja: error" "$SITL_LOG" 2>/dev/null && { echo "!! SITL 기동 실패"; tail -20 "$SITL_LOG"; exit 1; }; sleep 1; printf "."; done; echo
  grep -a -q "New NED origin" "$SITL_LOG" || { echo "!! 글로벌 원점 설정 실패 ($SITL_LOG)"; exit 1; }
  start_gui
# the instance's origin setter (set_global_origin.py, udpin on the offboard port) must have released the port first,
# otherwise mavsdk_server fails with "bind error: Address already in use" and every mission hangs at "Connecting..."
for i in $(seq 1 30); do docker exec starling-gz pgrep -f "[s]et_global_origin.py udpin:0.0.0.0:$((14540 + N))" >/dev/null || break; sleep 1; done
  step "mavsdk_server (gRPC $((50051 + N)) <- UDP $((14540 + N))), 시뮬 시계 중계 (UDP $((14600 + N)))"
  docker exec -d starling-flight bash -c "setsid $MAVSDK_BIN -p $((50051 + N)) udp://:$((14540 + N)) > ~/mavsdk_server_bench$N.log 2>&1"
  docker exec -d -e GZ_PARTITION=$PART starling-gz bash -c "setsid python3 $SC/sim_clock_udp.py qvix $((14600 + N)) 50 > /tmp/sim_clock_bench$N.log 2>&1"
  start_ctl
  start_map
  bench_camera
  start_panel
  echo
  echo "작업대 준비 완료. VNC 화면(:1)의 Gazebo 창에서:"
  echo "  - 재생/일시정지: 왼쪽 아래 ▶/❚❚ 버튼 (비행도 함께 멈춤), 단축키 P"
  echo "  - 3D 화면을 한 번 클릭한 뒤 숫자 1-9 = 프리셋 비행, R = 리셋, X = 중지, L = 반복 켬/끔"
  echo "  - 오른쪽 '작업대 조종' 창: 경로 선택, 비행 시작, 일시정지/재개, 리셋, 중지, N회 반복, 계속 반복, 상태 표시"
  echo "종료: $0 stop"
  ;;
stop)
  [ -f "$LOGS/.bench_ctl_pid" ] && kill "$(cat "$LOGS/.bench_ctl_pid")" 2>/dev/null; rm -f "$LOGS/.bench_ctl_pid"
  pkill -f "[b]ench_ctl.py" 2>/dev/null; pkill -f "[b]ench_panel.py" 2>/dev/null
  docker exec starling-flight bash -c "pkill -f '[m]avsdk_server -p $((50051 + N)) '; for p in \$(pgrep -f '[g]rid_mission.py|[p]ath_flight_phase1'); do tr '\0' ' ' < /proc/\$p/cmdline | grep -q -- '--port $((50051 + N))' && kill \$p; done; true" >/dev/null 2>&1
  docker exec starling-gz bash -c "for p in \$(pgrep -f '[g]z sim -g'); do tr '\0' '\n' < /proc/\$p/environ | grep -q -x 'GZ_PARTITION=$PART' && kill \$p; done; true" >/dev/null 2>&1
  "$SC/stop_sitl_instance.sh" "$N"
  ;;
ui)   # restart only the Gazebo GUI and the controller (after editing gui_bench.config or bench_ctl.py); the flight state stays
  pkill -f "[b]ench_ctl.py" 2>/dev/null
  docker exec starling-gz bash -c "for p in \$(pgrep -f '[g]z sim -g'); do tr '\0' '\n' < /proc/\$p/environ | grep -q -x 'GZ_PARTITION=$PART' && kill \$p; done; true"
  docker exec -e GZ_PARTITION=$PART starling-gz pkill -f "[l]ive_map.py" >/dev/null 2>&1
  sleep 2; start_gui; start_ctl; start_map; bench_camera; start_panel; echo "GUI, 제어기, 조종 패널 재시작 완료"
  ;;
status)
  echo "제어기: $(pgrep -f '[b]ench_ctl.py' >/dev/null && echo 실행 중 || echo 없음)"
  docker exec starling-gz bash -c "for p in \$(pgrep -x px4); do tr '\0' ' ' < /proc/\$p/cmdline | grep -q -- '-i $N ' && echo \"PX4 -i $N: 실행 중 (pid \$p)\"; done;
    for p in \$(pgrep -f '[g]z sim'); do tr '\0' '\n' < /proc/\$p/environ | grep -q -x 'GZ_PARTITION=$PART' && echo \"gz: \$(tr '\0' ' ' < /proc/\$p/cmdline | cut -c1-60)\"; done; true"
  docker exec starling-flight bash -c "pgrep -af '[m]avsdk_server -p $((50051 + N)) ' >/dev/null && echo 'mavsdk_server $((50051 + N)): 실행 중' || echo 'mavsdk_server $((50051 + N)): 없음'"
  tail -3 "$LOGS/bench_ctl.log" 2>/dev/null
  ;;
cmd)
  docker exec -e GZ_PARTITION=$PART starling-gz gz topic -t /bench/cmd -m gz.msgs.StringMsg -p "data: \"${2:-list}\""
  ;;
*) echo "usage: $0 start [cell] | stop | status | ui | cmd \"<command>\""; exit 1 ;;
esac
