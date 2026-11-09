#!/bin/bash
# Live Gazebo GUI for a wk_fly.sh run (run on the HOST, 2026-10-30). wk_fly.sh restarts the instance, so this waits for the
# NEW run's SITL to set its origin, then attaches a GUI client on VNC :1 (GPU), makes the QVIX module see-through and
# starts the aisle follow camera + markers (cam_follow.py).
#   INSTANCE=2 wk_live_gui.sh            # start it, then start wk_fly.sh with the same INSTANCE
#   wk_live_gui.sh stop
set -u
N=${INSTANCE:-2}; SC=$HOME/starling_sim/v3_marker/scripts; LOGS=$HOME/starling_sim/v3_marker/logs
DISP=${GUI_DISPLAY:-:1}; VIEW=${FOLLOW_VIEW:-aisle}
E="-e GZ_PARTITION=starling_$N -e QVIX_YAW_DEG=${QVIX_YAW_DEG:-0} -e CAM_FOLLOW_MODEL=starling2_tof_$N"
stop_gui(){
  docker exec starling-gz bash -c "for p in \$(pgrep -f '^gz sim -g'); do grep -qz 'GZ_PARTITION=starling_$N' /proc/\$p/environ && kill \$p; done
    for p in \$(pgrep -f '[c]am_follow.py'); do grep -qz 'GZ_PARTITION=starling_$N' /proc/\$p/environ && kill \$p; done" 2>/dev/null
}
[ "${1:-}" = "stop" ] && { stop_gui; exit 0; }

OLD=$(cat "$LOGS/.last_run_i$N" 2>/dev/null)
echo "새 wk_fly.sh 실행(인스턴스 $N) 대기..."
while :; do RUN=$(cat "$LOGS/.last_run_i$N" 2>/dev/null); [ -n "$RUN" ] && [ "$RUN" != "$OLD" ] && break; sleep 1; done
until grep -a -q "New NED origin" "$RUN/sitl.log" 2>/dev/null; do sleep 1; done
echo "SITL 기동 확인 ($RUN) -> GUI 연결"
stop_gui
docker exec $E starling-gz bash -c "VGL=''; python3 -c 'open(\"/dev/nvidiactl\",\"rb\")' 2>/dev/null && command -v vglrun >/dev/null && VGL='vglrun -d egl'
  export __EGL_VENDOR_LIBRARY_FILENAMES=\$HOME/starling_sim/repo_export/gz/glvnd/10_nvidia.json
  DISPLAY=$DISP setsid \$VGL gz sim -g --gui-config \$HOME/starling_sim/repo_export/gz/gui_record.config > /tmp/gzgui_live$N.log 2>&1 < /dev/null &"
# the transparency service exists only once the GUI scene is up; it TOGGLES, so call it once after it answers
for i in $(seq 1 30); do
  docker exec $E starling-gz bash -c "gz service -l | grep -q '^/gui/view/transparent$'" && break; sleep 1
done
sleep 3
docker exec $E starling-gz bash -c "timeout 10 gz service -s /gui/view/transparent --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: \"qvix_r_module\"'" >/dev/null && echo "  QVIX 모듈 반투명"
docker exec -d $E starling-gz bash -c "CAM_FOLLOW_ROLE=camera python3 $SC/cam_follow.py $VIEW 10 3600 > /tmp/cam_follow_live$N.log 2>&1"
docker exec -d $E starling-gz bash -c "CAM_FOLLOW_ROLE=markers python3 $SC/cam_follow.py $VIEW 10 3600 > /tmp/cam_follow_markers_live$N.log 2>&1"
echo "  추적 시점 시작 (view=$VIEW). VNC :1 (포트 5901)에서 보기"
