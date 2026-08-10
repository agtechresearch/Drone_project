#!/bin/bash
# 직접 제어 모드 시작 — navigation stack 없이 cmd_vel 직접 제어
# AMCL/move_base 불필요, myagv_odometry + 라이다만 실행
#
# 이후 다른 터미널에서:
#   move.sh forward 0.3
#   move.sh turn 90
#   stop_all.sh

source "$(dirname "$0")/../setup.sh"

echo "============================================"
echo " 직접 제어 모드 (navigation stack 없음)"
echo ""
echo " 이동 명령어 (다른 터미널에서):"
echo "   move.sh forward 0.3    # 전진 0.3m"
echo "   move.sh back 0.5       # 후진 0.5m"
echo "   move.sh left 0.3       # 왼쪽 0.3m"
echo "   move.sh right 0.3      # 오른쪽 0.3m"
echo "   move.sh turn 90        # 90도 회전"
echo "   stop_all.sh            # 비상 정지"
echo "============================================"

# 라이다 모터 전원 ON
echo "[1/2] 라이다 모터 전원 켜는 중..."
python3 /home/er/autonomous_nav/python/lidar_power_daemon.py &
LIDAR_PID=$!
sleep 1

# 오도메트리 노드 (cmd_vel → 시리얼 변환 + /odom 퍼블리시)
echo "[2/2] 오도메트리 노드 시작..."
roslaunch myagv_odometry myagv_active.launch

# 종료 시 정리
kill $LIDAR_PID 2>/dev/null
