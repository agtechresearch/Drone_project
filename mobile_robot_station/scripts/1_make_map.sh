#!/bin/bash
# SLAM 맵 제작 — gmapping
# myagv_active(오도메트리+라이다) + gmapping 실행
#
# 순서:
#   터미널 1: bash 1_make_map.sh          (이 스크립트)
#   터미널 2: bash move.sh forward 0.5    (이동하며 맵 구성)
#   터미널 2: bash 2_save_map.sh          (다 돌았으면 저장)

source "$(dirname "$0")/../setup.sh"

echo "============================================"
echo " SLAM 맵 제작 모드 (gmapping)"
echo ""
echo " 다른 터미널에서 이동 명령으로 공간을 탐색하세요:"
echo "   move.sh forward 0.5"
echo "   move.sh turn 90"
echo "   move.sh back 0.3"
echo ""
echo " 맵 저장 (탐색 완료 후):"
echo "   bash 2_save_map.sh"
echo "============================================"

# 라이다 모터 전원 ON
echo "[1/3] 라이다 모터 전원 켜는 중..."
python3 /home/er/autonomous_nav/python/lidar_power_daemon.py &
LIDAR_PID=$!
sleep 1

# 오도메트리 + 라이다 드라이버 + TF
echo "[2/3] 오도메트리 + 라이다 시작..."
roslaunch myagv_odometry myagv_active.launch &
ODOM_PID=$!
sleep 5

# gmapping SLAM
echo "[3/3] gmapping SLAM 시작..."
roslaunch myagv_navigation gmapping.launch

# 종료 시 정리
kill $ODOM_PID 2>/dev/null
kill $LIDAR_PID 2>/dev/null
