#!/bin/bash
# 저장된 지도로 자율주행 네비게이션 시작
# AMCL + move_base(메카넘 파라미터 포함) + YDLidar + 오도메트리

source "$(dirname "$0")/../setup.sh"

# --no-rviz 플래그 분리 (터치 UI에서 RViz를 버튼으로 켤 때 사용)
RVIZ_ARG="true"
ARGS=()
for a in "$@"; do
    if [ "$a" = "--no-rviz" ]; then RVIZ_ARG="false"; else ARGS+=("$a"); fi
done

MAP_FILE="${ARGS[0]:-/home/er/autonomous_nav/maps/current_map.yaml}"

if [ ! -f "$MAP_FILE" ]; then
    echo "[ERROR] 지도 파일이 없습니다: $MAP_FILE"
    exit 1
fi

echo "============================================"
echo " 자율주행 네비게이션 시작"
echo " 지도: $MAP_FILE"
echo ""
echo " 이동 명령어 (다른 터미널에서):"
echo "   go.sh <웨이포인트명>       # 예: go.sh A"
echo "   go.sh <x> <y> [yaw]       # 예: go.sh 1.5 2.0 90"
echo "   patrol.sh                  # 웨이포인트 순찰"
echo "   stop_all.sh                # 비상 정지"
echo "============================================"

# 라이다 모터 전원 ON (GPIO 20 HIGH)
echo "[1/3] 라이다 모터 전원 켜는 중..."
python3 /home/er/autonomous_nav/python/lidar_power_daemon.py &
LIDAR_POWER_PID=$!
sleep 1

# 오도메트리 + 라이다 드라이버 (백그라운드)
echo "[2/3] 오도메트리 + 라이다 드라이버 시작..."
roslaunch myagv_odometry myagv_active.launch &
ODOM_PID=$!
sleep 5

# 네비게이션 스택 (map_server + AMCL + move_base + mecanum 파라미터)
echo "[3/3] 네비게이션 스택 시작..."
roslaunch /home/er/autonomous_nav/launch/navigation.launch map_file:="$MAP_FILE" rviz:="$RVIZ_ARG"

# 종료 시 정리
kill $ODOM_PID 2>/dev/null
kill $LIDAR_POWER_PID 2>/dev/null  # GPIO 20 LOW → 모터 정지
