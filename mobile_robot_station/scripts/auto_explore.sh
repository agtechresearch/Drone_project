#!/bin/bash
# 자율 탐색 + 지도 동시 작성 실행 스크립트
# explore_lite가 레이저 레이더 데이터로 frontier를 찾아 자동으로 이동하며 지도를 완성합니다.
#
# 사전 조건:
#   터미널 1: roscore
#   터미널 2: roslaunch myagv_odometry myagv_active.launch  ← 레이저 레이더 + 오도메트리
#   터미널 3: 이 스크립트

source "$(dirname "$0")/../setup.sh"

# VNC 화면(:0)에 RViz 표시
export DISPLAY=:0

# explore_lite 설치 확인
if ! rospack find explore_lite &>/dev/null; then
    echo "[ERROR] explore_lite가 설치되어 있지 않습니다."
    echo "  다음 명령으로 설치하세요:"
    echo "  sudo apt install ros-noetic-explore-lite"
    exit 1
fi

echo "============================================"
echo " 자율 탐색 시작"
echo " - gmapping: 레이저 레이더로 지도 실시간 작성"
echo " - move_base: 경로 계획 및 장애물 회피"
echo " - explore_lite: frontier 탐색 목표 자동 생성"
echo ""
echo " 탐색 완료 후 다른 터미널에서:"
echo "   bash $(dirname "$0")/2_save_map.sh"
echo "============================================"

roslaunch /home/er/autonomous_nav/launch/auto_explore.launch
