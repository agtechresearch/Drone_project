#!/bin/bash
# 1단계: SLAM으로 지도 작성
# YDLidar + 오도메트리 + gmapping을 실행합니다.
# 실행 후 다른 터미널에서 teleop으로 로봇을 조종하여 지도를 완성하세요.

source "$(dirname "$0")/../setup.sh"

export DISPLAY=:0

echo "============================================"
echo " SLAM 지도 작성 시작 (myagv SLAM laser)"
echo " 완료 후 Ctrl+C로 종료하고 2_save_map.sh 실행"
echo "============================================"

roslaunch myagv_navigation myagv_slam_laser.launch
