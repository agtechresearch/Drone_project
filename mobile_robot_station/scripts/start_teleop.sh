#!/bin/bash
# 키보드 텔레오퍼레이션 - SLAM 지도 작성 시 로봇 조종에 사용
# u i o
# j k l   (k = 정지)
# m , .

source "$(dirname "$0")/../setup.sh"

echo "키보드 조종 시작 (q/e: 속도 조절, Ctrl+C: 종료)"
roslaunch myagv_teleop myagv_teleop.launch
