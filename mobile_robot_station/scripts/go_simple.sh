#!/bin/bash
# 순수동작 웨이포인트 주행 (move_base 없이 회전→직진→회전).
# 메카넘 저속 mix 데드밴드 우회용. 장애물 회피 없음(직선 경로).
#
#   go_simple.sh <웨이포인트명>       # 예: go_simple.sh 예냉실
#   go_simple.sh <x> <y> [yaw°]      # 예: go_simple.sh 1.0 0.5 90
#
# 전제: 네비게이션(AMCL) 실행 중 + set_initialpose.sh 로 초기위치 설정됨.

source "$(dirname "$0")/../setup.sh"

python3 "$(dirname "$0")/../python/go_simple.py" "$@"
