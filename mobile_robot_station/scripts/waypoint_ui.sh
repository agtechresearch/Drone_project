#!/bin/bash
# 터치스크린 웨이포인트 이동 UI (모니터에 전체화면)
# ※ 3_start_navigation.sh 로 네비게이션이 실행 중이어야 목표 전송이 됨.
#
# 웨이포인트는 config/waypoints.yaml 에서 읽음 (편집 후 재실행하면 버튼 갱신)
# 종료: 화면의 "종료" 버튼

source "$(dirname "$0")/../setup.sh"

export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/er/.Xauthority}"

python3 /home/er/autonomous_nav/python/waypoint_ui.py "$@"
