#!/bin/bash
# 매핑용 반복 직진 주행 — 전진 0.5m × N회, 사이에 대기.
# 주행 중 아무 키나 누르면(또는 Ctrl+C) 즉시 정지.
# ※ 라이다 장애물 감지 없음 (매핑 중 수동 감시). 1_make_map.sh 실행 중에 사용.
#
# 사용법:
#   map_drive.sh 50                     # 전진 0.5m × 50 = 25m (사이 2초 대기)
#   map_drive.sh 50 --wait 3 --step 0.5 # 대기/스텝 지정
#   map_drive.sh 20 --dir left          # 방향 (forward/back/left/right)
#   map_drive.sh 40 --speed 0.12        # 속도 지정

source "$(dirname "$0")/../setup.sh"

python3 /home/er/autonomous_nav/python/map_drive.py "$@"
