#!/bin/bash
# 라이다 통로 중앙 유지 주행
#
# 사용법:
#   corridor.sh               # 전진 + 통로 중앙 유지
#   corridor.sh --scan        # 좌/우/전방 거리값만 출력 (센서 방향 확인)
#   corridor.sh --center      # 제자리에서 중앙 정렬만
#   corridor.sh --speed 0.10  # 전진 속도 지정

source "$(dirname "$0")/../setup.sh"

python3 /home/er/autonomous_nav/python/corridor_follow.py "$@"
