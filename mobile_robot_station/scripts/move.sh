#!/bin/bash
# 로봇 기준 상대 이동
#
# 기본(--direct): move_base 없이 cmd_vel 직접 제어 + 오도메트리 피드백
#   myagv_odometry_node 만 실행되면 동작 (navigation stack 불필요)
#
# --nav: move_base 경유 (navigation stack 실행 중일 때)
# --timed: 오도메트리 없이 시간 기반 (비상용)
#
# 사용법:
#   move.sh forward 0.3            # 전진 0.3m  (직접 제어)
#   move.sh back 0.5               # 후진 0.5m
#   move.sh left 0.3               # 왼쪽 0.3m  (메카넘 측면)
#   move.sh right 0.3              # 오른쪽 0.3m
#   move.sh turn 90                # 반시계 90도
#   move.sh turn -45               # 시계 45도
#   move.sh dx 0.3 dy 0.2          # 대각선 이동
#   move.sh forward 0.3 --timed    # 시간 기반 (오도메트리 없어도 됨)
#   move.sh forward 0.3 --nav      # move_base 경유

source "$(dirname "$0")/../setup.sh"

if [ $# -eq 0 ]; then
    echo "사용법:"
    echo "  move.sh forward 0.3        # 전진 0.3m"
    echo "  move.sh back 0.5           # 후진 0.5m"
    echo "  move.sh left 0.3           # 왼쪽 0.3m (메카넘)"
    echo "  move.sh right 0.3          # 오른쪽 0.3m (메카넘)"
    echo "  move.sh turn 90            # 반시계 90도"
    echo "  move.sh turn -45           # 시계 45도"
    echo "  move.sh dx 0.3 dy 0.2      # 대각선 이동"
    echo ""
    echo "  옵션:"
    echo "  --timed   오도메트리 없이 시간 기반 (비상용)"
    echo "  --nav     move_base 경유 (navigation stack 필요)"
    exit 0
fi

# --nav 플래그: 기존 navigation stack 방식
if [[ "$*" == *"--nav"* ]]; then
    ARGS=("${@/--nav/}")
    python3 /home/er/autonomous_nav/python/move_relative.py "${ARGS[@]}"
else
    python3 /home/er/autonomous_nav/python/move_direct.py "$@"
fi
