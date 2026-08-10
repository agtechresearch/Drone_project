#!/bin/bash
# 카메라 기반 바닥 테이프 라인 추종
#
# 사용법:
#   line_follow.sh               # 주행 시작
#   line_follow.sh --debug       # 모니터에 검출 시각화 창 띄우기
#   line_follow.sh --speed 0.10  # 전진 속도 지정
#   line_follow.sh --no-drive    # cmd_vel 발행 없이 검출만 확인 (튜닝용)

source "$(dirname "$0")/../setup.sh"

python3 /home/er/autonomous_nav/python/line_follow.py "$@"
