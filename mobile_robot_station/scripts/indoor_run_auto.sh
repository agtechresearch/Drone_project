#!/bin/bash
# 실내 경로 완전 자동 왕복 주행
# 시작점 좌우 기준 측정 → 전진 → 정지 → 회전 → 귀환 → 정지
#
# 사용법:
#   indoor_run_auto.sh                          # 기본 파라미터
#   indoor_run_auto.sh --scan                   # 전/후/좌/우 거리 확인
#   indoor_run_auto.sh --front-stop 0.9         # 전방 정지 거리 (기본 0.5m)
#   indoor_run_auto.sh --fspeed 0.50            # 전진 속도 (기본 0.12 m/s)

source "$(dirname "$0")/../setup.sh"

python3 /home/er/autonomous_nav/python/indoor_run_auto.py "$@"
