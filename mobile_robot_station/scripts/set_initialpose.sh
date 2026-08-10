#!/bin/bash
# AMCL 초기위치(/initialpose) 발행 — RViz "2D Pose Estimate" 대체.
# 로봇을 해당 map 좌표·방향에 물리적으로 둔 상태에서 실행할 것.
#
#   set_initialpose.sh                # 원점 (0,0,0deg) — 로봇을 매핑 시작점에 둔 경우
#   set_initialpose.sh 1.5 2.0 90     # x=1.5m y=2.0m yaw=90deg
#
# 발행 후 /amcl_pose 로 반영 여부까지 확인해준다.

source "$(dirname "$0")/../setup.sh"

python3 "$(dirname "$0")/../python/set_initialpose.py" "$@"
