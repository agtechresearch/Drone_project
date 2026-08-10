#!/bin/bash
# 로봇 상태 실시간 모니터 (Ctrl+C 로 종료)

source "$(dirname "$0")/../setup.sh"

python3 /home/er/autonomous_nav/python/robot_status.py
