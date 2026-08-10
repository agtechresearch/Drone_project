#!/bin/bash
# 현재 로봇 위치를 웨이포인트로 저장
#
# 사용법:
#   save_waypoint.sh <이름>              # 현재 위치 저장
#   save_waypoint.sh <이름> --wait 3.0   # 도착 후 대기 시간 지정
#   save_waypoint.sh --list              # 저장된 웨이포인트 목록
#   save_waypoint.sh --delete <이름>     # 웨이포인트 삭제

source "$(dirname "$0")/../setup.sh"

if [ $# -eq 0 ]; then
    echo "사용법:"
    echo "  save_waypoint.sh <이름>              # 현재 위치 저장"
    echo "  save_waypoint.sh <이름> --wait 3.0   # 도착 후 대기 시간 지정"
    echo "  save_waypoint.sh --list              # 저장된 웨이포인트 목록"
    echo "  save_waypoint.sh --delete <이름>     # 웨이포인트 삭제"
    exit 0
fi

python3 /home/er/autonomous_nav/python/save_waypoint.py "$@"
