#!/bin/bash
# 웨이포인트 순찰 시작 (네비게이션 실행 중일 때 사용)
#
# 사용법:
#   patrol.sh                    # waypoints.yaml 전체 무한 순찰
#   patrol.sh --once             # 한 바퀴만
#   patrol.sh --loops 3          # 3바퀴
#   patrol.sh A B home           # 특정 웨이포인트만 순찰

source "$(dirname "$0")/../setup.sh"

WAYPOINTS_FILE="/home/er/autonomous_nav/config/waypoints.yaml"

# 웨이포인트 이름 인자와 옵션 분리
WAYPOINT_ARGS=()
OTHER_ARGS=()
for arg in "$@"; do
    if [[ "$arg" == --* ]]; then
        OTHER_ARGS+=("$arg")
    else
        WAYPOINT_ARGS+=("$arg")
    fi
done

echo "============================================"
echo " 웨이포인트 순찰 시작"
if [ ${#WAYPOINT_ARGS[@]} -gt 0 ]; then
    echo " 경로: ${WAYPOINT_ARGS[*]}"
else
    python3 -c "
import yaml
d = yaml.safe_load(open('$WAYPOINTS_FILE'))
names = [w['name'] for w in d.get('waypoints', [])]
print(' 경로:', ' → '.join(names), '(반복)')
" 2>/dev/null
fi
echo " 중단: Ctrl+C"
echo "============================================"

CMD=(python3 /home/er/autonomous_nav/python/patrol_navigator.py "${OTHER_ARGS[@]}")
if [ ${#WAYPOINT_ARGS[@]} -gt 0 ]; then
    CMD+=(--waypoints "${WAYPOINT_ARGS[@]}")
fi

"${CMD[@]}"
