#!/bin/bash
# 목표 지점으로 이동 (네비게이션 실행 중일 때 사용)
#
# 사용법:
#   go.sh <웨이포인트명>          # 예: go.sh A
#   go.sh <x> <y> [yaw_deg]     # 예: go.sh 1.5 2.0 90

source "$(dirname "$0")/../setup.sh"

WAYPOINTS_FILE="/home/er/autonomous_nav/config/waypoints.yaml"
NAV_SCRIPT="/home/er/autonomous_nav/python/nav_goal_sender.py"

if [ $# -eq 0 ]; then
    echo "사용법:"
    echo "  go.sh <웨이포인트명>          # 예: go.sh A"
    echo "  go.sh <x> <y> [yaw_deg]     # 예: go.sh 1.5 2.0 90"
    echo ""
    echo "웨이포인트 목록:"
    python3 -c "
import yaml
d = yaml.safe_load(open('$WAYPOINTS_FILE'))
for w in d.get('waypoints', []):
    print(f\"  {w['name']:<10}  x={w['x']:.2f}  y={w['y']:.2f}  yaw={w.get('yaw',0):.0f}deg\")
" 2>/dev/null || echo "  (waypoints.yaml 읽기 실패)"
    exit 1
fi

# 첫 번째 인자가 숫자이면 직접 좌표로 이동
if [[ "$1" =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; then
    X="$1"
    Y="${2:?'[ERROR] Y 좌표를 입력하세요'}"
    YAW="${3:-0}"
    echo "목표: x=$X, y=$Y, yaw=${YAW}deg"
    python3 "$NAV_SCRIPT" "$X" "$Y" "$YAW"
    exit $?
fi

# 웨이포인트 이름으로 좌표 조회
RESULT=$(python3 -c "
import yaml, sys
try:
    d = yaml.safe_load(open('$WAYPOINTS_FILE'))
except Exception as e:
    sys.stderr.write(f'YAML 읽기 실패: {e}\n')
    sys.exit(2)
name = sys.argv[1]
for w in d.get('waypoints', []):
    if w['name'] == name:
        print(w['x'], w['y'], w.get('yaw', 0))
        sys.exit(0)
sys.stderr.write(f\"웨이포인트 '{name}' 없음. go.sh 로 목록 확인\n\")
sys.exit(1)
" "$1" 2>&1)

EXIT=$?
if [ $EXIT -ne 0 ]; then
    echo "[ERROR] $RESULT"
    exit 1
fi

X=$(echo "$RESULT" | awk '{print $1}')
Y=$(echo "$RESULT" | awk '{print $2}')
YAW=$(echo "$RESULT" | awk '{print $3}')
echo "목표 '$1': x=$X, y=$Y, yaw=${YAW}deg"
python3 "$NAV_SCRIPT" "$X" "$Y" "$YAW"
