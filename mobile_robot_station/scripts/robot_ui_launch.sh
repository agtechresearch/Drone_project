#!/bin/bash
# 터치 한 번으로 로봇 UI 앱 실행:
#   저장된 맵으로 네비게이션(RViz 없이) 시작 → 터치 웨이포인트 GUI 실행.
#   GUI 안에서 RViz 버튼으로 켜고, 배터리 잔량도 표시됨.
#   (네비게이션이 이미 실행 중이면 GUI만 띄우고, 이 스크립트가 켠 경우에만 종료 시 정리)

source "$(dirname "$0")/../setup.sh"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/er/.Xauthority}"

LOG=/tmp/robot_ui_nav.log
STARTED_NAV=0

# 이미 move_base 가 떠 있는지 확인 (마스터 + move_base 액션)
if ! rostopic list 2>/dev/null | grep -q "/move_base/goal"; then
    echo "[robot_ui] 네비게이션 시작 (RViz 없이)... 로그: $LOG"
    bash "$(dirname "$0")/3_start_navigation.sh" --no-rviz > "$LOG" 2>&1 &
    STARTED_NAV=1
    # 마스터가 올라올 때까지 잠깐 대기 (GUI는 뜬 뒤 move_base 연결을 계속 재시도함)
    sleep 3
else
    echo "[robot_ui] 네비게이션이 이미 실행 중 — GUI만 실행"
fi

# 터치 GUI (블로킹). 네비게이션이 준비되면 GUI가 자동으로 "준비됨"으로 바뀜.
python3 "$(dirname "$0")/../python/waypoint_ui.py"

# 이 스크립트가 네비게이션을 켰다면 종료 시 정리
if [ "$STARTED_NAV" = "1" ]; then
    echo "[robot_ui] 종료 — 네비게이션 정리"
    bash "$(dirname "$0")/stop_all.sh" > /dev/null 2>&1
fi
