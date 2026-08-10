#!/bin/bash
# 메카넘 휠 최적화 DWA 파라미터 적용
# 네비게이션(3_start_navigation.sh)이 실행 중일 때 사용하세요.

source "$(dirname "$0")/../setup.sh"

PARAM_FILE="/home/er/autonomous_nav/config/mecanum_dwa_params.yaml"

echo "메카넘 DWA 파라미터 적용 중..."
rosrun dynamic_reconfigure dynparam load /move_base/DWAPlannerROS "$PARAM_FILE"

if [ $? -eq 0 ]; then
    echo "[OK] 메카넘 파라미터 적용 완료 (측면 이동 활성화)"
else
    echo "[ERROR] 파라미터 적용 실패. move_base가 실행 중인지 확인하세요."
fi
