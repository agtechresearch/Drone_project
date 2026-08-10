#!/bin/bash
# 2단계: SLAM으로 완성한 지도 저장
# 1_slam_mapping.sh 실행 중에 다른 터미널에서 이 스크립트를 실행하세요.

source "$(dirname "$0")/../setup.sh"

MAP_DIR="/home/er/autonomous_nav/maps"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
MAP_NAME="${MAP_DIR}/map_${TIMESTAMP}"

echo "지도를 저장합니다: ${MAP_NAME}"
rosrun map_server map_saver -f "${MAP_NAME}"

if [ $? -eq 0 ]; then
    echo "[OK] 지도 저장 완료: ${MAP_NAME}.pgm / ${MAP_NAME}.yaml"
    # 최신 지도를 기본 링크로 업데이트
    ln -sf "${MAP_NAME}.yaml" "${MAP_DIR}/current_map.yaml"
    ln -sf "${MAP_NAME}.pgm"  "${MAP_DIR}/current_map.pgm"
    echo "[OK] current_map.yaml -> ${MAP_NAME}.yaml"
else
    echo "[ERROR] 지도 저장 실패. roscore와 gmapping이 실행 중인지 확인하세요."
fi
