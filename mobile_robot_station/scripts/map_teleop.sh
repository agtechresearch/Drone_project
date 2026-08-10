#!/bin/bash
# 매핑용 단일키 스텝 드라이버
#   한 키 = 한 스텝(오도메트리 피드백 이동). move_direct 재사용.
#   1_make_map.sh(오도+라이다+gmapping) 실행 중에 별도 터미널에서 이 스크립트 실행.
#   이 터미널 창에 포커스를 두고 키를 누르세요.
#
#   i/k 전/후진  j/l 좌/우회전  u/o 좌/우 횡이동  r 360°스윕  +/- 스텝  space 정지  q 종료

source "$(dirname "$0")/../setup.sh"

python3 "$(dirname "$0")/../python/map_teleop.py" "$@"
