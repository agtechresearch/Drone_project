#!/bin/bash
# 바닥 테이프 색상 캘리브레이션 GUI (모니터 + 마우스 필요)
# ROS 불필요 — 카메라만 사용
#
# 조작법: l=라인 샘플, f=바닥 샘플, 드래그로 영역 선택,
#        p=마스크 미리보기, m=미리보기 모드 전환, r=초기화, s=저장, q=종료

export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/er/.Xauthority}"

python3 /home/er/autonomous_nav/python/line_calibrate.py "$@"
