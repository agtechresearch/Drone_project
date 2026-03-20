#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pi Camera + AprilTag 감지 테스트 (손으로 정렬용)
- 좌우(x) + 상하(y) + 앞뒤(마커크기) 3축 정렬 판정
- 비행 코드는 x + 마커크기만 제어하므로, y(높이)는 rangefinder로 별도 제어
- 실행 전 터미널1에서 먼저 실행:
  libcamera-vid --width 640 --height 480 --codec mjpeg -t 0 --listen -o tcp://127.0.0.1:8888
- 'q' 키로 종료
"""

import cv2
import sys
import time
import numpy as np
from pupil_apriltags import Detector

# ─────────────────────────────────────────
# 설정값 (비행 코드와 동일)
# ─────────────────────────────────────────
STREAM_URL         = "tcp://127.0.0.1:8888"
CAM_W, CAM_H       = 640, 480
ALIGN_THRESHOLD_PX = 60    # 좌우(x) 오차 허용
TAG_FAMILY         = "tag36h11"

TARGET_MARKER_PX   = 110   # 목표 마커 크기 → 앞뒤 거리 기준
PITCH_THRESHOLD_PX = 30    # 앞뒤 오차 허용

# 높이(y) 허용범위 — 비행 코드에서는 사용 안 하지만 손 정렬 확인용
Y_THRESHOLD_PX     = 60

# ─────────────────────────────────────────
# 초기화
# ─────────────────────────────────────────
detector = Detector(
    families=TAG_FAMILY,
    nthreads=2,
    quad_decimate=2.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
)

print(f"[INFO] 스트림 열기 시도: {STREAM_URL}")
cap = cv2.VideoCapture(STREAM_URL, cv2.CAP_FFMPEG)

if not cap.isOpened():
    print("[ERROR] 스트림을 열 수 없습니다.")
    sys.exit(1)

print("[INFO] 스트림 연결 성공! — 'q' 키로 종료")
print(f"[INFO] 좌우 허용: ±{ALIGN_THRESHOLD_PX}px  |  앞뒤 기준: {TARGET_MARKER_PX}px ±{PITCH_THRESHOLD_PX}px  |  높이 허용: ±{Y_THRESHOLD_PX}px")

cam_cx = CAM_W // 2
cam_cy = CAM_H // 2

# ─────────────────────────────────────────
# 메인 루프
# ─────────────────────────────────────────
while True:
    ret, frame_bgr = cap.read()
    if not ret or frame_bgr is None:
        time.sleep(0.05)
        continue

    frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_180)
    gray      = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    tags      = detector.detect(gray)

    # ── 카메라 중심 십자선 ──
    cv2.line(frame_bgr, (cam_cx - 20, cam_cy), (cam_cx + 20, cam_cy), (255, 255, 255), 1)
    cv2.line(frame_bgr, (cam_cx, cam_cy - 20), (cam_cx, cam_cy + 20), (255, 255, 255), 1)

    # ── 허용 오차 범위 박스 (x, y) ──
    cv2.rectangle(
        frame_bgr,
        (cam_cx - ALIGN_THRESHOLD_PX, cam_cy - Y_THRESHOLD_PX),
        (cam_cx + ALIGN_THRESHOLD_PX, cam_cy + Y_THRESHOLD_PX),
        (0, 255, 255), 1
    )

    if tags:
        tag = tags[0]
        cx  = int(tag.center[0])
        cy  = int(tag.center[1])

        err_x = cx - cam_cx
        err_y = cy - cam_cy

        # 마커 크기 계산 (4변 평균) → 앞뒤 거리
        side_lens = [
            np.linalg.norm(tag.corners[i] - tag.corners[(i + 1) % 4])
            for i in range(4)
        ]
        marker_px = int(sum(side_lens) / 4)
        err_pitch = marker_px - TARGET_MARKER_PX  # 양수: 너무 가까움(앞), 음수: 너무 멈(뒤)

        aligned_x     = abs(err_x)     <= ALIGN_THRESHOLD_PX
        aligned_y     = abs(err_y)     <= Y_THRESHOLD_PX
        aligned_pitch = abs(err_pitch) <= PITCH_THRESHOLD_PX
        aligned       = aligned_x and aligned_y and aligned_pitch

        # 태그 코너
        corners = tag.corners.astype(int)
        for i in range(4):
            cv2.line(frame_bgr, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)

        # 마커 중심점
        color_center = (0, 255, 0) if aligned else (0, 100, 255)
        cv2.circle(frame_bgr, (cx, cy), 6, color_center, -1)
        cv2.line(frame_bgr, (cam_cx, cam_cy), (cx, cy), (0, 180, 255), 1)

        # 태그 ID
        cv2.putText(frame_bgr, f"ID:{tag.tag_id}",
                    (cx + 8, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # 좌우 오차
        x_color = (0, 255, 0) if aligned_x else (0, 100, 255)
        cv2.putText(frame_bgr,
                    f"좌우 : {err_x:+d}px  (허용 +/-{ALIGN_THRESHOLD_PX})",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.52, x_color, 1)

        # 높이 오차
        y_color     = (0, 255, 0) if aligned_y else (0, 100, 255)
        y_hint      = "내려가세요" if err_y > 0 else "올라가세요" if err_y < 0 else "OK"
        cv2.putText(frame_bgr,
                    f"높이 : {err_y:+d}px  (허용 +/-{Y_THRESHOLD_PX})  {y_hint}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.52, y_color, 1)

        # 앞뒤 오차 (마커 크기)
        p_color     = (0, 255, 0) if aligned_pitch else (0, 100, 255)
        pitch_hint  = "뒤로가세요" if err_pitch > 0 else "앞으로가세요" if err_pitch < 0 else "OK"
        cv2.putText(frame_bgr,
                    f"앞뒤 : {marker_px}px / 목표 {TARGET_MARKER_PX}px  err={err_pitch:+d}  {pitch_hint}",
                    (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.52, p_color, 1)

        # 정렬 상태
        if aligned:
            status_text  = "[ ALIGNED ]"
            status_color = (0, 255, 0)
        else:
            parts = []
            if not aligned_x:
                parts.append("우측" if err_x > 0 else "좌측")
            if not aligned_y:
                parts.append(y_hint)
            if not aligned_pitch:
                parts.append(pitch_hint)
            status_text  = "조정: " + "  ".join(parts)
            status_color = (0, 100, 255)

        cv2.putText(frame_bgr, status_text,
                    (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.75, status_color, 2)

        print(f"[TAG] x={err_x:+d} y={err_y:+d} marker={marker_px}px(err={err_pitch:+d}) | {'ALIGNED' if aligned else 'ALIGNING'}",
              end="\r")

    else:
        cv2.putText(frame_bgr, "NO TAG",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        print("[TAG] 감지 없음", end="\r")

    cv2.imshow("AprilTag Tracker", frame_bgr)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("\n[CAM] 종료")