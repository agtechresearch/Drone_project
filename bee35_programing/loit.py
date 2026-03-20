#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
드론 자동이륙 + 순수 LOITER 호버링 측정 (비교용)
- 첫 정렬 감지 후 60초간 오버라이드 없이 LOITER 자체 호버링만 유지
- 로깅 구조는 정렬 비행(alighn_l.py)과 동일

실행 순서:
  터미널1: libcamera-vid --width 640 --height 480 --codec mjpeg --framerate 60 -t 0 --listen -o tcp://127.0.0.1:8888
  터미널2: mavproxy.py --master=/dev/ttyAMA0 --baudrate 115200 --out=udp:192.168.45.242:14550 --out=udp:127.0.0.1:14551
  터미널3: python loit.py
"""

import sys
import time
import threading
import csv
import math
import os
from datetime import datetime
import cv2
import numpy as np
from dronekit import connect, VehicleMode
from pupil_apriltags import Detector

# ─────────────────────────────────────────
# 설정값 (정렬 비행과 동일)
# ─────────────────────────────────────────
CONN               = "udp:127.0.0.1:14551"

TARGET_ALT_M       = 0.85
THR_START          = 1500
THR_STEP           = 2
THR_MAX            = 1630
THR_HOVER          = 1500
UPDATE_HZ          = 60
TIMEOUT_S          = 20.0
KILL_THRESHOLD     = 1000

STREAM_URL         = "tcp://127.0.0.1:8888"
CAM_W, CAM_H       = 640, 480
ALIGN_THRESHOLD_PX = 60
TAG_FAMILY         = "tag36h11"
MAX_CORRECTION     = 100

TARGET_MARKER_PX   = 90
PITCH_THRESHOLD_PX = 30

RECORD_DURATION    = 60.0   # 첫 정렬 후 측정 시간 (초)

# PID 게인 (정렬 비행과 동일 — 로그 비교용, 실제 제어는 안 함)
# Roll — 정밀
KP_ROLL  = 0.15
KI_ROLL  = 0.005
KD_ROLL  = 0.10

# Pitch — 오차 줄이기
KP_PITCH = 0.30
KI_PITCH = 0.02
KD_PITCH = 0.05

I_LIMIT  = 20

# ─────────────────────────────────────────
# 공유 상태
# ─────────────────────────────────────────
_ch5_value = 0

_marker = {
    "detected":    False,
    "error_x":     0,
    "error_pitch": 0,
    "aligned":     False,
}
_marker_lock    = threading.Lock()
_camera_running = False


# ─────────────────────────────────────────
# RC 리스너
# ─────────────────────────────────────────
def start_rc_listener(vehicle):
    @vehicle.on_message('RC_CHANNELS')
    def rc_handler(self, name, msg):
        global _ch5_value
        _ch5_value = msg.chan5_raw


def manual_override_requested():
    if _ch5_value > KILL_THRESHOLD:
        print(f"\n[SAFETY] MANUAL SWITCH DETECTED (CH5={_ch5_value})")
        return True
    return False


# ─────────────────────────────────────────
# 드론 유틸
# ─────────────────────────────────────────
def get_range_alt(vehicle):
    try:
        d = vehicle.rangefinder.distance
        if d is None:
            return -1.0
        return float(d)
    except Exception:
        return -1.0


def set_overrides(vehicle, roll=1500, pitch=1500, thr=1500, yaw=1500):
    vehicle.channels.overrides = {
        "1": int(roll),
        "2": int(pitch),
        "3": int(thr),
        "4": int(yaw),
    }


def clear_overrides(vehicle):
    vehicle.channels.overrides = {}


def wait_mode(vehicle, name, timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if vehicle.mode.name == name:
            return True
        time.sleep(0.1)
    return False


def get_attitude_deg(vehicle):
    try:
        att = vehicle.attitude
        return math.degrees(att.roll), math.degrees(att.pitch)
    except Exception:
        return 0.0, 0.0


# VIBRATION 메시지 직접 수신
_vibration = {"x": 0.0, "y": 0.0, "z": 0.0}

def start_vibration_listener(vehicle):
    @vehicle.on_message('VIBRATION')
    def vib_handler(self, name, msg):
        _vibration["x"] = msg.vibration_x
        _vibration["y"] = msg.vibration_y
        _vibration["z"] = msg.vibration_z

def get_vibration():
    return _vibration["x"], _vibration["y"], _vibration["z"]


# ─────────────────────────────────────────
# 로그 초기화
# ─────────────────────────────────────────
def init_logs():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs("logs", exist_ok=True)

    rt_path   = f"logs/loiter_realtime_{ts}.csv"
    rt_file   = open(rt_path, "w", newline="")
    rt_writer = csv.writer(rt_file)
    rt_writer.writerow([
        "elapsed_s",
        "err_x_px", "err_pitch_px",
        "target_roll_us", "target_pitch_us",
        "actual_roll_deg", "actual_pitch_deg",
        "vib_x", "vib_y", "vib_z",
        "aligned",
    ])

    sum_path   = f"logs/loiter_summary_{ts}.csv"
    sum_file   = open(sum_path, "w", newline="")
    sum_writer = csv.writer(sum_file)
    sum_writer.writerow([
        "attempt",
        "align_time_s",
        "total_abs_err_px",
        "rms_vib",
    ])

    print(f"[LOG] 실시간: {rt_path}")
    print(f"[LOG] 요약:   {sum_path}")
    return rt_writer, rt_file, sum_writer, sum_file


# ─────────────────────────────────────────
# 카메라 스레드
# ─────────────────────────────────────────
def camera_thread_fn():
    global _camera_running

    cam_cx = CAM_W // 2
    cam_cy = CAM_H // 2

    detector = Detector(
        families=TAG_FAMILY,
        nthreads=2,
        quad_decimate=2.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
    )

    print(f"[CAM] 스트림 연결 시도: {STREAM_URL}")
    cap = cv2.VideoCapture(STREAM_URL, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("[CAM] 스트림 연결 실패 — libcamera-vid 실행 중인지 확인하세요.")
        _camera_running = False
        return

    print("[CAM] 카메라 스레드 시작")

    while _camera_running:
        ret, frame_bgr = cap.read()
        if not ret or frame_bgr is None:
            time.sleep(0.05)
            continue

        frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_180)
        gray      = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        tags      = detector.detect(gray)

        cv2.line(frame_bgr, (cam_cx - 20, cam_cy), (cam_cx + 20, cam_cy), (255, 255, 255), 1)
        cv2.line(frame_bgr, (cam_cx, cam_cy - 20), (cam_cx, cam_cy + 20), (255, 255, 255), 1)
        cv2.rectangle(
            frame_bgr,
            (cam_cx - ALIGN_THRESHOLD_PX, cam_cy - ALIGN_THRESHOLD_PX),
            (cam_cx + ALIGN_THRESHOLD_PX, cam_cy + ALIGN_THRESHOLD_PX),
            (0, 255, 255), 1
        )

        if tags:
            tag = tags[0]
            cx  = int(tag.center[0])
            cy  = int(tag.center[1])

            err_x = cx - cam_cx

            corners   = tag.corners.astype(int)
            side_lens = [
                np.linalg.norm(tag.corners[i] - tag.corners[(i + 1) % 4])
                for i in range(4)
            ]
            marker_px = int(sum(side_lens) / 4)
            err_pitch = marker_px - TARGET_MARKER_PX

            aligned = (abs(err_x)     <= ALIGN_THRESHOLD_PX and
                       abs(err_pitch) <= PITCH_THRESHOLD_PX)

            for i in range(4):
                cv2.line(frame_bgr, tuple(corners[i]), tuple(corners[(i+1)%4]), (0, 255, 0), 2)

            color_center = (0, 255, 0) if aligned else (0, 100, 255)
            cv2.circle(frame_bgr, (cx, cy), 6, color_center, -1)
            cv2.line(frame_bgr, (cam_cx, cam_cy), (cx, cy), (0, 180, 255), 1)

            cv2.putText(frame_bgr, f"ID:{tag.tag_id}",
                        (cx + 8, cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(frame_bgr,
                        f"err_x:{err_x:+d}px  pitch:{err_pitch:+d}px",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            cv2.putText(frame_bgr,
                        f"marker:{marker_px}px  target:{TARGET_MARKER_PX}px",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 0), 1)

            status_text  = "ALIGNED" if aligned else "ALIGNING..."
            status_color = (0, 255, 0) if aligned else (0, 100, 255)
            cv2.putText(frame_bgr, status_text,
                        (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

            cv2.putText(frame_bgr, "[ LOITER ONLY ]",
                        (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 0), 2)

            with _marker_lock:
                _marker["detected"]    = True
                _marker["error_x"]     = err_x
                _marker["error_pitch"] = err_pitch
                _marker["aligned"]     = aligned

        else:
            cv2.putText(frame_bgr, "NO TAG",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            with _marker_lock:
                _marker["detected"]    = False
                _marker["error_x"]     = 0
                _marker["error_pitch"] = 0
                _marker["aligned"]     = False

        cv2.imshow("LOITER ONLY", frame_bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("\n[CAM] 카메라 스레드 종료")


# ─────────────────────────────────────────
# 메인 비행 루프
# ─────────────────────────────────────────
def main():
    global _camera_running

    _camera_running = True
    cam_thread = threading.Thread(target=camera_thread_fn, daemon=True)
    cam_thread.start()

    time.sleep(1.0)
    if not _camera_running:
        print("[ERROR] 카메라 연결 실패 — 종료합니다.")
        sys.exit(1)

    print(f"[CONNECT] {CONN}")
    vehicle = connect(CONN, wait_ready=True, timeout=60)
    start_rc_listener(vehicle)

    vehicle.mode = VehicleMode("LOITER")
    if not wait_mode(vehicle, "LOITER", timeout=8.0):
        raise RuntimeError("Mode change failed")

    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.1)
    print("[ARM] 완료")

    start_vibration_listener(vehicle)
    rt_writer, rt_file, sum_writer, sum_file = init_logs()

    thr   = THR_START
    start = time.time()

    try:
        # ── 1단계: 이륙 ──────────────────────────
        print(f"[TAKEOFF] 목표 고도 {TARGET_ALT_M}m 상승 중...")
        while True:
            if manual_override_requested():
                break

            alt = get_range_alt(vehicle)
            if alt < 0:
                raise RuntimeError("Rangefinder not available")

            if alt >= TARGET_ALT_M:
                print(f"[TAKEOFF] 목표 고도 도달 ({alt:.2f}m)")
                break

            if time.time() - start > TIMEOUT_S:
                raise RuntimeError("Takeoff timeout")

            thr = min(thr + THR_STEP, THR_MAX)
            set_overrides(vehicle, thr=thr)
            time.sleep(1 / UPDATE_HZ)

        # ── 2단계: 안정화 (2초) ──────────────────
        print("[STABILIZE] 2초 안정화 중...")
        manual_during_stabilize = False
        for _ in range(int(2 * UPDATE_HZ)):
            if manual_override_requested():
                manual_during_stabilize = True
                break
            set_overrides(vehicle, thr=THR_HOVER)
            time.sleep(1 / UPDATE_HZ)

        # ── 3단계: 전 채널 1500 고정 후 첫 정렬 대기 ──
        if not manual_during_stabilize:
            print("[LOITER] 스로틀 인계 중 (1초)...")
            for _ in range(int(1.0 * UPDATE_HZ)):
                if manual_override_requested():
                    break
                set_overrides(vehicle, roll=1500, pitch=1500, thr=THR_HOVER, yaw=1500)
                time.sleep(1 / UPDATE_HZ)
            vehicle.channels.overrides = {"1": 1500, "2": 1500, "3": 1500, "4": 1500}

            print("[LOITER] 스로틀 인계 완료 — 첫 정렬 감지 대기 중...")
            print("         (전 채널 중립 고정, 조종기 입력 차단)")
            print("         CH5 스위치로 수동 전환 가능")

            log_start        = time.time()
            segment_start    = time.time()
            first_align_time = None
            was_aligned      = False
            attempt          = 0

            sum_abs_err_x     = 0
            sum_abs_err_pitch = 0
            sum_vib_sq        = 0.0
            loop_count        = 0

            while True:
                if manual_override_requested():
                    break

                if first_align_time is not None:
                    remaining = RECORD_DURATION - (time.time() - first_align_time)
                    if remaining <= 0:
                        print("\n[TIMER] 완료 — 착륙합니다.")
                        break
                    if int(remaining) % 5 == 0 and remaining % 1 < (1 / UPDATE_HZ) * 2:
                        print(f"\n[TIMER] 착륙까지 {int(remaining)}초...")

                with _marker_lock:
                    detected  = _marker["detected"]
                    aligned   = _marker["aligned"]
                    err_x     = _marker["error_x"]
                    err_pitch = _marker["error_pitch"]

                # 전 채널 중립 유지 (매 틱마다 갱신)
                vehicle.channels.overrides = {"1": 1500, "2": 1500, "3": 1500, "4": 1500}

                actual_roll, actual_pitch = get_attitude_deg(vehicle)
                vib_x, vib_y, vib_z      = get_vibration()
                elapsed = time.time() - log_start

                # 첫 정렬 이후부터만 로그 기록
                if first_align_time is not None:
                    rt_writer.writerow([
                        f"{elapsed:.3f}",
                        err_x, err_pitch,
                        1500, 1500,
                        f"{actual_roll:.2f}", f"{actual_pitch:.2f}",
                        f"{vib_x:.4f}", f"{vib_y:.4f}", f"{vib_z:.4f}",
                        1 if aligned else 0,
                    ])

                    if not aligned:
                        sum_abs_err_x     += abs(err_x)
                        sum_abs_err_pitch += abs(err_pitch)
                    sum_vib_sq += vib_x**2 + vib_y**2 + vib_z**2
                    loop_count += 1

                # 정렬 진입 시 처리
                if aligned and not was_aligned:
                    if first_align_time is None:
                        first_align_time = time.time()
                        log_start        = first_align_time
                        print(f"\n[TIMER] 첫 정렬 감지 — {RECORD_DURATION:.0f}초 측정 시작")
                    else:
                        attempt += 1
                        align_time = time.time() - segment_start
                        rms_vib    = math.sqrt(sum_vib_sq / loop_count) if loop_count else 0
                        sum_writer.writerow([
                            attempt,
                            f"{align_time:.2f}",
                            sum_abs_err_x + sum_abs_err_pitch,
                            f"{rms_vib:.4f}",
                        ])
                        sum_file.flush()
                        print(f"\n[LOG] #{attempt} 재진입 — 소요시간:{align_time:.2f}s "
                              f"누적오차:{sum_abs_err_x + sum_abs_err_pitch} rms_vib:{rms_vib:.4f}")

                # 정렬 이탈 시 누적값 및 구간 타이머 리셋
                if not aligned and was_aligned:
                    sum_abs_err_x = sum_abs_err_pitch = 0
                    sum_vib_sq    = 0.0
                    loop_count    = 0
                    segment_start = time.time()

                was_aligned = aligned

                if detected:
                    status    = "IN RANGE ✓" if aligned else f"err_x={err_x:+d} pitch={err_pitch:+d}"
                    timer_str = f" | 남은시간:{RECORD_DURATION - (time.time() - first_align_time):.1f}s" \
                                if first_align_time else " | 첫 정렬 대기 중..."
                    print(f"[LOITER] {status}{timer_str}", end="\r")
                else:
                    print("[LOITER] 마커 미감지", end="\r")

                time.sleep(1 / UPDATE_HZ)

        # ── 4단계: 착륙 ──────────────────────────
        print("[LAND] 착륙 모드 전환...")
        vehicle.mode = VehicleMode("LAND")
        wait_mode(vehicle, "LAND", timeout=5.0)

    finally:
        print("\n[CLEANUP] 종료")
        _camera_running = False
        rt_file.flush()
        rt_file.close()
        sum_file.close()
        clear_overrides(vehicle)
        vehicle.close()
        cam_thread.join(timeout=3.0)


if __name__ == "__main__":
    main()