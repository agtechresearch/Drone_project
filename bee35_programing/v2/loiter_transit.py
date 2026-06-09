#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[대조군] AprilTag 정렬 + 우 이동 + LOITER 단독 측정 실험

실험 흐름:
  1. 이륙 (TARGET_ALT_M)
  2. 시작 마커(START_TAG_ID) 정렬       — PID 알고리즘 적용 (실험군과 동일)
  3. 시작 위치 호버링 (START_HOVER_S)   — LOITER 단독
  4. 우 이동 (TRANSIT_ROLL, TRANSIT_S)
  5. 도착 마커(GOAL_TAG_ID) 감지 대기   — 최대 GOAL_ACQUIRE_TIMEOUT_S
  6. 도착지 측정 (GOAL_MEASURE_S)       — LOITER 단독 (PID 미적용)
  7. 착륙

실험군(align_transit.py)과 동일한 흐름이며, 6단계에서만 차이가 있음:
  - 실험군: PID 정렬 적용
  - 대조군: 오버라이드 없이 LOITER 단독

실행 순서:
  터미널1: libcamera-vid --width 640 --height 480 --codec mjpeg --framerate 60 -t 0 --listen -o tcp://127.0.0.1:8888
  터미널2: mavproxy.py --master=/dev/ttyAMA0 --baudrate 115200 --out=udp:192.168.45.242:14550 --out=udp:127.0.0.1:14551
  터미널3: python loiter_transit.py
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
# 설정값
# ─────────────────────────────────────────
CONN               = "udp:127.0.0.1:14551"

# 그룹 식별 (로그/요약 파일명에 사용)
GROUP_NAME         = "loiter"   # 실험군은 "align"

# 이륙/호버 기본
TARGET_ALT_M       = 0.85
THR_START          = 1500
THR_STEP           = 2
THR_MAX            = 1630
THR_HOVER          = 1500
UPDATE_HZ          = 60
TIMEOUT_S          = 20.0
KILL_THRESHOLD     = 1000

# 카메라
STREAM_URL         = "tcp://127.0.0.1:8888"
CAM_W, CAM_H       = 640, 480
TAG_FAMILY         = "tag36h11"

# 마커 ID
START_TAG_ID       = 1
GOAL_TAG_ID        = 2

# 정렬 임계값
ALIGN_THRESHOLD_PX = 60       # 좌우 허용 오차
PITCH_THRESHOLD_PX = 30       # 거리 허용 오차
TARGET_MARKER_PX   = 90       # 목표 거리에서의 마커 크기

# PID 보정 최대값 (μs)
MAX_CORRECTION     = 100

# 실험 단계 시간
START_ALIGN_TIMEOUT_S   = 15.0   # 시작 정렬 최대 대기 (실패 시 강제 진행)
START_ALIGN_HOLD_S      = 0.5    # 정렬 성공 인정 지속 시간
START_HOVER_S           = 10.0   # 시작 정렬 후 호버링 시간 (LOITER 단독)
TRANSIT_ROLL            = 1570   # 우 이동 시 roll 오버라이드
TRANSIT_DURATION_S      = 3.0    # 우 이동 지속 시간
GOAL_ACQUIRE_TIMEOUT_S  = 5.0    # 도착지 마커 감지 대기 (실패 임계)
GOAL_MEASURE_S          = 10.0   # 도착지 오차 측정 시간

# PID 게인 (alighn_l.py 기반)
KP_ROLL  = 0.15
KI_ROLL  = 0.005
KD_ROLL  = 0.18

KP_PITCH = 0.30
KI_PITCH = 0.02
KD_PITCH = 0.12

I_LIMIT  = 20

# 미탐지 처리
NO_DETECT_HOLD_S   = 0.5     # 이 시간 안에서는 마지막 보정값 유지

# 실험군 플래그 (대조군이므로 False)
APPLY_PID_AT_GOAL  = False

# ─────────────────────────────────────────
# 공유 상태 (스레드 간)
# ─────────────────────────────────────────
_ch5_value = 0

# 현재 추적 대상 마커 ID (None이면 모든 마커)
_target_id      = None
_target_id_lock = threading.Lock()

_marker = {
    "detected":    False,
    "tag_id":      -1,
    "error_x":     0,
    "error_pitch": 0,
    "aligned":     False,
    "last_seen":   0.0,
}
_marker_lock    = threading.Lock()
_camera_running = False


def set_target_marker(tag_id):
    """카메라 스레드가 추적할 마커 ID를 변경."""
    global _target_id
    with _target_id_lock:
        _target_id = tag_id
    # 마커 상태 초기화 (이전 ID의 잔재 제거)
    with _marker_lock:
        _marker["detected"] = False
        _marker["aligned"]  = False
        _marker["error_x"]     = 0
        _marker["error_pitch"] = 0


def get_target_marker():
    with _target_id_lock:
        return _target_id


# ─────────────────────────────────────────
# RC 리스너 (CH5 킬스위치)
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

    # 실시간 로그 (phase, marker_id 컬럼 추가)
    rt_path = f"logs/{GROUP_NAME}_transit_realtime_{ts}.csv"
    rt_file = open(rt_path, "w", newline="")
    rt_writer = csv.writer(rt_file)
    rt_writer.writerow([
        "elapsed_s", "phase", "marker_id",
        "err_x_px", "err_pitch_px",
        "target_roll_us", "target_pitch_us",
        "actual_roll_deg", "actual_pitch_deg",
        "vib_x", "vib_y", "vib_z",
        "aligned",
    ])

    # 요약 로그 (실험 전체 결과 1행)
    sum_path = f"logs/{GROUP_NAME}_transit_summary_{ts}.csv"
    sum_file = open(sum_path, "w", newline="")
    sum_writer = csv.writer(sum_file)
    sum_writer.writerow([
        "timestamp", "group", "success",
        "start_align_time_s", "start_align_max_err_x",
        "goal_acquire_time_s",
        "goal_mean_err_x", "goal_mean_err_pitch",
        "goal_rms_err_x",  "goal_rms_err_pitch",
        "goal_final_err_x","goal_final_err_pitch",
        "goal_aligned_ratio",
        "goal_rms_vib",
        "fail_reason",
    ])

    print(f"[LOG] 실시간: {rt_path}")
    print(f"[LOG] 요약:   {sum_path}")
    return rt_writer, rt_file, sum_writer, sum_file, ts


# ─────────────────────────────────────────
# 카메라 스레드 (ID 기반 마커 필터링)
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

        # 중심 십자와 허용범위 박스
        cv2.line(frame_bgr, (cam_cx - 20, cam_cy), (cam_cx + 20, cam_cy), (255, 255, 255), 1)
        cv2.line(frame_bgr, (cam_cx, cam_cy - 20), (cam_cx, cam_cy + 20), (255, 255, 255), 1)
        cv2.rectangle(
            frame_bgr,
            (cam_cx - ALIGN_THRESHOLD_PX, cam_cy - ALIGN_THRESHOLD_PX),
            (cam_cx + ALIGN_THRESHOLD_PX, cam_cy + ALIGN_THRESHOLD_PX),
            (0, 255, 255), 1
        )

        # 현재 타겟 ID
        target = get_target_marker()

        # 타겟 ID와 일치하는 태그 선택 (없으면 첫 번째)
        chosen = None
        if tags:
            if target is None:
                chosen = tags[0]
            else:
                for t in tags:
                    if t.tag_id == target:
                        chosen = t
                        break

        # 모든 검출된 태그를 화면에 표시 (디버그용)
        for t in tags:
            corners = t.corners.astype(int)
            is_target = (target is None) or (t.tag_id == target)
            color = (0, 255, 0) if is_target else (100, 100, 100)
            thick = 2 if is_target else 1
            for i in range(4):
                pt1 = tuple(corners[i])
                pt2 = tuple(corners[(i + 1) % 4])
                cv2.line(frame_bgr, pt1, pt2, color, thick)
            cv2.putText(frame_bgr, f"ID:{t.tag_id}",
                        (corners[0][0] + 5, corners[0][1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        if chosen is not None:
            cx = int(chosen.center[0])
            cy = int(chosen.center[1])
            err_x = cx - cam_cx

            side_lens = [
                np.linalg.norm(chosen.corners[i] - chosen.corners[(i + 1) % 4])
                for i in range(4)
            ]
            marker_px = int(sum(side_lens) / 4)
            err_pitch = marker_px - TARGET_MARKER_PX

            aligned = (abs(err_x)     <= ALIGN_THRESHOLD_PX and
                       abs(err_pitch) <= PITCH_THRESHOLD_PX)

            color_center = (0, 255, 0) if aligned else (0, 100, 255)
            cv2.circle(frame_bgr, (cx, cy), 6, color_center, -1)
            cv2.line(frame_bgr, (cam_cx, cam_cy), (cx, cy), (0, 180, 255), 1)

            cv2.putText(frame_bgr,
                        f"TARGET ID:{chosen.tag_id}  err_x:{err_x:+d}  pitch:{err_pitch:+d}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            cv2.putText(frame_bgr,
                        f"marker:{marker_px}px  target:{TARGET_MARKER_PX}px",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 0), 1)

            status_text  = "ALIGNED" if aligned else "ALIGNING..."
            status_color = (0, 255, 0) if aligned else (0, 100, 255)
            cv2.putText(frame_bgr, status_text,
                        (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

            with _marker_lock:
                _marker["detected"]    = True
                _marker["tag_id"]      = chosen.tag_id
                _marker["error_x"]     = err_x
                _marker["error_pitch"] = err_pitch
                _marker["aligned"]     = aligned
                _marker["last_seen"]   = time.time()

        else:
            msg = "NO TARGET TAG" if target is not None else "NO TAG"
            cv2.putText(frame_bgr, msg,
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            if target is not None:
                cv2.putText(frame_bgr, f"looking for ID:{target}",
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1)

            with _marker_lock:
                _marker["detected"]    = False
                _marker["error_x"]     = 0
                _marker["error_pitch"] = 0
                _marker["aligned"]     = False

        # 타겟 ID 표시
        if target is not None:
            cv2.putText(frame_bgr, f"[ TARGET: ID {target} ]",
                        (10, CAM_H - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 0), 2)

        cv2.imshow("AprilTag Transit Experiment", frame_bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("\n[CAM] 카메라 스레드 종료")


# ─────────────────────────────────────────
# PID 상태
# ─────────────────────────────────────────
_pid = {
    "integral_x":     0.0,
    "integral_pitch": 0.0,
    "prev_err_x":     0.0,
    "prev_err_pitch": 0.0,
    "prev_time":      None,
    "last_corr":      (0, 0),
    "last_corr_time": 0.0,
}


def reset_pid():
    _pid["integral_x"]     = 0.0
    _pid["integral_pitch"] = 0.0
    _pid["prev_err_x"]     = 0.0
    _pid["prev_err_pitch"] = 0.0
    _pid["prev_time"]      = None
    _pid["last_corr"]      = (0, 0)
    _pid["last_corr_time"] = 0.0


def calc_marker_correction():
    """마커 오차 → roll/pitch 보정값 (μs). 미감지 시 NO_DETECT_HOLD_S 동안 마지막 값 유지."""
    with _marker_lock:
        detected  = _marker["detected"]
        err_x     = _marker["error_x"]
        err_pitch = _marker["error_pitch"]

    now = time.time()

    if not detected:
        # 마지막 감지 후 NO_DETECT_HOLD_S 안이면 마지막 보정값 유지
        if (now - _pid["last_corr_time"]) < NO_DETECT_HOLD_S and _pid["last_corr_time"] > 0:
            return _pid["last_corr"]
        # 그 이상이면 적분 리셋하고 0
        _pid["integral_x"]     = 0.0
        _pid["integral_pitch"] = 0.0
        _pid["prev_time"]      = None
        return 0, 0

    dt = (now - _pid["prev_time"]) if _pid["prev_time"] else (1.0 / UPDATE_HZ)
    dt = max(dt, 1e-4)
    _pid["prev_time"] = now

    # Roll
    _pid["integral_x"] += err_x * dt
    _pid["integral_x"]  = max(-I_LIMIT, min(I_LIMIT, _pid["integral_x"]))
    d_x = (err_x - _pid["prev_err_x"]) / dt
    _pid["prev_err_x"] = err_x

    corr_roll = KP_ROLL * err_x + KI_ROLL * _pid["integral_x"] + KD_ROLL * d_x
    corr_roll = int(max(-MAX_CORRECTION, min(MAX_CORRECTION, corr_roll)))

    # Pitch
    _pid["integral_pitch"] += err_pitch * dt
    _pid["integral_pitch"]  = max(-I_LIMIT, min(I_LIMIT, _pid["integral_pitch"]))
    d_p = (err_pitch - _pid["prev_err_pitch"]) / dt
    _pid["prev_err_pitch"] = err_pitch

    corr_pitch = KP_PITCH * err_pitch + KI_PITCH * _pid["integral_pitch"] + KD_PITCH * d_p
    corr_pitch = int(max(-MAX_CORRECTION, min(MAX_CORRECTION, corr_pitch)))

    _pid["last_corr"]      = (corr_roll, corr_pitch)
    _pid["last_corr_time"] = now
    return corr_roll, corr_pitch


# ─────────────────────────────────────────
# 로그 기록 헬퍼
# ─────────────────────────────────────────
def log_row(rt_writer, log_start, phase, marker_id,
            err_x, err_pitch, roll_us, pitch_us,
            actual_roll, actual_pitch, vib_x, vib_y, vib_z, aligned):
    rt_writer.writerow([
        f"{time.time() - log_start:.3f}",
        phase, marker_id,
        err_x, err_pitch,
        roll_us, pitch_us,
        f"{actual_roll:.2f}", f"{actual_pitch:.2f}",
        f"{vib_x:.4f}", f"{vib_y:.4f}", f"{vib_z:.4f}",
        1 if aligned else 0,
    ])


# ─────────────────────────────────────────
# 단계 1: 이륙
# ─────────────────────────────────────────
def run_takeoff(vehicle, rt_writer, log_start):
    print(f"[TAKEOFF] 목표 고도 {TARGET_ALT_M}m 상승 중...")
    thr = THR_START
    t0  = time.time()

    while True:
        if manual_override_requested():
            return False
        alt = get_range_alt(vehicle)
        if alt < 0:
            raise RuntimeError("Rangefinder not available")
        if alt >= TARGET_ALT_M:
            print(f"[TAKEOFF] 목표 고도 도달 ({alt:.2f}m)")
            break
        if time.time() - t0 > TIMEOUT_S:
            raise RuntimeError("Takeoff timeout")

        thr = min(thr + THR_STEP, THR_MAX)
        set_overrides(vehicle, thr=thr)

        actual_roll, actual_pitch = get_attitude_deg(vehicle)
        vib_x, vib_y, vib_z = get_vibration()
        log_row(rt_writer, log_start, "takeoff", -1,
                0, 0, 1500, 1500, actual_roll, actual_pitch,
                vib_x, vib_y, vib_z, False)

        time.sleep(1 / UPDATE_HZ)

    # 2초 안정화
    print("[STABILIZE] 2초 안정화...")
    for _ in range(int(2 * UPDATE_HZ)):
        if manual_override_requested():
            return False
        set_overrides(vehicle, thr=THR_HOVER)
        actual_roll, actual_pitch = get_attitude_deg(vehicle)
        vib_x, vib_y, vib_z = get_vibration()
        log_row(rt_writer, log_start, "takeoff", -1,
                0, 0, 1500, 1500, actual_roll, actual_pitch,
                vib_x, vib_y, vib_z, False)
        time.sleep(1 / UPDATE_HZ)

    return True


# ─────────────────────────────────────────
# 단계 2: 시작 마커 정렬 (PID)
# ─────────────────────────────────────────
def run_start_align(vehicle, rt_writer, log_start):
    """START_TAG_ID 마커에 정렬. (성공시각, 최대오차) 반환. 실패 시 (None, max_err)."""
    print(f"[START ALIGN] 마커 ID {START_TAG_ID} 정렬 시도 (최대 {START_ALIGN_TIMEOUT_S}s)...")
    set_target_marker(START_TAG_ID)
    reset_pid()

    t_phase_start = time.time()
    aligned_since = None       # 정렬 상태 진입 시각
    max_abs_err_x = 0
    align_time    = None

    while True:
        if manual_override_requested():
            return None, max_abs_err_x

        if time.time() - t_phase_start > START_ALIGN_TIMEOUT_S:
            print(f"\n[START ALIGN] 타임아웃 — 정렬 미완료로 진행")
            return None, max_abs_err_x

        corr_roll, corr_pitch = calc_marker_correction()
        roll_us  = 1500 + corr_roll
        pitch_us = 1500 + corr_pitch
        set_overrides(vehicle, roll=roll_us, pitch=pitch_us, thr=THR_HOVER)

        with _marker_lock:
            detected = _marker["detected"]
            aligned  = _marker["aligned"]
            err_x    = _marker["error_x"]
            err_pitch= _marker["error_pitch"]
            tag_id   = _marker["tag_id"]

        if detected:
            max_abs_err_x = max(max_abs_err_x, abs(err_x))

        actual_roll, actual_pitch = get_attitude_deg(vehicle)
        vib_x, vib_y, vib_z = get_vibration()
        log_row(rt_writer, log_start, "start_align", tag_id if detected else -1,
                err_x, err_pitch, roll_us, pitch_us,
                actual_roll, actual_pitch, vib_x, vib_y, vib_z, aligned)

        # 정렬 지속 시간 체크
        if aligned:
            if aligned_since is None:
                aligned_since = time.time()
            elif time.time() - aligned_since >= START_ALIGN_HOLD_S:
                align_time = time.time() - t_phase_start
                print(f"\n[START ALIGN] 정렬 완료 ({align_time:.2f}s)")
                return align_time, max_abs_err_x
        else:
            aligned_since = None

        if detected:
            print(f"[START ALIGN] err_x={err_x:+d} pitch={err_pitch:+d} "
                  f"roll={roll_us} pitch_us={pitch_us}", end="\r")
        else:
            print("[START ALIGN] 마커 미감지", end="\r")

        time.sleep(1 / UPDATE_HZ)


# ─────────────────────────────────────────
# 단계 3, 5: LOITER 단독 호버링 (오버라이드 중립)
# ─────────────────────────────────────────
def run_hover(vehicle, rt_writer, log_start, duration_s, phase_label):
    """주어진 시간 동안 모든 채널 중립으로 LOITER 호버링."""
    print(f"[{phase_label.upper()}] {duration_s:.1f}초 호버링 (LOITER 단독)...")
    t0 = time.time()

    while time.time() - t0 < duration_s:
        if manual_override_requested():
            return False

        # 전 채널 중립 유지
        set_overrides(vehicle, roll=1500, pitch=1500, thr=THR_HOVER, yaw=1500)

        with _marker_lock:
            detected = _marker["detected"]
            aligned  = _marker["aligned"]
            err_x    = _marker["error_x"]
            err_pitch= _marker["error_pitch"]
            tag_id   = _marker["tag_id"]

        actual_roll, actual_pitch = get_attitude_deg(vehicle)
        vib_x, vib_y, vib_z = get_vibration()
        log_row(rt_writer, log_start, phase_label, tag_id if detected else -1,
                err_x, err_pitch, 1500, 1500,
                actual_roll, actual_pitch, vib_x, vib_y, vib_z, aligned)

        remaining = duration_s - (time.time() - t0)
        print(f"[{phase_label.upper()}] 남은시간:{remaining:.1f}s "
              f"err_x={err_x:+d} pitch={err_pitch:+d}", end="\r")

        time.sleep(1 / UPDATE_HZ)

    return True


# ─────────────────────────────────────────
# 단계 4: 우 이동 (RC 오버라이드)
# ─────────────────────────────────────────
def run_transit(vehicle, rt_writer, log_start):
    """우측으로 roll 오버라이드 적용. (성공여부, 실제소요시간)."""
    print(f"[TRANSIT] roll={TRANSIT_ROLL}, {TRANSIT_DURATION_S:.1f}s 우 이동...")
    # 이동 중에는 마커 ID 추적 미사용 (양쪽 다 시야에 들어왔다 나갔다 함)
    set_target_marker(None)

    t0 = time.time()
    while time.time() - t0 < TRANSIT_DURATION_S:
        if manual_override_requested():
            return False, time.time() - t0

        set_overrides(vehicle, roll=TRANSIT_ROLL, pitch=1500, thr=THR_HOVER, yaw=1500)

        with _marker_lock:
            detected = _marker["detected"]
            err_x    = _marker["error_x"]
            err_pitch= _marker["error_pitch"]
            tag_id   = _marker["tag_id"]

        actual_roll, actual_pitch = get_attitude_deg(vehicle)
        vib_x, vib_y, vib_z = get_vibration()
        log_row(rt_writer, log_start, "transit", tag_id if detected else -1,
                err_x, err_pitch, TRANSIT_ROLL, 1500,
                actual_roll, actual_pitch, vib_x, vib_y, vib_z, False)

        remaining = TRANSIT_DURATION_S - (time.time() - t0)
        print(f"[TRANSIT] 남은시간:{remaining:.2f}s", end="\r")
        time.sleep(1 / UPDATE_HZ)

    print(f"\n[TRANSIT] 이동 완료")
    return True, time.time() - t0


# ─────────────────────────────────────────
# 단계 5: 도착 마커 감지 대기 (LOITER 호버링 + 마커 감지)
# ─────────────────────────────────────────
def run_goal_acquire(vehicle, rt_writer, log_start):
    """도착 마커 첫 감지까지 LOITER로 대기. (감지시각|None, 소요시간) 반환."""
    print(f"[GOAL ACQUIRE] 마커 ID {GOAL_TAG_ID} 감지 대기 (최대 {GOAL_ACQUIRE_TIMEOUT_S}s)...")
    set_target_marker(GOAL_TAG_ID)

    t0 = time.time()
    while time.time() - t0 < GOAL_ACQUIRE_TIMEOUT_S:
        if manual_override_requested():
            return None, time.time() - t0

        # 중립 호버링
        set_overrides(vehicle, roll=1500, pitch=1500, thr=THR_HOVER, yaw=1500)

        with _marker_lock:
            detected = _marker["detected"]
            aligned  = _marker["aligned"]
            err_x    = _marker["error_x"]
            err_pitch= _marker["error_pitch"]
            tag_id   = _marker["tag_id"]

        actual_roll, actual_pitch = get_attitude_deg(vehicle)
        vib_x, vib_y, vib_z = get_vibration()
        log_row(rt_writer, log_start, "goal_acquire", tag_id if detected else -1,
                err_x, err_pitch, 1500, 1500,
                actual_roll, actual_pitch, vib_x, vib_y, vib_z, aligned)

        if detected:
            acquire_time = time.time() - t0
            print(f"\n[GOAL ACQUIRE] 마커 감지! ({acquire_time:.2f}s)")
            return acquire_time, acquire_time

        remaining = GOAL_ACQUIRE_TIMEOUT_S - (time.time() - t0)
        print(f"[GOAL ACQUIRE] 남은시간:{remaining:.1f}s — 마커 탐색 중", end="\r")
        time.sleep(1 / UPDATE_HZ)

    print(f"\n[GOAL ACQUIRE] 타임아웃 — 도착 마커 미감지")
    return None, GOAL_ACQUIRE_TIMEOUT_S


# ─────────────────────────────────────────
# 단계 6: 도착지 측정 (10초)
# ─────────────────────────────────────────
def run_goal_measure(vehicle, rt_writer, log_start, apply_pid):
    """10초간 도착지 오차 측정. apply_pid=True면 PID 보정 적용 (실험군)."""
    label = "PID 적용" if apply_pid else "LOITER 단독"
    print(f"[GOAL MEASURE] {GOAL_MEASURE_S:.1f}초 측정 ({label})...")

    # PID 적용 전 리셋 (도착지 새 기준으로 시작)
    if apply_pid:
        reset_pid()

    err_x_buf      = []
    err_pitch_buf  = []
    vib_sq_sum     = 0.0
    aligned_count  = 0
    sample_count   = 0
    final_err_x    = 0
    final_err_pitch= 0

    t0 = time.time()
    while time.time() - t0 < GOAL_MEASURE_S:
        if manual_override_requested():
            break

        if apply_pid:
            corr_roll, corr_pitch = calc_marker_correction()
            roll_us  = 1500 + corr_roll
            pitch_us = 1500 + corr_pitch
        else:
            roll_us  = 1500
            pitch_us = 1500

        set_overrides(vehicle, roll=roll_us, pitch=pitch_us, thr=THR_HOVER, yaw=1500)

        with _marker_lock:
            detected = _marker["detected"]
            aligned  = _marker["aligned"]
            err_x    = _marker["error_x"]
            err_pitch= _marker["error_pitch"]
            tag_id   = _marker["tag_id"]

        actual_roll, actual_pitch = get_attitude_deg(vehicle)
        vib_x, vib_y, vib_z = get_vibration()
        log_row(rt_writer, log_start, "goal_measure", tag_id if detected else -1,
                err_x, err_pitch, roll_us, pitch_us,
                actual_roll, actual_pitch, vib_x, vib_y, vib_z, aligned)

        if detected:
            err_x_buf.append(err_x)
            err_pitch_buf.append(err_pitch)
            final_err_x     = err_x
            final_err_pitch = err_pitch
            if aligned:
                aligned_count += 1
        vib_sq_sum += vib_x**2 + vib_y**2 + vib_z**2
        sample_count += 1

        remaining = GOAL_MEASURE_S - (time.time() - t0)
        status = "ALIGNED" if aligned else f"err_x={err_x:+d} pitch={err_pitch:+d}"
        print(f"[GOAL MEASURE] 남은시간:{remaining:.1f}s | {status}", end="\r")
        time.sleep(1 / UPDATE_HZ)

    print()  # 줄바꿈

    # 통계 계산
    if err_x_buf:
        mean_err_x     = float(np.mean(np.abs(err_x_buf)))
        mean_err_pitch = float(np.mean(np.abs(err_pitch_buf)))
        rms_err_x      = float(np.sqrt(np.mean(np.square(err_x_buf))))
        rms_err_pitch  = float(np.sqrt(np.mean(np.square(err_pitch_buf))))
    else:
        mean_err_x = mean_err_pitch = rms_err_x = rms_err_pitch = float("nan")

    rms_vib = math.sqrt(vib_sq_sum / sample_count) if sample_count else 0.0
    aligned_ratio = aligned_count / sample_count if sample_count else 0.0

    return {
        "mean_err_x":      mean_err_x,
        "mean_err_pitch":  mean_err_pitch,
        "rms_err_x":       rms_err_x,
        "rms_err_pitch":   rms_err_pitch,
        "final_err_x":     final_err_x,
        "final_err_pitch": final_err_pitch,
        "aligned_ratio":   aligned_ratio,
        "rms_vib":         rms_vib,
        "sample_count":    sample_count,
    }


# ─────────────────────────────────────────
# 메인 비행 시퀀스
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
    rt_writer, rt_file, sum_writer, sum_file, ts = init_logs()

    log_start = time.time()
    success = False
    fail_reason = ""

    # 결과값 초기화
    start_align_time    = None
    start_align_max_err = 0
    goal_acquire_time   = None
    measure_stats       = None

    try:
        # 1. 이륙
        if not run_takeoff(vehicle, rt_writer, log_start):
            fail_reason = "manual_override_at_takeoff"
            raise RuntimeError(fail_reason)

        # 2. 시작 마커 정렬
        start_align_time, start_align_max_err = run_start_align(vehicle, rt_writer, log_start)
        if manual_override_requested():
            fail_reason = "manual_override_at_start_align"
            raise RuntimeError(fail_reason)
        # 정렬 타임아웃이라도 실험은 계속 진행 (start_align_time=None일 수 있음)

        # 3. 시작 호버링
        if not run_hover(vehicle, rt_writer, log_start, START_HOVER_S, "start_hover"):
            fail_reason = "manual_override_at_start_hover"
            raise RuntimeError(fail_reason)

        # 4. 우 이동
        ok, _ = run_transit(vehicle, rt_writer, log_start)
        if not ok:
            fail_reason = "manual_override_at_transit"
            raise RuntimeError(fail_reason)

        # 5. 도착 마커 감지 대기
        goal_acquire_time, _ = run_goal_acquire(vehicle, rt_writer, log_start)
        if manual_override_requested():
            fail_reason = "manual_override_at_goal_acquire"
            raise RuntimeError(fail_reason)
        if goal_acquire_time is None:
            fail_reason = "goal_marker_not_found"
            print(f"[FAIL] {fail_reason}")
            # 측정 단계 skip하고 착륙
        else:
            # 6. 도착지 측정
            measure_stats = run_goal_measure(vehicle, rt_writer, log_start, apply_pid=APPLY_PID_AT_GOAL)
            success = True

    except RuntimeError as e:
        if not fail_reason:
            fail_reason = str(e)
        print(f"[ERROR] {e}")

    finally:
        # 7. 착륙
        print("[LAND] 착륙 모드 전환...")
        clear_overrides(vehicle)
        vehicle.mode = VehicleMode("LAND")
        wait_mode(vehicle, "LAND", timeout=5.0)

        # 요약 기록
        if measure_stats:
            sum_writer.writerow([
                ts, GROUP_NAME, 1 if success else 0,
                f"{start_align_time:.2f}" if start_align_time is not None else "",
                start_align_max_err,
                f"{goal_acquire_time:.2f}" if goal_acquire_time is not None else "",
                f"{measure_stats['mean_err_x']:.2f}",
                f"{measure_stats['mean_err_pitch']:.2f}",
                f"{measure_stats['rms_err_x']:.2f}",
                f"{measure_stats['rms_err_pitch']:.2f}",
                measure_stats['final_err_x'],
                measure_stats['final_err_pitch'],
                f"{measure_stats['aligned_ratio']:.3f}",
                f"{measure_stats['rms_vib']:.4f}",
                fail_reason,
            ])
            print(f"\n[RESULT] mean_err_x={measure_stats['mean_err_x']:.2f}px "
                  f"rms_err_x={measure_stats['rms_err_x']:.2f}px "
                  f"aligned_ratio={measure_stats['aligned_ratio']*100:.1f}%")
        else:
            # 측정 없이 끝난 경우도 한 줄 기록
            sum_writer.writerow([
                ts, GROUP_NAME, 0,
                f"{start_align_time:.2f}" if start_align_time is not None else "",
                start_align_max_err,
                f"{goal_acquire_time:.2f}" if goal_acquire_time is not None else "",
                "", "", "", "", "", "", "", "",
                fail_reason or "no_measurement",
            ])
            print(f"\n[RESULT] FAIL — {fail_reason}")

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
