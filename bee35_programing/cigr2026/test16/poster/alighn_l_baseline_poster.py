#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
드론 자동이륙 + AprilTag 정렬 (baseline_poster — CIGR 본 실험 대조군)

목적: 정렬 알고리즘(alighn_l_poster.py)의 대조군
      LOITER + OF만으로 정렬 유지 가능한지 측정
      poster (PID 작동) vs baseline_poster (PID 없음) 비교

흐름:
  이륙 + 안정화: LOITER (poster와 동일)
  정렬 시도: PID로 첫 정렬까지만
  ↓ 첫 정렬 도달
  측정 시작 (60초): PID 비활성 → LOITER + OF만 (RC override 해제)

환경 (poster와 동일):
  마커 30mm × 30mm, 마커 1.5m 부착
  운용 거리 40cm 중앙 (25~55cm 비대칭)
  좌우 임계: 가용의 70%
  TARGET_ALT_M = 1.3m

ArduPilot 설정 (필수):
  EK3_SRC3_YAW = 8  (GSF, mag anomaly 회피)
  THR_MAX = 1580
  LAND_SPEED = 25

로그: logs/baseline_poster/loiter_baseline_poster_realtime_*.csv

실행:
  python alighn_l_baseline_poster.py
"""

import sys
import time
import threading
import queue
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
CONN               = "udp:127.0.0.1:14552"

TARGET_ALT_M       = 1.3      # 마커 1.5m 부착, 안전 고도 1.3m
THR_START          = 1500
THR_STEP           = 2
THR_MAX            = 1630
THR_HOVER          = 1500
UPDATE_HZ          = 60
TIMEOUT_S          = 20.0
KILL_THRESHOLD     = 1000

STREAM_URL         = "tcp://127.0.0.1:8888"
CAM_W, CAM_H       = 640, 480
TAG_FAMILY         = "tag36h11"

# ─────────────────────────────────────────
# 마커 및 거리 (v14: cm 직접 환산)
# ─────────────────────────────────────────
MARKER_SIZE_MM     = 30.0     # 3cm × 3cm
FOCAL_LENGTH_PX    = 950.3    # 캘리브레이션 결과 (40cm + 3cm 마커)

# 운용 거리 (앞뒤)
TARGET_DISTANCE_CM = 40.0     # 호버링 목표 거리
DISTANCE_MIN_CM    = 25.0     # 앞 한계 (가장 가까이)
DISTANCE_MAX_CM    = 55.0     # 뒤 한계 (가장 멀리)

# TARGET_MARKER_PX (PID용) — 거리 cm 기반으로 자동 계산
# distance_cm = (f × marker_real_mm) / marker_px / 10
# 역산: marker_px = f × marker_real_mm / (distance_cm × 10)
TARGET_MARKER_PX = int(FOCAL_LENGTH_PX * MARKER_SIZE_MM / (TARGET_DISTANCE_CM * 10))
# 40cm 거리 → 71 px

# 좌우 임계: 가용 좌우의 일정 비율 (동적)
LATERAL_RATIO      = 0.7      # 가용 좌우의 70% (사용자 요청: 넉넉하게)

# PITCH (거리) 임계는 cm 직접 사용 (DISTANCE_MIN/MAX_CM)

# PID 게인 (v13 그대로 유지)
# PID 게인 (KP_PITCH는 거리 px에 둔감해서 KP_ROLL보다 큰 값)
KP_ROLL  = 0.25
KI_ROLL  = 0.005
KD_ROLL  = 0.12
KP_PITCH = 0.80      # poster와 통일 (첫 정렬 도달 시간 공정)
KI_PITCH = 0.010
KD_PITCH = 0.12
I_LIMIT  = 20

MAX_CORRECTION   = 100

# ─────────────────────────────────────────
# 로그 저장 폴더 (실험별 분리)
# ─────────────────────────────────────────
LOG_SUBDIR = "baseline_poster"

# ─────────────────────────────────────────
# 데드밴드 보상 (FC RC_DZ ±20μs 우회)
# ─────────────────────────────────────────
USE_DEADBAND_COMPENSATION = True
DEADBAND_BOOST = 22      # RC_DZ(20)보다 살짝 크게


def deadband_compensate(corr):
    """PID 출력이 데드밴드 안이면 부스트, 0은 그대로."""
    if not USE_DEADBAND_COMPENSATION:
        return corr
    if corr == 0:
        return 0
    if 0 < corr < DEADBAND_BOOST:
        return DEADBAND_BOOST
    if -DEADBAND_BOOST < corr < 0:
        return -DEADBAND_BOOST
    return corr


# ─────────────────────────────────────────
# cm 환산 함수 (v14)
# ─────────────────────────────────────────
def marker_px_to_distance_cm(marker_px):
    """마커 화면 크기 (px) → 카메라까지 거리 (cm)."""
    if marker_px <= 0:
        return -1.0
    return (FOCAL_LENGTH_PX * MARKER_SIZE_MM) / marker_px / 10.0


def err_x_px_to_cm(err_x_px, marker_px):
    """좌우 픽셀 오차 → 실제 cm (해당 프레임 마커 크기 기준)."""
    if marker_px <= 0:
        return 0.0
    return err_x_px * (MARKER_SIZE_MM / marker_px) / 10.0


def lateral_threshold_px(marker_px):
    """현재 마커 px 기준 좌우 임계 (px). 가용 좌우의 일정 비율."""
    available_px = (CAM_W - marker_px) / 2
    return available_px * LATERAL_RATIO


def is_aligned(err_x_px, marker_px):
    """cm 단위로 정렬 판정.
    - 거리: DISTANCE_MIN_CM ~ DISTANCE_MAX_CM 안에 있어야
    - 좌우: 가용 좌우의 LATERAL_RATIO 안에 있어야
    """
    distance_cm = marker_px_to_distance_cm(marker_px)
    if distance_cm < 0:
        return False
    distance_ok = (DISTANCE_MIN_CM <= distance_cm <= DISTANCE_MAX_CM)
    
    lateral_thr = lateral_threshold_px(marker_px)
    lateral_ok = (abs(err_x_px) <= lateral_thr)
    
    return distance_ok and lateral_ok

# ─────────────────────────────────────────
# v4 적응 메커니즘 (외삽만 유지, 적응 게인 OFF)
# ─────────────────────────────────────────
MAX_EXTRAPOLATE_S = 0.5
DERIV_LPF_ALPHA   = 0.5

# 적응 게인 OFF — 영상 점프 해결되어 검출 지연 없음
ADAPTIVE_GAIN     = False
GAIN_SCALE_MAX    = 1.0      # OFF여도 안전상 1.0 고정
D_NORM            = 100.0
GAIN_LPF_ALPHA    = 0.3

# 첫 정렬 후 측정 시간
RECORD_DURATION    = 60.0

# v10: 정렬 시 PID 약화 (오버슈트 방지)
# - 정렬됨: 명령 × ALIGN_GAIN_SCALE + 데드밴드 보상 X (작은 명령 무시)
# - 미정렬: 정상 명령 + 데드밴드 보상 O (빠른 복귀)
ALIGN_GAIN_SCALE = 0.6

# ─────────────────────────────────────────
# 공유 상태
# ─────────────────────────────────────────
_ch5_value = 0
_marker = {
    "detected":    False,
    "error_x":     0,
    "error_pitch": 0,
    "marker_px":   0,        # v14: 좌우 임계 계산 + 거리 환산용
    "aligned":     False,
}
_marker_lock    = threading.Lock()
_camera_running = False
_vibration = {"x": 0.0, "y": 0.0, "z": 0.0}


def start_rc_listener(vehicle):
    @vehicle.on_message('RC_CHANNELS')
    def rc_handler(self, name, msg):
        global _ch5_value
        _ch5_value = msg.chan5_raw


def manual_override_requested():
    if _ch5_value > KILL_THRESHOLD:
        print(f"\n[SAFETY] MANUAL SWITCH (CH5={_ch5_value})")
        return True
    return False


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
        "1": int(roll), "2": int(pitch), "3": int(thr), "4": int(yaw),
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
        return math.degrees(att.roll), math.degrees(att.pitch), math.degrees(att.yaw)
    except Exception:
        return 0.0, 0.0, 0.0


def start_vibration_listener(vehicle):
    @vehicle.on_message('VIBRATION')
    def vib_handler(self, name, msg):
        _vibration["x"] = msg.vibration_x
        _vibration["y"] = msg.vibration_y
        _vibration["z"] = msg.vibration_z


def init_logs():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = f"logs/{LOG_SUBDIR}"
    os.makedirs(log_dir, exist_ok=True)
    rt_path  = f"{log_dir}/loiter_baseline_poster_realtime_{ts}.csv"
    sum_path = f"{log_dir}/loiter_baseline_poster_summary_{ts}.csv"
    rt_file = open(rt_path, "w", newline="")
    rt_writer = csv.writer(rt_file)
    rt_writer.writerow([
        "elapsed_s",
        "err_x_px", "err_pitch_px", "marker_px",        # v14: marker_px 추가
        "err_x_cm", "distance_cm", "lateral_thr_px",    # v14: cm 환산 + 동적 임계
        "target_roll_us", "target_pitch_us",
        "actual_roll_deg", "actual_pitch_deg", "actual_yaw_deg",
        "vib_x", "vib_y", "vib_z",
        "aligned",
    ])
    sum_file = open(sum_path, "w", newline="")
    sum_writer = csv.writer(sum_file)
    sum_writer.writerow(["attempt", "align_time_s", "total_abs_err_px", "rms_vib"])
    print(f"[LOG] 실시간: {rt_path}")
    print(f"[LOG] 요약:   {sum_path}")
    return rt_writer, rt_file, sum_writer, sum_file


def camera_thread_fn():
    global _camera_running
    cam_cx = CAM_W // 2
    cam_cy = CAM_H // 2

    detector = Detector(
        families=TAG_FAMILY,
        nthreads=1,           # v14: 4 → 1 (단일 스레드, 오버헤드 ↓)
        quad_decimate=2.0,    # v14: 1.0 → 2.0 (해상도 절반 검출, 빠름)
        quad_sigma=0.0,       # v14: 0.8 → 0.0 (블러 제거)
        refine_edges=1,       # 유지 (코너 정밀도)
        decode_sharpening=0.25,  # v14: 0.5 → 0.25 (기본)
    )

    print(f"[CAM] 스트림 연결: {STREAM_URL}")
    cap = cv2.VideoCapture(STREAM_URL, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("[CAM] 스트림 연결 실패")
        _camera_running = False
        return

    # ─── 영상 저장: 별도 스레드 + 큐 ───
    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = f"logs/{LOG_SUBDIR}"
    os.makedirs(log_dir, exist_ok=True)
    video_path = f"{log_dir}/flight_video_baseline_v14_{ts_str}.mp4"
    ts_path    = f"{log_dir}/flight_video_baseline_v14_{ts_str}_timestamps.csv"
    
    # 저장 큐 (max 30 = 1초 분량, 가득차면 drop)
    video_queue = queue.Queue(maxsize=30)
    ts_file = open(ts_path, "w", newline="")
    ts_writer = csv.writer(ts_file)
    ts_writer.writerow(["frame_idx", "elapsed_s", "dropped"])
    
    # 저장 스레드 시작
    saver_running = [True]   # 리스트로 감싸서 클로저 사용
    dropped_count = [0]
    
    def video_saver():
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(video_path, fourcc, 30.0, (CAM_W, CAM_H))
        saved = 0
        while saver_running[0] or not video_queue.empty():
            try:
                frame = video_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            writer.write(frame)
            saved += 1
        writer.release()
        print(f"\n[SAVER] 영상 저장 완료: {saved} frames, drop: {dropped_count[0]}")
    
    saver_thread = threading.Thread(target=video_saver, daemon=True)
    saver_thread.start()
    
    print(f"[CAM] 영상 녹화: {video_path}")
    print(f"[CAM] 타임스탬프: {ts_path}")
    print("[CAM] 카메라 스레드 시작")

    frame_idx = 0
    cam_start = time.time()

    while _camera_running:
        ret, frame_bgr = cap.read()
        if not ret or frame_bgr is None:
            time.sleep(0.05)
            continue

        frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_180)
        elapsed = time.time() - cam_start

        # ─── 영상 큐에 넣기 (논블로킹, 가득차면 drop) ───
        was_dropped = 0
        try:
            video_queue.put_nowait(frame_bgr.copy())   # 복사 (다음 detect와 분리)
        except queue.Full:
            dropped_count[0] += 1
            was_dropped = 1
        ts_writer.writerow([frame_idx, f"{elapsed:.3f}", was_dropped])
        frame_idx += 1

        # ─── 마커 검출 (경량화: 전처리 블러 제거) ───
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray)

        if tags:
            tag = tags[0]
            cx, cy = int(tag.center[0]), int(tag.center[1])
            err_x = cx - cam_cx
            side_lens = [
                np.linalg.norm(tag.corners[i] - tag.corners[(i + 1) % 4])
                for i in range(4)
            ]
            marker_px = int(sum(side_lens) / 4)
            err_pitch = marker_px - TARGET_MARKER_PX
            # v14: cm 기준 동적 임계 판정
            aligned = is_aligned(err_x, marker_px)

            with _marker_lock:
                _marker["detected"]    = True
                _marker["error_x"]     = err_x
                _marker["error_pitch"] = err_pitch
                _marker["marker_px"]   = marker_px   # v14: 좌우 임계 계산용
                _marker["aligned"]     = aligned
        else:
            with _marker_lock:
                _marker["detected"]    = False
                _marker["error_x"]     = 0
                _marker["error_pitch"] = 0
                _marker["marker_px"]   = 0
                _marker["aligned"]     = False

    # 종료
    saver_running[0] = False
    saver_thread.join(timeout=5.0)
    ts_file.flush()
    ts_file.close()
    cap.release()
    print(f"[CAM] 카메라 스레드 종료 ({frame_idx} frames captured)")


# ─────────────────────────────────────────
# PID (단순 — LPF/Rate limit 없음)
# ─────────────────────────────────────────
_pid = {
    "integral_x":     0.0,
    "integral_pitch": 0.0,
    "prev_err_x":     0.0,
    "prev_err_pitch": 0.0,
    "prev_time":      None,
    # v4: 적응 메커니즘 상태
    "filt_d_x":       0.0,    # LPF 적용된 변화율
    "filt_d_pitch":   0.0,
    "last_err_x":     0.0,    # 마지막 감지된 err_x
    "last_err_pitch": 0.0,
    "no_detect_time": 0.0,    # 미감지 누적 시간
    "filt_gain_scale": 1.0,   # LPF 적용된 게인 스케일
}


def reset_pid():
    _pid["integral_x"]     = 0.0
    _pid["integral_pitch"] = 0.0
    _pid["prev_err_x"]     = 0.0
    _pid["prev_err_pitch"] = 0.0
    _pid["prev_time"]      = None
    _pid["filt_d_x"]       = 0.0
    _pid["filt_d_pitch"]   = 0.0
    _pid["last_err_x"]     = 0.0
    _pid["last_err_pitch"] = 0.0
    _pid["no_detect_time"] = 0.0
    _pid["filt_gain_scale"] = 1.0


def calc_marker_correction():
    """
    적응 메커니즘:
    - 감지: 정상 PID + 변화율 기반 적응 게인
    - 미감지 (≤ MAX_EXTRAPOLATE_S): 변화율로 err_x 외삽 + PID 계산
    - 미감지 (> MAX_EXTRAPOLATE_S): 명령 0, 적분 리셋
    """
    with _marker_lock:
        detected      = _marker["detected"]
        err_x_raw     = _marker["error_x"]
        err_pitch_raw = _marker["error_pitch"]

    now = time.time()
    dt  = (now - _pid["prev_time"]) if _pid["prev_time"] else (1.0 / UPDATE_HZ)
    dt  = max(dt, 1e-4)
    _pid["prev_time"] = now

    if detected:
        # 정상 감지 — 변화율 갱신
        d_x_raw = (err_x_raw - _pid["last_err_x"]) / dt if _pid["no_detect_time"] < 1e-3 else 0.0
        d_p_raw = (err_pitch_raw - _pid["last_err_pitch"]) / dt if _pid["no_detect_time"] < 1e-3 else 0.0
        # LPF 적용
        _pid["filt_d_x"]     = DERIV_LPF_ALPHA * d_x_raw + (1-DERIV_LPF_ALPHA) * _pid["filt_d_x"]
        _pid["filt_d_pitch"] = DERIV_LPF_ALPHA * d_p_raw + (1-DERIV_LPF_ALPHA) * _pid["filt_d_pitch"]
        _pid["last_err_x"]     = err_x_raw
        _pid["last_err_pitch"] = err_pitch_raw
        _pid["no_detect_time"] = 0.0
        
        err_x     = err_x_raw
        err_pitch = err_pitch_raw
    else:
        # 미감지 — 외삽 시도
        _pid["no_detect_time"] += dt
        if _pid["no_detect_time"] > MAX_EXTRAPOLATE_S:
            # 외삽 한계 초과 — 명령 0, 적분 리셋
            _pid["integral_x"]     = 0.0
            _pid["integral_pitch"] = 0.0
            _pid["filt_d_x"]       *= (1 - DERIV_LPF_ALPHA)  # 변화율 천천히 감쇠
            _pid["filt_d_pitch"]   *= (1 - DERIV_LPF_ALPHA)
            return 0, 0
        
        # 외삽: 마지막 err + 변화율 * 경과시간
        err_x     = _pid["last_err_x"]     + _pid["filt_d_x"]     * _pid["no_detect_time"]
        err_pitch = _pid["last_err_pitch"] + _pid["filt_d_pitch"] * _pid["no_detect_time"]

    # ── 적응 게인 (변화율 크기 기반) ──
    if ADAPTIVE_GAIN and detected:
        d_mag = max(abs(_pid["filt_d_x"]), abs(_pid["filt_d_pitch"]))
        target_scale = 1.0 + (GAIN_SCALE_MAX - 1.0) * min(d_mag / D_NORM, 1.0)
        # 게인 변화 부드럽게
        _pid["filt_gain_scale"] = GAIN_LPF_ALPHA * target_scale + (1-GAIN_LPF_ALPHA) * _pid["filt_gain_scale"]
    else:
        # 미감지 중에는 게인 1.0 유지 (위험 방지)
        _pid["filt_gain_scale"] = GAIN_LPF_ALPHA * 1.0 + (1-GAIN_LPF_ALPHA) * _pid["filt_gain_scale"]
    gs = _pid["filt_gain_scale"]

    # ── PID 계산 ──
    # Roll
    _pid["integral_x"] += err_x * dt
    _pid["integral_x"]  = max(-I_LIMIT, min(I_LIMIT, _pid["integral_x"]))
    _pid["prev_err_x"]  = err_x
    corr_roll = (KP_ROLL * gs) * err_x + KI_ROLL * _pid["integral_x"] + KD_ROLL * _pid["filt_d_x"]
    corr_roll = int(max(-MAX_CORRECTION, min(MAX_CORRECTION, corr_roll)))

    # Pitch
    _pid["integral_pitch"] += err_pitch * dt
    _pid["integral_pitch"]  = max(-I_LIMIT, min(I_LIMIT, _pid["integral_pitch"]))
    _pid["prev_err_pitch"]  = err_pitch
    corr_pitch = (KP_PITCH * gs) * err_pitch + KI_PITCH * _pid["integral_pitch"] + KD_PITCH * _pid["filt_d_pitch"]
    corr_pitch = int(max(-MAX_CORRECTION, min(MAX_CORRECTION, corr_pitch)))

    # v11: 정렬 상태에 따라 약화만 분기 (데드밴드 보상은 항상 적용)
    with _marker_lock:
        currently_aligned = _marker["aligned"]
    
    if currently_aligned:
        # 정렬됨: 명령 약화 (작은 값은 데드밴드 보상으로 다시 22로 부스트됨)
        corr_roll  = int(corr_roll  * ALIGN_GAIN_SCALE)
        corr_pitch = int(corr_pitch * ALIGN_GAIN_SCALE)

    # 항상 데드밴드 보상 (정렬 안/밖 모두)
    corr_roll  = deadband_compensate(corr_roll)
    corr_pitch = deadband_compensate(corr_pitch)

    return corr_roll, corr_pitch


def main():
    global _camera_running

    print("=" * 60)
    print("baseline v14 — LOITER + OF 단독 (PID 없음)")
    print(f"  마커: {MARKER_SIZE_MM:.0f}mm × {MARKER_SIZE_MM:.0f}mm")
    print(f"  운용 거리: {DISTANCE_MIN_CM:.0f} ~ {DISTANCE_MAX_CM:.0f} cm (중앙 {TARGET_DISTANCE_CM:.0f}cm)")
    print(f"  좌우 임계: 가용 좌우의 {LATERAL_RATIO*100:.0f}%")
    print(f"  측정: 첫 정렬 후 {RECORD_DURATION:.0f}s 동안 LOITER+OF만")
    print("=" * 60)

    _camera_running = True
    cam_thread = threading.Thread(target=camera_thread_fn, daemon=True)
    cam_thread.start()
    time.sleep(1.0)
    if not _camera_running:
        print("[ERROR] 카메라 연결 실패")
        sys.exit(1)

    print(f"[CONNECT] {CONN}")
    vehicle = connect(CONN, wait_ready=True, timeout=60)
    start_rc_listener(vehicle)
    start_vibration_listener(vehicle)

    vehicle.mode = VehicleMode("LOITER")
    if not wait_mode(vehicle, "LOITER", timeout=8.0):
        raise RuntimeError("Mode change failed")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.1)
    print("[ARM] 완료")

    rt_writer, rt_file, sum_writer, sum_file = init_logs()
    thr = THR_START
    start = time.time()

    try:
        # 이륙
        print(f"[TAKEOFF] {TARGET_ALT_M}m 상승")
        while True:
            if manual_override_requested(): break
            alt = get_range_alt(vehicle)
            if alt < 0: raise RuntimeError("Rangefinder unavailable")
            if alt >= TARGET_ALT_M:
                print(f"[TAKEOFF] 도달 ({alt:.2f}m)")
                break
            if time.time() - start > TIMEOUT_S:
                raise RuntimeError("Takeoff timeout")
            thr = min(thr + THR_STEP, THR_MAX)
            set_overrides(vehicle, thr=thr)
            time.sleep(1 / UPDATE_HZ)

        # 안정화
        print("[STABILIZE] 2초")
        manual_during_stabilize = False
        for _ in range(int(2 * UPDATE_HZ)):
            if manual_override_requested():
                manual_during_stabilize = True
                break
            set_overrides(vehicle, thr=THR_HOVER)
            time.sleep(1 / UPDATE_HZ)

        # 정렬 + 측정
        if not manual_during_stabilize:
            print("[ALIGN] LOITER + PID 정렬 시도 (첫 정렬까지만)")
            print(f"        첫 정렬 후 {RECORD_DURATION:.0f}s 동안 LOITER+OF만 측정")
            log_start         = time.time()
            segment_start     = time.time()
            attempt           = 0
            was_aligned       = False
            first_align_time  = None
            sum_abs_err_x     = 0
            sum_abs_err_pitch = 0
            sum_vib_sq        = 0.0
            loop_count        = 0
            overrides_cleared = False   # 측정 시작 시 1회만 clear

            while True:
                if manual_override_requested(): break
                if first_align_time is not None:
                    remaining = RECORD_DURATION - (time.time() - first_align_time)
                    if remaining <= 0:
                        print(f"\n[TIMER] {RECORD_DURATION:.0f}s 완료")
                        break

                # ─── BASELINE 핵심: 측정 시작 후 PID 비활성 ───
                if first_align_time is not None:
                    # 측정 모드 — LOITER + OF만 (PID 명령 X)
                    if not overrides_cleared:
                        clear_overrides(vehicle)
                        overrides_cleared = True
                        print(f"\n[BASELINE] LOITER 단독 측정 시작 — PID 비활성")
                    roll_override  = 1500  # 로그용 (실제 명령 X)
                    pitch_override = 1500
                else:
                    # 정렬 시도 중 — 정상 PID
                    corr_roll, corr_pitch = calc_marker_correction()
                    roll_override  = 1500 + corr_roll
                    pitch_override = 1500 + corr_pitch
                    set_overrides(vehicle, roll=roll_override, pitch=pitch_override)

                with _marker_lock:
                    detected  = _marker["detected"]
                    aligned   = _marker["aligned"]
                    err_x     = _marker["error_x"]
                    err_pitch = _marker["error_pitch"]
                    marker_px = _marker["marker_px"]

                # v14: cm 환산
                if marker_px > 0:
                    err_x_cm    = err_x_px_to_cm(err_x, marker_px)
                    distance_cm = marker_px_to_distance_cm(marker_px)
                    lateral_thr = lateral_threshold_px(marker_px)
                else:
                    err_x_cm = 0.0
                    distance_cm = -1.0
                    lateral_thr = 0.0

                actual_roll, actual_pitch, actual_yaw = get_attitude_deg(vehicle)
                vib_x = _vibration["x"]; vib_y = _vibration["y"]; vib_z = _vibration["z"]
                elapsed = time.time() - log_start

                rt_writer.writerow([
                    f"{elapsed:.3f}", err_x, err_pitch, marker_px,
                    f"{err_x_cm:.2f}", f"{distance_cm:.2f}", f"{lateral_thr:.1f}",
                    roll_override, pitch_override,
                    f"{actual_roll:.2f}", f"{actual_pitch:.2f}", f"{actual_yaw:.2f}",
                    f"{vib_x:.4f}", f"{vib_y:.4f}", f"{vib_z:.4f}",
                    1 if aligned else 0,
                ])

                if not aligned:
                    sum_abs_err_x     += abs(err_x)
                    sum_abs_err_pitch += abs(err_pitch)
                sum_vib_sq += vib_x**2 + vib_y**2 + vib_z**2
                loop_count += 1

                if aligned and not was_aligned:
                    attempt += 1
                    align_time = time.time() - segment_start
                    rms_vib = math.sqrt(sum_vib_sq / loop_count) if loop_count else 0
                    sum_writer.writerow([
                        attempt, f"{align_time:.2f}",
                        sum_abs_err_x + sum_abs_err_pitch,
                        f"{rms_vib:.4f}",
                    ])
                    sum_file.flush()
                    print(f"\n[LOG] #{attempt} 정렬 {align_time:.2f}s err_x:{sum_abs_err_x} rms_vib:{rms_vib:.4f}")
                    if first_align_time is None:
                        first_align_time = time.time()
                        print(f"[TIMER] 첫 정렬 — {RECORD_DURATION:.0f}s 측정")

                if not aligned and was_aligned:
                    sum_abs_err_x = sum_abs_err_pitch = 0
                    sum_vib_sq = 0.0; loop_count = 0
                    segment_start = time.time()
                was_aligned = aligned

                if detected:
                    if aligned:
                        s = "ALIGNED ✓"
                    else:
                        s = f"err_x={err_x_cm:+.1f}cm dist={distance_cm:.1f}cm thr=±{lateral_thr*MARKER_SIZE_MM/marker_px/10:.1f}cm"
                    print(f"[LOITER] {s} | r={roll_override} p={pitch_override}    ", end="\r")
                else:
                    nodt = _pid["no_detect_time"]
                    if nodt <= MAX_EXTRAPOLATE_S:
                        print(f"[LOITER] 외삽 중 ({nodt:.2f}s/{MAX_EXTRAPOLATE_S:.1f}s) | r={roll_override} p={pitch_override}    ", end="\r")
                    else:
                        print(f"[LOITER] 미감지 {nodt:.2f}s              ", end="\r")

                time.sleep(1 / UPDATE_HZ)

        # 착륙 (천천히)
        print("\n[LAND] 착륙 (천천히)")
        clear_overrides(vehicle)
        vehicle.parameters['LAND_SPEED'] = 25   # cm/s (기본 50)
        vehicle.mode = VehicleMode("LAND")
        wait_mode(vehicle, "LAND", timeout=5.0)

    finally:
        print("\n[CLEANUP]")
        _camera_running = False
        rt_file.flush(); rt_file.close()
        sum_file.close()
        clear_overrides(vehicle)
        vehicle.close()
        cam_thread.join(timeout=3.0)


if __name__ == "__main__":
    main()