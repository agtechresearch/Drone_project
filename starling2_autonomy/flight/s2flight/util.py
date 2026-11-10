"""시계·각도·좌표 도우미와 로그. (v14 모듈 함수들을 옮김)

시계
    mono_time()  : CLOCK_MONOTONIC 초. 리눅스에서 time.monotonic() 은 CLOCK_MONOTONIC 이고
                   asyncio loop.time() 도 같은 시계다. VOXL MPA 파이프의 timestamp(ns) 도
                   CLOCK_MONOTONIC 이므로(docs/10 §3) 이 값에 1e9 를 곱하면 카메라 프레임
                   타임스탬프와 직접 비교할 수 있다.
    wallclock()  : 사람이 읽는 벽시계 문자열 (로그용).
"""

import math
import re
import time
from datetime import datetime

_LOG_FILE = None


def mono_time():
    """CLOCK_MONOTONIC 초 (float)."""
    if hasattr(time, "clock_gettime") and hasattr(time, "CLOCK_MONOTONIC"):
        return time.clock_gettime(time.CLOCK_MONOTONIC)
    return time.monotonic()


def mono_ns():
    return int(round(mono_time() * 1e9))


def wallclock():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def set_log_file(path):
    """콘솔 로그를 파일에도 쓴다. None 이면 끈다."""
    global _LOG_FILE
    if _LOG_FILE is not None:
        try:
            _LOG_FILE.close()
        except Exception:
            pass
        _LOG_FILE = None
    if path:
        _LOG_FILE = open(path, "a", encoding="utf-8")


def log(msg):
    line = "[{}] {}".format(wallclock(), msg)
    print(line, flush=True)
    if _LOG_FILE is not None:
        try:
            _LOG_FILE.write(line + "\n")
            _LOG_FILE.flush()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 각도 / 좌표
# ---------------------------------------------------------------------------

def horizontal_norm(n, e):
    return math.sqrt(n * n + e * e)


def normalize_angle_deg(angle):
    """[-180, 180) 로 정규화."""
    return (float(angle) + 180.0) % 360.0 - 180.0


def angle_diff_deg(target, current):
    """target - current 의 최단 부호 차이 (deg)."""
    return normalize_angle_deg(float(target) - float(current))


def yaw_to_360(angle):
    return float(angle) % 360.0


def body_delta_to_ned(forward_m, right_m, yaw_deg):
    """기체 전방/우측 변위 → 로컬 N/E 변위.

    PX4/MAVSDK NED yaw: 0 = 북, 90 = 동 (시계방향 양수).
    yaw=0: forward=N+, right=E+.  yaw=90: forward=E+, right=N-.
    """
    psi = math.radians(float(yaw_deg))
    dn = forward_m * math.cos(psi) - right_m * math.sin(psi)
    de = forward_m * math.sin(psi) + right_m * math.cos(psi)
    return dn, de


def direction_to_body_fr(direction, distance):
    if direction == "forward":
        return distance, 0.0
    if direction == "backward":
        return -distance, 0.0
    if direction == "right":
        return 0.0, distance
    if direction == "left":
        return 0.0, -distance
    raise ValueError("Unknown direction: {}".format(direction))


def direction_to_ned(direction, distance, yaw_deg=0.0, move_frame="world"):
    """world: NED 고정축. body: 현재 yaw 기준."""
    if move_frame == "body":
        fwd, right = direction_to_body_fr(direction, distance)
        return body_delta_to_ned(fwd, right, yaw_deg)
    if direction == "forward":
        return distance, 0.0
    if direction == "backward":
        return -distance, 0.0
    if direction == "right":
        return 0.0, distance
    if direction == "left":
        return 0.0, -distance
    raise ValueError("Unknown direction: {}".format(direction))


# ---------------------------------------------------------------------------
# 추종 평가 (v14 evaluate_horizontal_tracking)
# ---------------------------------------------------------------------------

MIN_H_MOVE_FOR_TRACK_EVAL = 0.20
TRACK_RATIO_OK = 0.70
CROSS_TRACK_OK = 0.20


def evaluate_horizontal_tracking(expected_dn, expected_de, actual_dn, actual_de):
    """(status, ratio, cross). 기대 이동이 20 cm 미만이면 평가하지 않는다."""
    expected_len = horizontal_norm(expected_dn, expected_de)
    actual_len = horizontal_norm(actual_dn, actual_de)

    if expected_len < MIN_H_MOVE_FOR_TRACK_EVAL:
        return "N/A", None, None

    ux = expected_dn / expected_len
    uy = expected_de / expected_len
    along = actual_dn * ux + actual_de * uy
    cross = abs(actual_dn * uy - actual_de * ux)
    ratio = along / expected_len

    if actual_len < 0.10:
        status = "NO_TRACK"
    elif ratio < 0.0:
        status = "WRONG_DIRECTION"
    elif ratio < 0.20:
        status = "NO_TRACK"
    elif ratio >= TRACK_RATIO_OK and cross <= CROSS_TRACK_OK:
        status = "OK"
    else:
        status = "POOR_TRACK"
    return status, ratio, cross


_NUM_RE = re.compile(r"[-+]?\d*\.\d+|[-+]?\d+")


def nums(s):
    return [float(x) for x in _NUM_RE.findall(s)]
