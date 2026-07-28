#!/usr/bin/env python3
"""
Starling 2 - Custom Mission Test v13
MAVSDK command + PX4/VOXL pipe based reach judgement + editable mission plan

핵심 목적:
  - Offboard 명령 전송은 MAVSDK PositionNedYaw를 사용한다.
  - 도달 판정은 MAVSDK telemetry.position_velocity_ned()가 아니라
    voxl-inspect-pose -n px4_vehicle_local_position 값을 직접 읽어서 수행한다.
  - 코드 상단의 MISSION_PLAN 리스트만 수정해서 상대 이동, hover, 상승, 하강, 고도 변경을
    원하는 순서로 실행한다.
  - yaw/turn 회전 명령을 지원한다.
  - forward/backward/right/left는 현재 위치 기준 상대 이동이다.
  - MOVE_FRAME="body"이면 회전 후 기체가 바라보는 방향 기준으로 이동한다.
  - MOVE_FRAME="world"이면 기존처럼 NED 축 기준으로 이동한다.
  - home/goto는 원점/절대 좌표 이동이다.
  - speed_mps는 PositionNedYaw의 velocity 필드가 아니라, position setpoint를
    중간 목표점으로 나누어 이동시키는 ramp 방식으로 구현한다.
  - 정밀 이동을 위해 h_tol/v_tol/stable_time을 사용한다.
  - 정밀 착륙을 위해 land() 전 낮은 고도(preland altitude)로 먼저 하강한다.
  - MAVSDK NED 값은 비교/진단 로그와 CSV에만 저장한다.

추천 실행:
  python3 /home/root/path_flight_phase1_custom_mission_v12_relative_moves.py \
    --csv auto \
    --csv-sample-sec 1.0

주의:
  - PositionNedYaw는 local NED frame 기준이다.
  - MOVE_FRAME="body"일 때 forward/right/left/backward는 현재 yaw를 이용해 local NED 목표로 변환된다.
  - MOVE_FRAME="world"일 때 forward=N+, right=E+이다.
  - goto/home은 항상 원점 기준 절대 목표 이동이다.
  - px4_vehicle_local_position의 x/y/z가 MAVSDK N/E/D와 거의 같은 축이라고 가정하되,
    초기 offset을 자동으로 보정해서 target 비교에 사용한다.
"""

import argparse
import asyncio
import csv
import math
import os
import re
import signal
import sys
import time
from datetime import datetime

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


# ================= DEFAULT CONFIG =================
DEFAULT_TAKEOFF_ALT = 0.4
DEFAULT_MOVE_DISTANCE = 0.5
DEFAULT_HOVER_TIME = 3.0
DEFAULT_LATE_OBSERVE = 5.0

DEFAULT_H_TOL = 0.02
DEFAULT_V_TOL = 0.06
DEFAULT_STABLE_TIME = 0.8
DEFAULT_PRELAND_ALT = 0.25
DEFAULT_PRELAND_HOVER_TIME = 2.0
MOVE_TIMEOUT = 15.0

SETPOINT_HZ = 20.0
POS_OK_HOLD_SEC = 5.0
LAND_WAIT_TIMEOUT = 15.0

MAX_ALT_DEVIATION = 1.5
MAX_HORIZONTAL_DEV = 2.5
MAX_STREAM_FAIL = 5
MAV_STALE_WARN_SEC = 1.5
FEEDBACK_STALE_WARN_SEC = 0.8

MIN_H_MOVE_FOR_TRACK_EVAL = 0.20
TRACK_RATIO_OK = 0.70
CROSS_TRACK_OK = 0.20
DEFAULT_CSV_SAMPLE_SEC = 1.0
DEFAULT_YAW_TOL = 5.0
DEFAULT_YAW_STABLE_TIME = 0.5

# MOVE_FRAME controls how forward/backward/right/left are interpreted.
#   "body"  : relative to the current planned yaw/heading. After turn/yaw, forward follows the new heading.
#   "world" : fixed local NED frame. forward=N+, right=E+.
MOVE_FRAME = "body"

# ==================================================
# USER MISSION PLAN CONFIG
# ==================================================
# 이 리스트만 바꾸면 원하는 순서대로 이동할 수 있다.
#
# v12 핵심 규칙:
#   - forward/backward/right/left = 현재 위치 기준 상대 이동
#   - home                         = 원점(N=0, E=0)으로 복귀
#   - goto                         = 원점 기준 절대 좌표로 이동
#   - altitude_m을 생략하면 현재 고도를 유지한다.
#
# 상대 이동 tuple 형식:
#   (direction, distance_m, speed_mps, hover_sec)
#     -> 현재 고도 유지
#   (direction, distance_m, speed_mps, hover_sec, altitude_m)
#     -> 이동하면서 altitude_m 절대고도로 변경
#
# 위치 유지:
#   ("hover", hover_sec)
#   ("hold",  hover_sec)
#
# 상승/하강:
#   ("up",   delta_alt_m, speed_mps, hover_sec)    # 현재 N/E 위치에서 delta_alt_m 만큼 상승
#   ("down", delta_alt_m, speed_mps, hover_sec)    # 현재 N/E 위치에서 delta_alt_m 만큼 하강
#   ("alt",  target_alt_m, speed_mps, hover_sec)   # 현재 N/E 위치에서 target_alt_m 절대고도로 변경
#
# 원점/절대 이동:
#   ("home", speed_mps, hover_sec)
#   ("home", speed_mps, hover_sec, altitude_m)
#   ("goto", n_rel, e_rel, speed_mps, hover_sec)
#   ("goto", n_rel, e_rel, speed_mps, hover_sec, altitude_m)
#
# 이름까지 직접 지정하는 tuple 형식:
#   (name, command, value..., speed_mps, hover_sec, altitude_m)
#   예: ("move_right", "right", 1.0, 0.3, 2.0)
#       ("go_center",  "home",  0.3, 2.0)
#       ("go_abs",     "goto",  0.5, -0.5, 0.3, 2.0, 1.2)
#
# dict 형식도 사용 가능:
#   {"name":"diag_abs", "command":"goto", "n":0.5, "e":0.5, "speed":0.3, "hover":2.0}
#   {"name":"rel_right", "command":"right", "distance":0.5, "speed":0.3, "hover":2.0}
#   {"name":"wait_here", "command":"hover", "hover":3.0}
#
# speed_mps:
#   None 또는 0 이하이면 최종 위치 setpoint를 바로 보낸다.
#   양수이면 target setpoint를 해당 속도로 천천히 이동시켜 속도를 제어한다.
USE_TOP_MISSION_PLAN = True
DEFAULT_STEP_SPEED = 0.45
DEFAULT_STEP_HOVER_SEC = 2.0
MIN_COMMAND_ALT = 0.15

MISSION_PLAN = [
    # MOVE_FRAME = "body"이면 forward/right/left/backward는 현재 yaw 기준 상대 이동이다.
    # MOVE_FRAME = "world"이면 기존처럼 forward=N+, right=E+ 기준 이동이다.
    # direction, distance_m, speed_mps, hover_sec, altitude_m
    
    ("right", 1.35, 0.50, 0.0),       # 현재 위치에서 오른쪽 4m 이동, 현재 고도 유지
    ("hover", 2.0),                  # 현재 위치/고도 3초 유지
    ("turn", -90, 30, 2.0),
    ("right", 10.0, 0.50, 0.0),        # 현재 위치에서 왼쪽 6m 이동, 상승 후 고도 유지
    ("up", 0.6, 0.40, 0.0),          # 현재 위치에서 0.6m 상승
    ("hover", 2.0),                  # 현재 위치/고도 3초 유지
    ("left", 9.5, 0.50, 0.0),        # 현재 위치에서 왼쪽 6m 이동, 상승 후 고도 유지
    ("up", 0.6, 0.40, 0.0),          # 현재 위치에서 0.6m 상승
    ("hover", 2.0),                  # 현재 위치/고도 3초 유지
    ("right", 9.5, 0.50, 0.0),        # 현재 위치에서 왼쪽 6m 이동, 상승 후 고도 유지
    ("up", 0.6, 0.40, 0.0),          # 현재 위치에서 0.6m 상승
    ("hover", 2.0),                  # 현재 위치/고도 3초 유지
    ("left", 9.5, 0.50, 0.0),        # 현재 위치에서 왼쪽 6m 이동, 상승 후 고도 유지
    ("up", 0.6, 0.40, 0.0),          # 현재 위치에서 0.6m 상승
    ("hover", 2.0),                  # 현재 위치/고도 3초 유지
    ("right", 9.5, 0.50, 0.0),        # 현재 위치에서 왼쪽 6m 이동, 상승 후 고도 유지
    ("down", 1.8, 0.40, 0.0),          # 현재 위치에서 0.6m 상승
    ("left", 10.0, 0.50, 0.0),        # 현재 위치에서 왼쪽 6m 이동, 상승 후 고도 유지
    ("turn", 90, 30, 2.0),
    ("left", 1.35, 0.50, 0.0),       # 현재 위치에서 오른쪽 4m 이동, 현재 고도 유지
]
# ==================================================
# ==================================================


def wallclock():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def log(msg):
    print("[{}] {}".format(wallclock(), msg), flush=True)


def horizontal_norm(n, e):
    return math.sqrt(n * n + e * e)


def normalize_angle_deg(angle):
    """Normalize angle to [-180, 180)."""
    return (float(angle) + 180.0) % 360.0 - 180.0


def angle_diff_deg(target, current):
    """Shortest signed difference target-current in degrees."""
    return normalize_angle_deg(float(target) - float(current))


def yaw_to_360(angle):
    return float(angle) % 360.0


def body_delta_to_ned(forward_m, right_m, yaw_deg):
    """Convert body-frame forward/right displacement to local N/E displacement.

    PX4/MAVSDK NED yaw convention is assumed: yaw=0 faces North, yaw=90 faces East.
    At yaw=0: forward=N+, right=E+.
    At yaw=90: forward=E+, right=N-.
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
    # world: local NED fixed axes. body: current yaw/heading axes.
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


def normalize_mission_step(item, idx, default_alt):
    """Convert a tuple/list/dict mission item into a normalized step dict.

    Normalized kinds:
      - move_rel  : relative movement from the current planned N/E position
      - goto      : absolute mission target in relative N/E/alt coordinates
      - hover     : keep the current target/position for hover seconds
      - alt_delta : keep current N/E and change altitude by +delta (up) or -delta (down)
      - alt_abs   : keep current N/E and change to an absolute altitude

    v12 rule:
      forward/backward/right/left are RELATIVE to the current planned position.
      home/goto are ABSOLUTE targets in the initial local NED-relative frame.
    """
    rel_dirs = {"forward", "backward", "right", "left"}
    abs_cmds = {"home", "return", "origin", "goto"}
    hover_cmds = {"hover", "hold", "wait", "stay"}
    alt_delta_cmds = {"up", "ascend", "rise", "down", "descend"}
    alt_abs_cmds = {"alt", "altitude", "height"}
    yaw_abs_cmds = {"yaw", "heading"}
    yaw_delta_cmds = {"turn", "rotate"}
    all_cmds = rel_dirs | abs_cmds | hover_cmds | alt_delta_cmds | alt_abs_cmds | yaw_abs_cmds | yaw_delta_cmds

    def base_step(name, label, kind, n_rel=None, e_rel=None, alt=None,
                  speed=DEFAULT_STEP_SPEED, hover=DEFAULT_STEP_HOVER_SEC,
                  alt_delta=None, alt_target=None, dn=None, de=None,
                  direction=None, distance=None, frame=None,
                  yaw_target=None, yaw_delta=None, yaw_speed=None):
        return {
            "name": str(name),
            "label": str(label),
            "kind": kind,
            # For absolute goto/home:
            "n_rel": n_rel,
            "e_rel": e_rel,
            # For relative movement. dn/de may be recalculated at runtime if frame=body.
            "dn": dn,
            "de": de,
            "direction": direction,
            "distance": distance,
            "frame": frame,
            # Optional absolute altitude; None means inherit current planned altitude.
            "alt": alt,
            "speed": None if speed is None else float(speed),
            "hover": float(hover),
            "alt_delta": alt_delta,
            "alt_target": alt_target,
            # Yaw commands
            "yaw_target": yaw_target,
            "yaw_delta": yaw_delta,
            "yaw_speed": None if yaw_speed is None else float(yaw_speed),
        }

    def optional_alt_from_dict(step):
        if "alt" in step:
            return float(step["alt"])
        if "altitude" in step:
            return float(step["altitude"])
        if "height" in step:
            return float(step["height"])
        return None

    if isinstance(item, dict):
        step = dict(item)
        name = step.get("name", "step_{:02d}".format(idx))
        command = step.get("command", step.get("cmd", step.get("target", step.get("direction", step.get("dir", None)))))
        command = str(command).lower() if command is not None else None
        speed = step.get("speed", step.get("speed_mps", DEFAULT_STEP_SPEED))
        hover = step.get("hover", step.get("hover_sec", DEFAULT_STEP_HOVER_SEC))

        if command in hover_cmds:
            hover = step.get("hover", step.get("hover_sec", step.get("sec", step.get("duration", DEFAULT_STEP_HOVER_SEC))))
            return base_step(name, "HOVER", "hover", speed=None, hover=hover)

        if command in alt_delta_cmds:
            delta = float(step.get("delta", step.get("distance", step.get("dist", step.get("alt_delta", 0.0)))))
            if command in ("down", "descend"):
                delta = -abs(delta)
            else:
                delta = abs(delta)
            return base_step(name, command.upper(), "alt_delta", speed=speed, hover=hover, alt_delta=delta)

        if command in alt_abs_cmds:
            target_alt = float(step.get("alt", step.get("altitude", step.get("height", default_alt))))
            return base_step(name, "ALT", "alt_abs", speed=speed, hover=hover, alt_target=target_alt)

        if command in yaw_abs_cmds:
            target_yaw = float(step.get("yaw", step.get("target_yaw", step.get("heading", step.get("angle", 0.0)))))
            yaw_speed = step.get("yaw_speed", step.get("speed", step.get("speed_deg_s", 30.0)))
            return base_step(name, "YAW", "yaw_abs", speed=None, hover=hover, yaw_target=target_yaw, yaw_speed=yaw_speed)

        if command in yaw_delta_cmds:
            delta_yaw = float(step.get("delta", step.get("yaw_delta", step.get("angle", 0.0))))
            yaw_speed = step.get("yaw_speed", step.get("speed", step.get("speed_deg_s", 30.0)))
            return base_step(name, "TURN", "yaw_delta", speed=None, hover=hover, yaw_delta=delta_yaw, yaw_speed=yaw_speed)

        alt = optional_alt_from_dict(step)
        if command in ("home", "return", "origin"):
            return base_step(name, "HOME", "goto", n_rel=0.0, e_rel=0.0, alt=alt, speed=speed, hover=hover)

        if command == "goto" or ("n" in step or "e" in step or "n_rel" in step or "e_rel" in step):
            n_rel = float(step.get("n", step.get("n_rel", 0.0)))
            e_rel = float(step.get("e", step.get("e_rel", 0.0)))
            label = step.get("label", "GOTO")
            return base_step(name, label, "goto", n_rel=n_rel, e_rel=e_rel, alt=alt, speed=speed, hover=hover)

        if command in rel_dirs:
            distance = float(step.get("distance", step.get("dist", 0.0)))
            frame = str(step.get("frame", step.get("move_frame", MOVE_FRAME))).lower()
            dn, de = direction_to_ned(command, distance, 0.0, "world")
            return base_step(name, command.upper(), "move_rel", dn=dn, de=de, alt=alt, speed=speed, hover=hover,
                             direction=command, distance=distance, frame=frame)

        raise ValueError("Mission dict step {} needs command/direction/target or n/e: {}".format(idx, item))

    if not isinstance(item, (tuple, list)):
        raise ValueError("Mission step {} must be tuple/list/dict, got {}".format(idx, type(item)))

    vals = list(item)
    if len(vals) < 1:
        raise ValueError("Mission tuple step {} is empty: {}".format(idx, item))

    # Named tuple form: (name, command, value..., speed, hover, alt)
    if len(vals) >= 2 and isinstance(vals[0], str) and isinstance(vals[1], str) and vals[1].lower() in all_cmds:
        name = vals[0]
        command = vals[1].lower()
        rest = vals[2:]
    else:
        command = str(vals[0]).lower()
        name = "{}_{}".format(command, idx)
        rest = vals[1:]

    if command not in all_cmds:
        raise ValueError("Unknown mission command at step {}: {}".format(idx, vals[0]))

    if command in hover_cmds:
        hover = float(rest[0]) if len(rest) >= 1 else DEFAULT_STEP_HOVER_SEC
        return base_step(name, "HOVER", "hover", speed=None, hover=hover)

    if command in alt_delta_cmds:
        if len(rest) < 1:
            raise ValueError("{} command needs delta altitude, e.g. ('{}', 0.5, 0.2, 2.0)".format(command, command))
        delta = float(rest[0])
        if command in ("down", "descend"):
            delta = -abs(delta)
        else:
            delta = abs(delta)
        speed = rest[1] if len(rest) >= 2 else DEFAULT_STEP_SPEED
        hover = float(rest[2]) if len(rest) >= 3 else DEFAULT_STEP_HOVER_SEC
        return base_step(name, command.upper(), "alt_delta", speed=speed, hover=hover, alt_delta=delta)

    if command in alt_abs_cmds:
        if len(rest) < 1:
            raise ValueError("alt command needs target altitude, e.g. ('alt', 1.2, 0.2, 2.0)")
        target_alt = float(rest[0])
        speed = rest[1] if len(rest) >= 2 else DEFAULT_STEP_SPEED
        hover = float(rest[2]) if len(rest) >= 3 else DEFAULT_STEP_HOVER_SEC
        return base_step(name, "ALT", "alt_abs", speed=speed, hover=hover, alt_target=target_alt)

    if command in yaw_abs_cmds:
        if len(rest) < 1:
            raise ValueError("yaw command needs target yaw deg, e.g. ('yaw', 90, 30, 2.0)")
        target_yaw = float(rest[0])
        yaw_speed = rest[1] if len(rest) >= 2 else 30.0
        hover = float(rest[2]) if len(rest) >= 3 else DEFAULT_STEP_HOVER_SEC
        return base_step(name, "YAW", "yaw_abs", speed=None, hover=hover, yaw_target=target_yaw, yaw_speed=yaw_speed)

    if command in yaw_delta_cmds:
        if len(rest) < 1:
            raise ValueError("turn command needs delta yaw deg, e.g. ('turn', 90, 30, 2.0)")
        delta_yaw = float(rest[0])
        yaw_speed = rest[1] if len(rest) >= 2 else 30.0
        hover = float(rest[2]) if len(rest) >= 3 else DEFAULT_STEP_HOVER_SEC
        return base_step(name, "TURN", "yaw_delta", speed=None, hover=hover, yaw_delta=delta_yaw, yaw_speed=yaw_speed)

    if command in ("home", "return", "origin"):
        # New v12 form: ("home", speed, hover[, alt])
        # Backward compatible old form: ("home", 0.0, speed, hover[, alt])
        if len(rest) >= 3 and abs(float(rest[0])) < 1e-9:
            speed = rest[1]
            hover = float(rest[2])
            alt = float(rest[3]) if len(rest) >= 4 else None
        else:
            speed = rest[0] if len(rest) >= 1 else DEFAULT_STEP_SPEED
            hover = float(rest[1]) if len(rest) >= 2 else DEFAULT_STEP_HOVER_SEC
            alt = float(rest[2]) if len(rest) >= 3 else None
        return base_step(name, "HOME", "goto", n_rel=0.0, e_rel=0.0, alt=alt, speed=speed, hover=hover)

    if command == "goto":
        # Absolute target in initial local frame.
        # ("goto", n_rel, e_rel, speed, hover[, alt])
        if len(rest) < 4:
            raise ValueError("goto command needs n, e, speed, hover, e.g. ('goto', 0.5, -0.5, 0.3, 2.0)")
        n_rel = float(rest[0])
        e_rel = float(rest[1])
        speed = rest[2]
        hover = float(rest[3])
        alt = float(rest[4]) if len(rest) >= 5 else None
        return base_step(name, "GOTO", "goto", n_rel=n_rel, e_rel=e_rel, alt=alt, speed=speed, hover=hover)

    # Relative movement commands.
    # (direction, distance, speed, hover[, alt])
    if len(rest) < 1:
        raise ValueError("Movement command {} needs distance, e.g. ('{}', 1.0, 0.3, 2.0)".format(command, command))
    distance = float(rest[0])
    speed = rest[1] if len(rest) >= 2 else DEFAULT_STEP_SPEED
    hover = float(rest[2]) if len(rest) >= 3 else DEFAULT_STEP_HOVER_SEC
    alt = float(rest[3]) if len(rest) >= 4 else None
    dn, de = direction_to_ned(command, distance, 0.0, "world")
    return base_step(name, command.upper(), "move_rel", dn=dn, de=de, alt=alt, speed=speed, hover=hover,
                     direction=command, distance=distance, frame=MOVE_FRAME)


def build_default_direction_plan(args):
    """Fallback plan from CLI --directions/--distance.

    In v12, direction commands are relative. The optional return step uses HOME, which is absolute.
    """
    steps = []
    idx = 1
    for direction in args.directions:
        steps.append(normalize_mission_step(
            ("{}".format(direction), direction, args.distance, args.default_speed, args.hover_time, args.altitude),
            idx, args.altitude))
        idx += 1
        if not args.no_return_between:
            steps.append(normalize_mission_step(
                ("back_from_{}".format(direction), "home", args.default_speed, args.hover_time, args.altitude),
                idx, args.altitude))
            steps[-1]["label"] = "RETURN_FROM_{}".format(direction.upper())
            idx += 1
    return steps

def build_mission_plan(args):
    if USE_TOP_MISSION_PLAN and MISSION_PLAN and not args.ignore_mission_plan:
        return [normalize_mission_step(item, i + 1, args.altitude) for i, item in enumerate(MISSION_PLAN)]
    return build_default_direction_plan(args)


def parse_voxl_inspect_pose_line(line):
    """Parse one row from `voxl-inspect-pose -n <pose_pipe>`.

    Robust against the previous parser bug where timestamp was accidentally treated as x.
    We split by '|' and parse position from the second column only.

    Expected data line:
      timestamp(ms)| x y z | roll pitch yaw | vx vy vz | wx wy wz
    """
    if not line:
        return None
    text = line.strip()
    if not text or "|" not in text:
        return None

    upper = text.upper()
    if "TIMESTAMP" in upper or "POSITION" in upper or "ERROR" in upper:
        return None

    parts = [p.strip() for p in text.split("|")]
    if len(parts) < 4:
        return None

    def nums(s):
        return [float(x) for x in re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", s)]

    try:
        ts_nums = nums(parts[0])
        pos_nums = nums(parts[1])
        rpy_nums = nums(parts[2])
        vel_nums = nums(parts[3])
        ang_nums = nums(parts[4]) if len(parts) > 4 else []

        if len(ts_nums) < 1 or len(pos_nums) < 3:
            return None

        # Some pipes may not include velocity. Keep blanks as None.
        while len(rpy_nums) < 3:
            rpy_nums.append(float("nan"))
        while len(vel_nums) < 3:
            vel_nums.append(float("nan"))
        while len(ang_nums) < 3:
            ang_nums.append(float("nan"))

        return {
            "voxl_ts_ms": ts_nums[0],
            "pos_x": pos_nums[0],
            "pos_y": pos_nums[1],
            "pos_z": pos_nums[2],
            "roll_deg": rpy_nums[0],
            "pitch_deg": rpy_nums[1],
            "yaw_deg": rpy_nums[2],
            "vel_x": vel_nums[0],
            "vel_y": vel_nums[1],
            "vel_z": vel_nums[2],
            "ang_x": ang_nums[0],
            "ang_y": ang_nums[1],
            "ang_z": ang_nums[2],
            "raw": text,
        }
    except Exception:
        return None


def evaluate_horizontal_tracking(expected_dn, expected_de, actual_dn, actual_de):
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


class DroneController:
    def __init__(self, args):
        self.args = args
        self.drone = System(mavsdk_server_address=args.server_address, port=args.port)

        self.current_target_yaw = float(args.yaw)
        self.target = PositionNedYaw(0.0, 0.0, 0.0, self.current_target_yaw)

        self.streaming = False
        self.monitoring = False
        self.aborted = False

        self.stream_task = None
        self.mav_task = None
        self.health_task = None
        self.voxl_tasks = []
        self.sync_csv_task = None

        # MAVSDK NED, used for command-frame initialization and comparison only.
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_z = 0.0
        self.mav_x = 0.0
        self.mav_y = 0.0
        self.mav_z = 0.0
        self.mav_vn = 0.0
        self.mav_ve = 0.0
        self.mav_vd = 0.0
        self._mav_update_count = 0
        self._last_mav_update_time = 0.0
        self._last_mav_value = (0.0, 0.0, 0.0)

        # VOXL/PX4 pipe data.
        self.voxl_latest = {}
        self.voxl_initial = {}
        self.voxl_reader_errors = {}

        # Reach feedback alignment: pipe x/y/z -> MAVSDK command N/E/D frame.
        self.reach_ready = False
        self.reach_pipe = args.reach_pipe
        self.reach_offset_n = 0.0
        self.reach_offset_e = 0.0
        self.reach_offset_d = 0.0
        self.reach_initial_raw = None

        self.takeoff_started = False
        self._emergency_lock = asyncio.Lock()
        self._emergency_triggered = False
        self._stream_fail_count = 0

        self.active_stage = None
        self.active_phase = "IDLE"
        self.stage_samples = {}
        self.sync_samples = []
        self.results = []
        self.mission_end_before_preland = None
        self.preland_end_before_land = None
        self.mission_end_before_land = None
        self.land_end_snapshot = None

    # ---------- MAVSDK connection / health ----------

    async def connect(self):
        log("[1/7] Connecting...")
        await self.drone.connect()
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                log("       Connected!")
                return
            await asyncio.sleep(0.1)

    async def wait_position_stable(self, hold_sec=POS_OK_HOLD_SEC, timeout=60.0):
        log("[2/7] Waiting for MAVSDK local position estimate health ({:.1f}s)...".format(hold_sec))
        ok_start = None
        deadline = asyncio.get_event_loop().time() + timeout
        async for health in self.drone.telemetry.health():
            now = asyncio.get_event_loop().time()
            if now > deadline:
                raise RuntimeError("MAVSDK/PX4 local position health did not stabilize")
            if health.is_local_position_ok:
                if ok_start is None:
                    ok_start = now
                elif now - ok_start >= hold_sec:
                    log("       Local position health OK.")
                    return
            else:
                ok_start = None
            await asyncio.sleep(0.2)

    async def check_vio_quality_preflight(self):
        log("[3/7] Checking local-position health (5 samples)...")
        count = 0
        async for health in self.drone.telemetry.health():
            if not health.is_local_position_ok:
                raise RuntimeError("Local position degraded during pre-check")
            count += 1
            if count >= 5:
                break
            await asyncio.sleep(0.5)
        log("       Local position health stable.")

    async def capture_initial_mavsdk_position(self):
        log("[4/7] Capturing initial MAVSDK NED position (10 samples over ~3s)...")
        xs, ys, zs = [], [], []
        count = 0
        async for pos in self.drone.telemetry.position_velocity_ned():
            xs.append(pos.position.north_m)
            ys.append(pos.position.east_m)
            zs.append(pos.position.down_m)
            count += 1
            if count >= 10:
                break
            await asyncio.sleep(0.3)

        self.initial_x = sum(xs) / len(xs)
        self.initial_y = sum(ys) / len(ys)
        self.initial_z = sum(zs) / len(zs)
        self.mav_x = self.initial_x
        self.mav_y = self.initial_y
        self.mav_z = self.initial_z

        x_range = max(xs) - min(xs)
        y_range = max(ys) - min(ys)
        z_range = max(zs) - min(zs)
        log("       Initial MAVSDK NED: N={:+.3f} E={:+.3f} D={:+.3f}".format(
            self.initial_x, self.initial_y, self.initial_z))
        log("       Initial MAVSDK ranges: N={:.6f} E={:.6f} D={:.6f}".format(
            x_range, y_range, z_range))

        if x_range < 1e-9 and y_range < 1e-9 and z_range < 1e-9:
            raise RuntimeError("MAVSDK NED data appears STALE. Shake drone 10s and retry.")
        if x_range > 0.05 or y_range > 0.05 or z_range > 0.05:
            raise RuntimeError(
                "Initial MAVSDK position UNSTABLE: N_r={:.3f} E_r={:.3f} D_r={:.3f}".format(
                    x_range, y_range, z_range))

        self.target = PositionNedYaw(self.initial_x, self.initial_y, self.initial_z, self.current_target_yaw)

    # ---------- coordinate helpers ----------

    def mav_rel_position(self):
        return (
            self.mav_x - self.initial_x,
            self.mav_y - self.initial_y,
            self.initial_z - self.mav_z,
        )

    def rel_target(self):
        return (
            self.target.north_m - self.initial_x,
            self.target.east_m - self.initial_y,
            self.initial_z - self.target.down_m,
        )

    def _pipe_prefix(self, pipe):
        return "voxl_{}_".format(pipe)

    def feedback_state(self):
        """Return reach feedback in the MAVSDK command N/E/D frame.

        The raw pipe x/y/z is aligned by an initial offset:
          cmd_n = raw_x + reach_offset_n
          cmd_e = raw_y + reach_offset_e
          cmd_d = raw_z + reach_offset_d
        """
        latest = self.voxl_latest.get(self.reach_pipe)
        if latest is None or not self.reach_ready:
            return None

        now = asyncio.get_event_loop().time()
        cmd_n = latest["pos_x"] + self.reach_offset_n
        cmd_e = latest["pos_y"] + self.reach_offset_e
        cmd_d = latest["pos_z"] + self.reach_offset_d
        return {
            "source": self.reach_pipe,
            "cmd_n": cmd_n,
            "cmd_e": cmd_e,
            "cmd_d": cmd_d,
            "rel_n": cmd_n - self.initial_x,
            "rel_e": cmd_e - self.initial_y,
            "rel_u": self.initial_z - cmd_d,
            "vn": latest.get("vel_x", float("nan")),
            "ve": latest.get("vel_y", float("nan")),
            "vd": latest.get("vel_z", float("nan")),
            "raw_x": latest["pos_x"],
            "raw_y": latest["pos_y"],
            "raw_z": latest["pos_z"],
            "yaw_deg": latest.get("yaw_deg", float("nan")),
            "roll_deg": latest.get("roll_deg", float("nan")),
            "pitch_deg": latest.get("pitch_deg", float("nan")),
            "age_sec": now - latest["mono_time"],
            "wallclock": latest["wallclock"],
        }

    def feedback_rel_position(self):
        fb = self.feedback_state()
        if fb is None:
            return None
        return fb["rel_n"], fb["rel_e"], fb["rel_u"]

    async def wait_reach_pipe_ready(self, timeout=8.0):
        """Wait until --reach-pipe has valid pose and align it to MAVSDK command frame."""
        log("       Waiting for reach feedback pipe '{}'...".format(self.reach_pipe))
        deadline = asyncio.get_event_loop().time() + timeout
        while asyncio.get_event_loop().time() < deadline:
            latest = self.voxl_latest.get(self.reach_pipe)
            if latest is not None:
                self.reach_initial_raw = dict(latest)
                self.reach_offset_n = self.initial_x - latest["pos_x"]
                self.reach_offset_e = self.initial_y - latest["pos_y"]
                self.reach_offset_d = self.initial_z - latest["pos_z"]
                self.reach_ready = True
                log("       Reach feedback ready: {} raw=(x{:+.3f},y{:+.3f},z{:+.3f})".format(
                    self.reach_pipe, latest["pos_x"], latest["pos_y"], latest["pos_z"]))
                log("       Alignment offset to command NED: dN{:+.3f} dE{:+.3f} dD{:+.3f}".format(
                    self.reach_offset_n, self.reach_offset_e, self.reach_offset_d))
                return
            await asyncio.sleep(0.05)
        raise RuntimeError("Reach feedback pipe '{}' did not produce valid pose within {:.1f}s".format(
            self.reach_pipe, timeout))

    # ---------- VOXL/PX4 pipe readers ----------

    async def voxl_pose_reader(self, pipe_name):
        if not pipe_name:
            return
        log("[voxl] starting pose reader: voxl-inspect-pose -n {}".format(pipe_name))
        backoff = 1.0

        while self.monitoring and not self.aborted:
            proc = None
            try:
                cmd = ["voxl-inspect-pose", "-n"]
                if pipe_name in ("local", "--local", "-l"):
                    cmd.append("-l")
                elif pipe_name in ("fixed", "--fixed", "-f"):
                    cmd.append("-f")
                else:
                    cmd.append(pipe_name)

                proc = await asyncio.create_subprocess_exec(
                    *cmd,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.STDOUT,
                )
                backoff = 1.0

                while self.monitoring and not self.aborted:
                    try:
                        raw = await asyncio.wait_for(proc.stdout.readline(), timeout=2.0)
                    except asyncio.TimeoutError:
                        continue
                    if not raw:
                        break
                    line = raw.decode("utf-8", errors="replace").strip()
                    parsed = parse_voxl_inspect_pose_line(line)
                    if parsed is None:
                        if "ERROR" in line.upper():
                            self.voxl_reader_errors[pipe_name] = line
                        continue

                    now = asyncio.get_event_loop().time()
                    parsed["mono_time"] = now
                    parsed["unix_time"] = time.time()
                    parsed["wallclock"] = wallclock()
                    self.voxl_latest[pipe_name] = parsed

                    if pipe_name not in self.voxl_initial:
                        self.voxl_initial[pipe_name] = {
                            "pos_x": parsed["pos_x"],
                            "pos_y": parsed["pos_y"],
                            "pos_z": parsed["pos_z"],
                            "voxl_ts_ms": parsed["voxl_ts_ms"],
                            "mono_time": now,
                        }
                        log("[voxl] {} initial pose: x={:+.3f} y={:+.3f} z={:+.3f} ts_ms={:.0f}".format(
                            pipe_name, parsed["pos_x"], parsed["pos_y"], parsed["pos_z"], parsed["voxl_ts_ms"]))

            except FileNotFoundError:
                log("[voxl] voxl-inspect-pose not found. Pipe '{}' disabled.".format(pipe_name))
                return
            except asyncio.CancelledError:
                raise
            except Exception as e:
                self.voxl_reader_errors[pipe_name] = str(e)
                log("[voxl] reader error for '{}': {}".format(pipe_name, e))
            finally:
                if proc is not None and proc.returncode is None:
                    try:
                        proc.terminate()
                        await asyncio.wait_for(proc.wait(), timeout=1.0)
                    except Exception:
                        try:
                            proc.kill()
                        except Exception:
                            pass

            if self.monitoring and not self.aborted:
                await asyncio.sleep(backoff)
                backoff = min(backoff * 1.5, 5.0)

    # ---------- recorders ----------

    def _append_stage_sample(self):
        if self.active_stage is None:
            return
        fb = self.feedback_state()
        now = asyncio.get_event_loop().time()
        t0 = self.active_stage["t_start"]
        mav_rel_n, mav_rel_e, mav_rel_u = self.mav_rel_position()
        target_n, target_e, target_u = self.rel_target()

        if fb is not None:
            h_dist = horizontal_norm(fb["cmd_n"] - self.target.north_m,
                                     fb["cmd_e"] - self.target.east_m)
            v_dist = abs(fb["cmd_d"] - self.target.down_m)
            fb_rel_n = fb["rel_n"]
            fb_rel_e = fb["rel_e"]
            fb_rel_u = fb["rel_u"]
            fb_vn = fb["vn"]
            fb_ve = fb["ve"]
            fb_age = fb["age_sec"]
        else:
            h_dist = float("nan")
            v_dist = float("nan")
            fb_rel_n = fb_rel_e = fb_rel_u = float("nan")
            fb_vn = fb_ve = fb_age = float("nan")

        sample = {
            "stage": self.active_stage["name"],
            "phase": self.active_phase,
            "t": now - t0,
            "wallclock": wallclock(),
            "fb_rel_n": fb_rel_n,
            "fb_rel_e": fb_rel_e,
            "fb_rel_u": fb_rel_u,
            "fb_vn": fb_vn,
            "fb_ve": fb_ve,
            "fb_age": fb_age,
            "mav_rel_n": mav_rel_n,
            "mav_rel_e": mav_rel_e,
            "mav_rel_u": mav_rel_u,
            "mav_vn": self.mav_vn,
            "mav_ve": self.mav_ve,
            "target_n": target_n,
            "target_e": target_e,
            "target_u": target_u,
            "h_dist": h_dist,
            "v_dist": v_dist,
        }
        self.stage_samples.setdefault(self.active_stage["name"], []).append(sample)

    def _append_sync_csv_sample(self):
        now = asyncio.get_event_loop().time()
        mav_rel_n, mav_rel_e, mav_rel_u = self.mav_rel_position()
        target_n, target_e, target_u = self.rel_target()
        fb = self.feedback_state()

        if fb is not None:
            fb_h = horizontal_norm(fb["cmd_n"] - self.target.north_m,
                                   fb["cmd_e"] - self.target.east_m)
            fb_v = abs(fb["cmd_d"] - self.target.down_m)
        else:
            fb_h = fb_v = ""

        row = {
            "unix_time": time.time(),
            "wallclock": wallclock(),
            "mono_time": now,
            "stage": self.active_stage["name"] if self.active_stage else "NONE",
            "phase": self.active_phase,
            "target_rel_n": target_n,
            "target_rel_e": target_e,
            "target_rel_u": target_u,
            "target_yaw_deg": self.current_target_yaw,

            "reach_pipe": self.reach_pipe,
            "fb_cmd_n": fb["cmd_n"] if fb else "",
            "fb_cmd_e": fb["cmd_e"] if fb else "",
            "fb_cmd_d": fb["cmd_d"] if fb else "",
            "fb_yaw_deg": fb.get("yaw_deg", "") if fb else "",
            "fb_rel_n": fb["rel_n"] if fb else "",
            "fb_rel_e": fb["rel_e"] if fb else "",
            "fb_rel_u": fb["rel_u"] if fb else "",
            "fb_vn": fb["vn"] if fb else "",
            "fb_ve": fb["ve"] if fb else "",
            "fb_vd": fb["vd"] if fb else "",
            "fb_age_sec": fb["age_sec"] if fb else "",
            "fb_h_dist": fb_h,
            "fb_v_dist": fb_v,

            "mav_abs_n": self.mav_x,
            "mav_abs_e": self.mav_y,
            "mav_abs_d": self.mav_z,
            "mav_rel_n": mav_rel_n,
            "mav_rel_e": mav_rel_e,
            "mav_rel_u": mav_rel_u,
            "mav_vn": self.mav_vn,
            "mav_ve": self.mav_ve,
            "mav_vd": self.mav_vd,
            "mav_updates": self._mav_update_count,
        }

        for pipe in self.args.voxl_pipes:
            prefix = self._pipe_prefix(pipe)
            latest = self.voxl_latest.get(pipe)
            initial = self.voxl_initial.get(pipe)
            if latest is None:
                for key in ["age_sec", "ts_ms", "x", "y", "z", "rel_x", "rel_y", "rel_z",
                            "vx", "vy", "vz", "roll_deg", "pitch_deg", "yaw_deg", "raw"]:
                    row[prefix + key] = ""
                continue
            row[prefix + "age_sec"] = now - latest["mono_time"]
            row[prefix + "ts_ms"] = latest["voxl_ts_ms"]
            row[prefix + "x"] = latest["pos_x"]
            row[prefix + "y"] = latest["pos_y"]
            row[prefix + "z"] = latest["pos_z"]
            if initial:
                row[prefix + "rel_x"] = latest["pos_x"] - initial["pos_x"]
                row[prefix + "rel_y"] = latest["pos_y"] - initial["pos_y"]
                row[prefix + "rel_z"] = latest["pos_z"] - initial["pos_z"]
            else:
                row[prefix + "rel_x"] = ""
                row[prefix + "rel_y"] = ""
                row[prefix + "rel_z"] = ""
            row[prefix + "vx"] = latest["vel_x"]
            row[prefix + "vy"] = latest["vel_y"]
            row[prefix + "vz"] = latest["vel_z"]
            row[prefix + "roll_deg"] = latest["roll_deg"]
            row[prefix + "pitch_deg"] = latest["pitch_deg"]
            row[prefix + "yaw_deg"] = latest["yaw_deg"]
            row[prefix + "raw"] = latest["raw"]

        self.sync_samples.append(row)

    async def sync_csv_sampler(self):
        period = max(0.2, float(self.args.csv_sample_sec))
        log("[csv] synchronized sampler running every {:.2f}s".format(period))
        while self.monitoring and not self.aborted:
            self._append_sync_csv_sample()
            await asyncio.sleep(period)

    # ---------- monitors / setpoint streamer ----------

    async def mavsdk_position_monitor(self):
        async for pos in self.drone.telemetry.position_velocity_ned():
            if not self.monitoring:
                break
            self.mav_x = pos.position.north_m
            self.mav_y = pos.position.east_m
            self.mav_z = pos.position.down_m
            self.mav_vn = pos.velocity.north_m_s
            self.mav_ve = pos.velocity.east_m_s
            self.mav_vd = pos.velocity.down_m_s

            new_value = (self.mav_x, self.mav_y, self.mav_z)
            if new_value != self._last_mav_value:
                self._last_mav_update_time = asyncio.get_event_loop().time()
                self._last_mav_value = new_value
                self._mav_update_count += 1

            self._append_stage_sample()
            await asyncio.sleep(0.05)

    async def health_monitor(self):
        consecutive_bad = 0
        async for health in self.drone.telemetry.health():
            if not self.monitoring:
                break
            if not health.is_local_position_ok:
                consecutive_bad += 1
                if consecutive_bad >= 3:
                    asyncio.ensure_future(self.trigger_emergency_land(
                        "local position health lost (3 consecutive bad samples)"))
            else:
                consecutive_bad = 0
            await asyncio.sleep(0.3)

    async def setpoint_streamer(self):
        period = 1.0 / SETPOINT_HZ
        while self.streaming:
            try:
                await self.drone.offboard.set_position_ned(self.target)
                self._stream_fail_count = 0
            except Exception as e:
                self._stream_fail_count += 1
                log("       [streamer] send failed ({}/{}): {}".format(
                    self._stream_fail_count, MAX_STREAM_FAIL, e))
                if self._stream_fail_count >= MAX_STREAM_FAIL:
                    asyncio.ensure_future(self.trigger_emergency_land(
                        "setpoint stream failed {} times".format(MAX_STREAM_FAIL)))
                    break
            await asyncio.sleep(period)

    async def arm_and_start_offboard(self):
        log("[5/7] Starting monitors + priming setpoint stream (3s)...")
        self.monitoring = True
        self.streaming = True

        # Ensure reach pipe is included in voxl readers.
        pipes = []
        for p in [self.reach_pipe] + list(self.args.voxl_pipes):
            if p and p not in pipes:
                pipes.append(p)
        self.args.voxl_pipes = pipes

        self.stream_task = asyncio.ensure_future(self.setpoint_streamer())
        self.mav_task = asyncio.ensure_future(self.mavsdk_position_monitor())
        self.health_task = asyncio.ensure_future(self.health_monitor())
        for pipe in self.args.voxl_pipes:
            self.voxl_tasks.append(asyncio.ensure_future(self.voxl_pose_reader(pipe)))
        if self.args.csv:
            self.sync_csv_task = asyncio.ensure_future(self.sync_csv_sampler())

        await asyncio.sleep(1.0)
        await self.wait_reach_pipe_ready(timeout=self.args.reach_pipe_timeout)
        await asyncio.sleep(2.0)

        log("[6/7] Arming...")
        await self.drone.action.arm()
        await asyncio.sleep(1.0)

        log("       *** RC must be in OFFBOARD mode ***")
        log("       Starting offboard...")
        try:
            await self.drone.offboard.start()
            log("       Offboard active!")
        except OffboardError as e:
            log("       Offboard FAILED: {}".format(e._result.result))
            await self.emergency_stop()
            raise

    # ---------- reach judgement using pipe feedback ----------

    async def wait_until_reached(self, name, timeout):
        log("       [wait] reaching {} using {} (h_tol={:.2f}m, v_tol={:.2f}m, timeout={:.1f}s)".format(
            name, self.reach_pipe, self.args.h_tol, self.args.v_tol, timeout))
        self.active_phase = "WAIT"
        start_time = asyncio.get_event_loop().time()
        in_tolerance_since = None
        last_print = -1.0

        while True:
            if self.aborted:
                return False, "ABORTED", asyncio.get_event_loop().time() - self.active_stage["t_start"]

            now = asyncio.get_event_loop().time()
            elapsed = now - start_time
            stage_elapsed = now - self.active_stage["t_start"]
            fb = self.feedback_state()
            if fb is None:
                h_dist = v_dist = float("inf")
            else:
                h_dist = horizontal_norm(fb["cmd_n"] - self.target.north_m,
                                         fb["cmd_e"] - self.target.east_m)
                v_dist = abs(fb["cmd_d"] - self.target.down_m)

            if elapsed - last_print >= 1.0:
                last_print = elapsed
                mav_rel_n, mav_rel_e, mav_rel_u = self.mav_rel_position()
                if fb is None:
                    log("       [{}] t={:.1f}s feedback not ready | mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                        name, elapsed, mav_rel_n, mav_rel_e, mav_rel_u))
                else:
                    stale = " FB_STALE!" if fb["age_sec"] > FEEDBACK_STALE_WARN_SEC else ""
                    mav_stale = " MAV_STALE!" if now - self._last_mav_update_time > MAV_STALE_WARN_SEC else ""
                    log("       [{}] t={:.1f}s fb_h={:.3f} fb_v={:.3f} "
                        "fb_rel=(N{:+.2f},E{:+.2f},U{:+.2f}) fb_vel=(N{:+.2f},E{:+.2f}) age={:.2f}s{} | "
                        "mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f}) mav_vel=(N{:+.2f},E{:+.2f}){}".format(
                            name, elapsed, h_dist, v_dist,
                            fb["rel_n"], fb["rel_e"], fb["rel_u"],
                            fb["vn"], fb["ve"], fb["age_sec"], stale,
                            mav_rel_n, mav_rel_e, mav_rel_u,
                            self.mav_vn, self.mav_ve, mav_stale))

            if h_dist <= self.args.h_tol and v_dist <= self.args.v_tol:
                if in_tolerance_since is None:
                    in_tolerance_since = now
                elif now - in_tolerance_since >= self.args.stable_time:
                    log("       [{}] REACHED by {} (stable {:.1f}s)".format(
                        name, self.reach_pipe, self.args.stable_time))
                    return True, "REACHED", stage_elapsed
            else:
                in_tolerance_since = None

            if elapsed > timeout:
                log("!!! TIMEOUT {} after {:.1f}s by {}, h={:.3f}m v={:.3f}m !!!".format(
                    name, timeout, self.reach_pipe, h_dist, v_dist))
                return False, "TIMEOUT", stage_elapsed

            await asyncio.sleep(0.1)

    async def observe_late_reach(self, name, duration):
        if duration <= 0.0 or self.aborted:
            return False, "NO_LATE_OBSERVE", None
        log("       [late] keeping same target for {:.1f}s, still judged by {}...".format(
            duration, self.reach_pipe))
        self.active_phase = "LATE_OBSERVE"
        start_time = asyncio.get_event_loop().time()
        in_tolerance_since = None
        last_print = -1.0

        while True:
            if self.aborted:
                return False, "ABORTED", asyncio.get_event_loop().time() - self.active_stage["t_start"]
            now = asyncio.get_event_loop().time()
            elapsed = now - start_time
            stage_elapsed = now - self.active_stage["t_start"]
            fb = self.feedback_state()
            if fb is None:
                h_dist = v_dist = float("inf")
            else:
                h_dist = horizontal_norm(fb["cmd_n"] - self.target.north_m,
                                         fb["cmd_e"] - self.target.east_m)
                v_dist = abs(fb["cmd_d"] - self.target.down_m)

            if elapsed - last_print >= 0.5:
                last_print = elapsed
                mav_rel_n, mav_rel_e, mav_rel_u = self.mav_rel_position()
                if fb:
                    log("       [{} LATE] +{:.1f}s fb_h={:.3f} fb_v={:.3f} "
                        "fb_rel=(N{:+.2f},E{:+.2f},U{:+.2f}) | mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                            name, elapsed, h_dist, v_dist,
                            fb["rel_n"], fb["rel_e"], fb["rel_u"],
                            mav_rel_n, mav_rel_e, mav_rel_u))
                else:
                    log("       [{} LATE] +{:.1f}s feedback not ready".format(name, elapsed))

            if h_dist <= self.args.h_tol and v_dist <= self.args.v_tol:
                if in_tolerance_since is None:
                    in_tolerance_since = now
                elif now - in_tolerance_since >= self.args.stable_time:
                    log("       [{}] LATE_REACHED by {} after +{:.1f}s".format(
                        name, self.reach_pipe, elapsed))
                    return True, "LATE_REACHED", stage_elapsed
            else:
                in_tolerance_since = None

            if elapsed > duration:
                log("       [{}] LATE_NOT_REACHED after {:.1f}s".format(name, duration))
                return False, "LATE_NOT_REACHED", stage_elapsed
            await asyncio.sleep(0.1)

    def hold_current_feedback_position(self, label):
        fb = self.feedback_state()
        if fb is None:
            # Last-resort fallback: hold current MAVSDK position.
            self.target = PositionNedYaw(self.mav_x, self.mav_y, self.mav_z, self.current_target_yaw)
            mav_rel_n, mav_rel_e, mav_rel_u = self.mav_rel_position()
            log("       [hold_current {}] feedback unavailable, fallback MAVSDK rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                label, mav_rel_n, mav_rel_e, mav_rel_u))
            return
        self.target = PositionNedYaw(fb["cmd_n"], fb["cmd_e"], fb["cmd_d"], self.current_target_yaw)
        log("       [hold_current {}] new target from {} rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
            label, self.reach_pipe, fb["rel_n"], fb["rel_e"], fb["rel_u"]))

    async def hover(self, duration, name, hold_current=False):
        if self.aborted:
            return
        if hold_current:
            self.hold_current_feedback_position(name)
        log("[hover {}] for {:.1f}s ({})".format(
            name, duration, "hold_current_feedback" if hold_current else "hold_target"))
        self.active_phase = "HOVER"
        start = asyncio.get_event_loop().time()
        last_print = -1.0
        while True:
            if self.aborted:
                return
            now = asyncio.get_event_loop().time()
            elapsed = now - start
            if elapsed > duration:
                return
            if elapsed - last_print >= 1.0:
                last_print = elapsed
                fb = self.feedback_state()
                mav_rel_n, mav_rel_e, mav_rel_u = self.mav_rel_position()
                if fb:
                    log("       [hover {}] t={:.1f}s fb_rel=(N{:+.2f},E{:+.2f},U{:+.2f}) | mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                        name, elapsed, fb["rel_n"], fb["rel_e"], fb["rel_u"],
                        mav_rel_n, mav_rel_e, mav_rel_u))
                else:
                    log("       [hover {}] t={:.1f}s feedback not ready | mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                        name, elapsed, mav_rel_n, mav_rel_e, mav_rel_u))
            await asyncio.sleep(0.2)

    async def ramp_setpoint_to(self, name, start_n, start_e, start_d, end_n, end_e, end_d, speed_mps):
        """Move the offboard position setpoint gradually to approximate a commanded speed.

        MAVSDK PositionNedYaw has no per-setpoint speed field. This function creates
        a moving target by updating self.target along the line at speed_mps.
        """
        if speed_mps is None or speed_mps <= 0:
            self.target = PositionNedYaw(end_n, end_e, end_d, self.current_target_yaw)
            return 0.0

        dn = end_n - start_n
        de = end_e - start_e
        dd = end_d - start_d
        dist = math.sqrt(dn * dn + de * de + dd * dd)
        if dist < 1e-6:
            self.target = PositionNedYaw(end_n, end_e, end_d, self.current_target_yaw)
            return 0.0

        duration = dist / max(0.05, float(speed_mps))
        log("       [ramp {}] moving setpoint {:.2f}m at {:.2f}m/s -> {:.1f}s".format(
            name, dist, speed_mps, duration))

        self.active_phase = "RAMP_SETPOINT"
        start_time = asyncio.get_event_loop().time()
        last_print = -1.0
        period = 1.0 / max(5.0, SETPOINT_HZ)

        while True:
            if self.aborted:
                return asyncio.get_event_loop().time() - start_time
            now = asyncio.get_event_loop().time()
            elapsed = now - start_time
            frac = min(1.0, elapsed / duration)
            tn = start_n + dn * frac
            te = start_e + de * frac
            td = start_d + dd * frac
            self.target = PositionNedYaw(tn, te, td, self.current_target_yaw)

            if elapsed - last_print >= 1.0 or frac >= 1.0:
                last_print = elapsed
                fb = self.feedback_state()
                target_rel_n, target_rel_e, target_rel_u = self.rel_target()
                if fb:
                    log("       [ramp {}] t={:.1f}/{:.1f}s target_rel=(N{:+.2f},E{:+.2f},U{:+.2f}) fb_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                        name, elapsed, duration, target_rel_n, target_rel_e, target_rel_u,
                        fb["rel_n"], fb["rel_e"], fb["rel_u"]))
                else:
                    log("       [ramp {}] t={:.1f}/{:.1f}s target_rel=(N{:+.2f},E{:+.2f},U{:+.2f}) feedback not ready".format(
                        name, elapsed, duration, target_rel_n, target_rel_e, target_rel_u))

            if frac >= 1.0:
                break
            await asyncio.sleep(period)

        self.target = PositionNedYaw(end_n, end_e, end_d, self.current_target_yaw)
        return duration

    async def go_to_relative(self, n_rel, e_rel, alt, name, expected_label, speed_mps=None):
        if self.aborted:
            return False

        fb_start = self.feedback_state()
        if fb_start is None:
            raise RuntimeError("Reach feedback not available before stage '{}'".format(name))
        mav_start_rel = self.mav_rel_position()

        end_x = self.initial_x + n_rel
        end_y = self.initial_y + e_rel
        end_z = self.initial_z - alt

        expected_dn = end_x - fb_start["cmd_n"]
        expected_de = end_y - fb_start["cmd_e"]
        expected_du = fb_start["cmd_d"] - end_z
        dist = math.sqrt(expected_dn * expected_dn + expected_de * expected_de + expected_du * expected_du)
        ramp_time_est = 0.0 if speed_mps is None or speed_mps <= 0 else dist / max(0.05, float(speed_mps))
        timeout = max(dist * 8.0 + 8.0, MOVE_TIMEOUT)

        log("[goto {}] [START] [{}] target rel N{:+.2f} E{:+.2f} U{:+.2f} "
            "expected_by_{} dN{:+.2f} dE{:+.2f} dU{:+.2f} dist={:.2f}m speed={}".format(
                name, expected_label, n_rel, e_rel, alt, self.reach_pipe,
                expected_dn, expected_de, expected_du, dist,
                "direct" if speed_mps is None or speed_mps <= 0 else "{:.2f}m/s".format(speed_mps)))

        self.active_stage = {
            "name": name,
            "t_start": asyncio.get_event_loop().time(),
            "expected_label": expected_label,
            "fb_start": dict(fb_start),
            "mav_start_rel": mav_start_rel,
            "target_rel": (n_rel, e_rel, alt),
            "expected_move": (expected_dn, expected_de, expected_du),
            "speed_mps": speed_mps,
        }
        self.active_phase = "RAMP_SETPOINT" if speed_mps and speed_mps > 0 else "WAIT"
        self.stage_samples[name] = []

        # Speed control by moving the target setpoint from current feedback position to final target.
        # After the ramp, perform the normal final-position reach judgement.
        if speed_mps is not None and speed_mps > 0:
            await self.ramp_setpoint_to(
                name,
                fb_start["cmd_n"], fb_start["cmd_e"], fb_start["cmd_d"],
                end_x, end_y, end_z,
                speed_mps,
            )
        else:
            self.target = PositionNedYaw(end_x, end_y, end_z, self.current_target_yaw)

        self.active_phase = "WAIT"
        reached, wait_reason, wait_reach_time = await self.wait_until_reached(name, timeout)
        late_reached = False
        late_reason = "NOT_NEEDED"
        late_reach_time = None
        if not reached and wait_reason == "TIMEOUT" and self.args.late_observe > 0:
            late_reached, late_reason, late_reach_time = await self.observe_late_reach(name, self.args.late_observe)

        fb_end = self.feedback_state()
        if fb_end is None:
            raise RuntimeError("Reach feedback not available after stage '{}'".format(name))
        mav_end_rel = self.mav_rel_position()

        actual_dn = fb_end["cmd_n"] - fb_start["cmd_n"]
        actual_de = fb_end["cmd_e"] - fb_start["cmd_e"]
        actual_du = fb_start["cmd_d"] - fb_end["cmd_d"]

        mav_actual_dn = mav_end_rel[0] - mav_start_rel[0]
        mav_actual_de = mav_end_rel[1] - mav_start_rel[1]
        mav_actual_du = mav_end_rel[2] - mav_start_rel[2]

        track_status, ratio, cross = evaluate_horizontal_tracking(expected_dn, expected_de, actual_dn, actual_de)

        if reached:
            status = "OK" if track_status in ("OK", "N/A") else "REACHED_POOR_DELTA"
            reach_time = wait_reach_time
        elif late_reached:
            status = "LATE_REACHED" if track_status in ("OK", "N/A") else "LATE_REACHED_POOR_DELTA"
            reach_time = late_reach_time
        else:
            status = track_status if track_status != "OK" else "TIMEOUT"
            reach_time = None

        # Compare MAVSDK lag against pipe feedback for diagnostics.
        samples = self.stage_samples.get(name, [])
        mav_progress_at_reach = None
        fb_progress_at_reach = None
        expected_len = horizontal_norm(expected_dn, expected_de)
        if expected_len >= MIN_H_MOVE_FOR_TRACK_EVAL and reach_time is not None:
            ux = expected_dn / expected_len
            uy = expected_de / expected_len
            closest = min(samples, key=lambda s: abs(s["t"] - reach_time)) if samples else None
            if closest:
                fb_progress_at_reach = (closest["fb_rel_n"] - fb_start["rel_n"]) * ux + \
                                       (closest["fb_rel_e"] - fb_start["rel_e"]) * uy
                mav_progress_at_reach = (closest["mav_rel_n"] - mav_start_rel[0]) * ux + \
                                        (closest["mav_rel_e"] - mav_start_rel[1]) * uy

        result = {
            "name": name,
            "dir": expected_label,
            "target_rel": (n_rel, e_rel, alt),
            "expected_move": (expected_dn, expected_de, expected_du),
            "fb_delta": (actual_dn, actual_de, actual_du),
            "mav_delta": (mav_actual_dn, mav_actual_de, mav_actual_du),
            "ratio": ratio,
            "cross": cross,
            "status": status,
            "speed_mps": speed_mps,
            "wait_reason": wait_reason,
            "late_reason": late_reason,
            "reach_time": reach_time,
            "fb_progress_at_reach": fb_progress_at_reach,
            "mav_progress_at_reach": mav_progress_at_reach,
        }
        self.results.append(result)

        log("[goto {}] [END] {} by {} - fb_delta dN={:+.3f} dE={:+.3f} dU={:+.3f} "
            "ratio={} cross={} | mav_delta dN={:+.3f} dE={:+.3f} dU={:+.3f} wait={} late={}".format(
                name, status, self.reach_pipe,
                actual_dn, actual_de, actual_du,
                "n/a" if ratio is None else "{:.2f}".format(ratio),
                "n/a" if cross is None else "{:.2f}m".format(cross),
                mav_actual_dn, mav_actual_de, mav_actual_du,
                wait_reason, late_reason))

        # If pipe feedback says failure, prevent target from being held far away indefinitely.
        if status not in ("OK", "LATE_REACHED") and name != "takeoff":
            self.hold_current_feedback_position("after_failed_{}".format(name))

        self.active_phase = "IDLE"
        self.active_stage = None
        return status in ("OK", "LATE_REACHED")


    def motion_delta_for_step(self, step, current_yaw_deg):
        """Return dN/dE for a relative move step using world/body mode."""
        direction = step.get("direction")
        distance = step.get("distance")
        frame = str(step.get("frame") or self.args.move_frame or MOVE_FRAME).lower()
        if direction is None or distance is None:
            return float(step.get("dn", 0.0)), float(step.get("de", 0.0)), frame
        dn, de = direction_to_ned(direction, float(distance), current_yaw_deg, frame)
        return dn, de, frame

    async def ramp_yaw_to(self, name, start_yaw, target_yaw, yaw_speed_deg_s):
        """Ramp the commanded yaw while holding the current position setpoint."""
        diff = angle_diff_deg(target_yaw, start_yaw)
        if yaw_speed_deg_s is None or yaw_speed_deg_s <= 0 or abs(diff) < 1e-6:
            self.current_target_yaw = yaw_to_360(target_yaw)
            self.target = PositionNedYaw(self.target.north_m, self.target.east_m, self.target.down_m, self.current_target_yaw)
            return 0.0

        duration = abs(diff) / max(1.0, float(yaw_speed_deg_s))
        log("       [yaw_ramp {}] yaw {:+.1f} -> {:+.1f} deg at {:.1f}deg/s -> {:.1f}s".format(
            name, start_yaw, yaw_to_360(target_yaw), yaw_speed_deg_s, duration))
        self.active_phase = "RAMP_YAW"
        start_time = asyncio.get_event_loop().time()
        last_print = -1.0
        period = 1.0 / max(5.0, SETPOINT_HZ)
        while True:
            if self.aborted:
                return asyncio.get_event_loop().time() - start_time
            now = asyncio.get_event_loop().time()
            elapsed = now - start_time
            frac = min(1.0, elapsed / duration)
            yaw = yaw_to_360(start_yaw + diff * frac)
            self.current_target_yaw = yaw
            self.target = PositionNedYaw(self.target.north_m, self.target.east_m, self.target.down_m, yaw)
            if elapsed - last_print >= 1.0 or frac >= 1.0:
                last_print = elapsed
                fb = self.feedback_state()
                fb_yaw = fb.get("yaw_deg", float("nan")) if fb else float("nan")
                log("       [yaw_ramp {}] t={:.1f}/{:.1f}s target_yaw={:+.1f} fb_yaw={}".format(
                    name, elapsed, duration, yaw, "n/a" if math.isnan(fb_yaw) else "{:+.1f}".format(fb_yaw)))
            if frac >= 1.0:
                break
            await asyncio.sleep(period)
        self.current_target_yaw = yaw_to_360(target_yaw)
        self.target = PositionNedYaw(self.target.north_m, self.target.east_m, self.target.down_m, self.current_target_yaw)
        return duration

    async def wait_until_yaw_reached(self, name, target_yaw, timeout):
        """Wait until feedback yaw is close to target yaw. If yaw feedback is invalid, return after a short hold."""
        log("       [wait_yaw] reaching {} target_yaw={:+.1f} tol={:.1f}deg timeout={:.1f}s".format(
            name, yaw_to_360(target_yaw), self.args.yaw_tol, timeout))
        start = asyncio.get_event_loop().time()
        in_tol_since = None
        last_print = -1.0
        valid_seen = False
        while True:
            if self.aborted:
                return False, "ABORTED", None
            now = asyncio.get_event_loop().time()
            elapsed = now - start
            fb = self.feedback_state()
            fb_yaw = fb.get("yaw_deg", float("nan")) if fb else float("nan")
            valid = fb is not None and not math.isnan(fb_yaw)
            err = abs(angle_diff_deg(target_yaw, fb_yaw)) if valid else float("nan")
            if elapsed - last_print >= 0.5:
                last_print = elapsed
                log("       [{} yaw] t={:.1f}s target={:+.1f} fb_yaw={} err={}".format(
                    name, elapsed, yaw_to_360(target_yaw),
                    "n/a" if not valid else "{:+.1f}".format(fb_yaw),
                    "n/a" if not valid else "{:.1f}deg".format(err)))
            if valid:
                valid_seen = True
                if err <= self.args.yaw_tol:
                    if in_tol_since is None:
                        in_tol_since = now
                    elif now - in_tol_since >= self.args.yaw_stable_time:
                        return True, "REACHED", elapsed
                else:
                    in_tol_since = None
            # If no yaw feedback exists, don't block forever; yaw setpoint has already been commanded.
            if (not valid_seen) and elapsed >= min(2.0, timeout):
                return True, "NO_YAW_FEEDBACK_ASSUMED", elapsed
            if elapsed > timeout:
                return False, "TIMEOUT", elapsed
            await asyncio.sleep(0.1)

    async def go_yaw(self, target_yaw, name, label, yaw_speed_deg_s=None, relative_delta=None):
        """Command yaw/turn while holding current position target."""
        if self.aborted:
            return False
        fb_start = self.feedback_state()
        mav_start_rel = self.mav_rel_position()
        start_yaw = self.current_target_yaw
        target_yaw = yaw_to_360(target_yaw)
        diff = angle_diff_deg(target_yaw, start_yaw)
        speed = 30.0 if yaw_speed_deg_s is None else float(yaw_speed_deg_s)
        timeout = max(abs(diff) / max(1.0, speed) + 5.0, 6.0)
        log("[yaw {}] [START] [{}] yaw {:+.1f} -> {:+.1f} diff={:+.1f}deg speed={:.1f}deg/s".format(
            name, label, start_yaw, target_yaw, diff, speed))

        # Hold the current planned target position during yaw. If target is uninitialized, hold feedback position.
        if fb_start is not None:
            self.target = PositionNedYaw(self.target.north_m, self.target.east_m, self.target.down_m, self.current_target_yaw)

        self.active_stage = {
            "name": name,
            "t_start": asyncio.get_event_loop().time(),
            "expected_label": label,
            "fb_start": dict(fb_start) if fb_start else None,
            "mav_start_rel": mav_start_rel,
            "target_rel": self.rel_target(),
            "expected_move": (0.0, 0.0, 0.0),
            "speed_mps": None,
            "yaw_target": target_yaw,
        }
        self.stage_samples[name] = []
        await self.ramp_yaw_to(name, start_yaw, target_yaw, speed)
        self.active_phase = "WAIT_YAW"
        ok, reason, reach_time = await self.wait_until_yaw_reached(name, target_yaw, timeout)
        fb_end = self.feedback_state()
        mav_end_rel = self.mav_rel_position()
        yaw_end = fb_end.get("yaw_deg", float("nan")) if fb_end else float("nan")
        yaw_error = None if math.isnan(yaw_end) else abs(angle_diff_deg(target_yaw, yaw_end))
        status = "OK" if ok else "YAW_TIMEOUT"
        self.results.append({
            "name": name,
            "dir": label,
            "target_rel": self.rel_target(),
            "expected_move": (0.0, 0.0, 0.0),
            "fb_delta": (0.0, 0.0, 0.0),
            "mav_delta": (mav_end_rel[0] - mav_start_rel[0], mav_end_rel[1] - mav_start_rel[1], mav_end_rel[2] - mav_start_rel[2]),
            "ratio": None,
            "cross": None,
            "status": status,
            "speed_mps": None,
            "wait_reason": reason,
            "late_reason": "NOT_NEEDED",
            "reach_time": reach_time,
            "fb_progress_at_reach": None,
            "mav_progress_at_reach": None,
            "yaw_start": start_yaw,
            "yaw_target": target_yaw,
            "yaw_end": yaw_end,
            "yaw_error": yaw_error,
            "yaw_speed_deg_s": speed,
        })
        log("[yaw {}] [END] {} target_yaw={:+.1f} fb_yaw={} err={} reason={}".format(
            name, status, target_yaw,
            "n/a" if math.isnan(yaw_end) else "{:+.1f}".format(yaw_end),
            "n/a" if yaw_error is None else "{:.1f}deg".format(yaw_error), reason))
        self.active_phase = "IDLE"
        self.active_stage = None
        return ok

    def make_position_snapshot(self, label):
        fb = self.feedback_state()
        mav_rel_n, mav_rel_e, mav_rel_u = self.mav_rel_position()
        snap = {
            "label": label,
            "wallclock": wallclock(),
            "unix_time": time.time(),
            "mav_abs_n": self.mav_x,
            "mav_abs_e": self.mav_y,
            "mav_abs_d": self.mav_z,
            "mav_rel_n": mav_rel_n,
            "mav_rel_e": mav_rel_e,
            "mav_rel_u": mav_rel_u,
        }
        if fb:
            snap.update({
                "fb_cmd_n": fb["cmd_n"],
                "fb_cmd_e": fb["cmd_e"],
                "fb_cmd_d": fb["cmd_d"],
                "fb_rel_n": fb["rel_n"],
                "fb_rel_e": fb["rel_e"],
                "fb_rel_u": fb["rel_u"],
                "fb_age_sec": fb["age_sec"],
                "fb_available": True,
            })
        else:
            snap.update({
                "fb_cmd_n": None,
                "fb_cmd_e": None,
                "fb_cmd_d": None,
                "fb_rel_n": None,
                "fb_rel_e": None,
                "fb_rel_u": None,
                "fb_age_sec": None,
                "fb_available": False,
            })
        return snap

    def log_position_snapshot(self, snap):
        if not snap:
            return
        if snap.get("fb_available"):
            log("       [{}] fb_rel=(N{:+.2f},E{:+.2f},U{:+.2f}) | mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                snap["label"], snap["fb_rel_n"], snap["fb_rel_e"], snap["fb_rel_u"],
                snap["mav_rel_n"], snap["mav_rel_e"], snap["mav_rel_u"]))
        else:
            log("       [{}] feedback unavailable | mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                snap["label"], snap["mav_rel_n"], snap["mav_rel_e"], snap["mav_rel_u"]))

    # ---------- mission / landing ----------

    async def execute_mission(self):
        self.takeoff_started = True
        alt = self.args.altitude
        plan = list(self.args.plan_steps)

        log("=" * 78)
        log("CUSTOM MISSION TEST - MAVSDK COMMAND + {} REACH".format(self.reach_pipe))
        log("Takeoff altitude: {:.2f}m | Mission steps: {}".format(alt, len(plan)))
        log("Speed is implemented by ramping the PositionNedYaw setpoint, not by MAVSDK velocity fields.")
        log("=" * 78)
        # Preview the plan while simulating target state so relative movements and yaw/body frame are clear.
        preview_n, preview_e, preview_alt, preview_yaw = 0.0, 0.0, alt, yaw_to_360(self.args.yaw)
        for i, step in enumerate(plan, 1):
            speed_str = "direct" if step["speed"] is None or step["speed"] <= 0 else "{:.2f}m/s".format(step["speed"])
            kind = step.get("kind")
            if kind == "hover":
                desc = "HOVER current target N{:+.2f} E{:+.2f} U{:+.2f}".format(preview_n, preview_e, preview_alt)
            elif kind == "alt_delta":
                next_alt = max(MIN_COMMAND_ALT, preview_alt + float(step["alt_delta"]))
                desc = "ALT_DELTA {:+.2f}m: U{:+.2f} -> U{:+.2f}".format(step["alt_delta"], preview_alt, next_alt)
                preview_alt = next_alt
            elif kind == "alt_abs":
                next_alt = max(MIN_COMMAND_ALT, float(step["alt_target"]))
                desc = "ALT_ABS: U{:+.2f} -> U{:+.2f}".format(preview_alt, next_alt)
                preview_alt = next_alt
            elif kind == "yaw_delta":
                next_yaw = yaw_to_360(preview_yaw + float(step["yaw_delta"]))
                desc = "TURN dYaw{:+.1f}deg: yaw {:+.1f} -> {:+.1f}".format(
                    step["yaw_delta"], preview_yaw, next_yaw)
                preview_yaw = next_yaw
            elif kind == "yaw_abs":
                next_yaw = yaw_to_360(float(step["yaw_target"]))
                desc = "YAW absolute: yaw {:+.1f} -> {:+.1f}".format(preview_yaw, next_yaw)
                preview_yaw = next_yaw
            elif kind == "move_rel":
                frame = str(step.get("frame") or self.args.move_frame or MOVE_FRAME).lower()
                dn, de = direction_to_ned(step.get("direction"), float(step.get("distance")), preview_yaw, frame)
                next_n = preview_n + dn
                next_e = preview_e + de
                next_alt = preview_alt if step.get("alt") is None else float(step["alt"])
                desc = "REL({}) {} {:.2f}m -> dN{:+.2f} dE{:+.2f} -> N{:+.2f} E{:+.2f} U{:+.2f} yaw{:+.1f}".format(
                    frame, step.get("direction"), step.get("distance"), dn, de, next_n, next_e, next_alt, preview_yaw)
                preview_n, preview_e, preview_alt = next_n, next_e, next_alt
            else:
                next_n = float(step["n_rel"])
                next_e = float(step["e_rel"])
                next_alt = preview_alt if step.get("alt") is None else float(step["alt"])
                desc = "ABS target N{:+.2f} E{:+.2f} U{:+.2f}".format(next_n, next_e, next_alt)
                preview_n, preview_e, preview_alt = next_n, next_e, next_alt
            log("  PLAN {:02d}: {:<18} label={:<14} {:<48} speed={} hover={:.1f}s".format(
                i, step["name"], step["label"], desc, speed_str, step["hover"]))
        log("=" * 78)

        log(">>> STAGE 1: TAKEOFF <<<")
        self.current_target_yaw = yaw_to_360(self.args.yaw)
        ok = await self.go_to_relative(
            0.0, 0.0, alt, "takeoff", "UP", speed_mps=self.args.takeoff_speed
        )
        await self.hover(self.args.takeoff_hover_time, "post_takeoff", hold_current=False)
        if self.aborted:
            return
        if self.args.stop_on_fail and not ok:
            log("!!! stop_on_fail: takeoff did not pass. Mission will stop before custom steps.")
            self.mission_end_before_land = self.make_position_snapshot("mission_end_before_land")
            self.log_position_snapshot(self.mission_end_before_land)
            return

        stage_no = 2
        used_names = set(["takeoff"])
        # Planned target state. Dynamic commands such as hover/up/down use this state.
        current_plan_n = 0.0
        current_plan_e = 0.0
        current_plan_alt = alt
        current_plan_yaw = yaw_to_360(self.args.yaw)
        self.current_target_yaw = current_plan_yaw

        for idx, step in enumerate(plan, 1):
            base_name = step["name"] or "step_{:02d}".format(idx)
            name = base_name
            if name in used_names:
                name = "{}_{}".format(base_name, idx)
            used_names.add(name)

            kind = step.get("kind", "goto")
            log(">>> STAGE {}: {} <<<".format(stage_no, name))

            if kind == "hover":
                # Keep the current target. This is a true standalone hover/wait command.
                log("       [command hover] keep current target N{:+.2f} E{:+.2f} U{:+.2f} yaw{:+.1f} for {:.1f}s".format(
                    current_plan_n, current_plan_e, current_plan_alt, current_plan_yaw, step["hover"]))
                await self.hover(step["hover"], name, hold_current=False)
                ok = True

            elif kind == "alt_delta":
                target_alt = max(MIN_COMMAND_ALT, current_plan_alt + float(step["alt_delta"]))
                log("       [command {}] current U{:.2f} -> target U{:.2f}".format(
                    step["label"], current_plan_alt, target_alt))
                self.current_target_yaw = current_plan_yaw
                ok = await self.go_to_relative(
                    current_plan_n, current_plan_e, target_alt,
                    name, step["label"], speed_mps=step["speed"]
                )
                await self.hover(step["hover"], "after_{}".format(name), hold_current=False)
                if ok:
                    current_plan_alt = target_alt

            elif kind == "alt_abs":
                target_alt = max(MIN_COMMAND_ALT, float(step["alt_target"]))
                log("       [command ALT] current U{:.2f} -> target U{:.2f}".format(
                    current_plan_alt, target_alt))
                self.current_target_yaw = current_plan_yaw
                ok = await self.go_to_relative(
                    current_plan_n, current_plan_e, target_alt,
                    name, step["label"], speed_mps=step["speed"]
                )
                await self.hover(step["hover"], "after_{}".format(name), hold_current=False)
                if ok:
                    current_plan_alt = target_alt

            elif kind == "yaw_delta":
                target_yaw = yaw_to_360(current_plan_yaw + float(step["yaw_delta"]))
                self.current_target_yaw = current_plan_yaw
                log("       [command TURN] yaw {:+.1f} -> {:+.1f} dYaw={:+.1f}".format(
                    current_plan_yaw, target_yaw, float(step["yaw_delta"])))
                ok = await self.go_yaw(
                    target_yaw, name, step["label"],
                    yaw_speed_deg_s=step.get("yaw_speed"),
                    relative_delta=step.get("yaw_delta"),
                )
                await self.hover(step["hover"], "after_{}".format(name), hold_current=False)
                if ok:
                    current_plan_yaw = target_yaw

            elif kind == "yaw_abs":
                target_yaw = yaw_to_360(float(step["yaw_target"]))
                self.current_target_yaw = current_plan_yaw
                log("       [command YAW] yaw {:+.1f} -> {:+.1f}".format(current_plan_yaw, target_yaw))
                ok = await self.go_yaw(
                    target_yaw, name, step["label"],
                    yaw_speed_deg_s=step.get("yaw_speed"),
                )
                await self.hover(step["hover"], "after_{}".format(name), hold_current=False)
                if ok:
                    current_plan_yaw = target_yaw

            elif kind == "move_rel":
                # Relative movement from the current planned N/E position.
                # If MOVE_FRAME/body is used, direction is converted using current_plan_yaw.
                # If altitude is omitted, keep the current planned altitude.
                dn, de, frame = self.motion_delta_for_step(step, current_plan_yaw)
                target_n = current_plan_n + dn
                target_e = current_plan_e + de
                target_alt = current_plan_alt if step.get("alt") is None else float(step["alt"])
                self.current_target_yaw = current_plan_yaw
                log("       [command {}] REL({}) {} {:.2f}m yaw{:+.1f} -> dN{:+.2f} dE{:+.2f} -> target N{:+.2f} E{:+.2f} U{:+.2f}".format(
                    step["label"], frame, step.get("direction"), float(step.get("distance")), current_plan_yaw,
                    dn, de, target_n, target_e, target_alt))
                ok = await self.go_to_relative(
                    target_n, target_e, target_alt,
                    name, step["label"], speed_mps=step["speed"]
                )
                await self.hover(step["hover"], "after_{}".format(name), hold_current=False)
                if ok:
                    current_plan_n = target_n
                    current_plan_e = target_e
                    current_plan_alt = target_alt

            else:
                # Absolute goto/home commands. Altitude is optional; omitted means keep current planned altitude.
                target_n = float(step["n_rel"])
                target_e = float(step["e_rel"])
                target_alt = current_plan_alt if step.get("alt") is None else float(step["alt"])
                log("       [command {}] ABS target N{:+.2f} E{:+.2f} U{:+.2f}".format(
                    step["label"], target_n, target_e, target_alt))
                self.current_target_yaw = current_plan_yaw
                ok = await self.go_to_relative(
                    target_n, target_e, target_alt,
                    name, step["label"], speed_mps=step["speed"]
                )
                await self.hover(step["hover"], "after_{}".format(name), hold_current=False)
                if ok:
                    current_plan_n = target_n
                    current_plan_e = target_e
                    current_plan_alt = target_alt

            if self.aborted:
                return
            if self.args.stop_on_fail and not ok:
                log("!!! stop_on_fail: {} did not pass. Mission will stop before next step.".format(name))
                break
            stage_no += 1

        # Mission result at normal mission altitude, before the precision pre-landing descent.
        self.mission_end_before_preland = self.make_position_snapshot("mission_end_before_preland")
        log("=" * 78)
        log("CUSTOM MISSION COMPLETE - position before preland descent")
        self.log_position_snapshot(self.mission_end_before_preland)
        log("=" * 78)

        # Precision landing preparation: descend to a low altitude first, then use action.land().
        if (not self.args.skip_preland) and self.args.preland_alt > 0.0 and self.args.preland_alt < alt:
            preland_alt = self.args.preland_alt
            log(">>> STAGE {}: PRELAND DESCENT (U{:.2f}m) <<<".format(stage_no, preland_alt))
            ok = await self.go_to_relative(
                0.0, 0.0, preland_alt, "preland_descend", "PRELAND_DESCEND",
                speed_mps=self.args.preland_speed,
            )
            await self.hover(self.args.preland_hover_time, "preland_stabilize", hold_current=False)
            if self.aborted:
                return
            if self.args.stop_on_fail and not ok:
                log("!!! stop_on_fail: preland descent did not pass. Holding current position before landing.")
                self.hold_current_feedback_position("preland_failed")
            self.preland_end_before_land = self.make_position_snapshot("preland_end_before_land")
            self.mission_end_before_land = self.preland_end_before_land
            log("=" * 78)
            log("POSITION BEFORE LAND COMMAND - after preland descent")
            self.log_position_snapshot(self.mission_end_before_land)
            log("=" * 78)
        else:
            if self.args.skip_preland:
                log("       [preland] skipped by --skip-preland")
            elif self.args.preland_alt >= alt:
                log("       [preland] skipped: --preland-alt {:.2f}m is not lower than flight altitude {:.2f}m".format(
                    self.args.preland_alt, alt))
            self.preland_end_before_land = None
            self.mission_end_before_land = self.mission_end_before_preland
            log("=" * 78)
            log("POSITION BEFORE LAND COMMAND - no preland descent")
            self.log_position_snapshot(self.mission_end_before_land)
            log("=" * 78)

    async def land_and_disarm(self):
        log("[7/7] Landing...")
        if not self.aborted:
            self.streaming = False
            await asyncio.sleep(0.2)
            try:
                await self.drone.action.land()
                log("       Land command sent.")
            except Exception as e:
                log("       Land command warning: {}".format(e))

        start = asyncio.get_event_loop().time()
        async for in_air in self.drone.telemetry.in_air():
            elapsed = asyncio.get_event_loop().time() - start
            fb = self.feedback_state()
            mav_rel_n, mav_rel_e, mav_rel_u = self.mav_rel_position()
            if fb:
                log("       [land] t={:.1f}s in_air={} fb_rel=(N{:+.2f},E{:+.2f},U{:+.2f}) | mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                    elapsed, in_air,
                    fb["rel_n"], fb["rel_e"], fb["rel_u"],
                    mav_rel_n, mav_rel_e, mav_rel_u))
            else:
                log("       [land] t={:.1f}s in_air={} feedback not ready | mav_rel=(N{:+.2f},E{:+.2f},U{:+.2f})".format(
                    elapsed, in_air, mav_rel_n, mav_rel_e, mav_rel_u))
            if not in_air:
                log("       *** LANDED ***")
                break
            if elapsed > LAND_WAIT_TIMEOUT:
                log("       [land] timeout, breaking")
                break
            await asyncio.sleep(0.5)

        self.land_end_snapshot = self.make_position_snapshot("land_end")
        await self._cancel_background_tasks()
        self._write_csv_if_needed()
        self._print_summary()

        log("[disarm]")
        try:
            await self.drone.action.disarm()
            log("       Disarmed.")
        except Exception as e:
            log("       Disarm note: {}".format(e))
        log("Done!")

    # ---------- emergency / cleanup ----------

    async def trigger_emergency_land(self, reason="unknown"):
        async with self._emergency_lock:
            if self._emergency_triggered:
                return
            self._emergency_triggered = True
        log("!!! EMERGENCY LAND TRIGGERED: {} !!!".format(reason))
        self.streaming = False
        self.aborted = True
        await asyncio.sleep(0.2)
        try:
            await self.drone.action.land()
            log("       Land command sent.")
        except Exception as e:
            log("       Emergency land failed: {}, trying kill".format(e))
            try:
                await self.drone.action.kill()
                log("       Kill command sent.")
            except Exception as ke:
                log("       Kill also failed: {}".format(ke))

    async def emergency_stop(self):
        log("!!! EMERGENCY STOP !!!")
        await self.trigger_emergency_land("emergency_stop called")
        await asyncio.sleep(0.5)
        try:
            await self.drone.action.kill()
        except Exception:
            pass
        await self._cancel_background_tasks()

    async def _cancel_background_tasks(self):
        self.streaming = False
        self.monitoring = False
        tasks = [self.stream_task, self.mav_task, self.health_task, self.sync_csv_task] + list(self.voxl_tasks)
        for task in tasks:
            if task and not task.done():
                task.cancel()
                try:
                    await asyncio.wait_for(task, timeout=2.0)
                except (asyncio.CancelledError, asyncio.TimeoutError):
                    pass
                except Exception:
                    pass

    # ---------- CSV / summary ----------

    def _write_csv_if_needed(self):
        if not self.args.csv:
            return
        path = self.args.csv
        if path == "auto":
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = "/home/root/px4_reach_sync_custom_v13_{}.csv".format(stamp)
        try:
            if not self.sync_samples:
                self._append_sync_csv_sample()
            if not self.sync_samples:
                log("[csv] no synchronized samples to write")
                return
            fieldnames = sorted(set().union(*(row.keys() for row in self.sync_samples)))
            with open(path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for row in self.sync_samples:
                    writer.writerow(row)
            log("[csv] wrote synchronized MAVSDK + VOXL/PX4 samples: {}".format(path))
        except Exception as e:
            log("[csv] failed to write {}: {}".format(path, e))

        if self.voxl_reader_errors:
            for pipe, err in self.voxl_reader_errors.items():
                log("[voxl] last reader note for {}: {}".format(pipe, err))

    def _print_summary(self):
        log("")
        log("=" * 150)
        log("POST-FLIGHT SUMMARY - CUSTOM MISSION PIPE-BASED REACH JUDGEMENT")
        log("=" * 150)
        log("Command frame initial MAVSDK NED: N={:+.3f} E={:+.3f} D={:+.3f}".format(
            self.initial_x, self.initial_y, self.initial_z))

        def fmt_snap(prefix, snap):
            if not snap:
                log("{}: n/a".format(prefix))
                return
            if snap.get("fb_available"):
                log("{} by {} aligned: rel=(N{:+.3f},E{:+.3f},U{:+.3f}) | MAVSDK rel=(N{:+.3f},E{:+.3f},U{:+.3f})".format(
                    prefix, self.reach_pipe,
                    snap["fb_rel_n"], snap["fb_rel_e"], snap["fb_rel_u"],
                    snap["mav_rel_n"], snap["mav_rel_e"], snap["mav_rel_u"]))
            else:
                log("{}: feedback unavailable | MAVSDK rel=(N{:+.3f},E{:+.3f},U{:+.3f})".format(
                    prefix, snap["mav_rel_n"], snap["mav_rel_e"], snap["mav_rel_u"]))

        fmt_snap("Mission end before preland descent", self.mission_end_before_preland)
        fmt_snap("Position before land command", self.mission_end_before_land)
        fmt_snap("After landing", self.land_end_snapshot)

        if self.mission_end_before_land and self.land_end_snapshot:
            a = self.mission_end_before_land
            b = self.land_end_snapshot
            if a.get("fb_available") and b.get("fb_available"):
                dn = b["fb_rel_n"] - a["fb_rel_n"]
                de = b["fb_rel_e"] - a["fb_rel_e"]
                du = b["fb_rel_u"] - a["fb_rel_u"]
                log("Landing drift by {}: dN={:+.3f} dE={:+.3f} dU={:+.3f} horizontal={:.3f}m".format(
                    self.reach_pipe, dn, de, du, horizontal_norm(dn, de)))
            mdn = b["mav_rel_n"] - a["mav_rel_n"]
            mde = b["mav_rel_e"] - a["mav_rel_e"]
            mdu = b["mav_rel_u"] - a["mav_rel_u"]
            log("Landing drift by MAVSDK telemetry: dN={:+.3f} dE={:+.3f} dU={:+.3f} horizontal={:.3f}m".format(
                mdn, mde, mdu, horizontal_norm(mdn, mde)))

        log("")
        log("{:<22} {:<18} {:<18} {:<18} {:<18} {:<8} {:<7} {:<8} {:<22}".format(
            "Stage", "Dir", "ExpectedMove", "FB_Delta", "MAV_Delta", "Speed", "Ratio", "Cross", "Status"))
        log("-" * 150)
        for r in self.results:
            if "yaw_target" in r:
                exp = "yaw->{:+.0f}".format(r["yaw_target"])
                fb_delta = "fb_yaw={}".format("n/a" if r.get("yaw_end") is None or math.isnan(r.get("yaw_end", float("nan"))) else "{:+.0f}".format(r["yaw_end"]))
                mav_delta = "dN{:+.2f},dE{:+.2f}".format(r["mav_delta"][0], r["mav_delta"][1])
            else:
                exp = "dN{:+.2f},dE{:+.2f}".format(r["expected_move"][0], r["expected_move"][1])
                fb_delta = "dN{:+.2f},dE{:+.2f}".format(r["fb_delta"][0], r["fb_delta"][1])
                mav_delta = "dN{:+.2f},dE{:+.2f}".format(r["mav_delta"][0], r["mav_delta"][1])
            ratio = "n/a" if r["ratio"] is None else "{:.2f}".format(r["ratio"])
            cross = "n/a" if r["cross"] is None else "{:.2f}m".format(r["cross"])
            speed = "direct" if r.get("speed_mps") is None or r.get("speed_mps") <= 0 else "{:.2f}".format(r.get("speed_mps"))
            log("{:<22} {:<18} {:<18} {:<18} {:<18} {:<8} {:<7} {:<8} {:<22}".format(
                r["name"], r["dir"], exp, fb_delta, mav_delta, speed, ratio, cross, r["status"]))
        log("-" * 150)
        log("Reach judgement source: {}".format(self.reach_pipe))
        log("MAVSDK telemetry is logged only for comparison. It is NOT used for reached/not-reached decisions.")
        log("Precision settings: h_tol={:.2f}m, v_tol={:.2f}m, stable_time={:.1f}s".format(
            self.args.h_tol, self.args.v_tol, self.args.stable_time))
        log("Preland: {}".format(
            "skipped" if self.args.skip_preland else "target U={:.2f}m before land".format(self.args.preland_alt)))
        log("Mission end before preland, position before land command, and after-landing final position are separated.")
        log("If FB_Delta is correct but MAV_Delta is delayed/small, the old MAVSDK-based judgement was the issue.")
        log("=" * 150)


async def main_async(args):
    ctrl = DroneController(args)

    def sigint_handler():
        log("SIGINT received")
        asyncio.ensure_future(ctrl.emergency_stop())

    loop = asyncio.get_event_loop()
    try:
        loop.add_signal_handler(signal.SIGINT, sigint_handler)
    except NotImplementedError:
        pass

    try:
        await ctrl.connect()
        await ctrl.wait_position_stable()
        await ctrl.check_vio_quality_preflight()
        await ctrl.capture_initial_mavsdk_position()
        await ctrl.arm_and_start_offboard()
        await ctrl.execute_mission()
        await ctrl.land_and_disarm()
    except Exception as e:
        log("Exception: {}".format(e))
        try:
            ctrl._write_csv_if_needed()
            ctrl._print_summary()
        except Exception:
            pass
        await ctrl.emergency_stop()
        sys.exit(1)


def build_arg_parser():
    p = argparse.ArgumentParser(
        description="Starling2 custom mission plan using MAVSDK commands and px4_vehicle_local_position reach judgement"
    )
    p.add_argument("--directions", nargs="+", default=["forward", "right", "left", "backward"],
                   choices=["forward", "backward", "right", "left"],
                   help="Horizontal directions to test in order. Default: forward right left backward")
    p.add_argument("--distance", type=float, default=DEFAULT_MOVE_DISTANCE)
    p.add_argument("--altitude", type=float, default=DEFAULT_TAKEOFF_ALT)
    p.add_argument("--hover-time", type=float, default=DEFAULT_HOVER_TIME,
                   help="Fallback hover time when --ignore-mission-plan is used.")
    p.add_argument("--default-speed", type=float, default=DEFAULT_STEP_SPEED,
                   help="Fallback speed for CLI-generated direction plan. <=0 means direct final setpoint.")
    p.add_argument("--takeoff-speed", type=float, default=0.25,
                   help="Setpoint ramp speed for takeoff/pre-mission climb. <=0 means direct final setpoint.")
    p.add_argument("--takeoff-hover-time", type=float, default=DEFAULT_HOVER_TIME,
                   help="Hover/stabilize time after takeoff before the mission plan starts.")
    p.add_argument("--ignore-mission-plan", action="store_true",
                   help="Ignore the top-of-file MISSION_PLAN and use --directions/--distance instead.")
    p.add_argument("--h-tol", type=float, default=DEFAULT_H_TOL,
                   help="Horizontal reach tolerance in meters. Default is stricter for precision movement.")
    p.add_argument("--v-tol", type=float, default=DEFAULT_V_TOL,
                   help="Vertical reach tolerance in meters. Default is stricter than v7.")
    p.add_argument("--stable-time", type=float, default=DEFAULT_STABLE_TIME,
                   help="How long the feedback position must remain within tolerance before REACHED.")
    p.add_argument("--preland-alt", type=float, default=DEFAULT_PRELAND_ALT,
                   help="Before action.land(), descend to this altitude above start/home using offboard position control.")
    p.add_argument("--preland-hover-time", type=float, default=DEFAULT_PRELAND_HOVER_TIME,
                   help="Hover/stabilize time after preland descent before sending land().")
    p.add_argument("--preland-speed", type=float, default=0.20,
                   help="Setpoint ramp speed for preland descent. <=0 means direct final setpoint.")
    p.add_argument("--skip-preland", action="store_true",
                   help="Skip low-altitude preland descent and call action.land() directly.")
    p.add_argument("--late-observe", type=float, default=DEFAULT_LATE_OBSERVE)
    p.add_argument("--yaw", type=float, default=0.0, help="Initial/starting commanded yaw in degrees. 0=N, 90=E.")
    p.add_argument("--move-frame", choices=["world", "body"], default=MOVE_FRAME,
                   help="Frame for forward/backward/right/left. world=NED axes, body=current planned yaw heading.")
    p.add_argument("--yaw-tol", type=float, default=DEFAULT_YAW_TOL,
                   help="Yaw tolerance in degrees for yaw/turn reach judgement.")
    p.add_argument("--yaw-stable-time", type=float, default=DEFAULT_YAW_STABLE_TIME,
                   help="How long yaw must stay within tolerance before yaw/turn is OK.")
    p.add_argument("--server-address", default="localhost")
    p.add_argument("--port", type=int, default=50051)
    p.add_argument("--max-horizontal-dev", type=float, default=MAX_HORIZONTAL_DEV,
                   help="Reserved safety parameter. Main judgement uses --reach-pipe.")
    p.add_argument("--reach-pipe", default="px4_vehicle_local_position",
                   help="Pose pipe used for reached/not-reached judgement. Recommended: px4_vehicle_local_position")
    p.add_argument("--reach-pipe-timeout", type=float, default=8.0)
    p.add_argument("--csv", default="", help="Write synchronized CSV. Use 'auto' or a path. Default: disabled")
    p.add_argument("--csv-sample-sec", type=float, default=DEFAULT_CSV_SAMPLE_SEC)
    p.add_argument("--voxl-pipes", nargs="*", default=["px4_vehicle_local_position", "vvhub_body_wrt_local"],
                   help="Extra pose pipes to log with voxl-inspect-pose -n. reach-pipe is always added automatically.")
    p.add_argument("--no-return-between", action="store_true",
                   help="Do not return to home between directions. Default is to return home after every direction.")
    p.add_argument("--stop-on-fail", action="store_true",
                   help="Stop the mission after the first failed stage. Default is to continue while holding current feedback position after failures.")
    return p


def print_banner(args):
    print("=" * 78)
    print("Starling 2 - CUSTOM MISSION TEST v13")
    print("  Command      : MAVSDK Offboard PositionNedYaw")
    print("  Reach judge  : voxl-inspect-pose -n {}".format(args.reach_pipe))
    print("  Mission plan : {}".format(
        "top-of-file MISSION_PLAN" if USE_TOP_MISSION_PLAN and MISSION_PLAN and not args.ignore_mission_plan else "CLI --directions/--distance"))
    print("  Steps        : {}".format(len(args.plan_steps)))
    print("  Altitude     : default {:.2f}m".format(args.altitude))
    print("  Move frame   : {} (forward/back/right/left)".format(args.move_frame))
    print("  Initial yaw  : {:.1f} deg".format(args.yaw))
    print("  Precision    : h_tol={:.2f}m, v_tol={:.2f}m, stable_time={:.1f}s".format(
        args.h_tol, args.v_tol, args.stable_time))
    print("  Takeoff      : speed={}, hover {:.1f}s".format(
        "direct" if args.takeoff_speed <= 0 else "{:.2f}m/s".format(args.takeoff_speed),
        args.takeoff_hover_time))
    print("  Preland      : {}".format(
        "SKIP" if args.skip_preland else "descend to U={:.2f}m at {}, hover {:.1f}s, then land".format(
            args.preland_alt,
            "direct" if args.preland_speed <= 0 else "{:.2f}m/s".format(args.preland_speed),
            args.preland_hover_time)))
    print("=" * 78)
    print("MISSION_PLAN preview:")
    max_range = 0.0
    preview_n, preview_e, preview_alt, preview_yaw = 0.0, 0.0, args.altitude, yaw_to_360(args.yaw)
    for i, step in enumerate(args.plan_steps, 1):
        speed_str = "direct" if step["speed"] is None or step["speed"] <= 0 else "{:.2f}m/s".format(step["speed"])
        kind = step.get("kind")
        if kind == "hover":
            desc = "HOVER current target N{:+.2f} E{:+.2f} U{:+.2f} yaw{:+.1f}".format(
                preview_n, preview_e, preview_alt, preview_yaw)
        elif kind == "alt_delta":
            next_alt = max(MIN_COMMAND_ALT, preview_alt + float(step["alt_delta"]))
            desc = "ALT_DELTA {:+.2f}m: U{:+.2f} -> U{:+.2f}".format(step["alt_delta"], preview_alt, next_alt)
            preview_alt = next_alt
        elif kind == "alt_abs":
            next_alt = max(MIN_COMMAND_ALT, float(step["alt_target"]))
            desc = "ALT_ABS: U{:+.2f} -> U{:+.2f}".format(preview_alt, next_alt)
            preview_alt = next_alt
        elif kind == "yaw_delta":
            next_yaw = yaw_to_360(preview_yaw + float(step["yaw_delta"]))
            desc = "TURN dYaw{:+.1f}deg: yaw {:+.1f} -> {:+.1f}".format(
                step["yaw_delta"], preview_yaw, next_yaw)
            preview_yaw = next_yaw
        elif kind == "yaw_abs":
            next_yaw = yaw_to_360(float(step["yaw_target"]))
            desc = "YAW absolute: yaw {:+.1f} -> {:+.1f}".format(preview_yaw, next_yaw)
            preview_yaw = next_yaw
        elif kind == "move_rel":
            frame = str(step.get("frame") or args.move_frame or MOVE_FRAME).lower()
            dn, de = direction_to_ned(step.get("direction"), float(step.get("distance")), preview_yaw, frame)
            next_n = preview_n + dn
            next_e = preview_e + de
            next_alt = preview_alt if step.get("alt") is None else float(step["alt"])
            desc = "REL({}) {} {:.2f}m -> dN{:+.2f} dE{:+.2f} -> N{:+.2f} E{:+.2f} U{:+.2f} yaw{:+.1f}".format(
                frame, step.get("direction"), float(step.get("distance")), dn, de, next_n, next_e, next_alt, preview_yaw)
            preview_n, preview_e, preview_alt = next_n, next_e, next_alt
            max_range = max(max_range, horizontal_norm(preview_n, preview_e))
        else:
            next_n = float(step["n_rel"])
            next_e = float(step["e_rel"])
            next_alt = preview_alt if step.get("alt") is None else float(step["alt"])
            desc = "ABS target N{:+.2f} E{:+.2f} U{:+.2f} yaw{:+.1f}".format(next_n, next_e, next_alt, preview_yaw)
            preview_n, preview_e, preview_alt = next_n, next_e, next_alt
            max_range = max(max_range, horizontal_norm(preview_n, preview_e))
        print("  {:02d}. {:<18} {:<12} {:<48} speed={} hover={:.1f}s".format(
            i, step["name"], step["label"], desc, speed_str, step["hover"]))
    print("=" * 78)
    print("CHECKLIST:")
    print("  [ ] Propellers attached and tight")
    print("  [ ] Clear space for the configured mission, at least {:.1f}m from start/home".format(max(1.5, max_range + 0.8)))
    print("  [ ] RC ON, OFFBOARD mode, in HAND")
    print("  [ ] Kill switch confirmed")
    print("  [ ] Battery sufficient")
    print("  [ ] Drone on FLAT GROUND")
    print("  [ ] Drone facing intended forward direction if using world frame N+ as forward")
    print("  [ ] Floor has texture, not pure white/single-color")
    print()
    print("NOTE:")
    print("  PositionNedYaw uses LOCAL NED, not body frame.")
    print("  Speed is approximated by ramping the position setpoint, not by a MAVSDK velocity field.")
    print("  MAVSDK position_velocity_ned() is NOT used for reach judgement in this v13 code.")
    print("  Reach judgement source: {}".format(args.reach_pipe))
    print("  CSV sample interval: {:.2f}s".format(args.csv_sample_sec) if args.csv else "  CSV: disabled")
    print("  Logged VOXL/PX4 pose pipes: {}".format(", ".join(args.voxl_pipes)))
    print()

def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    if args.distance <= 0:
        raise ValueError("--distance must be positive")
    if args.altitude <= 0:
        raise ValueError("--altitude must be positive")
    if args.h_tol <= 0 or args.v_tol <= 0:
        raise ValueError("--h-tol and --v-tol must be positive")
    if args.stable_time < 0:
        raise ValueError("--stable-time must be >= 0")
    if args.preland_alt < 0:
        raise ValueError("--preland-alt must be >= 0")
    if args.preland_hover_time < 0:
        raise ValueError("--preland-hover-time must be >= 0")
    if args.takeoff_hover_time < 0:
        raise ValueError("--takeoff-hover-time must be >= 0")
    args.plan_steps = build_mission_plan(args)
    if not args.plan_steps:
        raise ValueError("Mission plan is empty")
    if args.reach_pipe not in args.voxl_pipes:
        args.voxl_pipes = [args.reach_pipe] + list(args.voxl_pipes)

    print_banner(args)
    input("Type Enter when confirmed...")
    asyncio.get_event_loop().run_until_complete(main_async(args))


if __name__ == "__main__":
    main()
