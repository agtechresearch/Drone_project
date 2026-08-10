#!/usr/bin/env python3
"""
순수동작 웨이포인트 주행 — move_base 없이 '제자리 회전 → 직진 → (1회 보정)'.

메카넘 저속 mix(전진+회전 동시)가 모터 데드밴드에 걸리는 문제를 우회한다.
로봇이 확실히 하는 동작(순수 회전 / 순수 직진)만 사용.

동작(무한루프 없음 — 런어웨이 방지):
  1) 시작 시 AMCL(map) 위치 1회 읽기
  2) 목표 방향으로 제자리 회전 (odom 기반, 정확)
  3) 목표 거리만큼 직진 (odom 기반)
  4) AMCL로 도착 확인 → 오차 크면 1회만 보정
  5) 최종 방향으로 회전

사용법:
  go_simple.py <웨이포인트명>
  go_simple.py <x> <y> [yaw_deg]

전제: 네비게이션(AMCL) 실행 + initialpose 설정. ※ 장애물 회피 없음(직선).
안전: 목표가 8m 초과면 거부. Ctrl+C 시 즉시 정지.
"""
import sys
import os
import math
import time
import signal
import yaml
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import move_direct as md

WAYPOINTS_FILE = "/home/er/autonomous_nav/config/waypoints.yaml"
LOG_PATH = "/tmp/go_simple.log"
XY_TOL   = 0.20    # m
YAW_TOL  = 10.0    # deg
MAX_DIST = 8.0     # m  안전: 목표까지 이보다 멀면 거부(런어웨이/오국소화 방지)

pub = None
_amcl = {"x": None, "y": None, "yaw": None}
_logf = open(LOG_PATH, "w")

def log(s):
    print(s)
    _logf.write(s + "\n"); _logf.flush()

def emergency_stop():
    if pub is not None:
        for _ in range(8):
            pub.publish(Twist()); time.sleep(0.03)

def _sigint(sig, frame):
    log("\n[중단] Ctrl+C — 로봇 정지 후 종료")
    emergency_stop()
    os._exit(0)

def _amcl_cb(msg):
    p = msg.pose.pose.position
    _amcl["x"], _amcl["y"] = p.x, p.y
    _amcl["yaw"] = md._yaw_from_quat(msg.pose.pose.orientation)

def norm_deg(a):
    while a > 180.0:  a -= 360.0
    while a < -180.0: a += 360.0
    return a

def wait_amcl(timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if _amcl["x"] is not None:
            return True
        time.sleep(0.1)
    return False

def load_wp(name):
    try:
        with open(WAYPOINTS_FILE) as f:
            data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        return None
    for w in data.get("waypoints", []):
        if w["name"] == name:
            return float(w["x"]), float(w["y"]), float(w.get("yaw", 0.0))
    return None

def face_and_go(tx, ty):
    """현재 AMCL 기준으로 목표를 향해 회전 후 직진 (1회)."""
    cx, cy, cyaw = _amcl["x"], _amcl["y"], _amcl["yaw"]
    dist = math.hypot(tx - cx, ty - cy)
    if dist > MAX_DIST:
        log(f"[거부] 목표까지 {dist:.2f}m > 안전한계 {MAX_DIST}m. 위치추정 확인 요망."); return None
    bearing = math.degrees(math.atan2(ty - cy, tx - cx))
    dtheta = norm_deg(bearing - math.degrees(cyaw))
    log(f"현재 ({cx:.2f},{cy:.2f}) yaw={math.degrees(cyaw):.0f}° → 회전 {dtheta:.0f}°, 직진 {dist:.2f}m")
    if abs(dtheta) > YAW_TOL:
        md.move_rotate(pub, dtheta, md.ANGULAR_SPEED)
        time.sleep(0.6)
    if dist > 0.03:
        md.move_linear(pub, dist, 0.0, md.LINEAR_SPEED, f"직진 {dist:.2f}m")
    time.sleep(0.8)
    return dist

def main():
    global pub
    args = sys.argv[1:]
    if not args:
        print(__doc__); sys.exit(1)
    try:
        tx = float(args[0]); ty = float(args[1])
        tyaw = float(args[2]) if len(args) > 2 else 0.0
    except (ValueError, IndexError):
        wp = load_wp(args[0])
        if wp is None:
            print(f"[오류] 웨이포인트 '{args[0]}' 없음 (또는 'x y yaw')"); sys.exit(1)
        tx, ty, tyaw = wp

    rospy.init_node("go_simple", anonymous=True, disable_signals=True)
    signal.signal(signal.SIGINT, _sigint)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("/odom", Odometry, md._odom_cb)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, _amcl_cb)
    time.sleep(0.6)

    if not md._wait_odom(5.0):
        log("[오류] /odom 없음"); emergency_stop(); sys.exit(1)
    if not wait_amcl(5.0):
        log("[오류] /amcl_pose 없음 — 네비 + set_initialpose 필요"); emergency_stop(); sys.exit(1)

    log(f"목표: ({tx:.2f},{ty:.2f}) yaw={tyaw:.0f}°  (장애물 회피 없음)")

    # 1차 이동
    d = face_and_go(tx, ty)
    if d is None:
        emergency_stop(); sys.exit(1)

    # 도착 확인 + 1회만 보정
    err = math.hypot(tx - _amcl["x"], ty - _amcl["y"])
    log(f"도착 ({_amcl['x']:.2f},{_amcl['y']:.2f}), 오차 {err:.2f}m")
    if err > XY_TOL:
        log("→ 1회 보정")
        if face_and_go(tx, ty) is None:
            emergency_stop(); sys.exit(1)
        err = math.hypot(tx - _amcl["x"], ty - _amcl["y"])
        log(f"보정 후 오차 {err:.2f}m")

    # 최종 방향
    dfin = norm_deg(tyaw - math.degrees(_amcl["yaw"]))
    if abs(dfin) > YAW_TOL:
        log(f"최종 방향 회전 {dfin:.0f}°")
        md.move_rotate(pub, dfin, md.ANGULAR_SPEED)

    emergency_stop()
    log(f"=== 완료. 최종 ({_amcl['x']:.2f},{_amcl['y']:.2f}) 오차 {err:.2f}m ===")

if __name__ == "__main__":
    main()
