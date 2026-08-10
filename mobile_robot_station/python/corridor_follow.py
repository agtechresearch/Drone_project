#!/usr/bin/env python3
"""
라이다 기반 통로 중앙 유지 주행 — 헤딩 드리프트 보정 포함.

제어 구조:
  linear.y  = Kp*error + Kd*d_error     (P+D: 현재 위치 오차 즉시 보정)
  angular.z = Ki*integral(error)         (I:   누적 오차 → 헤딩 보정)

턱/장애물로 헤딩이 틀어지면 오차가 지속 누적 → angular.z가 헤딩 복구.
일시적 충격은 적분이 쌓이지 않아 헤딩 보정 미발동.

사용법:
  corridor_follow.py                        # 전진 + 중앙 유지
  corridor_follow.py --scan                 # 거리값만 출력 (센서 방향 확인)
  corridor_follow.py --center               # 제자리 중앙 정렬만
  corridor_follow.py --speed 0.12           # 전진 속도 지정
  corridor_follow.py --stop-dist 0.8        # 전방 정지 거리 지정 (기본 0.5m)
"""

import sys
import math
import collections
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# ── 속도 설정 ─────────────────────────────────────────────────────
FORWARD_SPEED    = 0.12   # m/s
MAX_LATERAL      = 0.15   # m/s  linear.y 최대값
MAX_YAW          = 0.25   # rad/s  angular.z 최대값

# ── PID 게인 ──────────────────────────────────────────────────────
KP = 0.40   # 비례: 현재 위치 오차 → linear.y
KD = 0.10   # 미분: 오차 변화율 (진동 억제)
KI = 0.08   # 적분: 누적 오차 → angular.z (헤딩 보정)

# 적분 와인드업 방지 — 누적값 상한
INTEGRAL_CLAMP = 3.0   # m·s (이 이상 쌓이지 않도록)

# 적분 데드존 — 이 미만의 오차는 적분에 포함 안 함 (작은 노이즈 무시)
INTEGRAL_DEADZONE = 0.03   # m

# 통로 인식 임계값 — 양쪽 벽이 모두 이 거리 이내일 때만 보정 활성화
CORRIDOR_WALL_MAX = 1.5    # m

# ── 라이다 설정 ───────────────────────────────────────────────────
LEFT_ANGLE_DEG   = -90    # 왼쪽 측정 각도
RIGHT_ANGLE_DEG  =  90    # 오른쪽 측정 각도
SECTOR_WIDTH_DEG =  30    # 측정 섹터 폭 (±15°)
RANGE_MIN = 0.10
RANGE_MAX = 5.00

# ── 전방 장애물 ───────────────────────────────────────────────────
FRONT_STOP_DIST  = 0.50   # m  이하면 정지
FRONT_SLOW_DIST  = 1.00   # m  이하면 감속

RATE_HZ = 20
# ─────────────────────────────────────────────────────────────────

_scan = None


def _scan_cb(msg):
    global _scan
    _scan = msg


def _sector_median(scan, center_deg, width_deg=SECTOR_WIDTH_DEG):
    center_rad = math.radians(center_deg)
    half       = math.radians(width_deg / 2.0)
    results = []
    for i, r in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        if abs(angle - center_rad) <= half:
            if RANGE_MIN < r < RANGE_MAX and math.isfinite(r):
                results.append(r)
    if not results:
        return float('inf')
    results.sort()
    return results[len(results) // 2]


def _sector_min(scan, center_deg, width_deg=20):
    center_rad = math.radians(center_deg)
    half       = math.radians(width_deg / 2.0)
    results = []
    for i, r in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        if abs(angle - center_rad) <= half:
            if RANGE_MIN < r < RANGE_MAX and math.isfinite(r):
                results.append(r)
    return min(results) if results else float('inf')


def get_distances(scan):
    left  = _sector_median(scan, LEFT_ANGLE_DEG)
    right = _sector_median(scan, RIGHT_ANGLE_DEG)
    front = _sector_min(scan, 180, width_deg=20)
    return left, right, front


def scan_mode(stop_dist):
    """센서 방향 확인 — 왼쪽에 손 가져다 댔을 때 좌값이 줄어야 정상."""
    rospy.loginfo("스캔 모드 (Ctrl+C 종료)")
    rospy.loginfo("좌/우 값이 반대라면 LEFT/RIGHT_ANGLE_DEG 부호를 바꾸세요.")
    rospy.loginfo(f"전방 정지 거리: {stop_dist:.2f}m")
    rate = rospy.Rate(4)
    while not rospy.is_shutdown():
        if _scan is None:
            rospy.loginfo("스캔 대기 중...")
        else:
            l, r, f = get_distances(_scan)
            front_warn = ""
            if f <= stop_dist:
                front_warn = "  *** 정지 ***"
            elif f <= stop_dist * 2:
                front_warn = "  (감속 구간)"
            rospy.loginfo(
                f"전방={f:5.2f}m{front_warn}  |  "
                f"좌={l:5.2f}m  우={r:5.2f}m  오차(좌-우)={l-r:+.3f}m"
            )
        rate.sleep()


def follow_mode(pub, forward_speed, center_only, stop_dist=FRONT_STOP_DIST):
    dt          = 1.0 / RATE_HZ
    rate        = rospy.Rate(RATE_HZ)
    prev_error  = 0.0
    integral    = 0.0

    # 로그용: 최근 N샘플 적분 방향 추적
    history     = collections.deque(maxlen=int(RATE_HZ * 2))  # 2초

    def emergency_stop():
        for _ in range(5):
            pub.publish(Twist())
            rospy.sleep(0.05)

    rospy.on_shutdown(emergency_stop)

    rospy.loginfo("통로 추종 시작 (Ctrl+C 정지)")
    rospy.loginfo(f"  전진속도={forward_speed} m/s  KP={KP}  KD={KD}  KI={KI}")

    while not rospy.is_shutdown():
        if _scan is None:
            rate.sleep()
            continue

        l, r, f = get_distances(_scan)

        # ── 오차 계산 ──────────────────────────────────────────
        # 통로 인식: 양쪽 벽 모두 CORRIDOR_WALL_MAX 이내일 때만 보정
        in_corridor = (l < CORRIDOR_WALL_MAX) and (r < CORRIDOR_WALL_MAX)

        if not in_corridor:
            error = 0.0   # 넓은 공간 → 직진만
        elif math.isinf(l):
            error = -0.30
        elif math.isinf(r):
            error =  0.30
        else:
            error = l - r

        history.append(error)

        # ── 적분 (헤딩 드리프트 감지) ──────────────────────────
        # 데드존: 작은 노이즈는 적분에서 제외
        if abs(error) > INTEGRAL_DEADZONE:
            integral += error * dt
        else:
            # 오차가 작아지면 적분 서서히 감쇠 (decay)
            integral *= 0.95

        # 와인드업 방지
        integral = max(-INTEGRAL_CLAMP, min(INTEGRAL_CLAMP, integral))

        # ── P+D → linear.y (위치 보정) ─────────────────────────
        d_error  = (error - prev_error) / dt
        lateral  = KP * error + KD * d_error
        lateral  = max(-MAX_LATERAL, min(MAX_LATERAL, lateral))
        prev_error = error

        # ── I → angular.z (헤딩 보정) ──────────────────────────
        yaw = KI * integral
        yaw = max(-MAX_YAW, min(MAX_YAW, yaw))

        # ── 전진 속도 (전방 장애물 감속/정지) ──────────────────
        slow_dist = stop_dist * 2.0
        if center_only:
            vx = 0.0
        elif f <= stop_dist:
            vx = 0.0
            rospy.loginfo_throttle(0.5, f"[정지] 전방 장애물 {f:.2f}m (정지거리 {stop_dist:.2f}m)")
        elif f <= slow_dist:
            scale = (f - stop_dist) / (slow_dist - stop_dist)
            vx = forward_speed * max(0.3, scale)
            rospy.loginfo_throttle(1.0, f"[감속] 전방 {f:.2f}m → vx={vx:.2f}")
        else:
            vx = forward_speed

        # ── 퍼블리시 ───────────────────────────────────────────
        cmd = Twist()
        cmd.linear.x  = vx
        cmd.linear.y  = lateral
        cmd.angular.z = yaw
        pub.publish(cmd)

        # ── 로그 ───────────────────────────────────────────────
        drift_dir = ""
        if len(history) == history.maxlen:
            avg = sum(history) / len(history)
            if abs(avg) > 0.05:
                drift_dir = f"  [드리프트→{'왼쪽' if avg > 0 else '오른쪽'} 헤딩보정중]"

        rospy.loginfo_throttle(
            0.5,
            f"전방={f:4.2f}m  좌={l:4.2f}m  우={r:4.2f}m  "
            f"err={error:+.3f}  적분={integral:+.3f}  "
            f"vy={lateral:+.3f}  wz={yaw:+.3f}{drift_dir}"
        )

        rate.sleep()

    pub.publish(Twist())


def main():
    args = sys.argv[1:]

    scan_only   = '--scan'   in args
    center_only = '--center' in args

    forward_speed = FORWARD_SPEED
    if '--speed' in args:
        idx = args.index('--speed')
        forward_speed = float(args[idx + 1])

    stop_dist = FRONT_STOP_DIST
    if '--stop-dist' in args:
        idx = args.index('--stop-dist')
        stop_dist = float(args[idx + 1])

    rospy.init_node('corridor_follow', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, _scan_cb)
    rospy.sleep(0.5)

    if scan_only:
        scan_mode(stop_dist)
    else:
        follow_mode(pub, forward_speed, center_only, stop_dist)


if __name__ == '__main__':
    main()
