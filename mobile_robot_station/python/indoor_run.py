#!/usr/bin/env python3
"""
실내 경로 왕복 주행 — 단계별 키보드 확인.

단계:
  1단계 [전진]  : 기준 거리 유지하며 전진 → 전방 장애물 정지
  2단계 [회전]  : R 키 → 제자리 180도 회전
  3단계 [귀환]  : G 키 → 동일 방식으로 복귀 → 전방 장애물 정지
  각 단계 전 Q 키로 중단 가능

사용법:
  indoor_run.py                    # 기본 파라미터
  indoor_run.py --front-stop 0.6   # 전방 정지 거리
  indoor_run.py --fspeed 0.10      # 전진 속도
  indoor_run.py --scan             # 센서값 확인
"""

import sys
import math
import termios
import tty
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# ════════════════════════════════════════════════════
#  ★ 사용자 파라미터 — 여기서 수정
# ════════════════════════════════════════════════════

FRONT_STOP_DIST  = 0.50   # m  전방 정지 거리

FORWARD_SPEED    = 0.12   # m/s  전진 속도
ROTATE_SPEED     = 0.30   # rad/s  회전 속도 (양수=반시계)

MAX_LATERAL      = 0.15   # m/s  횡이동 최대 속도
MAX_YAW          = 0.25   # rad/s  헤딩 보정 최대 각속도

# PID 게인
KP = 0.40
KD = 0.10
KI = 0.08

INTEGRAL_CLAMP    = 3.0
INTEGRAL_DEADZONE = 0.03  # m

# ── 기준 거리 측정 ─────────────────────────────────────────────────
REF_MEASURE_SEC   = 2.0   # s

# ── 기둥/일시 장애물 무시 ──────────────────────────────────────────
PILLAR_IGNORE_DIST = 0.40  # m  기준보다 이 값 이상 가까우면 기둥으로 판단

# ── 180도 회전 감지 ────────────────────────────────────────────────
ROTATE_MIN_SEC     = 2.0   # s  최소 회전 시간
ROTATE_TIMEOUT_SEC = 12.0  # s  타임아웃
SWAP_TOLERANCE     = 0.35  # m  좌/우 교환 감지 허용 오차

# ── 전방 벽 vs 기둥 구분 ──────────────────────────────────────────
# 전방 stop_dist 이내 포인트들의 각도 스팬이 이 값 이상이면 벽으로 판단
# 기둥은 좁은 각도(~5°), 벽은 넓은 각도(30°+)로 구분
FRONT_CHECK_WIDTH_DEG = 60   # 장애물 폭 검사에 쓸 전방 스캔 범위
WALL_SPAN_MIN_DEG     = 20   # 이 각도 이상 → 벽 (정지)  미만 → 기둥 (무시하고 계속)

# 감속 구간
SLOW_FACTOR = 2.0

# 라이다 각도
LEFT_ANGLE_DEG  = -90
RIGHT_ANGLE_DEG =  90
FRONT_ANGLE_DEG = 180
REAR_ANGLE_DEG  =   0

SECTOR_WIDTH_DEG = 30
FRONT_WIDTH_DEG  = 20

RANGE_MIN = 0.10
RANGE_MAX = 12.0

RATE_HZ = 20
# ════════════════════════════════════════════════════

_scan = None


def _scan_cb(msg):
    global _scan
    _scan = msg


def _sector_median(scan, center_deg, width_deg):
    center_rad = math.radians(center_deg)
    half = math.radians(width_deg / 2.0)
    vals = []
    for i, r in enumerate(scan.ranges):
        a = scan.angle_min + i * scan.angle_increment
        diff = abs(math.atan2(math.sin(a - center_rad), math.cos(a - center_rad)))
        if diff <= half and RANGE_MIN < r < RANGE_MAX and math.isfinite(r):
            vals.append(r)
    if not vals:
        return float('inf')
    vals.sort()
    return vals[len(vals) // 2]


def _sector_min(scan, center_deg, width_deg):
    center_rad = math.radians(center_deg)
    half = math.radians(width_deg / 2.0)
    vals = []
    for i, r in enumerate(scan.ranges):
        a = scan.angle_min + i * scan.angle_increment
        diff = abs(math.atan2(math.sin(a - center_rad), math.cos(a - center_rad)))
        if diff <= half and RANGE_MIN < r < RANGE_MAX and math.isfinite(r):
            vals.append(r)
    return min(vals) if vals else float('inf')


def get_all_distances(scan):
    left  = _sector_median(scan, LEFT_ANGLE_DEG,  SECTOR_WIDTH_DEG)
    right = _sector_median(scan, RIGHT_ANGLE_DEG, SECTOR_WIDTH_DEG)
    front = _sector_min(scan,    FRONT_ANGLE_DEG, FRONT_WIDTH_DEG)
    rear  = _sector_min(scan,    REAR_ANGLE_DEG,  FRONT_WIDTH_DEG)
    return left, right, front, rear


def front_obstacle_span(scan, center_deg, stop_dist, check_width_deg):
    """전방 check_width_deg 범위에서 stop_dist 이내 포인트들의 각도 스팬(도) 반환."""
    center_rad = math.radians(center_deg)
    half = math.radians(check_width_deg / 2.0)
    signed_angles = []
    for i, r in enumerate(scan.ranges):
        a = scan.angle_min + i * scan.angle_increment
        d = math.atan2(math.sin(a - center_rad), math.cos(a - center_rad))
        if abs(d) <= half and RANGE_MIN < r <= stop_dist and math.isfinite(r):
            signed_angles.append(math.degrees(d))
    if not signed_angles:
        return 0.0
    return max(signed_angles) - min(signed_angles)


def filter_pillar(current, reference):
    if not math.isinf(reference) and current < reference - PILLAR_IGNORE_DIST:
        return reference
    return current


def wall_error(l_f, r_f, ref_l, ref_r):
    if math.isinf(l_f) and math.isinf(r_f):
        return 0.0
    elif math.isinf(l_f):
        return -0.30
    elif math.isinf(r_f):
        return 0.30
    return (l_f - ref_l) - (r_f - ref_r)


def compute_control(error, prev_error, integral, dt):
    if abs(error) > INTEGRAL_DEADZONE:
        integral += error * dt
    else:
        integral *= 0.95
    integral = max(-INTEGRAL_CLAMP, min(INTEGRAL_CLAMP, integral))

    d_error = (error - prev_error) / dt
    lateral = KP * error + KD * d_error
    lateral = max(-MAX_LATERAL, min(MAX_LATERAL, lateral))
    yaw = KI * integral
    yaw = max(-MAX_YAW, min(MAX_YAW, yaw))
    return lateral, yaw, error, integral


def calc_speed(distance, stop_dist, max_speed):
    slow_dist = stop_dist * SLOW_FACTOR
    if distance <= stop_dist:
        return 0.0
    elif distance <= slow_dist:
        scale = (distance - stop_dist) / (slow_dist - stop_dist)
        return max_speed * max(0.3, scale)
    return max_speed


def wait_key(prompt):
    """엔터 없이 단일 키 입력. 소문자 반환."""
    sys.stdout.write(prompt)
    sys.stdout.flush()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    sys.stdout.write(ch.upper() + '\n')
    sys.stdout.flush()
    return ch.lower()


def measure_reference():
    rospy.loginfo(f"기준 거리 측정 중 ({REF_MEASURE_SEC:.1f}s)...")
    rate = rospy.Rate(RATE_HZ)
    left_vals, right_vals = [], []
    deadline = rospy.get_time() + REF_MEASURE_SEC
    while rospy.get_time() < deadline and not rospy.is_shutdown():
        if _scan is not None:
            l, r, _, _ = get_all_distances(_scan)
            if not math.isinf(l):
                left_vals.append(l)
            if not math.isinf(r):
                right_vals.append(r)
        rate.sleep()

    if not left_vals or not right_vals:
        rospy.logwarn("벽 감지 실패 — 기본값 1.0m 사용")
        return 1.0, 1.0

    left_vals.sort()
    right_vals.sort()
    ref_l = left_vals[len(left_vals) // 2]
    ref_r = right_vals[len(right_vals) // 2]
    rospy.loginfo(f"기준 거리 — 좌: {ref_l:.3f}m  우: {ref_r:.3f}m")
    return ref_l, ref_r


def forward_phase(pub, front_stop, fspeed, ref_l, ref_r, label):
    """전진 → 전방 장애물 정지. 정지 직전 좌/우 거리 반환."""
    rate = rospy.Rate(RATE_HZ)
    dt = 1.0 / RATE_HZ
    integral, prev_error = 0.0, 0.0
    last_l, last_r = ref_l, ref_r

    rospy.loginfo(f"[{label}] 전진 시작  기준 좌={ref_l:.3f}m 우={ref_r:.3f}m")

    while not rospy.is_shutdown():
        if _scan is None:
            rate.sleep()
            continue

        l, r, f, _ = get_all_distances(_scan)
        l_f = filter_pillar(l, ref_l)
        r_f = filter_pillar(r, ref_r)
        error = wall_error(l_f, r_f, ref_l, ref_r)

        lateral, yaw, prev_error, integral = compute_control(error, prev_error, integral, dt)
        vx = calc_speed(f, front_stop, fspeed)

        cmd = Twist()
        cmd.linear.x  = vx
        cmd.linear.y  = lateral
        cmd.angular.z = yaw
        pub.publish(cmd)

        pillar_str = ""
        if l_f != l:
            pillar_str += " [좌기둥]"
        if r_f != r:
            pillar_str += " [우기둥]"

        rospy.loginfo_throttle(
            0.5,
            f"[{label}]  전방={f:.2f}  좌={l:.2f}(기준{ref_l:.2f})  우={r:.2f}(기준{ref_r:.2f})"
            f"{pillar_str}  vx={vx:+.3f}  vy={lateral:+.3f}  wz={yaw:+.3f}  err={error:+.3f}"
        )

        if f <= front_stop:
            span = front_obstacle_span(_scan, FRONT_ANGLE_DEG, front_stop, FRONT_CHECK_WIDTH_DEG)
            # 가까울수록 기둥도 넓어 보이므로 임계값을 거리에 반비례해서 높임
            adaptive_min = WALL_SPAN_MIN_DEG * (front_stop / max(f, 0.05))
            if span >= adaptive_min:
                rospy.loginfo(
                    f"[{label}] 전방 도달 {f:.2f}m  장애물 폭={span:.1f}° ≥ {adaptive_min:.1f}° → 벽 → 정지"
                )
                last_l, last_r = l, r
                break
            else:
                rospy.loginfo_throttle(
                    1.0,
                    f"[{label}] 전방 {f:.2f}m 감지됐으나 폭={span:.1f}° < {adaptive_min:.1f}° → 기둥 무시"
                )

        rate.sleep()

    return last_l, last_r


def rotate_phase(pub, rot_left_before, rot_right_before):
    """제자리 회전 → 좌/우 교환 감지 또는 타임아웃. 교환된 (ref_l, ref_r) 반환."""
    rate = rospy.Rate(RATE_HZ)
    start = rospy.get_time()
    rospy.loginfo(
        f"[2단계] 회전 시작  "
        f"목표: 좌≈{rot_right_before:.2f}m  우≈{rot_left_before:.2f}m"
    )

    while not rospy.is_shutdown():
        elapsed = rospy.get_time() - start

        cmd = Twist()
        cmd.angular.z = ROTATE_SPEED
        pub.publish(cmd)

        if _scan is not None and elapsed >= ROTATE_MIN_SEC:
            l, r, _, _ = get_all_distances(_scan)
            swapped = (
                abs(l - rot_right_before) < SWAP_TOLERANCE and
                abs(r - rot_left_before)  < SWAP_TOLERANCE
            )
            if swapped or elapsed >= ROTATE_TIMEOUT_SEC:
                reason = "좌/우 교환 감지" if swapped else f"타임아웃({ROTATE_TIMEOUT_SEC:.0f}s)"
                rospy.loginfo(
                    f"[2단계] 회전 완료 ({reason})  "
                    f"좌={l:.2f}↔{rot_right_before:.2f}  우={r:.2f}↔{rot_left_before:.2f}"
                )
                break

        if _scan is not None:
            l, r, _, _ = get_all_distances(_scan)
            rospy.loginfo_throttle(
                0.5,
                f"[ROTATING] {elapsed:.1f}s  "
                f"좌={l:.2f}(목표≈{rot_right_before:.2f})  우={r:.2f}(목표≈{rot_left_before:.2f})"
            )

        rate.sleep()

    return rot_right_before, rot_left_before


def scan_mode():
    rospy.loginfo("스캔 모드 (Ctrl+C 종료) — 전/후/좌/우 거리 확인")
    rate = rospy.Rate(4)
    while not rospy.is_shutdown():
        if _scan is None:
            rospy.loginfo("스캔 대기...")
            rate.sleep()
            continue
        l, r, f, b = get_all_distances(_scan)

        def fmt(v, stop, label):
            if v <= stop:
                return f"{label}={v:.2f}m[★정지]"
            elif v <= stop * SLOW_FACTOR:
                return f"{label}={v:.2f}m[감속]"
            return f"{label}={v:.2f}m"

        rospy.loginfo(
            f"{fmt(f, FRONT_STOP_DIST,'전방')}  "
            f"후방={b:.2f}m  |  "
            f"좌={l:.2f}m  우={r:.2f}m  오차={l-r:+.3f}m"
        )
        rate.sleep()


def run(pub, front_stop, fspeed):
    def emergency_stop():
        for _ in range(5):
            pub.publish(Twist())
            rospy.sleep(0.05)

    rospy.on_shutdown(emergency_stop)

    ref_l, ref_r = measure_reference()

    rospy.loginfo("=" * 50)
    rospy.loginfo("실내 경로 단계별 주행")
    rospy.loginfo(f"  기준 좌/우 : {ref_l:.3f}m / {ref_r:.3f}m")
    rospy.loginfo(f"  전방 정지  : {front_stop:.2f}m")
    rospy.loginfo(f"  전진 속도  : {fspeed:.2f} m/s")
    rospy.loginfo(f"  회전 속도  : {ROTATE_SPEED:.2f} rad/s")
    rospy.loginfo("  키: [R]=회전  [G]=귀환  [Q]=중단")
    rospy.loginfo("=" * 50)

    # ── 1단계: 전진 ────────────────────────────────────────────────
    last_l, last_r = forward_phase(pub, front_stop, fspeed, ref_l, ref_r, "1단계 전진")
    emergency_stop()

    if rospy.is_shutdown():
        return

    # ── 키 입력: 회전 or 중단 ──────────────────────────────────────
    key = wait_key("\n>>> [R] 회전  [Q] 중단: ")
    if key != 'r':
        rospy.loginfo("중단")
        emergency_stop()
        return

    # ── 2단계: 회전 ────────────────────────────────────────────────
    new_ref_l, new_ref_r = rotate_phase(pub, last_l, last_r)
    emergency_stop()

    if rospy.is_shutdown():
        return

    # ── 키 입력: 귀환 or 중단 ─────────────────────────────────────
    key = wait_key("\n>>> [G] 귀환  [Q] 중단: ")
    if key != 'g':
        rospy.loginfo("중단")
        emergency_stop()
        return

    # ── 3단계: 귀환 ────────────────────────────────────────────────
    forward_phase(pub, front_stop, fspeed, new_ref_l, new_ref_r, "3단계 귀환")
    emergency_stop()

    rospy.loginfo("완료 — 정지")


def parse_arg(args, flag, default):
    if flag in args:
        return float(args[args.index(flag) + 1])
    return default


def main():
    args = sys.argv[1:]

    if '--scan' in args:
        rospy.init_node('indoor_run_scan', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, _scan_cb)
        rospy.sleep(0.5)
        scan_mode()
        return

    front_stop = parse_arg(args, '--front-stop', FRONT_STOP_DIST)
    fspeed     = parse_arg(args, '--fspeed',     FORWARD_SPEED)

    rospy.init_node('indoor_run', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, _scan_cb)
    rospy.sleep(0.5)

    run(pub, front_stop, fspeed)


if __name__ == '__main__':
    main()
