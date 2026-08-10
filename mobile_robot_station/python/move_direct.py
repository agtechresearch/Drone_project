#!/usr/bin/env python3
"""
직접 cmd_vel 제어 — move_base/AMCL 없이 오도메트리 피드백으로 이동.

myagv_odometry_node 가 실행 중이어야 함 (직렬포트 → cmd_vel 변환 담당).

사용법:
  move_direct.py forward  <meters>
  move_direct.py back     <meters>
  move_direct.py left     <meters>
  move_direct.py right    <meters>
  move_direct.py turn     <degrees>   # 양수=반시계, 음수=시계
  move_direct.py dx <m> dy <m>        # 로봇 기준 대각선 이동
"""

import sys
import math
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ── 속도 파라미터 ──────────────────────────────────────────────
LINEAR_SPEED  = 0.15   # m/s  (너무 빠르면 미끄러짐; 0.10~0.20 권장)
ANGULAR_SPEED = 0.4    # rad/s
RATE_HZ       = 20
TIMEOUT_MULT  = 3.0    # 예상 소요시간의 3배를 타임아웃으로

# ── 오도메트리 상태 ────────────────────────────────────────────
_odom_pos  = None   # (x, y)
_odom_yaw  = None


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _odom_cb(msg):
    global _odom_pos, _odom_yaw
    p = msg.pose.pose.position
    _odom_pos = (p.x, p.y)
    _odom_yaw = _yaw_from_quat(msg.pose.pose.orientation)


def _wait_odom(timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _odom_pos is not None:
            return True
        time.sleep(0.05)
    return False


def _publish_cmd(pub, vx=0.0, vy=0.0, wz=0.0):
    t = Twist()
    t.linear.x  = vx
    t.linear.y  = vy
    t.angular.z = wz
    pub.publish(t)


def _stop(pub):
    for _ in range(3):
        _publish_cmd(pub)
        time.sleep(0.05)


def move_linear(pub, dx_robot, dy_robot, speed, label):
    """로봇 기준 (dx_robot, dy_robot) 방향으로 이동."""
    dist = math.sqrt(dx_robot**2 + dy_robot**2)
    if dist < 1e-6:
        return

    # 로봇 프레임 속도 벡터
    vx = speed * dx_robot / dist
    vy = speed * dy_robot / dist

    start = _odom_pos
    yaw0  = _odom_yaw
    timeout = dist / speed * TIMEOUT_MULT

    rate = rospy.Rate(RATE_HZ)
    deadline = time.time() + timeout
    traveled = 0.0

    print(f"[move] {label}  목표={dist:.3f}m  vx={vx:.2f} vy={vy:.2f}")

    while not rospy.is_shutdown() and time.time() < deadline:
        # 오도메트리 이동거리: 세계좌표 변화를 로봇 초기 방향으로 투영
        wx = _odom_pos[0] - start[0]
        wy = _odom_pos[1] - start[1]
        cos_y, sin_y = math.cos(yaw0), math.sin(yaw0)
        local_x =  wx * cos_y + wy * sin_y
        local_y = -wx * sin_y + wy * cos_y
        traveled = math.sqrt(local_x**2 + local_y**2)

        if traveled >= dist:
            break

        remaining = dist - traveled
        # 목표에 근접하면 감속
        if remaining < 0.05:
            scale = max(0.3, remaining / 0.05)
            _publish_cmd(pub, vx * scale, vy * scale)
        else:
            _publish_cmd(pub, vx, vy)
        rate.sleep()

    _stop(pub)

    if traveled >= dist * 0.8:
        print(f"[완료] 이동거리={traveled:.3f}m")
    else:
        print(f"[경고] 타임아웃 — 이동거리={traveled:.3f}m / 목표={dist:.3f}m")
        print("       오도메트리 없이 시간 기반으로 재시도: move_direct.py --timed 사용")


def move_rotate(pub, target_deg, speed):
    """제자리 회전 — 양수=반시계(CCW), 음수=시계(CW)."""
    target_rad = math.radians(target_deg)
    wz = speed if target_deg > 0 else -speed

    start_yaw = _odom_yaw
    timeout = abs(target_rad) / speed * TIMEOUT_MULT

    rate = rospy.Rate(RATE_HZ)
    deadline = time.time() + timeout
    rotated = 0.0
    prev_yaw = start_yaw

    print(f"[move] 회전  목표={target_deg:.1f}°  wz={wz:.2f}")

    while not rospy.is_shutdown() and time.time() < deadline:
        delta = _odom_yaw - prev_yaw
        # 각도 wrap-around 처리
        if delta >  math.pi: delta -= 2 * math.pi
        if delta < -math.pi: delta += 2 * math.pi
        rotated += delta
        prev_yaw = _odom_yaw

        if abs(rotated) >= abs(target_rad):
            break

        remaining = abs(target_rad) - abs(rotated)
        if remaining < math.radians(10):
            scale = max(0.4, remaining / math.radians(10))
            _publish_cmd(pub, wz=wz * scale)
        else:
            _publish_cmd(pub, wz=wz)
        rate.sleep()

    _stop(pub)
    print(f"[완료] 회전={math.degrees(rotated):.1f}°")


def move_timed(pub, vx, vy, wz, seconds, label):
    """오도메트리 없이 시간 기반 이동 (폴백)."""
    print(f"[timed] {label}  {seconds:.2f}초")
    rate = rospy.Rate(RATE_HZ)
    deadline = time.time() + seconds
    while not rospy.is_shutdown() and time.time() < deadline:
        _publish_cmd(pub, vx, vy, wz)
        rate.sleep()
    _stop(pub)
    print("[완료] 시간 기반 이동 완료")


def _parse_flag_value(args, flag, default):
    """--flag value 형태의 인자 파싱."""
    if flag in args:
        idx = args.index(flag)
        val = float(args[idx + 1])
        args.pop(idx); args.pop(idx)
        return val
    return default


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        sys.exit(1)

    # 플래그 파싱
    use_timed = '--timed' in args
    args = [a for a in args if a != '--timed']

    linear_speed  = _parse_flag_value(args, '--speed',  LINEAR_SPEED)
    angular_speed = _parse_flag_value(args, '--aspeed', ANGULAR_SPEED)

    rospy.init_node('move_direct', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, _odom_cb)

    rospy.sleep(0.5)  # publisher 등록 대기

    cmd = args[0].lower()

    # dx dy 혼합 이동 (예: dx 0.3 dy 0.2)
    if cmd == 'dx':
        dx = float(args[1])
        dy = float(args[args.index('dy') + 1]) if 'dy' in args else 0.0
        dist = math.sqrt(dx**2 + dy**2)
        speed = linear_speed
    elif cmd in ('forward', 'back', 'left', 'right'):
        dist = float(args[1])
        dx = {'forward': 1, 'back': -1, 'left': 0, 'right': 0}[cmd] * dist
        dy = {'forward': 0, 'back': 0,  'left': 1, 'right': -1}[cmd] * dist
        speed = linear_speed
    elif cmd == 'turn':
        deg = float(args[1])
        if use_timed:
            seconds = abs(math.radians(deg)) / angular_speed * 1.1
            move_timed(pub, 0, 0, angular_speed if deg > 0 else -angular_speed, seconds, f"turn {deg}°")
        else:
            if not _wait_odom(5.0):
                print("[경고] /odom 없음 — 시간 기반으로 실행")
                seconds = abs(math.radians(deg)) / angular_speed * 1.1
                move_timed(pub, 0, 0, angular_speed if deg > 0 else -angular_speed, seconds, f"turn {deg}°")
            else:
                move_rotate(pub, deg, angular_speed)
        return
    else:
        print(f"알 수 없는 명령: {cmd}")
        print(__doc__)
        sys.exit(1)

    # 시간 기반 모드
    if use_timed:
        vx = speed * dx / dist if dist > 1e-6 else 0
        vy = speed * dy / dist if dist > 1e-6 else 0
        seconds = dist / speed * 1.1
        move_timed(pub, vx, vy, 0, seconds, f"{cmd} {dist:.2f}m")
        return

    # 오도메트리 기반 모드
    if not _wait_odom(5.0):
        print("[경고] /odom 수신 실패 — --timed 모드로 폴백")
        vx = speed * dx / dist if dist > 1e-6 else 0
        vy = speed * dy / dist if dist > 1e-6 else 0
        seconds = dist / speed * 1.1
        move_timed(pub, vx, vy, 0, seconds, f"{cmd} {dist:.2f}m")
        return

    move_linear(pub, dx, dy, speed, f"{cmd} {dist:.2f}m")


if __name__ == '__main__':
    main()
