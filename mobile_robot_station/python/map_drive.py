#!/usr/bin/env python3
"""
매핑용 반복 직진 주행.
전진 <step>m 이동 → <wait>초 대기 를 <count>회 반복한다. 각 대기 구간은
gmapping 스캔 정합이 안정될 시간을 준다. move_direct 와 동일하게 오도메트리
피드백으로 거리를 맞추며, 라이다 장애물 감지는 없다(매핑 중 수동 감시용).

주행 중 아무 키나 누르거나(터미널) Ctrl+C 를 누르면 즉시 정지·종료한다.

myagv_odometry_node 가 실행 중이어야 함 (/odom, /cmd_vel).

사용법:
  map_drive.py <count>                    # 전진 0.5m × count, 사이 2초 대기
  map_drive.py 50                         # 0.5m × 50 = 25m
  map_drive.py 50 --wait 3 --step 0.5     # 대기/스텝 지정
  map_drive.py 20 --dir left              # 방향 지정 (forward/back/left/right)
  map_drive.py 40 --speed 0.12            # 속도 지정
"""
import sys
import math
import time
import select
import termios
import tty
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

LINEAR_SPEED = 0.15   # m/s
RATE_HZ      = 20
TIMEOUT_MULT = 3.0

_odom_pos = None
_odom_yaw = None


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


def _publish_cmd(pub, vx=0.0, vy=0.0):
    t = Twist()
    t.linear.x = vx
    t.linear.y = vy
    pub.publish(t)


def _stop(pub):
    for _ in range(3):
        _publish_cmd(pub)
        time.sleep(0.05)


def _stop_key_pressed():
    """터미널에서 아무 키나 눌렸으면 True (정지 신호)."""
    if not sys.stdin.isatty():
        return False
    if select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.read(1)  # 버퍼 비우기
        return True
    return False


def move_step(pub, dx_robot, dy_robot, speed):
    """로봇 기준 한 스텝 이동. 반환: 'done' | 'stopped' | 'timeout'."""
    dist = math.sqrt(dx_robot**2 + dy_robot**2)
    if dist < 1e-6:
        return 'done'

    vx = speed * dx_robot / dist
    vy = speed * dy_robot / dist

    start = _odom_pos
    yaw0  = _odom_yaw
    timeout = dist / speed * TIMEOUT_MULT

    rate = rospy.Rate(RATE_HZ)
    deadline = time.time() + timeout
    traveled = 0.0

    while not rospy.is_shutdown() and time.time() < deadline:
        if _stop_key_pressed():
            _stop(pub)
            return 'stopped'

        wx = _odom_pos[0] - start[0]
        wy = _odom_pos[1] - start[1]
        cos_y, sin_y = math.cos(yaw0), math.sin(yaw0)
        local_x =  wx * cos_y + wy * sin_y
        local_y = -wx * sin_y + wy * cos_y
        traveled = math.sqrt(local_x**2 + local_y**2)

        if traveled >= dist:
            break

        remaining = dist - traveled
        if remaining < 0.05:
            scale = max(0.3, remaining / 0.05)
            _publish_cmd(pub, vx * scale, vy * scale)
        else:
            _publish_cmd(pub, vx, vy)
        rate.sleep()

    _stop(pub)
    return 'done' if traveled >= dist * 0.8 else 'timeout'


def wait_with_stop(pub, seconds):
    """대기 중에도 정지 키를 감시. 정지되면 False 반환."""
    deadline = time.time() + seconds
    while time.time() < deadline and not rospy.is_shutdown():
        if _stop_key_pressed():
            _stop(pub)
            return False
        time.sleep(0.05)
    return True


def _parse_flag(args, flag, cast, default):
    if flag in args:
        return cast(args[args.index(flag) + 1])
    return default


def main():
    args = sys.argv[1:]
    if not args or not args[0].lstrip('-').isdigit():
        print(__doc__)
        sys.exit(1)

    count = int(args[0])
    step  = _parse_flag(args, '--step',  float, 0.5)
    wait  = _parse_flag(args, '--wait',  float, 2.0)
    speed = _parse_flag(args, '--speed', float, LINEAR_SPEED)
    direction = _parse_flag(args, '--dir', str, 'forward')

    dirmap = {'forward': (1, 0), 'back': (-1, 0), 'left': (0, 1), 'right': (0, -1)}
    if direction not in dirmap:
        print(f"알 수 없는 방향: {direction} (forward/back/left/right)")
        sys.exit(1)
    sx, sy = dirmap[direction]
    dx, dy = sx * step, sy * step

    rospy.init_node('map_drive', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, _odom_cb)
    rospy.sleep(0.5)

    if not _wait_odom(5.0):
        print("[오류] /odom 수신 실패 — 오도메트리 노드(myagv_active)가 실행 중인지 확인하세요.")
        sys.exit(1)

    rospy.on_shutdown(lambda: _stop(pub))

    total = count * step
    print("=" * 50)
    print(f" 매핑 반복 주행: {direction} {step}m × {count} = {total:.1f}m")
    print(f" 스텝 사이 대기: {wait:.1f}s   속도: {speed:.2f} m/s")
    print(f" ★ 정지: 아무 키나 누르기 (또는 Ctrl+C)")
    print("=" * 50)

    # 터미널을 cbreak 모드로 → Enter 없이 키 1개 즉시 감지 (Ctrl+C 유지)
    old_term = None
    if sys.stdin.isatty():
        old_term = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())

    try:
        for i in range(1, count + 1):
            if rospy.is_shutdown():
                break
            print(f"[{i}/{count}] {direction} {step}m ...")
            result = move_step(pub, dx, dy, speed)

            if result == 'stopped':
                print(f"[정지] 사용자 정지 — {i-1}회 완료 후 중단")
                break
            if result == 'timeout':
                print(f"[경고] {i}회차 타임아웃 — 벽에 막혔거나 미끄러짐 의심. 정지·중단")
                break

            if i < count:
                if not wait_with_stop(pub, wait):
                    print(f"[정지] 대기 중 사용자 정지 — {i}회 완료 후 중단")
                    break
        else:
            print(f"[완료] {count}회 반복 주행 완료 (약 {total:.1f}m)")
    finally:
        _stop(pub)
        if old_term is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)


if __name__ == '__main__':
    main()
