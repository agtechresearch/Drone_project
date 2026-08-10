#!/usr/bin/env python3
"""
매핑용 단일키 스텝 드라이버 — 한 키 = 한 스텝(오도메트리 피드백 이동).

move_direct.py 의 move_linear/move_rotate 를 재사용한다. ROS 노드를 하나만
띄워 상주하므로 키마다 재초기화가 없다(타이핑보다 빠르고 연속 텔레옵보다 안전).
myagv_odometry_node 가 실행 중이어야 한다(직렬포트 → cmd_vel + /odom).

SSH 터미널에서 단일키 입력을 받는다(cbreak). 이 창에 포커스를 두고 키를 누르면 된다.

키:
  i / k   전진 / 후진 (STEP m)
  j / l   좌회전 / 우회전 (TURN°, j=반시계 CCW)
  u / o   좌 / 우 횡이동 (메카넘, STEP m)
  r       제자리 360° 스윕 (45°×8, 라이다 후방 실명 보완)
  + / -   STEP 크기 ±0.1m
  space   즉시 정지 (cmd_vel 0)
  q       종료
"""

import sys
import os
import termios
import tty
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import move_direct as md

STEP = 0.3          # m  (전/후/횡 기본 스텝)
TURN = 30.0         # deg (좌/우 회전 스텝)
SPEED = md.LINEAR_SPEED     # 0.15 m/s
ASPEED = md.ANGULAR_SPEED   # 0.4 rad/s

LEGEND = (
    "\r\n"
    "  i/k 전진/후진   j/l 좌/우회전   u/o 좌/우 횡이동\r\n"
    "  r 제자리360°스윕   +/- 스텝크기   space 정지   q 종료\r\n"
)


def main():
    rospy.init_node('map_teleop', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, md._odom_cb)
    rospy.sleep(0.5)

    if not md._wait_odom(5.0):
        print("[오류] /odom 수신 실패 — myagv_odometry_node(1_make_map.sh) 실행 중인지 확인하세요.")
        return

    step = STEP
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    sys.stdout.write("매핑 스텝 드라이버 시작. STEP=%.1fm TURN=%.0f°\r\n" % (step, TURN))
    sys.stdout.write(LEGEND)
    sys.stdout.flush()
    try:
        tty.setcbreak(fd)
        while not rospy.is_shutdown():
            termios.tcflush(fd, termios.TCIFLUSH)   # 이동 중 눌린 키 버림(폭주 방지)
            ch = sys.stdin.read(1)
            if ch in ('q', '\x03'):                 # q 또는 Ctrl+C
                break
            elif ch == 'i':
                md.move_linear(pub, step, 0, SPEED, "forward %.2f" % step)
            elif ch == 'k':
                md.move_linear(pub, -step, 0, SPEED, "back %.2f" % step)
            elif ch == 'u':
                md.move_linear(pub, 0, step, SPEED, "left %.2f" % step)
            elif ch == 'o':
                md.move_linear(pub, 0, -step, SPEED, "right %.2f" % step)
            elif ch == 'j':
                md.move_rotate(pub, TURN, ASPEED)
            elif ch == 'l':
                md.move_rotate(pub, -TURN, ASPEED)
            elif ch == 'r':
                sys.stdout.write("[스윕] 45°×8 회전...\r\n"); sys.stdout.flush()
                for _ in range(8):
                    if rospy.is_shutdown():
                        break
                    md.move_rotate(pub, 45, ASPEED)
                    rospy.sleep(0.4)   # gmapping 스캔 반영 대기
            elif ch in ('+', '='):
                step = min(1.0, round(step + 0.1, 1))
                sys.stdout.write("STEP=%.1fm\r\n" % step); sys.stdout.flush()
            elif ch in ('-', '_'):
                step = max(0.1, round(step - 0.1, 1))
                sys.stdout.write("STEP=%.1fm\r\n" % step); sys.stdout.flush()
            elif ch == ' ':
                md._stop(pub)
                sys.stdout.write("[정지]\r\n"); sys.stdout.flush()
            # 그 외 키는 무시
    except KeyboardInterrupt:
        pass
    finally:
        md._stop(pub)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\r\n종료. cmd_vel 0 발행 완료.")


if __name__ == '__main__':
    main()
