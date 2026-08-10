#!/usr/bin/env python3
"""
로봇 기준 상대 이동 스크립트.
현재 오도메트리 위치에서 지정한 만큼 이동합니다.
move_base를 사용하므로 장애물 회피가 동작합니다.

사용법:
    python3 move_relative.py forward 2.0        # 전진 2m
    python3 move_relative.py back 1.0           # 후진 1m
    python3 move_relative.py left 0.5           # 왼쪽 0.5m (메카넘)
    python3 move_relative.py right 0.5          # 오른쪽 0.5m (메카넘)
    python3 move_relative.py turn 90            # 제자리 우회전 90도
    python3 move_relative.py turn -45           # 제자리 좌회전 45도
    python3 move_relative.py dx 1.0 dy 0.5      # x,y 동시 지정
"""
import sys
import math
import rospy
import actionlib
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def get_current_pose(timeout=5.0):
    """오도메트리에서 현재 자세(x, y, yaw_rad) 반환."""
    try:
        msg = rospy.wait_for_message("/odom", Odometry, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("오도메트리 수신 실패. myagv_odometry가 실행 중인지 확인하세요.")
        sys.exit(1)

    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    return pos.x, pos.y, yaw


def compute_target(x0, y0, yaw0, dx_robot=0.0, dy_robot=0.0, dyaw_deg=0.0):
    """
    로봇 로컬 프레임의 상대 이동을 월드 프레임 절대 좌표로 변환.
    dx_robot: 전진(+) / 후진(-) 거리 (m)
    dy_robot: 왼쪽(+) / 오른쪽(-) 거리 (m)
    dyaw_deg: 반시계(+) / 시계(-) 회전 (도)
    """
    cos_y = math.cos(yaw0)
    sin_y = math.sin(yaw0)

    x_target = x0 + dx_robot * cos_y - dy_robot * sin_y
    y_target = y0 + dx_robot * sin_y + dy_robot * cos_y
    yaw_target = yaw0 + math.radians(dyaw_deg)

    return x_target, y_target, yaw_target


def send_goal(x, y, yaw_rad, timeout=60.0):
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("move_base 연결 대기 중...")
    if not client.wait_for_server(timeout=rospy.Duration(10.0)):
        rospy.logerr("move_base 연결 실패. 네비게이션이 실행 중인지 확인하세요.")
        sys.exit(1)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    q = quaternion_from_euler(0, 0, yaw_rad)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    rospy.loginfo(f"목표: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw_rad):.1f}deg")
    client.send_goal(goal)

    finished = client.wait_for_result(timeout=rospy.Duration(timeout))
    if not finished:
        rospy.logwarn("타임아웃 — 목표 취소")
        client.cancel_goal()
        return False

    state = client.get_state()
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("이동 완료")
        return True
    else:
        rospy.logwarn(f"이동 실패 (상태: {state})")
        return False


def parse_args(argv):
    """인자 파싱 → (dx_robot, dy_robot, dyaw_deg) 반환."""
    if not argv:
        print(__doc__)
        sys.exit(0)

    cmd = argv[0].lower()

    DIRECTIONS = {
        "forward": (1, 0, 0),
        "back":    (-1, 0, 0),
        "left":    (0, 1, 0),
        "right":   (0, -1, 0),
        "turn":    (0, 0, 1),
    }

    if cmd in DIRECTIONS:
        if len(argv) < 2:
            print(f"[ERROR] 거리/각도 값이 필요합니다. 예: {cmd} 1.0")
            sys.exit(1)
        val = float(argv[1])
        sx, sy, st = DIRECTIONS[cmd]
        return sx * val, sy * val, st * val

    # dx dy 키워드 형식: dx 1.0 dy 0.5 turn 30
    dx = dy = dyaw = 0.0
    i = 0
    while i < len(argv):
        key = argv[i].lower()
        if key in ("dx", "dy", "turn"):
            i += 1
            val = float(argv[i])
            if key == "dx":   dx = val
            elif key == "dy": dy = val
            else:             dyaw = val
        i += 1
    return dx, dy, dyaw


def main():
    rospy.init_node("move_relative", anonymous=True)

    dx_robot, dy_robot, dyaw_deg = parse_args(sys.argv[1:])

    rospy.loginfo(f"상대 이동: 전진={dx_robot:.3f}m  측면={dy_robot:.3f}m  회전={dyaw_deg:.1f}deg")

    x0, y0, yaw0 = get_current_pose()
    rospy.loginfo(f"현재 위치: x={x0:.3f}, y={y0:.3f}, yaw={math.degrees(yaw0):.1f}deg")

    xt, yt, yawt = compute_target(x0, y0, yaw0, dx_robot, dy_robot, dyaw_deg)

    success = send_goal(xt, yt, yawt)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
