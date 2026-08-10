#!/usr/bin/env python3
"""
단일 네비게이션 목표 지점 전송 스크립트.
Usage:
    python3 nav_goal_sender.py <x> <y> [yaw_deg]
    python3 nav_goal_sender.py 1.5 2.0 90
"""
import sys
import math
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


def send_goal(x: float, y: float, yaw_deg: float = 0.0) -> bool:
    rospy.init_node('nav_goal_sender', anonymous=True)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("move_base 액션 서버 대기 중...")
    if not client.wait_for_server(timeout=rospy.Duration(10.0)):
        rospy.logerr("move_base 서버에 연결할 수 없습니다. 네비게이션이 실행 중인지 확인하세요.")
        return False

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    yaw_rad = math.radians(yaw_deg)
    q = quaternion_from_euler(0, 0, yaw_rad)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    rospy.loginfo(f"목표 전송: x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}°")
    client.send_goal(goal)

    client.wait_for_result()
    state = client.get_state()

    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("목표 지점 도달 성공!")
        return True
    else:
        rospy.logwarn(f"목표 지점 도달 실패 (상태: {state})")
        return False


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

    success = send_goal(x, y, yaw)
    sys.exit(0 if success else 1)
