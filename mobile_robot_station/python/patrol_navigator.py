#!/usr/bin/env python3
"""
다중 웨이포인트 순찰 네비게이터.
config/waypoints.yaml 에 정의된 경로를 반복 순회합니다.

Usage:
    python3 patrol_navigator.py                    # waypoints.yaml 사용
    python3 patrol_navigator.py --once             # 한 바퀴만
    python3 patrol_navigator.py --loops 3          # 3바퀴
    python3 patrol_navigator.py --waypoints a b c  # 특정 웨이포인트만
"""
import sys
import math
import time
import argparse
import yaml
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist

WAYPOINTS_FILE = "/home/er/autonomous_nav/config/waypoints.yaml"


def load_waypoints(names: list = None) -> list:
    with open(WAYPOINTS_FILE, "r") as f:
        data = yaml.safe_load(f)
    waypoints = data.get("waypoints", [])
    if names:
        waypoints = [w for w in waypoints if w["name"] in names]
    return waypoints


def make_goal(wp: dict) -> MoveBaseGoal:
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = wp["x"]
    goal.target_pose.pose.position.y = wp["y"]

    yaw_rad = math.radians(wp.get("yaw", 0.0))
    q = quaternion_from_euler(0, 0, yaw_rad)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    return goal


def stop_robot(pub: rospy.Publisher):
    pub.publish(Twist())


def run_patrol(waypoints: list, loops: int = -1, stop_pub=None):
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("move_base 액션 서버 대기 중...")
    if not client.wait_for_server(timeout=rospy.Duration(15.0)):
        rospy.logerr("move_base 서버 연결 실패")
        return

    loop_count = 0
    total = loops if loops > 0 else float("inf")

    while loop_count < total and not rospy.is_shutdown():
        loop_count += 1
        label = f"루프 {loop_count}" if loops > 0 else f"루프 {loop_count} (무한 순찰)"
        rospy.loginfo(f"=== {label} 시작 ({len(waypoints)}개 웨이포인트) ===")

        for i, wp in enumerate(waypoints):
            if rospy.is_shutdown():
                break

            rospy.loginfo(f"  [{i+1}/{len(waypoints)}] '{wp['name']}' → x={wp['x']:.2f}, y={wp['y']:.2f}")
            goal = make_goal(wp)
            client.send_goal(goal)

            # 대기 시간 계산 (기본 60초 타임아웃)
            timeout = wp.get("timeout", 60.0)
            finished = client.wait_for_result(timeout=rospy.Duration(timeout))

            if not finished:
                rospy.logwarn(f"  '{wp['name']}' 타임아웃 ({timeout}s) - 다음 웨이포인트로 이동")
                client.cancel_goal()
                continue

            state = client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"  '{wp['name']}' 도달 완료")
                wait = wp.get("wait", 0.0)
                if wait > 0:
                    rospy.loginfo(f"  {wait}초 대기...")
                    time.sleep(wait)
            else:
                rospy.logwarn(f"  '{wp['name']}' 도달 실패 (상태: {state})")

    if stop_pub:
        stop_robot(stop_pub)
    rospy.loginfo("순찰 완료")


def main():
    parser = argparse.ArgumentParser(description="다중 웨이포인트 순찰")
    parser.add_argument("--once", action="store_true", help="한 바퀴만 순찰")
    parser.add_argument("--loops", type=int, default=-1, help="순찰 횟수 (-1=무한)")
    parser.add_argument("--waypoints", nargs="+", help="순회할 웨이포인트 이름 목록")
    args = parser.parse_args()

    rospy.init_node("patrol_navigator")
    stop_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    try:
        waypoints = load_waypoints(args.waypoints)
    except FileNotFoundError:
        rospy.logerr(f"웨이포인트 파일 없음: {WAYPOINTS_FILE}")
        sys.exit(1)

    if not waypoints:
        rospy.logerr("웨이포인트가 없습니다. config/waypoints.yaml 을 확인하세요.")
        sys.exit(1)

    rospy.loginfo(f"로드된 웨이포인트: {[w['name'] for w in waypoints]}")

    loops = 1 if args.once else args.loops

    try:
        run_patrol(waypoints, loops=loops, stop_pub=stop_pub)
    except rospy.ROSInterruptException:
        pass
    finally:
        stop_robot(stop_pub)


if __name__ == "__main__":
    main()
