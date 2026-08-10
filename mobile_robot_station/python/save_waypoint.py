#!/usr/bin/env python3
"""
현재 로봇 위치를 웨이포인트로 저장/관리.
AMCL 실행 중이면 map 프레임, 아니면 odom 프레임 위치를 사용합니다.

사용법:
    python3 save_waypoint.py <이름>                    # 현재 위치 저장
    python3 save_waypoint.py <이름> --wait 3.0         # 도착 후 대기 시간 지정
    python3 save_waypoint.py <이름> --timeout 90       # 이동 타임아웃 지정
    python3 save_waypoint.py --list                    # 저장된 웨이포인트 목록
    python3 save_waypoint.py --delete <이름>           # 웨이포인트 삭제
"""
import sys
import math
import argparse
import yaml
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

WAYPOINTS_FILE = "/home/er/autonomous_nav/config/waypoints.yaml"


# ── YAML 읽기/쓰기 ──────────────────────────────────────────────────────────

def load_yaml():
    try:
        with open(WAYPOINTS_FILE, "r") as f:
            data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        data = {}
    if "waypoints" not in data or data["waypoints"] is None:
        data["waypoints"] = []
    return data


def save_yaml(data):
    with open(WAYPOINTS_FILE, "w") as f:
        f.write("# 자율주행 웨이포인트 설정 파일\n")
        f.write("# save_waypoint.sh 로 자동 생성/수정됩니다.\n\n")
        f.write("waypoints:\n")
        for wp in data.get("waypoints", []):
            f.write(f"  - name: \"{wp['name']}\"\n")
            f.write(f"    x: {wp['x']:.4f}\n")
            f.write(f"    y: {wp['y']:.4f}\n")
            f.write(f"    yaw: {wp.get('yaw', 0.0):.1f}\n")
            f.write(f"    wait: {wp.get('wait', 2.0):.1f}\n")
            f.write(f"    timeout: {wp.get('timeout', 60.0):.1f}\n")
            f.write("\n")


# ── 현재 위치 취득 ──────────────────────────────────────────────────────────

def get_pose_amcl(timeout=3.0):
    """AMCL map 프레임 위치 (네비게이션 실행 중)."""
    try:
        msg = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped,
                                     timeout=timeout)
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        return pos.x, pos.y, math.degrees(yaw), "map"
    except rospy.ROSException:
        return None


def get_pose_odom(timeout=5.0):
    """odom 프레임 위치 (폴백)."""
    try:
        msg = rospy.wait_for_message("/odom", Odometry, timeout=timeout)
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        return pos.x, pos.y, math.degrees(yaw), "odom"
    except rospy.ROSException:
        rospy.logerr("위치 수신 실패. myagv_odometry가 실행 중인지 확인하세요.")
        sys.exit(1)


def get_current_pose():
    result = get_pose_amcl()
    if result:
        return result
    rospy.logwarn("AMCL 위치 없음 → odom 프레임으로 폴백")
    return get_pose_odom()


# ── 명령 처리 ────────────────────────────────────────────────────────────────

def cmd_save(name, wait, timeout):
    x, y, yaw_deg, frame = get_current_pose()
    rospy.loginfo(f"현재 위치 ({frame}): x={x:.4f}, y={y:.4f}, yaw={yaw_deg:.1f}deg")

    data = load_yaml()
    waypoints = data["waypoints"]

    existing = next((w for w in waypoints if w["name"] == name), None)
    if existing:
        old = f"x={existing['x']:.4f}, y={existing['y']:.4f}"
        existing.update({"x": round(x, 4), "y": round(y, 4),
                         "yaw": round(yaw_deg, 1), "wait": wait, "timeout": timeout})
        print(f"[수정] '{name}': {old} → x={x:.4f}, y={y:.4f}")
    else:
        waypoints.append({"name": name, "x": round(x, 4), "y": round(y, 4),
                           "yaw": round(yaw_deg, 1), "wait": wait, "timeout": timeout})
        print(f"[저장] '{name}': x={x:.4f}, y={y:.4f}, yaw={yaw_deg:.1f}deg")

    save_yaml(data)
    print(f"→ {WAYPOINTS_FILE} 저장 완료")


def cmd_list():
    data = load_yaml()
    waypoints = data.get("waypoints", [])
    if not waypoints:
        print("저장된 웨이포인트가 없습니다.")
        return
    print(f"{'이름':<12}  {'x':>8}  {'y':>8}  {'yaw':>7}  {'wait':>5}  {'timeout':>8}")
    print("-" * 58)
    for wp in waypoints:
        print(f"{wp['name']:<12}  {wp['x']:>8.4f}  {wp['y']:>8.4f}"
              f"  {wp.get('yaw',0):>6.1f}d  {wp.get('wait',2):>5.1f}s"
              f"  {wp.get('timeout',60):>7.1f}s")


def cmd_delete(name):
    data = load_yaml()
    before = len(data["waypoints"])
    data["waypoints"] = [w for w in data["waypoints"] if w["name"] != name]
    if len(data["waypoints"]) == before:
        print(f"[ERROR] '{name}' 웨이포인트가 없습니다.")
        sys.exit(1)
    save_yaml(data)
    print(f"[삭제] '{name}' 완료")


# ── 진입점 ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="웨이포인트 저장/관리")
    parser.add_argument("name", nargs="?", help="웨이포인트 이름")
    parser.add_argument("--wait",    type=float, default=2.0,  help="도착 후 대기 시간(초)")
    parser.add_argument("--timeout", type=float, default=60.0, help="이동 타임아웃(초)")
    parser.add_argument("--list",    action="store_true",       help="웨이포인트 목록 출력")
    parser.add_argument("--delete",  metavar="NAME",            help="웨이포인트 삭제")
    args = parser.parse_args()

    if args.list:
        cmd_list()
        return

    if args.delete:
        cmd_delete(args.delete)
        return

    if not args.name:
        parser.print_help()
        sys.exit(1)

    rospy.init_node("save_waypoint", anonymous=True)
    cmd_save(args.name, args.wait, args.timeout)


if __name__ == "__main__":
    main()
