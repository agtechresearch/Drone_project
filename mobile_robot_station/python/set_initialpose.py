#!/usr/bin/env python3
"""
AMCL 초기위치(/initialpose)를 CLI로 발행 (RViz "2D Pose Estimate" 대체).

네비게이션(AMCL) 실행 중에, 로봇의 현재 map 좌표를 알려줘 위치추정을 확정한다.
발행 후 /amcl_pose 가 뜨는지 확인해 성공 여부를 보고한다.

사용법:
    python3 set_initialpose.py                 # 원점 (0, 0, 0deg)
    python3 set_initialpose.py 1.5 2.0 90      # x=1.5m, y=2.0m, yaw=90deg
    python3 set_initialpose.py 1.5 2.0         # yaw 생략 시 0deg

※ 로봇을 그 좌표·방향에 물리적으로 맞춰 둔 상태에서 발행해야 한다.
  가장 흔한 경우: 로봇을 매핑 시작점(원점)에 같은 방향으로 두고 인자 없이 실행.
"""
import sys
import math
import argparse
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

# amcl.yaml 의 initial_cov 값과 동일 (xx, yy, aa)
COV_XX = 0.25
COV_YY = 0.25
COV_AA = 0.068


def build_msg(x, y, yaw_deg):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    yaw = math.radians(yaw_deg)
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    cov = [0.0] * 36
    cov[0] = COV_XX      # x-x
    cov[7] = COV_YY      # y-y
    cov[35] = COV_AA     # yaw-yaw
    msg.pose.covariance = cov
    return msg


def wait_for_amcl(pub, timeout=5.0):
    """AMCL(=/initialpose 구독자)이 붙을 때까지 대기. 안 붙으면 False."""
    rate = rospy.Rate(10)
    waited = 0
    while pub.get_num_connections() < 1 and waited < int(timeout * 10):
        if rospy.is_shutdown():
            return False
        rate.sleep()
        waited += 1
    return pub.get_num_connections() >= 1


def main():
    ap = argparse.ArgumentParser(description="AMCL 초기위치 발행")
    ap.add_argument("x", nargs="?", type=float, default=0.0, help="map x (m)")
    ap.add_argument("y", nargs="?", type=float, default=0.0, help="map y (m)")
    ap.add_argument("yaw", nargs="?", type=float, default=0.0, help="yaw (deg)")
    args = ap.parse_args()

    rospy.init_node("set_initialpose", anonymous=True)
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped,
                          queue_size=1, latch=True)

    print(f"[발행] initialpose: x={args.x:.3f}, y={args.y:.3f}, yaw={args.yaw:.1f}deg (frame=map)")
    if not wait_for_amcl(pub):
        print("[경고] /initialpose 구독자(AMCL) 없음 — 네비게이션이 실행 중인지 확인하세요.")
        # 그래도 latch로 발행은 해둔다 (나중에 AMCL이 붙으면 받음)

    pub.publish(build_msg(args.x, args.y, args.yaw))
    rospy.sleep(1.5)

    # 검증: /amcl_pose 가 반영됐는지
    try:
        ap_msg = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=8.0)
        p = ap_msg.pose.pose.position
        o = ap_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        c = ap_msg.pose.covariance
        print(f"[확인] /amcl_pose: x={p.x:.3f}, y={p.y:.3f}, yaw={math.degrees(yaw):.1f}deg "
              f"(cov xx={c[0]:.3f} yy={c[7]:.3f} aa={c[35]:.3f})")
        err = math.hypot(p.x - args.x, p.y - args.y)
        if err < 0.3:
            print(f"[OK] 요청 위치와 {err*100:.0f}cm 이내로 일치 → 초기위치 설정 완료.")
        else:
            print(f"[주의] 요청 위치와 {err:.2f}m 차이 — 로봇 물리 위치/방향을 다시 확인하세요.")
    except rospy.ROSException:
        print("[실패] /amcl_pose 응답 없음 — AMCL이 initialpose를 못 받았거나 미실행. "
              "네비게이션 상태를 확인하세요.")
        sys.exit(1)


if __name__ == "__main__":
    main()
