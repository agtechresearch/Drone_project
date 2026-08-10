#!/usr/bin/env python3
# 모터가 어떤 cmd_vel 패턴에 반응하는지 특정 (move_direct는 되는데 move_base는 안 되는 원인 규명)
# 각 패턴을 15Hz로 잠깐 직접 발행하고 odom 반응을 측정. 제자리 회전 위주 + 짧은 전진.
# ※ 로봇 앞뒤로 ~0.5m 공간 확보 후 실행.
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

st = {"odom": 0.0}
def odom_cb(m):
    t = m.twist.twist
    st["odom"] = max(st["odom"], abs(t.linear.x)+abs(t.linear.y)+abs(t.angular.z))

rospy.init_node("drive_probe", anonymous=True)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
rospy.Subscriber("/odom", Odometry, odom_cb)
rospy.sleep(0.8)

PATTERNS = [
    ("순수회전 az=0.40",       0.0, 0.0, 0.40, 2.5),
    ("순수회전 az=0.25",       0.0, 0.0, 0.25, 2.5),
    ("순수전진 lx=0.12",       0.12, 0.0, 0.0, 1.8),
    ("전진+회전 lx=0.10,az=0.30", 0.10, 0.0, 0.30, 2.0),
    ("약한mix lx=0.06,az=0.30",  0.06, 0.0, 0.30, 2.0),
]
def stop():
    for _ in range(6):
        pub.publish(Twist()); rospy.sleep(0.03)

print("=== 모터 반응 프로브 시작 (각 패턴 사이 정지) ===")
rate = rospy.Rate(15)
for name, vx, vy, wz, dur in PATTERNS:
    st["odom"] = 0.0
    t = Twist(); t.linear.x = vx; t.linear.y = vy; t.angular.z = wz
    n = int(dur * 15)
    for _ in range(n):
        if rospy.is_shutdown(): break
        pub.publish(t); rate.sleep()
    stop()
    resp = "움직임 ✓ (odom %.2f)" % st["odom"] if st["odom"] > 0.05 else "반응없음 ✗ (odom %.2f)" % st["odom"]
    print(f"  [{name:<26}] → {resp}")
    rospy.sleep(1.2)
stop()
print("=== 완료 ===")
