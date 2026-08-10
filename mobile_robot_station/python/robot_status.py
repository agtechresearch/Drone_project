#!/usr/bin/env python3
"""
로봇 상태 모니터 - 터미널에서 주요 토픽을 실시간으로 표시합니다.
Usage: python3 robot_status.py
"""
import math
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from actionlib_msgs.msg import GoalStatusArray


class RobotStatus:
    def __init__(self):
        rospy.init_node("robot_status_monitor", anonymous=True)

        self.odom_x = self.odom_y = self.odom_yaw = 0.0
        self.vel_x = self.vel_y = self.vel_th = 0.0
        self.scan_min = float("inf")
        self.nav_status = "대기"
        self.obstacle = False
        self.voltage = 0.0
        self.voltage_backup = 0.0

        rospy.Subscriber("/odom",              Odometry,        self._odom_cb)
        rospy.Subscriber("/cmd_vel",            Twist,           self._vel_cb)
        rospy.Subscriber("/scan",              LaserScan,       self._scan_cb)
        rospy.Subscriber("/move_base/status",  GoalStatusArray, self._nav_cb)
        rospy.Subscriber("/obstacle_detected", Bool,            self._obs_cb)
        rospy.Subscriber("/Voltage",           Float32,         self._volt_cb)
        rospy.Subscriber("/voltage_backup",    Float32,         self._volt_backup_cb)

        rospy.Timer(rospy.Duration(0.5), self._print_status)
        rospy.loginfo("상태 모니터 시작 (Ctrl+C 종료)")
        rospy.spin()

    def _odom_cb(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.degrees(math.atan2(siny, cosy))

    def _vel_cb(self, msg: Twist):
        self.vel_x  = msg.linear.x
        self.vel_y  = msg.linear.y
        self.vel_th = msg.angular.z

    def _scan_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        self.scan_min = min(valid) if valid else float("inf")

    def _nav_cb(self, msg: GoalStatusArray):
        if not msg.status_list:
            self.nav_status = "대기"
            return
        last = msg.status_list[-1]
        status_map = {0: "대기", 1: "활성(이동중)", 2: "취소됨", 3: "완료", 4: "중단됨"}
        self.nav_status = status_map.get(last.status, str(last.status))

    def _obs_cb(self, msg: Bool):
        self.obstacle = msg.data

    def _volt_cb(self, msg: Float32):
        self.voltage = msg.data

    def _volt_backup_cb(self, msg: Float32):
        self.voltage_backup = msg.data

    @staticmethod
    def _volt_to_pct(v: float) -> str:
        if v <= 0:
            return "N/A"
        pct = max(0.0, min(100.0, (v - 9) / (12 - 9) * 100))
        return f"{v:.1f}V({pct:.0f}%)"

    def _print_status(self, _event):
        obs_flag = " [!장애물]" if self.obstacle else ""
        lidar_disp = f"{self.scan_min:.2f}m" if self.scan_min < 9999 else "N/A"
        print(
            f"\r위치: ({self.odom_x:+.2f}, {self.odom_y:+.2f}) yaw={self.odom_yaw:+.1f}° | "
            f"속도: vx={self.vel_x:+.2f} vy={self.vel_y:+.2f} th={self.vel_th:+.2f} | "
            f"LiDAR: {lidar_disp} | "
            f"배터리: {self._volt_to_pct(self.voltage)} 백업: {self._volt_to_pct(self.voltage_backup)} | "
            f"네비: {self.nav_status}{obs_flag}          ",
            end="", flush=True
        )


if __name__ == "__main__":
    try:
        RobotStatus()
    except rospy.ROSInterruptException:
        print()
