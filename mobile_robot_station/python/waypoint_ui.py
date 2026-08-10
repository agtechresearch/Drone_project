#!/usr/bin/env python3
"""
터치스크린 웨이포인트 이동 UI.

config/waypoints.yaml 의 웨이포인트마다 큰 버튼을 만들고, 터치하면 그 위치로
move_base 목표를 전송한다. 정지 버튼으로 언제든 목표 취소 + 로봇 정지.

전제: 3_start_navigation.sh 로 네비게이션 스택(move_base)이 실행 중이어야 함.
      (이 UI는 실행 중인 move_base에 목표만 보냄)

모니터(터치스크린)에 전체화면으로 뜬다.
"""
import os
import math
import subprocess
import yaml
import rospy
import actionlib
import tkinter as tk
from tkinter import messagebox
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

WAYPOINTS_FILE = os.path.join(os.path.dirname(__file__), "..", "config", "waypoints.yaml")
RVIZ_CONFIG = "/home/er/myagv_ros/src/myagv_navigation/rviz/dwa.rviz"

# 배터리 전압 임계값 (V) — 실제 만충/방전 전압 관찰 후 조정하세요.
BATT_FULL_V = 12.6
BATT_LOW_V  = 10.5


class WaypointUI:
    def __init__(self):
        rospy.init_node("waypoint_ui", anonymous=True)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.initpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped,
                                            queue_size=1, latch=True)
        self._voltage = None
        rospy.Subscriber("Voltage", Float32, self._voltage_cb)
        self._rviz_proc = None
        self._rviz_overlay = None
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self._status = "move_base 연결 확인 중..."
        self._connected = self.client.wait_for_server(timeout=rospy.Duration(3.0))
        self._status = "준비됨 — 목적지를 터치하세요" if self._connected \
            else "move_base 연결 안 됨 — 3_start_navigation.sh 실행 확인"

        self.waypoints = self._load_waypoints()
        self._build_ui()

    def _load_waypoints(self):
        try:
            with open(WAYPOINTS_FILE) as f:
                return yaml.safe_load(f).get("waypoints", [])
        except Exception as e:
            self._status = f"waypoints.yaml 읽기 실패: {e}"
            return []

    # ── 목표 전송/정지 ──────────────────────────────────────────────
    def go(self, wp):
        if not (self._connected or self.client.wait_for_server(timeout=rospy.Duration(1.0))):
            self._status = "move_base 연결 안 됨 — 네비게이션 실행 확인"
            return
        self._connected = True

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(wp["x"])
        goal.target_pose.pose.position.y = float(wp["y"])
        q = quaternion_from_euler(0, 0, math.radians(float(wp.get("yaw", 0))))
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        name = wp["name"]
        self.client.send_goal(goal, done_cb=lambda st, res: self._on_done(name, st))
        self._status = f"이동 중 → {name}"

    def _on_done(self, name, state):
        # actionlib GoalStatus: 3=SUCCEEDED, 4=ABORTED, 2/8=취소류
        if state == actionlib.GoalStatus.SUCCEEDED:
            self._status = f"도착 ✓ {name}"
        elif state in (actionlib.GoalStatus.PREEMPTED, actionlib.GoalStatus.RECALLED):
            self._status = f"정지됨 ({name})"
        else:
            self._status = f"실패 ✗ {name} (상태 {state})"

    def stop(self):
        self.client.cancel_all_goals()
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.03)
        self._status = "정지"

    # ── 원점 초기화 (AMCL initialpose) ──────────────────────────────
    def set_origin(self):
        """로봇이 지정 원점에 있다는 전제로 AMCL 초기위치를 (0,0,0)으로 발행."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.orientation.w = 1.0            # yaw=0 (x=y=z=0 기본)
        cov = [0.0] * 36
        cov[0], cov[7], cov[35] = 0.25, 0.25, 0.068  # x, y, yaw (amcl.yaml 초기 공분산)
        msg.pose.covariance = cov
        self.initpose_pub.publish(msg)
        self._status = "원점 초기화 발행됨 (0,0,0) — 위치추정 설정"

    # ── 배터리 ──────────────────────────────────────────────────────
    def _voltage_cb(self, msg):
        self._voltage = msg.data

    def _battery_text_color(self):
        if self._voltage is None:
            return "배터리 --.- V", "#888"
        v = self._voltage
        if v >= BATT_FULL_V - 0.6:
            color = "#2ecc71"   # 양호
        elif v >= BATT_LOW_V:
            color = "#f1c40f"   # 주의
        else:
            color = "#e74c3c"   # 부족
        pct = max(0, min(100, (v - BATT_LOW_V) / (BATT_FULL_V - BATT_LOW_V) * 100))
        return f"배터리 {v:.1f} V (~{pct:.0f}%)", color

    # ── RViz 토글 ───────────────────────────────────────────────────
    def toggle_rviz(self):
        if self._rviz_proc and self._rviz_proc.poll() is None:
            self._close_rviz()
            return
        env = dict(os.environ, DISPLAY=":0", XAUTHORITY="/home/er/.Xauthority")
        self._rviz_proc = subprocess.Popen(
            ["rosrun", "rviz", "rviz", "-d", RVIZ_CONFIG], env=env)
        self.rviz_btn.config(text="RViz 닫기 (↖ 오버레이)")
        self._show_rviz_overlay()

    def _show_rviz_overlay(self):
        # RViz가 전체화면을 덮어도 항상 위에 뜨는 제어 패널 (RViz 닫기 + 정지)
        if self._rviz_overlay is not None:
            return
        win = tk.Toplevel(self.root)
        win.title("RViz 제어")
        win.attributes("-topmost", True)
        win.geometry("+20+20")
        win.configure(bg="#111")
        tk.Button(win, text="✕ RViz 닫기", font=("Sans", 18, "bold"),
                  fg="white", bg="#c0392b", activebackground="#e74c3c",
                  padx=24, pady=14, command=self._close_rviz).pack(side="left", padx=4, pady=4)
        tk.Button(win, text="■ 정지", font=("Sans", 18, "bold"),
                  fg="white", bg="#7f2d2d", activebackground="#c0392b",
                  padx=24, pady=14, command=self.stop).pack(side="left", padx=4, pady=4)
        self._rviz_overlay = win

    def _close_rviz(self):
        if self._rviz_proc and self._rviz_proc.poll() is None:
            self._rviz_proc.terminate()
        self._rviz_proc = None
        if self._rviz_overlay is not None:
            try:
                self._rviz_overlay.destroy()
            except tk.TclError:
                pass
            self._rviz_overlay = None
        self.rviz_btn.config(text="RViz 열기")

    # ── UI ──────────────────────────────────────────────────────────
    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title("Waypoint 이동")
        self.root.attributes("-fullscreen", True)
        self.root.configure(bg="#111")

        top = tk.Frame(self.root, bg="#222")
        top.pack(fill="x", side="top")
        self.status_lbl = tk.Label(top, text=self._status, font=("Sans", 20, "bold"),
                                    fg="#fff", bg="#222", pady=14, wraplength=560)
        self.status_lbl.pack(side="left", fill="x", expand=True)
        self.batt_lbl = tk.Label(top, text="배터리 --.- V", font=("Sans", 20, "bold"),
                                  fg="#888", bg="#222", padx=16)
        self.batt_lbl.pack(side="right")

        grid = tk.Frame(self.root, bg="#111")
        grid.pack(fill="both", expand=True, padx=8, pady=8)

        cols = 2
        colors = ["#2d6cdf", "#2e9e5b", "#c9852a", "#8e44ad", "#16a085", "#c0392b"]
        for i, wp in enumerate(self.waypoints):
            r, c = divmod(i, cols)
            label = f"{wp['name']}\n(x={float(wp['x']):.1f}, y={float(wp['y']):.1f})"
            b = tk.Button(grid, text=label, font=("Sans", 20, "bold"),
                          fg="white", bg=colors[i % len(colors)],
                          activebackground="#555", relief="raised", bd=3,
                          command=lambda w=wp: self.go(w))
            b.grid(row=r, column=c, sticky="nsew", padx=6, pady=6)
        for c in range(cols):
            grid.columnconfigure(c, weight=1)
        for r in range((len(self.waypoints) + cols - 1) // cols):
            grid.rowconfigure(r, weight=1)

        bottom = tk.Frame(self.root, bg="#111")
        bottom.pack(fill="x", side="bottom", padx=8, pady=8)
        tk.Button(bottom, text="■ 정지", font=("Sans", 24, "bold"),
                  fg="white", bg="#c0392b", activebackground="#e74c3c",
                  height=2, command=self.stop).pack(fill="x", side="top")
        row = tk.Frame(bottom, bg="#111")
        row.pack(fill="x", side="top", pady=(6, 0))
        tk.Button(row, text="⌂ 원점 초기화", font=("Sans", 16, "bold"),
                  fg="white", bg="#2c3e50", activebackground="#555",
                  command=self.set_origin).pack(side="left", fill="x", expand=True, padx=(0, 3))
        self.rviz_btn = tk.Button(row, text="RViz 열기", font=("Sans", 16, "bold"),
                                   fg="white", bg="#34495e", command=self.toggle_rviz)
        self.rviz_btn.pack(side="left", fill="x", expand=True, padx=3)
        tk.Button(row, text="종료", font=("Sans", 16),
                  fg="#ddd", bg="#333", command=self._quit).pack(side="right", fill="x", expand=True, padx=(3, 0))

        # 상태 라벨 주기적 갱신 (ROS 콜백 스레드 → Tk 스레드 안전하게 전달)
        self._poll_status()
        # 시작 시 원점 초기화 여부 확인 (창이 뜬 뒤)
        self.root.after(600, self._startup_origin_prompt)

    def _quit(self):
        self._close_rviz()
        self.root.destroy()

    def _startup_origin_prompt(self):
        yes = messagebox.askyesno(
            "초기위치 설정",
            "로봇을 지정 구역(원점)에 같은 방향으로 두셨습니까?\n\n"
            "[예]   → 현재 위치를 맵 원점(0,0,0)으로 초기화합니다.\n"
            "[아니오] → 나중에 '⌂ 원점 초기화' 버튼으로 설정하세요.\n\n"
            "※ 로봇이 원점에 없을 때 '예'를 누르면 위치추정이 틀어집니다.",
            parent=self.root)
        if yes:
            self.set_origin()

    def _poll_status(self):
        # 아직 연결 안 됐으면 주기적으로 move_base 재연결 시도
        if not self._connected and self.client.wait_for_server(rospy.Duration(0.05)):
            self._connected = True
            self._status = "준비됨 — 목적지를 터치하세요"
        # RViz 오버레이: 외부에서 RViz 종료됐으면 정리, 살아있으면 항상 위로 유지
        if self._rviz_overlay is not None:
            if self._rviz_proc is None or self._rviz_proc.poll() is not None:
                self._close_rviz()
            else:
                try:
                    self._rviz_overlay.lift()
                    self._rviz_overlay.attributes("-topmost", True)
                except tk.TclError:
                    pass
        self.status_lbl.config(text=self._status)
        batt_text, batt_color = self._battery_text_color()
        self.batt_lbl.config(text=batt_text, fg=batt_color)
        if not rospy.is_shutdown():
            self.root.after(200, self._poll_status)
        else:
            self.root.destroy()

    def run(self):
        rospy.on_shutdown(lambda: self.root.after(0, self.root.destroy))
        self.root.mainloop()


if __name__ == "__main__":
    WaypointUI().run()
