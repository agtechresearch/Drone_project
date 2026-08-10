#!/usr/bin/env python3
"""
카메라 기반 바닥 테이프 라인 추종.
OpenCV로 라즈베리파이 카메라(/dev/video0) 영상을 처리하여
전방 바닥의 테이프 라인을 검출하고 PID로 조향합니다.

기본은 "어두운 테이프(검정 등) vs 밝은 바닥" 가정의 적응형 이진화입니다.
테이프가 특정 색(예: 노랑/빨강)이면 LINE_COLOR_MODE를 "hsv"로 바꾸고
HSV_LOWER/HSV_UPPER를 실측값으로 맞추세요 (--debug로 마스크 확인).

사용법:
  line_follow.py                 # 주행 시작
  line_follow.py --debug         # 로컬 창(cv2.imshow)에 원본/마스크/조향 시각화
  line_follow.py --speed 0.10    # 전진 속도 지정
  line_follow.py --no-drive      # cmd_vel 발행 없이 검출만 확인 (튜닝용)
"""
import os
import sys
import json
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ════════════════════════════════════════════════════
#  ★ 사용자 파라미터 — 여기서 수정
# ════════════════════════════════════════════════════

CAMERA_INDEX = 0
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240
FPS          = 15

# ROI: 화면 하단 일부만 사용 (바로 앞 바닥)
ROI_TOP_RATIO = 0.55   # 화면 55% 지점부터 하단까지를 라인 검출에 사용

# 라인 색 검출 방식: "dark"(어두운 라인, 적응형/추천) | "hsv" | "lab" (고정 범위)
LINE_COLOR_MODE = "dark"

# dark 모드: Otsu 임계값에 오프셋을 더해 "가장 어두운 영역"만 남김
# python/line_calibrate.py 로 라인/바닥을 샘플링하면 자동 추천값이 계산됨
DARK_OFFSET = 25

# hsv/lab 모드: 고정 범위. line_calibrate.py 로 채워지는 값이며 조명이
# 크게 변하는 구간에서는 "dark" 모드보다 불안정할 수 있음
HSV_LOWER = np.array([0,   0,   0])
HSV_UPPER = np.array([180, 255, 60])
LAB_LOWER = np.array([0,   0,   0])
LAB_UPPER = np.array([60,  255, 255])

# line_calibrate.py 저장 파일 — 있으면 위 값들을 자동으로 덮어씀
CALIB_PATH = os.path.join(os.path.dirname(__file__), "..", "config", "line_calib.json")

MIN_LINE_AREA = 200     # 이 미만이면 노이즈로 무시

# 주행 속도
FORWARD_SPEED     = 0.12   # m/s  직진 속도
CURVE_SLOWDOWN    = 0.5    # 오차가 클 때 곱해지는 최소 속도 비율

# PID (조향 = angular.z)
KP = 0.9
KD = 0.25
KI = 0.05
INTEGRAL_CLAMP    = 1.0
INTEGRAL_DEADZONE = 0.03   # 정규화 오차 기준

MAX_YAW = 0.8   # rad/s

# 라인을 놓쳤을 때: 마지막으로 본 방향으로 제자리 탐색 회전
LOST_SEARCH_YAW      = 0.35   # rad/s
LOST_GRACE_FRAMES    = 5      # 이 프레임까지는 직전 명령 유지
LOST_STOP_AFTER_SEC  = 3.0    # 그래도 못 찾으면 정지


class LineFollower:
    def __init__(self, drive=True, debug=False, fspeed=FORWARD_SPEED):
        rospy.init_node("line_follow")

        self.drive = drive
        self.debug = debug
        self.fspeed = fspeed

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FPS)

        if not self.cap.isOpened():
            rospy.logerr(f"카메라 {CAMERA_INDEX} 열기 실패")
            self.enabled = False
            return
        self.enabled = True

        self.cmd_pub   = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.image_pub = rospy.Publisher("/camera/line_view", Image, queue_size=1)

        self.prev_error   = 0.0
        self.integral      = 0.0
        self.last_yaw      = 0.0
        self.lost_count     = 0
        self.lost_since     = None

        rospy.loginfo(f"라인 추종 노드 시작 (drive={drive}, mode={LINE_COLOR_MODE})")

    def detect_line(self, frame: np.ndarray):
        """반환: (found, error, viz) — error는 화면 중앙 기준 [-1, 1] 정규화 오차"""
        h, w = frame.shape[:2]
        roi_top = int(h * ROI_TOP_RATIO)
        roi = frame[roi_top:h, :]

        if LINE_COLOR_MODE == "hsv":
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
        elif LINE_COLOR_MODE == "lab":
            lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
            mask = cv2.inRange(lab, LAB_LOWER, LAB_UPPER)
        else:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            otsu_val, _ = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            thresh = max(0, otsu_val - DARK_OFFSET)
            _, mask = cv2.threshold(blurred, thresh, 255, cv2.THRESH_BINARY_INV)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                 cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                 cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        viz = frame.copy()
        cv2.line(viz, (0, roi_top), (w, roi_top), (0, 255, 255), 1)
        center_x = w // 2
        cv2.line(viz, (center_x, roi_top), (center_x, h), (255, 255, 0), 1)

        if not contours:
            return False, 0.0, viz

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < MIN_LINE_AREA:
            return False, 0.0, viz

        m = cv2.moments(largest)
        if m["m00"] == 0:
            return False, 0.0, viz
        cx = int(m["m10"] / m["m00"])
        cy = int(m["m01"] / m["m00"])

        error = (cx - center_x) / float(center_x)  # -1(왼쪽) ~ +1(오른쪽)

        cv2.drawContours(viz, [largest], -1, (0, 255, 0), 2, offset=(0, roi_top))
        cv2.circle(viz, (cx, cy + roi_top), 5, (0, 0, 255), -1)
        cv2.putText(viz, f"err={error:+.2f}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2)

        return True, error, viz

    def compute_yaw(self, error, dt):
        if abs(error) > INTEGRAL_DEADZONE:
            self.integral += error * dt
        else:
            self.integral *= 0.9
        self.integral = max(-INTEGRAL_CLAMP, min(INTEGRAL_CLAMP, self.integral))

        d_error = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        yaw = KP * error + KD * d_error + KI * self.integral
        return max(-MAX_YAW, min(MAX_YAW, yaw))

    def handle_lost(self):
        """라인을 못 찾았을 때: 잠시 직전 명령 유지 후 탐색 회전, 오래 지속되면 정지."""
        self.lost_count += 1
        now = rospy.get_time()
        if self.lost_since is None:
            self.lost_since = now

        if self.lost_count <= LOST_GRACE_FRAMES:
            return self.fspeed * 0.5, self.last_yaw

        if now - self.lost_since >= LOST_STOP_AFTER_SEC:
            rospy.logwarn_throttle(1.0, "라인 미검출 지속 — 정지")
            return 0.0, 0.0

        search_yaw = LOST_SEARCH_YAW if self.last_yaw >= 0 else -LOST_SEARCH_YAW
        rospy.loginfo_throttle(1.0, "라인 탐색 회전 중...")
        return 0.0, search_yaw

    def run(self):
        if not self.enabled:
            return

        rate = rospy.Rate(FPS)
        dt = 1.0 / FPS

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn_throttle(5, "카메라 프레임 읽기 실패")
                rate.sleep()
                continue

            found, error, viz = self.detect_line(frame)

            if found:
                self.lost_count = 0
                self.lost_since = None
                yaw = self.compute_yaw(error, dt)
                speed_scale = max(CURVE_SLOWDOWN, 1.0 - abs(error))
                vx = self.fspeed * speed_scale
                self.last_yaw = yaw
                status = f"라인 err={error:+.3f} vx={vx:.3f} wz={yaw:+.3f}"
            else:
                vx, yaw = self.handle_lost()
                status = f"라인 미검출 (lost={self.lost_count}) vx={vx:.3f} wz={yaw:+.3f}"

            rospy.loginfo_throttle(0.5, status)

            if self.drive:
                cmd = Twist()
                cmd.linear.x  = vx
                cmd.angular.z = yaw
                self.cmd_pub.publish(cmd)

            try:
                img_msg = self.bridge.cv2_to_imgmsg(viz, encoding="bgr8")
                self.image_pub.publish(img_msg)
            except Exception:
                pass

            if self.debug:
                cv2.imshow("line_follow", viz)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            rate.sleep()

        self.cap.release()
        if self.debug:
            cv2.destroyAllWindows()

    def __del__(self):
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()


def load_calibration():
    """python/line_calibrate.py 로 저장된 값을 읽어 전역 상수를 덮어씀."""
    global LINE_COLOR_MODE, DARK_OFFSET, HSV_LOWER, HSV_UPPER, LAB_LOWER, LAB_UPPER

    if not os.path.exists(CALIB_PATH):
        return False

    with open(CALIB_PATH) as f:
        data = json.load(f)

    LINE_COLOR_MODE = data.get("mode", LINE_COLOR_MODE)
    if data.get("dark_offset") is not None:
        DARK_OFFSET = data["dark_offset"]
    if "hsv_lower" in data:
        HSV_LOWER = np.array(data["hsv_lower"])
        HSV_UPPER = np.array(data["hsv_upper"])
    if "lab_lower" in data:
        LAB_LOWER = np.array(data["lab_lower"])
        LAB_UPPER = np.array(data["lab_upper"])
    return True


def main():
    args = sys.argv[1:]
    drive = "--no-drive" not in args
    debug = "--debug" in args
    fspeed = FORWARD_SPEED
    if "--speed" in args:
        fspeed = float(args[args.index("--speed") + 1])

    follower = LineFollower(drive=drive, debug=debug, fspeed=fspeed)

    if "--no-calib" in args:
        rospy.loginfo("--no-calib 지정 — 캘리브레이션 파일 무시, 코드 기본값 사용")
    elif load_calibration():
        rospy.loginfo(f"캘리브레이션 로드: {CALIB_PATH} (mode={LINE_COLOR_MODE}, dark_offset={DARK_OFFSET})")
    else:
        rospy.logwarn(f"캘리브레이션 파일 없음 ({CALIB_PATH}) — 코드 기본값 사용. line_calibrate.py로 먼저 캘리브레이션 권장")

    def emergency_stop():
        for _ in range(5):
            follower.cmd_pub.publish(Twist())
            rospy.sleep(0.05)

    rospy.on_shutdown(emergency_stop)

    try:
        follower.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
