#!/usr/bin/env python3
"""
카메라 기반 장애물 감지 노드.
OpenCV로 라즈베리파이 카메라(/dev/video0) 영상을 처리하여
전방 근거리 장애물을 감지하고 /obstacle_detected 토픽으로 발행합니다.

장애물 감지 시 move_base를 취소하고 로봇을 정지합니다.
"""
import cv2
import numpy as np
import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import actionlib
from move_base_msgs.msg import MoveBaseAction

# 감지 설정
CAMERA_INDEX = 0
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240
FPS          = 10

# 장애물 판단 파라미터 (ROI: 화면 중앙 하단 60%)
ROI_TOP_RATIO    = 0.4   # ROI 시작 (화면 40% 지점)
MIN_OBSTACLE_AREA = 3000  # 픽셀 수 (이 이상이면 장애물)

# 배경 차분 파라미터
BG_HISTORY       = 200
BG_THRESHOLD     = 25


class CameraObstacleDetector:
    def __init__(self):
        rospy.init_node("camera_obstacle_detector")

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FPS)

        if not self.cap.isOpened():
            rospy.logwarn(f"카메라 {CAMERA_INDEX} 열기 실패 — 장애물 감지 비활성화")
            self.enabled = False
            return
        self.enabled = True

        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=BG_HISTORY, varThreshold=BG_THRESHOLD, detectShadows=False
        )

        self.obstacle_pub = rospy.Publisher("/obstacle_detected", Bool, queue_size=1)
        self.image_pub    = rospy.Publisher("/camera/obstacle_view", Image, queue_size=1)
        self.cmd_pub      = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.move_base_client = None
        self._init_move_base_client()

        self.obstacle_count = 0  # 연속 감지 프레임 수
        self.CONFIRM_FRAMES = 3  # 이 프레임 이상 연속 감지 시 장애물로 판정

        rospy.loginfo("카메라 장애물 감지 노드 시작")

    def _init_move_base_client(self):
        try:
            self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            # 비동기 연결 (실패해도 노드는 계속 실행)
            self.move_base_client.wait_for_server(timeout=rospy.Duration(3.0))
        except Exception:
            self.move_base_client = None

    def detect(self, frame: np.ndarray) -> tuple[bool, np.ndarray]:
        """
        반환: (장애물_감지, 시각화_프레임)
        """
        h, w = frame.shape[:2]
        roi_top = int(h * ROI_TOP_RATIO)
        roi = frame[roi_top:h, :]  # 중앙 하단 영역

        # 그레이스케일 + 블러
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 배경 차분으로 움직이는 객체 감지
        fg_mask = self.bg_subtractor.apply(blurred)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN,
                                   cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))

        # 근거리 정적 장애물: 엣지 기반 감지 (배경 차분 보완)
        edges = cv2.Canny(blurred, 50, 150)
        edges_dilated = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=2)

        # 두 마스크 결합
        combined = cv2.bitwise_or(fg_mask, edges_dilated)

        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        obstacle = False
        viz = frame.copy()
        cv2.line(viz, (0, roi_top), (w, roi_top), (0, 255, 255), 1)  # ROI 경계선

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= MIN_OBSTACLE_AREA:
                obstacle = True
                x, y, bw, bh = cv2.boundingRect(cnt)
                cv2.rectangle(viz, (x, y + roi_top), (x + bw, y + bh + roi_top), (0, 0, 255), 2)

        status_color = (0, 0, 255) if obstacle else (0, 255, 0)
        status_text  = "OBSTACLE!" if obstacle else "Clear"
        cv2.putText(viz, status_text, (5, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, status_color, 2)
        return obstacle, viz

    def handle_obstacle(self, detected: bool):
        self.obstacle_pub.publish(Bool(data=detected))

        if detected:
            self.obstacle_count += 1
            if self.obstacle_count >= self.CONFIRM_FRAMES:
                rospy.logwarn("장애물 감지! 로봇 정지")
                if self.move_base_client:
                    self.move_base_client.cancel_all_goals()
                self.cmd_pub.publish(Twist())  # 정지
        else:
            self.obstacle_count = max(0, self.obstacle_count - 1)

    def run(self):
        if not self.enabled:
            rospy.spin()
            return

        rate = rospy.Rate(FPS)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn_throttle(5, "카메라 프레임 읽기 실패")
                rate.sleep()
                continue

            detected, viz = self.detect(frame)
            self.handle_obstacle(detected)

            # 시각화 이미지 발행 (rviz / rqt_image_view 로 확인 가능)
            try:
                img_msg = self.bridge.cv2_to_imgmsg(viz, encoding="bgr8")
                self.image_pub.publish(img_msg)
            except Exception:
                pass

            rate.sleep()

        self.cap.release()

    def __del__(self):
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()


if __name__ == "__main__":
    detector = CameraObstacleDetector()
    try:
        detector.run()
    except rospy.ROSInterruptException:
        pass
