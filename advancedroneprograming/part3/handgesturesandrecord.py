import time
import cv2
import cvzone
from cvzone.HandTrackingModule import HandDetector
from cvzone.FaceDetectionModule import FaceDetector
from djitellopy import Tello

# 손 및 얼굴 인식 모듈 초기화
detectorhand = HandDetector(maxHands=1)
detectorface = FaceDetector()
gesture = ""

# Tello 드론 객체 생성 및 연결
me = Tello()
me.connect()
print(me.get_battery())

# 스트리밍 및 비디오 녹화 초기화
me.streamoff()
me.streamon()

# 드론 이륙
me.takeoff()
me.move_up(50)

# 동영상 저장을 위한 설정
width = 960  # 프레임 너비 (Tello의 기본 해상도)
height = 720  # 프레임 높이 (Tello의 기본 해상도)
fps = 30  # 프레임 속도

# 비디오 코덱 설정 및 VideoWriter 객체 생성
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
video_writer = cv2.VideoWriter('tello_video2.avi', fourcc, fps, (width, height))

while True:
    # 프레임 읽기
    img = me.get_frame_read().frame
    img = cv2.resize(img, (640, 480))

    # 손과 얼굴 인식
    hands, img = detectorhand.findHands(img, draw=True)
    img, bboxs = detectorface.findFaces(img, draw=True)

    if bboxs:
        x, y, w, h = bboxs[0]["bbox"]
        bboxregion = x - 175 - 25, y - 75, 175, h + 75
        cvzone.cornerRect(img, bboxregion, rt=0, t=10, colorC=(0, 0, 255))

        if hands and hands[0]["type"] == "Right":
            handinfo = hands[0]
            handcenter = handinfo["center"]

            inside = bboxregion[0] < handcenter[0] < bboxregion[0] + bboxregion[2] and bboxregion[1] < handcenter[1] < \
                     bboxregion[1] + bboxregion[3]

            if inside:
                cvzone.cornerRect(img, bboxregion, rt=0, t=10, colorC=(0, 255, 0))
                fingers = detectorhand.fingersUp(handinfo)
                if fingers == [1, 1, 1, 1, 1]:
                    gesture = "  Open"
                elif fingers == [0, 1, 0, 0, 0]:
                    gesture = "  UP"
                    me.move_up(20)
                elif fingers == [0, 0, 0, 0, 0]:
                    gesture = "  STOP"
                elif fingers == [0, 0, 1, 0, 0]:
                    gesture = "  Middle"
                elif fingers == [1, 1, 0, 0, 1]:
                    gesture = "FLIP"
                    me.flip_forward()
                elif fingers == [0, 1, 1, 0, 0]:
                    gesture = " DOWN"
                    me.move_down(20)
                elif fingers == [0, 0, 0, 0, 1]:
                    gesture = "  LEFT"
                    me.move_left(30)
                elif fingers == [1, 0, 0, 0, 0]:
                    gesture = "  RIGHT"
                    me.move_right(30)
                elif fingers == [0, 1, 1, 1, 1]:
                    gesture = "  forward"
                    me.move_forward(30)
                elif fingers == [0, 1, 1, 1, 0]:
                    gesture = "  back"
                    me.move_back(30)

                # 제스처 텍스트 표시
                cv2.rectangle(img, (bboxregion[0], bboxregion[1] + bboxregion[3] + 10),
                              (bboxregion[0] + bboxregion[2], bboxregion[1] + bboxregion[3] + 60),
                              (0, 255, 0), cv2.FILLED)
                cv2.putText(img, f'{gesture}',
                            (bboxregion[0] + 10, bboxregion[1] + bboxregion[3] + 50),
                            cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

    # 프레임을 비디오 파일에 저장
    video_writer.write(cv2.resize(img, (width, height)))

    # 프레임을 화면에 표시
    cv2.imshow("Tello Video Stream", img)
    if cv2.waitKey(5) & 0xFF == ord('q'):
        me.land()
        break

# 모든 창 닫기 및 비디오 저장 종료
cv2.destroyAllWindows()
video_writer.release()
