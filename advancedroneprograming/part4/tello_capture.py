import time
import cv2
from djitellopy import Tello

# Tello 객체 생성 및 연결
tello = Tello()
tello.connect()

# 스트리밍 활성화
tello.streamon()

# 약간의 대기 시간 추가 (스트리밍 초기화 시간 확보)
time.sleep(5)  # 스트리밍 초기화에 더 많은 시간을 줍니다.

# 프레임 읽기 객체
frame_read = tello.get_frame_read()

def take_picture():
    # 프레임 읽기
    frame = frame_read.frame
    if frame is not None:
        # 사진 파일로 저장
        cv2.imwrite('tello_picture.jpg', frame)
        print("Picture taken and saved as tello_picture.jpg")
    else:
        print("Failed to capture frame")

# 사진 찍기
take_picture()

# 드론 연결 해제
tello.streamoff()
tello.end()
