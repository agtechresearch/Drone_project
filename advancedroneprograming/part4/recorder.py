import threading
import time
import cv2
from djitellopy import Tello

if hasattr(cv2, 'VideoWriter_fourcc'):
    print("cv2.VideoWriter_fourcc is available")
else:
    print("cv2.VideoWriter_fourcc is not available")

# Tello 객체 생성 및 연결
tello = Tello()
tello.connect()

keepRecording = True

# 스트리밍 활성화
tello.streamon()

# 약간의 대기 시간 추가 (스트리밍 초기화 시간 확보)
time.sleep(2)

# 프레임 읽기 객체
frame_read = tello.get_frame_read()
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 30  # 비디오의 FPS 설정

def videoRecorder():
    # VideoWriter 객체 생성
    height, width, _ = frame_read.frame.shape
    video = cv2.VideoWriter('tello_video.mp4', fourcc, fps, (width, height))

    # 초기 몇 프레임을 건너뛰기
    for _ in range(5):
        _ = frame_read.frame
        time.sleep(1 / fps)

    while keepRecording:
        # 프레임 복사 후 저장
        frame = frame_read.frame
        if frame is not None:
            video.write(frame)
            print("Frame is yes!")
        else:
            print("Frame is None!")

        time.sleep(1 / fps)  # 비디오 속도 조절

    video.release()

# 비디오 레코더를 별도 스레드에서 실행
recorder = threading.Thread(target=videoRecorder)
recorder.start()

# 5초 동안 녹화 진행
time.sleep(5)

# 레코딩 종료
keepRecording = False
recorder.join()

# 드론 연결 해제
tello.streamoff()
tello.end()
