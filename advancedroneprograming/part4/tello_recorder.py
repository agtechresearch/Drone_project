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

# 비디오 코덱 및 파일 설정
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 코덱 설정 (mp4v로 설정)
fps = 30  # 프레임 속도 설정
height, width, _ = frame_read.frame.shape  # 프레임의 크기 얻기
video_writer = cv2.VideoWriter('tello_video.mp4', fourcc, fps, (width, height))

def record_video():
    while True:
        # 프레임 읽기
        frame = frame_read.frame
        if frame is not None:
            
            # (im show 위로 위치변경)프레임을 비디오 파일로 저장
            video_writer.write(frame)
            
            cv2.imshow("Tello Frame", frame)

            """
            # 프레임을 비디오 파일로 저장
            video_writer.write(frame)
            """

            key = cv2.waitKey(1)
            if key == ord('q'):  # 'q' 키를 눌러 종료
                break
        else:
            print("Failed to capture frame")

    video_writer.release()  # 비디오 파일 닫기
    cv2.destroyAllWindows()

# 비디오 녹화 시작
record_video()

# 드론 연결 해제
tello.streamoff()
tello.end()
