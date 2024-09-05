import time
import cv2
from djitellopy import Tello

# Tello 객체 생성 및 연결
tello = Tello()
tello.connect()

# 스트리밍 활성화
tello.streamon()

# 약간의 대기 시간 추가 (스트리밍 초기화 시간 확보)
time.sleep(3)

# 프레임 읽기 객체
frame_read = tello.get_frame_read()

# 동영상 저장을 위한 설정
width = 960  # 프레임 너비 (Tello의 기본 해상도)
height = 720  # 프레임 높이 (Tello의 기본 해상도)
fps = 30  # 프레임 속도

# 비디오 코덱 설정 및 VideoWriter 객체 생성
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
video_writer = cv2.VideoWriter('tello_video.avi', fourcc, fps, (width, height))

def record_video():
    while True:
        # 프레임 읽기
        frame = frame_read.frame
        if frame is not None:
            cv2.imshow("Tello Frame", frame)
            video_writer.write(frame)  # 현재 프레임을 비디오 파일에 저장
            key = cv2.waitKey(1)
            if key == ord('s'):  # 's' 키를 눌러 사진 저장
                cv2.imwrite('tello_picture.jpg', frame)
                print("Picture taken and saved as tello_picture.jpg")
            elif key == ord('q'):  # 'q' 키를 눌러 종료
                break
        else:
            print("Failed to capture frame")

    # 모든 창 닫기 및 비디오 저장 종료
    cv2.destroyAllWindows()
    video_writer.release()

# 영상 녹화
record_video()
