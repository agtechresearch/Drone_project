import threading
import time
import cv2

# 웹캠 초기화
cap = cv2.VideoCapture(0)  # 0은 기본 웹캠을 의미

if not cap.isOpened():
    print("Cannot open webcam")
    exit()

keepRecording = True

# 약간의 대기 시간 추가 (웹캠 초기화 시간 확보)
time.sleep(2)

# 비디오 코덱 설정
fourcc = cv2.VideoWriter_fourcc(*'mp4v')


def videoRecorder():
    # VideoWriter 객체 생성
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        return

    height, width, _ = frame.shape
    video = cv2.VideoWriter('webcam_video.mp4', fourcc, 30, (width, height))

    # 초기 몇 프레임을 건너뛰기
    for _ in range(5):
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            continue
        time.sleep(1 / 60)

    while keepRecording:
        # 프레임 읽기
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        video.write(frame)

        if frame is None:
            print("Frame is None!")
        else:
            print("Frame is yes!")

        time.sleep(1 / 60)  # 비디오 속도 조절

    video.release()


# 비디오 레코더를 별도 스레드에서 실행
recorder = threading.Thread(target=videoRecorder)
recorder.start()

# 5초 동안 녹화 진행
time.sleep(5)

# 레코딩 종료
keepRecording = False
recorder.join()

# 웹캠 연결 해제
cap.release()
cv2.destroyAllWindows()
