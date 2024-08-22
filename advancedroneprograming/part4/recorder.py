import time, cv2
from threading import Thread
from djitellopy import Tello

tello = Tello()

tello.connect()
tello.streamoff()
time.sleep(1)
tello.streamon()
time.sleep(1)
keepRecording = True

frame_read = tello.get_frame_read()

def videoRecorder():
    # create a VideoWrite object, recoring to ./video.avi
    # 创建一个VideoWrite对象，存储画面至./video.avi
    height, width, _ = frame_read.frame.shape
    video = cv2.VideoWriter('video.mp4', cv2.VideoWriter.fourcc(*'mp4v'), 30, (width, height))

    while keepRecording:
        video.write(frame_read.frame)
        time.sleep(1 / 30)

    video.release()

# we need to run the recorder in a seperate thread, otherwise blocking options
#  would prevent frames from getting added to the video
# 我们需要在另一个线程中记录画面视频文件，否则其他的阻塞操作会阻止画面记录
recorder = Thread(target=videoRecorder)
recorder.start()

# tello.takeoff()
# tello.land()
time.sleep(5)
keepRecording = False
recorder.join()