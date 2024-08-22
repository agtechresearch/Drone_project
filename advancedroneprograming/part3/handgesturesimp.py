import cv2
import cvzone
from cvzone.HandTrackingModule import HandDetector
from cvzone.FaceDetectionModule import FaceDetector
from djitellopy import tello

detectorhand=HandDetector(maxHands=1)
detectorface=FaceDetector()
gesture=""

me=tello.Tello()
me.connect()
print(me.get_battery())
me.streamoff()
me.streamon()
me.takeoff()
me.move_up(50)

while True:

    img = me.get_frame_read().frame
    img = cv2.resize(img, (640, 480))
    hands,img=detectorhand.findHands(img,draw=True)
    img,bboxs=detectorface.findFaces(img,draw=True)

    if bboxs:
        x,y,w,h=bboxs[0]["bbox"]
        bboxregion=x-175-25,y-75,175,h+75
        cvzone.cornerRect(img,bboxregion,rt=0,t=10,colorC=(0,0,255))

        if hands and hands[0]["type"]=="Right":
            handinfo=hands[0]
            handcenter=handinfo["center"]

            inside=bboxregion[0]<handcenter[0]<bboxregion[0]+bboxregion[2] and bboxregion[1]<handcenter[1]<bboxregion[1]+bboxregion[3]

            if inside:
                cvzone.cornerRect(img,bboxregion,rt=0,t=10,colorC=(0,255,0))
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
                cv2.rectangle(img, (bboxregion[0],bboxregion[1]+bboxregion[3]+10),(bboxregion[0]+bboxregion[2],bboxregion[1]+bboxregion[3]+60),(0,255,0),cv2.FILLED)
                cv2.putText(img,f'{gesture}',(bboxregion[0]+10,bboxregion[1]+bboxregion[3]+50),cv2.FONT_HERSHEY_PLAIN,2,(0,0,0),2)

    cv2.imshow("Image",img)
    if cv2.waitKey(5) & 0xFF == ord('q'):
        me.land()
        break
cv2.destroyAllWindows()