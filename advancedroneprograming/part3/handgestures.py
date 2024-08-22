import cv2
import cvzone
from cvzone.HandTrackingModule import HandDetector
from cvzone.FaceDetectionModule import FaceDetector

cap=cv2.VideoCapture(0)
detectorhand=HandDetector(maxHands=1)
detectorface=FaceDetector()
gesture=""

while True:

    _,img=cap.read()
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
                    gesture = "  Index"
                elif fingers == [0, 0, 0, 0, 0]:
                    gesture = "  Fist"
                elif fingers == [0, 0, 1, 0, 0]:
                    gesture = "  Middle"
                elif fingers == [1, 1, 0, 0, 1]:
                    gesture = "SpiderMan"
                elif fingers == [0, 1, 1, 0, 0]:
                    gesture = " Victory"
                elif fingers == [0, 0, 0, 0, 1]:
                    gesture = "  Pinky"
                elif fingers == [1, 0, 0, 0, 0]:
                    gesture = "  Thumb"
                cv2.rectangle(img, (bboxregion[0],bboxregion[1]+bboxregion[3]+10),(bboxregion[0]+bboxregion[2],bboxregion[1]+bboxregion[3]+60),(0,255,0),cv2.FILLED)
                cv2.putText(img,f'{gesture}',(bboxregion[0]+10,bboxregion[1]+bboxregion[3]+50),cv2.FONT_HERSHEY_PLAIN,2,(0,0,0),2)

    cv2.imshow("Image",img)
    cv2.waitKey(1)