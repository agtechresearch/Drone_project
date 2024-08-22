import cv2
from cvzone.HandTrackingModule import HandDetector

cap=cv2.VideoCapture(0)
detector=HandDetector(maxHands=1)

while True:

    _,img=cap.read()
    hands,img=detector.findHands(img)
    if hands:
        lmlist=hands[0]["lmList"]
        print(f'Landmarks: {lmlist}')


    cv2.imshow("Image",img)
    cv2.waitKey(1)