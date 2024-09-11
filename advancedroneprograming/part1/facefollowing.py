import cvzone
from djitellopy import tello
import cv2
from cvzone.PIDModule import PID
from cvzone.FaceDetectionModule import FaceDetector
from cvzone.PlotModule import LivePlot


detector=FaceDetector(minDetectionCon=0.7)

# cap=cv2.VideoCapture(0)
# _, img = cap.read()
hi,wi= 640, 480
#print(hi,wi)

xpid = PID([0.3,0,0.1],wi//2)
ypid = PID([0.3,0,0.1],hi//2,axis=1)
zpid = PID([0.005,0,0.003],12000,limit=[-20,15])

myplotx=LivePlot(yLimit=[-100,100],char='X')
myploty=LivePlot(yLimit=[-100,100],char='Y')
myplotz=LivePlot(yLimit=[-100,100],char='Z')

me=tello.Tello()
me.connect()
print(me.get_battery())
me.streamoff()
me.streamon()
me.takeoff()
me.move_up(60)

while True:
    # _, img = cap.read()

    img=me.get_frame_read().frame
    img=cv2.resize(img,(970,720))
    img,bboxs = detector.findFaces(img,True)
    xval=0
    yval=0
    zval=0

    if bboxs:
        cx,cy = bboxs[0]['center']
        x,y,w,h = bboxs[0]['bbox']
        area = w * h

        xval = int(xpid.update(cx))
        yval = int(ypid.update(cy))
        zval = -(int(zpid.update(area)))
        # print(zval)
        imgplotx=myplotx.update(xval)
        imgploty=myploty.update(yval)
        imgplotz=myplotz.update(zval)

        img=xpid.draw(img,[cx,cy])
        img=ypid.draw(img,[cx,cy])
        # imgstacked=cvzone.stackImages([img,imgplotx,imgploty,imgplotz],2,0.5)
        imgstacked = cvzone.stackImages([img], 1, 0.75)
    else:
        imgstacked=cvzone.stackImages([img],1,0.75)

    me.send_rc_control(0,zval,-yval,xval)
    # me.send_rc_control(0, 0, 0, xval)
    cv2.imshow('Image stacked',imgstacked)

    '''cv2.imshow("Image",img)
    cv2.imshow("plotx",imgplotx)
    cv2.imshow("ploty", imgploty)
    cv2.imshow("plotz", imgplotz)'''


    if cv2.waitKey(5) & 0xFF == ord('q'):
        me.land()
        break
cv2.destroyAllWindows()