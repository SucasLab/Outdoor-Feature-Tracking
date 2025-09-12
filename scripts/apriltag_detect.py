import apriltag
import cv2
import numpy as np

cap = cv2.VideoCapture(0)
at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))

while(1):
    # 获得图像
    ret, frame = cap.read()
    # 检测按键
    k = cv2.waitKey(1)
    if k==27:
        break
        
    # 检测apriltag
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray)
    for tag in tags:
        #获取Apriltag角点及中心坐标
        a = tuple(tag.corners[0])   # left-top
        b = tuple(tag.corners[1])   # right-top
        c = tuple(tag.corners[2])   # right-bottom
        d = tuple(tag.corners[3])   # left-bottom
        cx = (tag.center[0])
        cy = (tag.center[1])
        #print(cx,cy)
        #绘制Apriltag4个角点
        cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2)
        cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2)
        cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2)
        cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2)
        #绘制Apriltag中心坐标
        (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
        cv2.circle(frame, (cX, cY), 5, (255, 0, 255), -1)

        dx1 = d[0] - a[0]
        dy1 = d[1] - a[1]
        dx2 = c[0] - b[0]
        dy2 = c[1] - b[1] 
        degree1 = (np.arctan2(dx1, dy1)) *180/np.pi
        degree2 = (np.arctan2(dx2, dy2)) *180/np.pi
        degree = (degree1 + degree2) / 2
        print(degree)

        #print(tags)
    # 显示检测结果
    cv2.imshow('capture', frame)

cap.release()
cv2.destroyAllWindows()
