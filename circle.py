import cv2
import numpy as np
import math

cam = cv2.VideoCapture(0)

ret, img = cam.read()

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)


while True:
    try:
        #cv2.imshow("test", img)
        ret, img = cam.read()
        if cv2.waitKey(1) == 27:
            break
        corners, ids, rej = cv2.aruco.detectMarkers(img,arucoDict)
        if(corners):
            imgDet = cv2.aruco.drawDetectedMarkers(img, corners, ids)

            cimg = cv2.cvtColor(img[int((corners[0][0][0][0]-100)):int((corners[0][0][0][0]+100)),
                                int((corners[0][0][0][0]-100)):int((corners[0][0][0][0]+100))], cv2.COLOR_RGB2GRAY)

            circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT, 1, 30, param1=150, param2=50, minRadius=0, maxRadius=400)

            if(not circles is None):
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    # draw the outer circle
                    cv2.circle(imgDet, (i[0]+int((corners[0][0][0][0]-100)), i[1]+int((corners[0][0][0][0]-100))), i[2], (0, 255, 0), 2)
                    # draw the center of the circle
                    cv2.circle(imgDet, (i[0]+int((corners[0][0][0][0]-100)), i[1]+int((corners[0][0][0][0]-100))), 2, (0, 0, 255), 3)


            cv2.imshow("det", imgDet)
            if cv2.waitKey(1) == 27:
                break
    except KeyboardInterrupt:
        break

cam.release()
cv2.destroyAllWindows()
