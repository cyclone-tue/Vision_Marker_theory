import cv2
import numpy as np
import time
import math

cam = cv2.VideoCapture(1)

ret, img = cam.read()

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
markerLength = 1.665
distCoeffs = np.array([0.0214991, 0.790737, 0.474273, -0.00920846])


# cameraMatrix = np.array([[9028.981, 0, 0], [0, 14583.126, 0], [959.5, 539.5, 1]])
cameraMatrix = np.array([[821.934, 0, 342.613], [0, 822.053, 226.966], [0, 0, 1]])

cornersOld = np.empty([1, 1, 1, 3])

while True:
    try:
        cv2.imshow("test", img)
        ret, img = cam.read()
        if cv2.waitKey(1) == 27:
            break
        corners, ids, rej = cv2.aruco.detectMarkers(img, arucoDict)
        # print(cameraMatrix)

        if(corners):
            cornersOld = corners
            imgDet = cv2.aruco.drawDetectedMarkers(img, corners, ids)

            rvecs, tvecs, objPoint = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs)
            # print(rvecs)
            # print(tvecs)
            if(rvecs.size == 3):
                rotMat, jacobian = cv2.Rodrigues(rvecs)
                markerPos = np.array([corners[0][0][0][0], corners[0][0][0][1], 0])
                # print(np.dot(markerPos,rotMat))
                # print(rotMat)
                roll = math.atan2(rotMat[2][1], rotMat[2][2])
                pitch = math.atan2(-rotMat[2][0], math.sqrt(math.pow(rotMat[2][1],2)+math.pow(rotMat[2][2],2)))
                yaw = math.atan2(rotMat[1][0], rotMat[0][0])

                # print("Pitch: " + str(math.degrees(roll)))
                # print("Y dist: " + str(tvecs[0][0][2]*math.sin(math.fabs(roll))))

                tvecsTran = [-1*tvecs[0][0][0], -1*tvecs[0][0][1], -1*tvecs[0][0][2]]
                dist = np.dot(np.linalg.inv(rotMat), tvecsTran)
                print(dist)

            # print("Roll: " + str(math.degrees(roll)))
            # print("Pitch: " + str(math.degrees(pitch)))
            # print("Yaw: " + str(math.degrees(yaw)))


            try:
                imgDet = cv2.aruco.drawAxis(imgDet, cameraMatrix, distCoeffs, rvecs,tvecs,1)
            except:
                continue

            # time.sleep(0.1)

            # cv2.aruco.
            # print(corners)



            cv2.imshow("det", imgDet)
            if cv2.waitKey(1) == 27:
                break
    except KeyboardInterrupt:
        break

cam.release()
cv2.destroyAllWindows()


