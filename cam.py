import cv2
import numpy as np

cam = cv2.VideoCapture(1)

ret, img = cam.read()

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
markerLength = 1.665
distCoeffs = np.array([0.0214991, 0.790737, 0.474273, -0.00920846])
cameraMatrix = np.array([[821.934, 0, 342.613], [0, 822.053, 226.966], [0, 0, 1]])

# Board = cv2.aruco.Board_create()

# cameraMatrix = [[9028.981, 0, 0], [0, 14583.126, 0], [959.5, 539.5, 1]]
# retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(corners, img, imageSize)
# distCoeffs

while True:
    try:
        cv2.imshow("test", img)
        ret, img = cam.read()
        if cv2.waitKey(1) == 27:
            break
        corners, ids, rej = cv2.aruco.detectMarkers(img,arucoDict)
        # print(cameraMatrix)

        if(corners):
            imgDet = cv2.aruco.drawDetectedMarkers(img, corners, ids)
            # #############################################################
            # #############################################################
            # 1. 1:00-2:00
            print(corners)


            # #############################################################
            # #############################################################
            # # 2.
            # cornersOld = corners
            # imgDet = cv2.aruco.drawDetectedMarkers(img, corners, ids)
            # rvecs, tvecs, objPoint = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix,
            #                                                              distCoeffs)
            # print('rvecs: ', rvecs)
            # print('tvecs: ', tvecs)
            #
            # # cv2.imshow("det", imgDet)
            # if cv2.waitKey(1) == 27:
            #     break
    except KeyboardInterrupt:
        break

cam.release()
cv2.destroyAllWindows()


