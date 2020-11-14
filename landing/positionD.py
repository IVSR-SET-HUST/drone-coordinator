import cv2
import numpy as np
from cv2 import aruco
from cameraCalibration import load_coefficients
from convertQ import quaternionToEurler
import math
from csvv import *
import argparse
import time


def draw_axis(frame, cx, cy):
    cx = int(cx)
    cy = int(cy)
    cv2.line(frame, (cx, cy), ((cx + 100), cy), (0, 0, 255), 2)
    cv2.line(frame, (cx, cy), (cx, (cy + 100)), (255, 0, 0), 2)
    return frame


fx, fy, cx, cy = (1.4134702038629678e+03, 1.4235239712132613e+03, 6.7342836691438674e+02, 3.4939651343710443e+02)
# fx, fy , cx, cy = (1.5498163188835867e+03, 1.5001743149690269e+03, 6.4353239121802733e+02, 3.6017899329422818e+02)
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", default='./4_mark.avi', help="name of video capture")
args = vars(ap.parse_args())
out = cv2.VideoWriter(args["video"], cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (1280, 720))

# create homo vector row
homoMat = np.array([0.0, 0.0, 0.0, 1.0])

# create R_T matrix from camera to UAV
r2 = np.array([0.0, 0.0, math.pi / 2])
r1 = np.array([math.pi, 0.0, 0.0])
rMat1, j2 = cv2.Rodrigues(r1)
rMat2, j3 = cv2.Rodrigues(r2)
rMat3 = np.dot(rMat1, rMat2)
tVec2 = np.array([[0.1], [-0.05], [-0.09]])
tVec2 = np.dot(-rMat3.transpose(), tVec2)
rtMat2 = np.hstack((rMat3, tVec2))
rtMat2 = np.vstack((rtMat2, homoMat))

# set up video capture
mtx, dist = load_coefficients("imLogi")
cap = cv2.VideoCapture(1)
cap.set(3, 1280)
cap.set(4, 720)

# set dictionary size depending on the aruco marker selected
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# detector parameters can be set here (List of detection parameters[3])
parameters = aruco.DetectorParameters_create()
parameters.adaptiveThreshConstant = 5

# font for displaying text (below)
font = cv2.FONT_HERSHEY_SIMPLEX

# x,y,z of ENU frame
position = []
time_array = []
while True:
    time1 = time.time()
    ret, frame = cap.read()
    # operations on the frame
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    # lists of ids and the corners belonging to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # cv2.line(frame, (640, 360), (860, 360), (0, 0, 255), 5)
    # cv2.line(frame, (640, 360), (640, 580), (0, 255, 0), 5)
    frame = draw_axis(frame, cx, cy)
    # index = 0

    if ids is not None:
        # time1 = time.time()
        ret1 = aruco.estimatePoseSingleMarkers(corners=corners, markerLength=0.1,
                                               cameraMatrix=mtx, distCoeffs=dist)
        rvec, tvec = ret1[0][0, 0, :], ret1[1][0, 0, :]
        # -- Draw the detected marker and put a reference frame over it
        aruco.drawDetectedMarkers(frame, corners, ids)
        aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.1)
        (rvec - tvec).any()  # get rid of that nasty numpy value array error
        str_position0 = "Marker Position in Camera frame: x=%f  y=%f  z=%f" % (tvec[0], tvec[1], tvec[2])
        cv2.putText(frame, str_position0, (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Transformation matrix from marker to camera
        rmat1, j5 = cv2.Rodrigues(rvec)
        pose = rmat1.transpose()
        tvec1 = np.dot(-pose, tvec)
        tvec1 = np.expand_dims(tvec1, axis=1)
        rtmat1 = np.hstack((pose, tvec1))
        rtmat1 = np.vstack((rtmat1, homoMat))
        str_position3 = "Camera Position in Marker frame: x=%f  y=%f  z=%f" % (rtmat1[0, 3],
                                                                               rtmat1[1, 3], rtmat1[2, 3])
        cv2.putText(frame, str_position3, (0, 100), font, 1, (255, 255, 0), 2, cv2.LINE_AA)
        # Drone position in Marker frame
        result2 = np.dot(rtmat1, rtMat2)
        str_position1 = "Drone Position in Marker frame: x=%f  y=%f  z=%f" % (
            result2[0, 3], result2[1, 3], result2[2, 3])
        cv2.putText(frame, str_position1, (0, 150), font, 1, (0, 255, 255), 2, cv2.LINE_AA)
        time2 = time.time()
        t_position = result2[:, 3]
        position.append(t_position)
        delta = time2 - time1
        time_array.append(delta)

    cv2.imshow("frame", frame)
    # record video
    out.write(frame)
    k = cv2.waitKey(1)
    if k == ord("q"):
        break

position = np.reshape(position, (-1, 4))
time_array = np.array(time_array)
write_csv(position, 0.1, time_array, './result_4.csv', 0.0, 0.0, 0.76, 0.1, -0.05, -0.09, 0.0, 0.0, 0.0)

cap.release()
cv2.destroyAllWindows()
out.release()
