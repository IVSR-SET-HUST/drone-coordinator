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


ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", default='./2_mark.avi', help="name of video capture")
args = vars(ap.parse_args())
out = cv2.VideoWriter(args["video"], cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (1280, 720))

# create homo vector row
homoMat = np.array([0.0, 0.0, 0.0, 1.0])

fx, fy, cx, cy = (1.4134702038629678e+03, 1.4235239712132613e+03, 6.7342836691438674e+02, 3.4939651343710443e+02)

# create R_T matrix from Global to UAV
x = 0.0
y = 0.0
z = 0.0
w = -1.0
r, p, y = quaternionToEurler(x, y, z, w)
rMat, j1 = cv2.Rodrigues(np.array([r, p, y]))
tVec = np.array([[0.0], [0.0], [0.76]])
rtMat1 = np.hstack((rMat, tVec))
rtMat1 = np.vstack((rtMat1, homoMat))

# create R_T matrix from UAV to camera
r2 = np.array([0.0, 0.0, math.pi / 2])
r1 = np.array([math.pi, 0.0, 0.0])
rMat1, j2 = cv2.Rodrigues(r1)
rMat2, j3 = cv2.Rodrigues(r2)
rMat3 = np.dot(rMat1, rMat2)
tVec2 = np.array([[0.1], [-0.05], [-0.09]])
rtMat2 = np.hstack((rMat3, tVec2))
rtMat2 = np.vstack((rtMat2, homoMat))

# transformation from global to camera
a = np.dot(rtMat1, rtMat2)
# set up video capture
mtx, dist = load_coefficients("imLogi")
cap = cv2.VideoCapture(1)
cap.set(3, 1280)
cap.set(4, 720)

# set dictionary size depending on the aruco marker selected
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# detector parameters can be set here (List of detection parameters[3])
parameters = aruco.DetectorParameters_create()
parameters.adaptiveThreshConstant = 7

# font for displaying text (below)
font = cv2.FONT_HERSHEY_SIMPLEX

# x,y,z of ENU frame
position = []
time_array = []
time1 = time.time()
while True:
    # time1 = time.time()
    ret, frame = cap.read()
    # operations on the frame
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    # lists of ids and the corners belonging to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # cv2.line(frame, (640, 360), (860, 360), (0, 0, 255), 5)
    # cv2.line(frame, (640, 360), (640, 580), (0, 255, 0), 5)
    draw_axis(frame, cx, cy)
    # index = 0

    if ids is not None:
        ret1 = aruco.estimatePoseSingleMarkers(corners=corners, markerLength=0.1,
                                               cameraMatrix=mtx, distCoeffs=dist)
        rvec, tvec = ret1[0][0, 0, :], ret1[1][0, 0, :]
        (rvec - tvec).any()  # get rid of that nasty numpy value array error
        str_position0 = "Marker Position in Camera frame: x=%f  y=%f  z=%f" % (tvec[0], tvec[1], tvec[2])
        # take the value position of the marker
        tvec1 = np.expand_dims(tvec, axis=1)
        tvec1 = np.concatenate((tvec1, [[1.0]]), axis=0)
        result = np.dot(a, tvec1)
        str_position1 = "Marker Position in Global frame: x=%f  y=%f  z=%f" % (result[0], result[1], result[2])
        aruco.drawDetectedMarkers(image=frame, corners=corners, ids=ids)
        cv2.putText(frame, str_position0, (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, str_position1, (0, 100), font, 1, (255, 255, 0), 2, cv2.LINE_AA)
        time2 = time.time()
        delta = time2 - time1
        time_array.append(delta)
        position.append(result)

    cv2.imshow("frame", frame)

    # record video
    out.write(frame)
    k = cv2.waitKey(1)
    if k == ord("q"):
        break

# position = np.reshape(position, (-1, 4))
# time_array = np.array(time_array)
# write_csv(position, 0.1, time_array, './result_2.csv', 0.15, -0.1, 0.0, 0.1, -0.05, -0.09, 0.0, 0.0, 0.76)

cap.release()
cv2.destroyAllWindows()
# out.release()
