import cv2
import numpy as np
from cv2 import aruco
import math
from cameraCalibration import load_coefficients

cap = cv2.VideoCapture(1)
mtx, dist = load_coefficients("imLogi")
print(dist)
print(mtx.shape)
homo_k = np.array([[0.0, 0.0, 0.0]])
print(homo_k.shape)
K_homo = np.concatenate((mtx,homo_k), axis = 0)
print(K_homo.shape)
homo_1 = np.array([[0.0, 0.0, 0.0, 1.0]])
k_last = np.concatenate((K_homo, homo_1.T), axis = -1)
print(k_last)
cap.set(3, 1280)
cap.set(4, 720)

while True:
    ret, frame = cap.read()

    # operations on the frame
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # set dictionary size depending on the aruco marker selected
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

    # detector parameters can be set here (List of detection parameters[3])
    parameters = aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 7

    # lists of ids and the corners belonging to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # font for displaying text (below)
    font = cv2.FONT_HERSHEY_SIMPLEX

    # check if the ids list is not empty
    # if no check is added the code will crash
    if np.all(ids is not None):

        # estimate pose of each marker and return the values
        # rvet and tvec-different from camera coefficients
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners=corners, markerLength=0.1,
                                                        cameraMatrix=mtx, distCoeffs=dist)
        print(rvec)
        print(tvec)
        (rvec - tvec).any()  # get rid of that nasty numpy value array error
        cv2.putText(frame, "X :" + str(tvec[0][0][0]), (0, 150), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, "Y:" + str(tvec[0][0][1]), (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, "Z:" + str(tvec[0][0][2]), (0, 250), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
        # tvec = np.array(tvec)
        # rvec = np.array(rvec)
        # vecD = rvec
        # vecD = vecD*180/math.pi
        # a = vecD[0][0][0]
        # b = vecD[0][0][1]
        # c = vecD[0][0][2]
        # a = round(a,2)
        # b = round(b,2)
        # c = round(c,2)

        # cv2.putText(frame, "Marker wrt camera coordinate" + str(rvec), (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        # rmat, jacobian = cv2.Rodrigues(rvec)
        # rtmat = np.concatenate((rmat,tvec),axis=-1)
        # print(tvec.shape)
        # print(rvec.shape)
        # print(rmat.shape)
        # print(rtmat.shape)
        # homo = np.array([[0.0, 0.0, 0.0, 1.0]])
        # rtmat = np.concatenate((rtmat,homo), axis=0)
        # print(rtmat.shape)
        # rmat = np.reshape(rmat,(-1, 3))
        # tvec = np.reshape(tvec,(-1, 3))
        # print(tvec.shape)
        # print(rmat.shape)
        # Rmat = np.concatenate((rmat, tvec.T), axis = -1)
        # print(Rmat.shape)
        # RRt = np.concatenate((Rmat, homo), axis = 0)
        # print(RRt)
        #
        # KR = np.dot(k_last,RRt)
        # print(KR)
        # anh = np.array([0.0,0.0,0.0,1.0])
        # pixel = np.dot(KR,anh.T)
        # print(pixel)
        # pixel = pixel/ pixel[2]
        # a2 = pixel[0:3]
        # print(a2)
        # rmat2 = np.linalg.inv(rmat)
        # cv2.putText(frame, "X :" + str(a2[0]), (0, 150), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        # cv2.putText(frame, "Y:" + str(tvec[0][0][1]), (0, 200), font, 1, (0, 255, 255), 2, cv2.LINE_AA)
        # cv2.putText(frame, "Z:" + str(tvec[0][0][2]), (0, 250), font, 1, (255, 0, 255), 2, cv2.LINE_AA)

        # cv2.putText(frame, "Roll:" + str(a), (0, 150), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        # cv2.putText(frame, "Pitch:" + str(c), (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        # cv2.putText(frame, "Yaw" + str(b), (0, 250), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
        # cv2.putText(frame, "Marker wrt camera coordinate" + str(rvec), (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        print(rvec)
        # print(rmat)
        # print(rmat2)
        # print(rmat*rmat2)

        # """In camera coordinate R-P-Y"""
        # roll = math.atan2(rmat[2,1], rmat[2,2])
        # yaw = math.atan2(rmat[1,0], rmat[0,0])
        # if math.cos(yaw) == 0:
        #     pitch = math.atan2(-rmat[2,0], rmat[1,0]/math.sin(yaw))
        # else:
        #     pitch = math.atan2(-rmat[2,0], rmat[0,0]/math.cos(yaw))
        # roll_Degrees = roll*180/math.pi
        # pitch_Degrees = pitch*180/math.pi
        # yaw_Degrees = yaw*180/math.pi
        # camC = np.array([roll_Degrees, pitch_Degrees, yaw_Degrees])
        # print('Marker wrt camera coordinate:')
        # print(camC)
        # cv2.putText(frame, "Marker wrt camera coordinate" + str(camC), (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # """In world coordinate R-P-Y"""
        # roll2 = math.atan2(rmat2[2, 1], rmat2[2, 2])
        # yaw2 = math.atan2(rmat2[1, 0], rmat2[0, 0])
        # if math.cos(yaw2) == 0:
        #     pitch2 = math.atan2(-rmat2[2, 0], rmat2[1, 0] / math.sin(yaw2))
        # else:
        #     pitch2 = math.atan2(-rmat2[2, 0], rmat2[0, 0] / math.cos(yaw2))
        # worldC = np.array([roll2, pitch2, yaw2])
        # print(worldC)
        # cv2.putText(frame, "Camera wrt marker coordinate" + str(worldC), (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # """In camera coordinate R-P-Y"""
        # roll3 = math.atan2(-rmat[2,0], math.sqrt(math.pow(rmat[0,0],2) + math.pow(rmat[1,0],2)))
        # yaw3 = -math.atan2(rmat[1,0]/math.cos(roll3), rmat[0,0]/math.cos(roll3))
        # pitch3 = math.atan2(rmat[2,1]/math.cos(roll3), rmat[2,2]/math.cos(roll3))
        # roll_Degrees3 = roll3*180/math.pi
        # pitch_Degrees3 = pitch3*180/math.pi
        # yaw_Degrees3 = yaw3*180/math.pi
        # camC3 = np.array([roll_Degrees3, pitch_Degrees3, yaw_Degrees3])
        # cv2.putText(frame, "Marker wrt camera coordinate" + str(camC3), (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        # print('Marker wrt camera coordinate:')
        # print(camC3)

        # """In world coordinate Y-P-R"""
        # roll4 = math.atan2(-rmat2[3, 1], math.sqrt(math.pow(rmat2[1, 1], 2) + math.pow(rmat2[2, 1], 2)))
        # yaw4 = -math.atan2(rmat2[2, 1] / math.cos(roll4), rmat2[1, 1] / math.cos(roll4))
        # pitch4 = math.atan2(rmat2[3, 2] / math.cos(roll4), rmat2[3, 3] / math.cos(roll4))
        # worldC4 = np.matrix[roll4, pitch4, yaw4]
        # print(worldC4)

        # P = np.hstack((rmat,tvec))
        # euler_angles_radians = -cv2.decomposeProjectionMatrix(P)
        # euler_angles_degrees = 180 * euler_angles_radians / math.pi
        # print(euler_angles_degrees)
        # cam_pos = -np.array(rmat).T*np.array(tvec)
        # print(cam_pos)

        length = np.linalg.norm(tvec)
        length = round(length,2)
        print(length)
        cv2.putText(frame, "Distance: " + str(length)+"m", (0, 100), font, 1, (0, 255, 255), 2, cv2.LINE_AA)

        for i in range(0, len(ids)):
            # draw axis for the aruco markers
            aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)

        # draw a square around the markers
        aruco.drawDetectedMarkers(frame, corners)

        # code to show ids of the marker found
        strg = ''
        for i in range(0, ids.size):
            strg += str(ids[i][0]) + ', '

        cv2.putText(frame, "Id: " + strg, (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        # print("rvec")
        # print(rvec)
        # print("rmatrix")
        # print(rmat)
        # print("tvec")
        # print(tvec)
    else:
        # code to show 'No Ids' when no markers are found
        cv2.putText(frame, "No Ids", (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

    # display the resulting frame
    cv2.imshow('frame', frame)
    k = cv2.waitKey(1)
    if k == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()