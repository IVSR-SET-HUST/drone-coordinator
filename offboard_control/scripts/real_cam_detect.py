#! /usr/bin/env python2

import math
import rospy
import mavros
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped as PS
# from nav_msgs.msg import Odometry
# from mavros_msgs.msg import PositionTarget as PT
import transform as tr
# from std_msgs.msg import Float32
# from tf import transformations as tr
# import mavros_msgs.msg
# from mavros import setpoint as SP
# from simple_pid import PID

class MarkerDetector:
    """ 
        Subscribe to the camera feed and detect the aruco marker in the scene
    """
    def __init__(self):
        
        # Init node
        rospy.init_node('marker_detector')
        mavros.set_namespace('mavros')

        # Setup subscribers
        ## Image
        # video_topic = "iris_fpv_cam/usb_cam/image_raw"
        video_topic = "/camera/color/image_raw"
        image_subscriber = rospy.Subscriber(video_topic, Image, self.image_callback)
        self.bridge = CvBridge()

        # ## Drone pose
        # /mavros/local_position/pose
        local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
            PS, self._local_position_callback)

        # Set up publishers
        self.aruco_marker_pos_pub = rospy.Publisher('/aruco_marker_pos', PS, queue_size=10)
        self.fly_pos_pub = rospy.Publisher('/target_pos', PS, queue_size=10)
        self.aruco_marker_img_pub = rospy.Publisher('/aruco_marker_img', Image, queue_size=10)
        self.check_marker_detection = rospy.Publisher('/ids_detection', Bool, queue_size=10) 
        # self.check_move_position = rospy.Publisher('/move_position', Bool, queue_size=10) 
       
        # Initialize variables
        self.frame = np.zeros((540, 960, 3), np.uint8)
        self.pos = [0.0] * 4
        # self.markerPos = [0.0] * 4
        # self.beta = [0.0] * 2
        self.ids_target = [0.0] * 2
        # initial max high the camera can detect maker                                    
        # self.altitude = 15.0
        self.corners = [0.0] * 4

        # transformation matrix from imu to camera 4x4
        self.imu_cam = np.zeros((4,4), dtype=np.float)

        ## camera intristic parameters fpv_cam
        # self.K = np.array([277.191356, 0.0, 160.5, 0.0, 277.191356, 120.5, 0.0, 0.0, 1.0]).reshape(3,3)
        # self.distCoeffs = np.array([0.0] * 5)

        ## camera intristic parameters real_cam
        self.K = np.array([1386.4139404296875, 0.0, 960.0, 0.0, 1386.4139404296875, 540.0, 0.0, 0.0, 1.0]).reshape(3,3)
        self.distCoeffs = np.array([0.0] * 5)

        #Load the dictionary that was used to generate the markers.
        # self.dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
        self.dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_ARUCO_ORIGINAL)
        # Initialize the detector parameters using default values
        self.parameters =  cv.aruco.DetectorParameters_create()
        
        # Setup rate
        self.rate = rospy.Rate(20)
        # self.rate.sleep()

    def _local_position_callback(self, topic):
        # Position data
        self.pos[0] = topic.pose.position.x
        self.pos[1] = topic.pose.position.y
        self.pos[2] = topic.pose.position.z

         # Orientation data
        (r, p, y) = tr.euler_from_quaternion(topic.pose.orientation.x, topic.pose.orientation.y, topic.pose.orientation.z, topic.pose.orientation.w)
        self.pos[3] = y
        
    def image_callback(self, data):
        """
            Save the image data everytime a new frame comes up
        """
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def Aruco_marker_detector(self):
        # define the ids of marker
        # a = 0 # the index of first marker need to detect
        self.ids_target[0] = 4
        self.ids_target[1] = 7
        
        #set up the real size of the marker
        # markerSize = 1.0
        # axisLength = 2.0

        # setup matrix from imu to cam
        self.imu_cam[0][1] = -1.0
        self.imu_cam[0][3] = 0.0
        self.imu_cam[1][0] = -1.0
        self.imu_cam[1][3] = 0.0
        self.imu_cam[2][2] = -1.0
        self.imu_cam[2][3] = 0.0
        self.imu_cam[3][3] = 1.0

        # create vector tvec1, tvec2
        tvec1 = np.zeros((4,1), dtype=np.float)
        tvec1[3][0] = 1.0
        tvec2 = np.zeros((4,1), dtype=np.float)
        
        while not rospy.is_shutdown():
            # print(self.pos[0])
            """ Use the build in python library to detect the aruco marker and its coordinates """
            img = self.frame.copy() 
        
            # Detect the markers in the image
            markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(img, self.dictionary, parameters=self.parameters)

            if np.all(markerIds != None):
                ids_marker = True
                self.check_marker_detection.publish(ids_marker)
                for i in range(0, markerIds.size):
                    if markerIds[i][0] == self.ids_target[0]:
                        # ids_marker = True
                        # self.check_marker_detection.publish(ids_marker)
                        # get corner at index i responsible id at index 0
                        self.corners = markerCorners[i]

                        markerSize = 1.0
                        axisLength = 2.0
            
                        ret1 = cv.aruco.estimatePoseSingleMarkers( self.corners, markerSize, self.K, self.distCoeffs)
                        rvecs, tvecs = ret1[0][0,0,:],ret1[1][0,0,:]
                        
                        tvec1[0][0] = tvecs[0]
                        tvec1[1][0] = tvecs[1]
                        tvec1[2][0] = tvecs[2]

                        # marker in the body (UAV frane)
                        marker_pos = PS()
                        tvec2 = np.matmul(self.imu_cam, tvec1)
                        marker_pos.pose.position.x = tvec2[0][0]
                        marker_pos.pose.position.y = tvec2[1][0]
                        marker_pos.pose.position.z = tvec2[2][0]
                        # publish marker in body frame
                        self.aruco_marker_pos_pub.publish(marker_pos)

                        ## marker in the global frame 
                        fly_pos = PS()
                        fly_pos.pose.position.x = tvec2[0][0] + self.pos[0]
                        fly_pos.pose.position.y = tvec2[1][0] + self.pos[1]
                        fly_pos.pose.position.z = tvec2[2][0] + self.pos[2]
                        # publish marker in body frame
                        self.fly_pos_pub.publish(fly_pos)

                        frame_out = cv.aruco.drawAxis(img, self.K, self.distCoeffs, rvecs, tvecs, axisLength)
                        # self.aruco_marker_pos_pub.publish(marker_pos)
                        self.aruco_marker_img_pub.publish(self.bridge.cv2_to_imgmsg(frame_out, "bgr8"))
                        # print(markerSize)
                        self.rate.sleep()
            else:
                ids_marker = False
                self.check_marker_detection.publish(ids_marker)



###################################################################################################
if __name__ == "__main__":
    MD = MarkerDetector()
    try:
        MD.Aruco_marker_detector()
    except rospy.ROSInterruptException:
        pass 
    # while not rospy.is_shutdown():
    #         MD.Aruco_marker_detector()

