from realsense2 import StreamInfo as sif, RealSense2 as rs2
import threading
import time

# Construct two objects of StreamInfo class
_color = sif('color', 640, 480, 'bgr8', 30)
_depth = sif('depth', 640, 480, 'z16', 30)

rsObj = rs2([_color.streamInfo(), _depth.streamInfo()])

#### TEST SCRIPTS ####

### SCRIPT 1 ###
## Two parallel threads perform capture() together
## A thread return False, a thread perform capture()
# t1 = threading.Thread(target=rsObj.capture)
# t2 = threading.Thread(target=rsObj.capture)
# t1.start()
# t2.start()
# t1.join()
# t2.join()
# print(rsObj.color_frame)
# print(rsObj.depth_frame)

### SCRIPT 2 ###
## Two parallel threads perform startRecording() together
## Or a thread is recording then another is recording
## 2nd thread returns False
# rsObj.startRecording(5)
# rsObj.startRecording(5)

### SCRIPT 3 ###
## recording() is non-blocking
## We can stop recording by stopRecording()
# rsObj.startRecording(5)
# print('recording() function is performing')
# print('we can do anything when is recording')
# time.sleep(2)
# rsObj.stopRecording()

### SCRIPT 4 ###
## when is recording, if we perform capture(), it will return False with capture()
## And recording() still continues
# rsObj.startRecording(5)
# time.sleep(2)
# rsObj.capture()
# print('Color frame is: ', rsObj.color_frame)
# print('Depth frame is: ', rsObj.depth_frame)










