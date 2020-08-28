from realsense2 import StreamInfo as sif, RealSense2 as rs2
import threading
import time

# Construct two objects of StreamInfo class
_color = sif('color', 640, 480, 'bgr8', 30)
_depth = sif('depth', 640, 480, 'z16', 30)

# A object of RealSense2 class include three argument
# List stream configuration, path to recording file and device's serial
# You need to change path to recording file or device's serial
rsObj = rs2([_color.streamInfo(), _depth.streamInfo()], 'record.bag', '001622072448')


##Capture
rsObj.capture()
print(rsObj.color_frame)
print(rsObj.depth_frame)


## 1st thread start recording, maximum recording time is 5s
#threading.Thread(target=rsObj.startRecording, args=(5,)).start()
#time.sleep(2)
## After 2s, 2nd thread start recording, but it must wait until 1st thread stop record
#threading.Thread(target=rsObj.startRecording, args=(5,)).start()

## If call rsObj.capture() when recording will return rsObj.color_frame or rsObj.depth_frame is []



