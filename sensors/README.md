# Instructions for using sensors
## Realsense D435
* **Requirements**

  Python3
  
  Python libraries: pyrealsense2, numpy 

* **Descriptions**
  
  [**realsense2.py**](https://github.com/IVSR-SET-HUST/drone-coordinator/blob/sprint_2020-09-15/sensors/realsense2.py) contains **StreamInfo** and **RealSense2** class

  StreamInfo class is used to construct a configuration for camera. A object has attributes: stream type, width, heigh, format, frame rate

  streamInfo() method will return a configuration can use for RealSense2 class. If a configuration is false, it will return false
  
  D435 can get color and depth data. So stream type can be 'color' or 'depth'
  
  format: 'rgb8' for 'color' and 'z16' for depth
  
  Example:
  
  ```
  _color = StreamInfo('color', 640, 480, 'bgr8', 30)  
  color_stream = _color.streamInfo() #color_stream can be used for RealSense2 class
  ```


  RealSense2 class includes some function: capture(), startRecording(), stopRecording(), updateConfig()
  
  Initialization function needs three arguments: a configuration list (color, depth or both), path to recording file (string, default is 'record_bag_file.bag'),   device's serial (string, default '001622072448')
  
  Example: 
  ```
  rsObj = RealSense2([color_stream], "path/to/your/bag/file", "your device's serial")
  ```

  capture() function: can get color, depth numpy data or both. It depends on configuration list. You can't capture when camera is recording. If it capture successfully, you can get color_frame or depth_frame
  
  Example: 
  ```
  rsObj.capture()
  rsObj.color_frame #numpy data or []
  rsObj.depth_frame #depth data or []
  ```

  startRecording(max_time_record): start recording to your bag file. Stop recording when timeout or stopRecording() is called. 

  stopRecording(): stop recording immediately 

  updateConfig(streamInfos): to change the stream configuration. Can't change when camera is recording
  Example:
  ```
  _depth = StreamInfo('depth', 640, 480, 'z16', 30)
  rsObj.updateConfig([_depth.streamInfo()])
  ```

* **Example**

  You can see the [test_realsense_d435.py](ttps://github.com/IVSR-SET-HUST/drone-coordinator/blob/sprint_2020-09-15/sensors/test_realsense2.py) to understand details


