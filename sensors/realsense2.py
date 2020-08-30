import pyrealsense2 as rs
import numpy as np
import threading
import time

class StreamInfo:
    def __init__(self, _streamType, _w, _h, _format, _fps):
        self.streamType = _streamType
        self.width = _w
        self.height = _h
        self.format = _format
        self.frameRate = _fps

    def streamInfo(self):
        if self.streamType == 'color':
            type = rs.stream.color
            if self.format == 'bgr8':
                data_format = rs.format.bgr8
            else:
                return False

        elif self.streamType == 'depth':
            type = rs.stream.depth
            if self.format == 'z16':
                data_format = rs.format.z16
            else:
                return False

        else:
            return False

        streamInfo = {'streamType' : type, 'width' : self.width, 'height' : self.height,
                          'format' : data_format, 'frameRate' : self.frameRate}
        return streamInfo


class RealSense2:
    #Device serial number is '001622072448'
    def __init__(self, _streamInfos, _recordFilePath = 'record_bag_file.bag',  _devSerial = '001622072448'):
        self.streamInfos = _streamInfos
        self.recordFilePath = _recordFilePath
        self.devSerial = _devSerial

        self.color_frame = []
        self.depth_frame = []

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(self.devSerial)

        self.lock = threading.Lock()

        self.types = []  # can be color, depth or both
        for streamInfo in self.streamInfos:
            self.config.enable_stream(streamInfo['streamType'], streamInfo['width'], streamInfo['height'],
                                      streamInfo['format'], streamInfo['frameRate'])
            self.types.append(streamInfo['streamType'])


    def record(self, max_recordTime):
        self.config.enable_record_to_file(self.recordFilePath)
        self.pipeline.start(self.config)
        record_thread = threading.Timer(max_recordTime, self.stopRecording)
        record_thread.start()
        while True:
            if not self.lock.locked():
                break
        record_thread.cancel()
        self.pipeline.stop()


    def capture(self):
        if self.lock.locked():
            return False
        self.lock.acquire()
        self.pipeline.start(self.config)
        color_frame = []
        depth_frame = []
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                if rs.stream.color in self.types:
                    color_frame = frames.get_color_frame()
                    self.color_frame = np.asanyarray(color_frame.get_data())
                if rs.stream.depth in self.types:
                    depth_frame = frames.get_depth_frame()
                    self.depth_frame = np.asanyarray(depth_frame.get_data())

                if not color_frame and not depth_frame:
                    continue
                break
        finally:
            self.pipeline.stop()
            print('Succeed capture')
            self.lock.release()

    def startRecording(self, max_recordTime):
        if self.lock.locked():
            return False
        self.lock.acquire()
        threading.Thread(target=self.record, args=(max_recordTime, )).start()

    def stopRecording(self):
        if not self.lock.locked():
            return False
        self.lock.release()

    def updateConfig(self, _streamInfos):
        if self.lock.locked():
            print("Is recording, can't change configuration")
            return False

        self.streamInfos = _streamInfos
        self.types = []  # can be color, depth or both
        for streamInfo in self.streamInfos:
            self.config.enable_stream(streamInfo['streamType'], streamInfo['width'], streamInfo['height'],
                                      streamInfo['format'], streamInfo['frameRate'])
            self.types.append(streamInfo['streamType'])




