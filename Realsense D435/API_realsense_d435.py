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

        self.isRecording = False
        self.color_frame = []
        self.depth_frame = []

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(self.devSerial)

        self.types = []  # can be color, depth or both
        for streamInfo in self.streamInfos:
            self.config.enable_stream(streamInfo['streamType'], streamInfo['width'], streamInfo['height'],
                                      streamInfo['format'], streamInfo['frameRate'])
            self.types.append(streamInfo['streamType'])

    def record(self, max_recordTime):
        print('Start record')
        self.isRecording = True
        self.config.enable_record_to_file(self.recordFilePath)
        self.pipeline.start(self.config)
        # while True:
        try:
            record_thread = threading.Timer(max_recordTime, self.stopRecording)
            record_thread.start()
            while self.isRecording:
                print('Recording')
                time.sleep(1)
            record_thread.cancel()
        finally:
            self.pipeline.stop()
                # break

    def capture(self):
        if self.isRecording:
            print('Device is busy')
            return False
        try:
            self.pipeline.start(self.config)
            while True:
                color_frame = []
                depth_frame = []
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

            print('Succeed capture')
            return True

        finally:
            self.pipeline.stop()


    #Two threads can't record on same time
    #When a thread is recording then a different thread also want to record
    #2nd thread will must wait until 1st thread stop record
    def startRecording(self, max_recordTime, lock):
        # lock = threading.Lock()
        lock.acquire()
        # self.safe = True
        self.record(max_recordTime)
        lock.release()


    def stopRecording(self):
        self.isRecording = False
        print('Stop record')

    def updateConfig(self, _streamInfos):
        if self.isRecording:
            print("Is recording, can't change configuration")
            return False

        self.streamInfos = _streamInfos
        self.types = []  # can be color, depth or both
        for streamInfo in self.streamInfos:
            self.config.enable_stream(streamInfo['streamType'], streamInfo['width'], streamInfo['height'],
                                      streamInfo['format'], streamInfo['frameRate'])
            self.types.append(streamInfo['streamType'])




