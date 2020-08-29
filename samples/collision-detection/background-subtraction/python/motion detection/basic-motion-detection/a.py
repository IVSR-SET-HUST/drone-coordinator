import numpy as np
import cv2
import pyrealsense2 as rs
import argparse
import datetime
import imutils
import time

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the video file")
ap.add_argument("-a", "--min-area", type=int, default=500, help="minimum area size")
args = vars(ap.parse_args())

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# initialize the first frame in the video stream
firstFrame = None
time.sleep(2)
try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frame = pipeline.wait_for_frames()
        color_frame = frame.get_color_frame()
        text = "Unoccupied"

        if not color_frame:
            break

        # Convert images to numpy arrays
        frame = np.asanyarray(color_frame.get_data())

	# resize the frame, convert it to grayscale, and blur it
        frame = imutils.resize(frame, width=500)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

	# if the first frame is None, initialize it
        if firstFrame is None:
                firstFrame = gray
                continue
        # compute the absolute difference between the current frame and
	# first frame
        frameDelta = cv2.absdiff(firstFrame, gray)
        thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

	# dilate the thresholded image to fill in holes, then find contours
	# on thresholded image
        thresh = cv2.dilate(thresh, None, iterations=4)
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

	# loop over the contours
        for c in cnts:
		# if the contour is too small, ignore it
                if cv2.contourArea(c) < args["min_area"]:
                      continue

		# compute the bounding box for the contour, draw it on the frame,
		# and update the text
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                text = "Occupied"

	# draw the text and timestamp on the frame
        cv2.putText(frame, "Room Status: {}".format(text), (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(frame, datetime.datetime.now().strftime("%A %d %B %Y %I:%M:%S%p"),
                (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

	# show the frame and record if the user presses a key
        cv2.imshow("Security Feed", frame)
        cv2.imshow("Thresh", thresh)
        cv2.imshow("Frame Delta", frameDelta)
        key = cv2.waitKey(1) & 0xFF

        cv2.imshow("first frame", firstFrame)
        cv2.imshow("current frame",gray)
	# if the `q` key is pressed, break from the lop
        if key == ord("q"):
                break

finally:

    # Stop streaming
       pipeline.stop()





