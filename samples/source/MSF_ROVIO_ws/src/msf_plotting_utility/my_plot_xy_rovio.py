import rosbag
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import csv

#from matplotlib.axes import Axes as axes
if len(sys.argv)!=2:
	print("usage python pyplot_rosbag.py <rosbag_file>")
	sys.exit()
#put topics to plot here
topic="/rovio/pose_with_covariance_stamped"
fig = plt.figure()
#give your plot a title
title = "Position Estimated from Rovio"
inputfile = sys.argv[1]
bag = rosbag.Bag(inputfile)

for topic, msg, t in bag.read_messages(topics=topic):
    if topic == "/rovio/pose_with_covariance_stamped":
        plt.scatter(msg.pose.pose.position.x, msg.pose.pose.position.y)
#plt.set_xlabel('X Label')
#plt.set_ylabel('Y Label')
plt.show()
