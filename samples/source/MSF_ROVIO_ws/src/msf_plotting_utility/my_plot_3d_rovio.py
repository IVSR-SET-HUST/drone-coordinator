import rosbag
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import csv
from mpl_toolkits.mplot3d import Axes3D

#from matplotlib.axes import Axes as axes
if len(sys.argv)!=2:
	print("usage python pyplot_rosbag.py <rosbag_file>")
	sys.exit()
#put topics to plot here
topic="/rovio/pose_with_covariance_stamped"
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
topicdata = []
patches = []
colors = ["#CC0000", "#0000CC", "#00994C"]
labels = ["Estimate x", "Estimate y", "Estimate z"]
linewidths = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
#give your plot a title
title = "Position Estimated from Rovio"
inputfile = sys.argv[1]
bag = rosbag.Bag(inputfile)

for topic, msg, t in bag.read_messages(topics=topic):
	#print(msg.pose.pose.position.x)
	#case switch how to access x coordinate for all topics
    if topic == "/rovio/pose_with_covariance_stamped":
        ax.scatter(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.show()
