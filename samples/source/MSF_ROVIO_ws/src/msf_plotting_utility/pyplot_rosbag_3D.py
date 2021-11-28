#!/usr/bin/env python

#code to read data from rosbag and plot them using pyplot
#usage: python pyplot_rosbag.py <rosbag_file>
import rosbag
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
if len(sys.argv)!=2:
	print("usage python pyplot_rosbag.py <rosbag_file>")
	sys.exit()
#put topics to plot here
topiclist = []
#topiclist.append("/msf_core/state_out")
#topiclist.append("/noise_drift_handler/pose_output")
#topiclist.append("/noise_drift_handler/position_output")
#topiclist.append("/pos_to_error/output")
topiclist.append("/vicon/firefly_sbx/firefly_sbx")
topicdata = []
patches = []
colors = ["red", "green"]
labels = ["Estimate", "Groundtruth"]
linewidths = [1,1]
#give your plot a title
title = "Error on perfect data"
inputfile = sys.argv[1]
bag = rosbag.Bag(inputfile)
for currtopic in topiclist:
	#saves t, x, y, z
	tempdata=np.array([[0],[0],[0],[0]])
	for topic, msg, t in bag.read_messages(topics=currtopic):
		#print(msg.pose.pose.position.x)
		#case switch how to access x coordinate for all topics
		if topic == "/msf_core/state_out":
			tempdata = np.append(tempdata, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.data[0]], [msg.data[1]], [msg.data[2]]], axis=1)
		elif topic == "/noise_drift_handler/pose_output":
			#need to look up precise transform for this
			tempdata = np.append(tempdata, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[-1*msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]], axis=1)
		elif topic == "/noise_drift_handler/position_output":
			tempdata = np.append(tempdata, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.point.x], [msg.point.y], [msg.point.z]], axis=1)
		elif topic == "/pos_to_error/output":
			tempdata = np.append(tempdata, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.data[2]], [msg.data[3]], [msg.data[4]]], axis=1)
		elif topic == "/vicon/firefly_sbx/firefly_sbx":
			tempdata = np.append(tempdata, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.transform.translation.x], [msg.transform.translation.y], [msg.transform.translation.z]], axis=1)
		#topicdata
	topicdata.append(tempdata)

fig = plt.figure()
ax = fig.gca(projection='3d')
for idx, topic in enumerate(topicdata):
	#plot all data in topicdata
	#print(topic)
	ax.plot(topic[1,1:], topic[2,1:], topic[3,1:], color=colors[idx], linewidth=linewidths[idx])
mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()
