#!/usr/bin/env python

#code to read data from rosbag and plot them using pyplot
#usage: python pyplot_rosbag.py <rosbag_file>
import rosbag
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
if len(sys.argv)!=2:
	print("usage python pyplot_rosbag.py <rosbag_file>")
	sys.exit()
#put topics to plot here
topiclist = []
#topiclist.append("/msf_core/state_out")
#topiclist.append("/noise_drift_handler/pose_output")
#topiclist.append("/noise_drift_handler/position_output")
topiclist.append("/pos_to_error/output")
#topiclist.append("/vicon/firefly_sbx/firefly_sbx")
topicdata = []
patches = []
colors = ["red", "green"]
labels = ["Error", "Groundtruth"]
linewidths = [1,1]
#give your plot a title
title = "Error on perfect data"
inputfile = sys.argv[1]
bag = rosbag.Bag(inputfile)
for currtopic in topiclist:
	tempdata=np.array([[0],[0]])
	for topic, msg, t in bag.read_messages(topics=currtopic):
		#print(msg.pose.pose.position.x)
		#case switch how to access x coordinate for all topics
		if topic == "/msf_core/state_out":
			tempdata = np.append(tempdata, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.data[0]]], axis=1)
		elif topic == "/noise_drift_handler/pose_output":
			tempdata = np.append(tempdata, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[-1*msg.pose.pose.position.x]], axis=1)
		elif topic == "/noise_drift_handler/position_output":
			tempdata = np.append(tempdata, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.point.x]], axis=1)
		elif topic == "/pos_to_error/output":
			tempdata = np.append(tempdata, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.data[1]]], axis=1)
		elif topic == "/vicon/firefly_sbx/firefly_sbx":
			tempdata = np.append(tempdata, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.transform.translation.x]], axis=1)
		#topicdata
	topicdata.append(tempdata)
for idx, topic in enumerate(topicdata):
	#plot all data in topicdata
	#print(topic)
	plt.plot(topic[0,1:], topic[1,1:], color=colors[idx], linewidth=linewidths[idx])
	patches.append(mpatches.Patch(color=colors[idx], label=labels[idx]))



plt.legend(handles=patches, prop={'size': 24})
plt.xlabel('time (s)', fontsize=24)
plt.ylabel('Error [L2 norm] (m)', fontsize=24)
plt.grid(True)
plt.suptitle(title, fontsize=36)
plt.tick_params(axis='both', which='major', labelsize=18)
mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()
