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
topicdata = []
patches = []
colors = ["#CC0000", "#0000CC", "#00994C"]
labels = ["Estimate x", "Estimate y", "Estimate z"]
linewidths = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
#give your plot a title
title = "Position Estimated from Rovio"
inputfile = sys.argv[1]
bag = rosbag.Bag(inputfile)
tempdata1=np.array([[0],[0]])
tempdata2=np.array([[0],[0]])
tempdata3=np.array([[0],[0]])
for topic, msg, t in bag.read_messages(topics=topic):
	a = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
	break

for topic, msg, t in bag.read_messages(topics=topic):
	#print(msg.pose.pose.position.x)
	#case switch how to access x coordinate for all topics
	if topic == "/rovio/pose_with_covariance_stamped":
		tempdata1 = np.append(tempdata1, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-a],[msg.pose.pose.position.x]], axis=1)
		tempdata2 = np.append(tempdata2, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-a],[msg.pose.pose.position.y]], axis=1)
		tempdata3 = np.append(tempdata3, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-a],[msg.pose.pose.position.z]], axis=1)
	#topicdata
topicdata.append(tempdata1)
topicdata.append(tempdata2)
topicdata.append(tempdata3)


for idx, topic in enumerate(topicdata):
	plt.plot(topic[0,1:], topic[1,1:], color=colors[idx], linewidth=linewidths[idx])
	patches.append(mpatches.Patch(color=colors[idx], label=labels[idx]))

plt.legend(handles=patches, prop={'size': 20})
plt.xlabel('Time (s)', fontsize=24)
plt.ylabel('Position (m)', fontsize=24)
plt.grid(True)
plt.suptitle(title, fontsize=36)
plt.tick_params(axis='both', which='major', labelsize=18)
#plt.ylim(-50,50)
axes = plt.gca()
ylims = axes.get_ylim()

mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()
