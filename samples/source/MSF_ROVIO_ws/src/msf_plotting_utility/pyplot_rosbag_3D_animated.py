#!/usr/bin/env python

#code to read data from rosbag and plot them using pyplot
#usage: python pyplot_rosbag.py <rosbag_file>
import rosbag
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import matplotlib.patheffects as patheffects

def update_lines(num, dataLines, lines):
    for line, data in zip(lines, dataLines):
        # NOTE: there is no .set_data() for 3 dim data...
        line.set_data(data[0:2, :num])
        line.set_3d_properties(data[2, :num])
    return lines
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
#ax = fig.gca(projection='3d')
ax = Axes3D(fig)

idx=0
targetnum=1000
topics=[]
for idx, topic in enumerate(topicdata):
	#select every ith number s.t. resulting are targetnum
	#not quite
	step=topic.shape[1]//targetnum
	topics.append(topic[1:,1::step])
#topics=[topicdata[idx][1:4,1:] for idx in range(1)]
lines = [ax.plot(topic[0,1:2], topic[1,1:2], topic[2,1:2], color=colors[idx], linewidth=linewidths[idx])[0] for topic in topics]
# Setting the axes properties
ax.set_xlim3d([-5.0, 5.0])
ax.set_xlabel('X')

ax.set_ylim3d([-5.0, 5.0])
ax.set_ylabel('Y')

ax.set_zlim3d([-1.0, 2.0])
ax.set_zlabel('Z')

ax.set_title(title)

# Creating the Animation object
print(topics[0].shape)
line_ani = animation.FuncAnimation(fig, update_lines, targetnum, fargs=(topics, lines),
                                   interval=10, blit=False)

mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()
