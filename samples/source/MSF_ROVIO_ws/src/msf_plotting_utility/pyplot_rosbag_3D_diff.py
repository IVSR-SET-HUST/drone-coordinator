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
topiclist.append("/msf_core/state_out")
#topiclist.append("/noise_drift_handler/pose_output")
#topiclist.append("/noise_drift_handler/position_output")
#topiclist.append("/pos_to_error/output")
topiclist.append("/vicon/firefly_sbx/firefly_sbx")
topicdata = []
patches = []
colors = ["red", "green"]
labels = ["Estimate", "Groundtruth"]
linewidths = [1.5,1.5]
#give your plot a title
title = "Error on perfect data"
inputfile = sys.argv[1]
bag = rosbag.Bag(inputfile)
for currtopic in topiclist:
	#saves t, x, y, z
	tempdata=np.array([[0],[0],[0],[0]])
	for topic, msg, unused in bag.read_messages(topics=currtopic):
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

zlims = ax.get_zlim3d()
#actually compute datapoints for line (i.e x,y,z=linspace[...])
#use ax.plot(x,y,z linewidth as in normal plot)
for topic, msg, unused in bag.read_messages(topics = "/noise_drift_handler/events"):
	if msg.vector.x==3:
		t = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs 
		index = (np.abs(topicdata[0][0,:]-t)).argmin()
		#print(index)
		x_temp = np.array([topicdata[0][1,index],topicdata[0][1,index]])
		y_temp = np.array([topicdata[0][2,index],topicdata[0][2,index]])
		z_temp = np.array([topicdata[0][3,index],0.05*zlims[0]+0.95*zlims[1]])
		ax.plot(x_temp, y_temp, z_temp, linewidth = 2, color="black")
		#plt.axvline(x=topicdata[0][1, index], color="black", ymin=0.5, ymax=0.95, linewidth = 2)
		ax.text(topicdata[0][1,index], topicdata[0][2,index], 0.05*zlims[0]+0.95*zlims[1], "gps starts diverging", None, rotation = 90, fontsize=14)
	elif msg.vector.x==4:
		t = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs 
		index = (np.abs(topicdata[0][0,:]-t)).argmin()
		x_temp = np.array([topicdata[0][1,index],topicdata[0][1,index]])
		y_temp = np.array([topicdata[0][2,index],topicdata[0][2,index]])
		z_temp = np.array([topicdata[0][3,index],0.05*zlims[0]+0.95*zlims[1]])
		ax.plot(x_temp, y_temp, z_temp, linewidth = 2, color="black")
		#plt.axvline(x=msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs, color="black", ymin=0.5, ymax=0.95, linewidth = 2)
		ax.text(topicdata[0][1,index], topicdata[0][2,index], 0.05*zlims[0]+0.95*zlims[1], "gps stops diverging", None, rotation = 90, fontsize=14)

for topic, msg, unused in bag.read_messages(topics = "/rovio_divergence_handler/events"):
	if msg.vector.x==1:
		t = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
		index = (np.abs(topicdata[0][0,:]-t)).argmin()
		#print(index)
		x_temp = np.array([topicdata[0][1,index],topicdata[0][1,index]])
		y_temp = np.array([topicdata[0][2,index],topicdata[0][2,index]])
		z_temp = np.array([topicdata[0][3,index],0.05*zlims[0]+0.95*zlims[1]])
		ax.plot(x_temp, y_temp, z_temp, linewidth = 2, color="black")
		#plt.axvline(x=msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs, color="black", ymin=0.5, ymax=0.95, linewidth = 2)
		ax.text(topicdata[0][1,index], topicdata[0][2,index], 0.05*zlims[0]+0.95*zlims[1], "rovio diverging", None, rotation = 90, fontsize=14)
	#elif msg.vector.x==2:
		#plt.axvline(x=msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs, color="black", ymin=0.5, ymax=0.95, linewidth = 2)
		#plt.text(msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-1.1, 0.05*ylims[0]+0.95*ylims[1], "gps stops diverging", rotation = 90, fontsize=14)
mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()
