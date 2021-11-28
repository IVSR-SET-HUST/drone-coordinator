import rosbag
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import csv

filename_rovio_origin = '/home/manh/catkin_ws/MSF_ROVIO_ws/src/msf_plotting_utility/recordings/rovio_real18.bag'
filename_gps = '/home/manh/Lab/bag_file/realdata/realdata18_fly.bag'

filename_rovio = '/home/manh/catkin_ws/MSF_ROVIO_ws/src/msf_plotting_utility/recordings/_2021-04-22-11-48-32.bag'

topicdata=[]
patches = []
colors = ["#CC0000", "#0000CC", "#00994C", "#FF9999", "#9999FF", "#66FF66","#f97306", "#00ffff", "#aaff32"]
labels = ["Estimate x", "Estimate y", "Estimate z", "GPS x", "GPS y", "GPS z","Origin Estimate x", "Origin Estimate y", "Origin Estimate z"]
linewidths = [1.5, 1.5, 1.5, 3, 3, 3, 1.5, 1.5, 1.5]
linestyles = ['-','-','-','-','-','-','--','--','--']
#give your plot a title
title = "Rovio Estimated and Local Position GPS"
bag3 = rosbag.Bag(filename_rovio_origin)
bag1 = rosbag.Bag(filename_rovio)
bag2 = rosbag.Bag(filename_gps)

tempdata1=np.array([[0],[0]])
tempdata2=np.array([[0],[0]])
tempdata3=np.array([[0],[0]])
tempdata4=np.array([[0],[0]])
tempdata5=np.array([[0],[0]])
tempdata6=np.array([[0],[0]])
tempdata7=np.array([[0],[0]])
tempdata8=np.array([[0],[0]])
tempdata9=np.array([[0],[0]])

for topic, msg, t in bag1.read_messages(topics="/rovio/pose_with_covariance_stamped"):
	tempdata1 = np.append(tempdata1, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.pose.pose.position.x]], axis=1)
	tempdata2 = np.append(tempdata2, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.pose.pose.position.y]], axis=1)
	tempdata3 = np.append(tempdata3, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.pose.pose.position.z]], axis=1)
topicdata.append(tempdata1)
topicdata.append(tempdata2)
topicdata.append(tempdata3)
for topic, msg, t in bag2.read_messages(topics="/mavros/global_position/local"):
	tempdata4 = np.append(tempdata4, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.pose.pose.position.x]], axis=1)
	tempdata5 = np.append(tempdata5, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.pose.pose.position.y]], axis=1)
	tempdata6 = np.append(tempdata6, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.pose.pose.position.z]], axis=1)
topicdata.append(tempdata4)
topicdata.append(tempdata5)
topicdata.append(tempdata6)
for topic, msg, t in bag3.read_messages(topics="/rovio/pose_with_covariance_stamped"):
	tempdata7 = np.append(tempdata7, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.pose.pose.position.x]], axis=1)
	tempdata8 = np.append(tempdata8, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.pose.pose.position.y]], axis=1)
	tempdata9 = np.append(tempdata9, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.pose.pose.position.z]], axis=1)
topicdata.append(tempdata7)
topicdata.append(tempdata8)
topicdata.append(tempdata9)
for idx, topic in enumerate(topicdata):
	plt.plot(topic[0,1:], topic[1,1:], color=colors[idx], linewidth=linewidths[idx],linestyle=linestyles[idx])
	patches.append(mpatches.Patch(linewidth=linewidths[idx],color=colors[idx], label=labels[idx],linestyle=linestyles[idx]))

plt.legend(handles=patches, prop={'size': 20})
plt.xlabel('Time (s)', fontsize=24)
plt.ylabel('Position (m)', fontsize=24)
plt.grid(True)
plt.suptitle(title, fontsize=36)
plt.tick_params(axis='both', which='major', labelsize=18)
plt.ylim(-50,50)
axes = plt.gca()
ylims = axes.get_ylim()

mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()
