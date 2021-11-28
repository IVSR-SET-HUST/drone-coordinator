import rosbag
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import csv

filename_msf_origin = '/home/manh/catkin_ws/MSF_ROVIO_ws/src/msf_plotting_utility/recordings/msf_real18.bag'
filename_msf = '/home/manh/catkin_ws/MSF_ROVIO_ws/src/msf_plotting_utility/recordings/_2021-04-22-13-59-29.bag'
filename_gps = '/home/manh/Lab/bag_file/realdata/realdata18_fly.bag'

topicdata = []
patches = []
colors = ["#CC0000", "#0000CC", "#00994C", "#FF9999", "#9999FF", "#66FF66","#f97306", "#00ffff", "#aaff32","#f97306", "#00ffff", "#aaff32"]
labels = ["MSF+Pose+Position x", "MSF+Pose+Position y", "MSF+Pose+Position z", "GPS x", "GPS y", "GPS z","MSF+Pose x", "MSF+Pose y", "MSF+Pose z","MSF+Pose x", "MSF+Pose y", "MSF+Pose z" ]
linewidths = [5,5,5,5,5,5,5,5,5]
linestyles = ['-','-','-','-','-','-','--','--','--','--','--','--']
#give your plot a title
title = "MSF+Position(GPS local)+Pose(Rovio)"

bag1 = rosbag.Bag(filename_msf)
bag2 = rosbag.Bag(filename_gps)
bag3 = rosbag.Bag(filename_msf_origin)


tempdata1=np.array([[0],[0]])
tempdata2=np.array([[0],[0]])
tempdata3=np.array([[0],[0]])
tempdata4=np.array([[0],[0]])
tempdata5=np.array([[0],[0]])
tempdata6=np.array([[0],[0]])
tempdata7=np.array([[0],[0]])
tempdata8=np.array([[0],[0]])
tempdata9=np.array([[0],[0]])

for topic, msg, t in bag1.read_messages(topics="/msf_core/state_out"):
	start_time_1 = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
	break

for topic, msg, t in bag1.read_messages(topics="/msf_core/state_out"):
	#print(msg.pose.pose.position.x)
	#case switch how to access x coordinate for all topics
	if topic == "/msf_core/state_out":
			tempdata2 = np.append(tempdata2, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-start_time_1],[msg.data[1]]], axis=1)
			tempdata1 = np.append(tempdata1, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-start_time_1],[msg.data[0]]], axis=1)
			tempdata3 = np.append(tempdata3, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-start_time_1],[msg.data[2]]], axis=1)
	#topicdata
topicdata.append(tempdata1)
topicdata.append(tempdata2)
topicdata.append(tempdata3)

for topic, msg, t in bag2.read_messages(topics="/mavros/global_position/local"):
	start_time_2 = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
	break

for topic, msg, t in bag2.read_messages(topics="/mavros/global_position/local"):
	#print(msg.pose.pose.position.x)
	#case switch how to access x coordinate for all topics
	if topic == "/mavros/global_position/local":
		tempdata4 = np.append(tempdata4, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-start_time_2],[msg.pose.pose.position.x]], axis=1)
		tempdata5 = np.append(tempdata5, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-start_time_2],[msg.pose.pose.position.y]], axis=1)
		tempdata6 = np.append(tempdata6, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-start_time_2],[msg.pose.pose.position.z]], axis=1)
	#topicdata
topicdata.append(tempdata4)
topicdata.append(tempdata5)
topicdata.append(tempdata6)

for topic, msg, t in bag3.read_messages(topics="/msf_core/state_out"):
	start_time_3 = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
	break
for topic, msg, t in bag3.read_messages(topics="/msf_core/state_out"):
	#print(msg.pose.pose.position.x)
	#case switch how to access x coordinate for all topics
	if topic == "/msf_core/state_out":
			tempdata7 = np.append(tempdata7, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-start_time_3],[msg.data[0]]], axis=1)
			tempdata8 = np.append(tempdata8, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-start_time_3],[msg.data[1]]], axis=1)
			tempdata9 = np.append(tempdata9, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-start_time_3],[msg.data[2]]], axis=1)
	#topicdata
topicdata.append(tempdata7)
topicdata.append(tempdata8)
topicdata.append(tempdata9)
for idx, topic in enumerate(topicdata):
	plt.plot(topic[0,1:], topic[1,1:], color=colors[idx], linewidth=linewidths[idx],linestyle=linestyles[idx])
	patches.append(mpatches.Patch(color=colors[idx], label=labels[idx]))

#plt.legend(handles=patches, prop={'size': 20})
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
