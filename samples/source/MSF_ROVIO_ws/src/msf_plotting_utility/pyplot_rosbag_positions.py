#!/usr/bin/env python
 
#code to read data from rosbag and plot them using pyplot
#usage: python pyplot_rosbag.py <rosbag_file>
import rosbag
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

#transformation function taken from EKF_MSF code
def InitReference(latitude, longitude, altitude):
	R=np.zeros((3,3))
	DEG2RAD=math.pi/180.0
	s_lat=math.sin(latitude * DEG2RAD)
	c_lat=math.cos(latitude * DEG2RAD)
	s_long=math.sin(longitude * DEG2RAD)
	c_long=math.cos(longitude * DEG2RAD)


	R[0, 0] = -s_long
	R[0, 1] = c_long
	R[0, 2] = 0

	R[1, 0] = -s_lat * c_long
	R[1, 1] = -s_lat * s_long
	R[1, 2] = c_lat

	R[2, 0] = c_lat * c_long
	R[2, 1] = c_lat * s_long
	R[2, 2] = s_lat

	ecef_ref_orientation = R
	ecef_ref_point = WGS84ToECEF(latitude, longitude, altitude)
	return [ecef_ref_orientation, ecef_ref_point]


#transformation for gps to ENU (i.e display frame)
def WGS84ToECEF(latitude, longitude, altitude):
	a = 6378137.0  # semi-major axis
	e_sq = 6.69437999014e-3  # first eccentricity squared

	DEG2RAD=math.pi/180.0
	s_lat=math.sin(latitude * DEG2RAD)
	c_lat=math.cos(latitude * DEG2RAD)
	s_long=math.sin(longitude * DEG2RAD)
	c_long=math.cos(longitude * DEG2RAD)

	N = a / math.sqrt(1 - e_sq * s_lat * s_lat)

	ecef=np.zeros(3)

	ecef[0] = (N + altitude) * c_lat * c_long
	ecef[1] = (N + altitude) * c_lat * s_long
	ecef[2] = (N * (1 - e_sq) + altitude) * s_lat

	return ecef


def ECEFToENU(ecef, ecef_ref_orientation, ecef_ref_point):
	return np.einsum('ij,j->i', ecef_ref_orientation, ecef - ecef_ref_point)

#from matplotlib.axes import Axes as axes
if len(sys.argv)!=2:
	print("usage python pyplot_rosbag.py <rosbag_file>")
	sys.exit()
#put topics to plot here
topiclist = []
topiclist.append("/msf_core/state_out")
#topiclist.append("/noise_drift_handler/pose_output")
#topiclist.append("/noise_drift_handler/position_output")
#topiclist.append("/pos_to_error/output")
#this is for euroc dataset
topiclist.append("/vicon/firefly_sbx/firefly_sbx")
#this is for real dataset
#topiclist.append("/raven/gps")
topicdata = []
patches = []
colors = ["#CC0000", "#0000CC", "#00994C", "#FF9999", "#9999FF", "#66FF66"]
labels = ["Estimate x", "Estimate y", "Estimate z", "Truth x", "Truth y", "Truth z"]
linewidths = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
#give your plot a title
title = "Error"
inputfile = sys.argv[1]
bag = rosbag.Bag(inputfile)
for currtopic in topiclist:
	tempdata1=np.array([[0],[0]])
	tempdata2=np.array([[0],[0]])
	tempdata3=np.array([[0],[0]])
	ecef_ref_orientation=0
	ecef_ref_point=0
	if currtopic=="/raven/gps":
		for topic, msg, t in bag.read_messages(topics=currtopic):
			[ecef_ref_orientation, ecef_ref_point]=InitReference(msg.latitude, msg.longitude, msg.altitude)
			break
	for topic, msg, t in bag.read_messages(topics=currtopic):
		#print(msg.pose.pose.position.x)
		#case switch how to access x coordinate for all topics
		if topic == "/msf_core/state_out":
			tempdata1 = np.append(tempdata1, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.data[0]]], axis=1)
			tempdata2 = np.append(tempdata2, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.data[1]]], axis=1)
			tempdata3 = np.append(tempdata3, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.data[2]]], axis=1)
		elif topic == "/vicon/firefly_sbx/firefly_sbx":
			tempdata1 = np.append(tempdata1, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.transform.translation.x]], axis=1)
			tempdata2 = np.append(tempdata2, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.transform.translation.y]], axis=1)
			tempdata3 = np.append(tempdata3, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.transform.translation.z]], axis=1)
		elif topic == "/raven/gps":
			enu=ECEFToENU(WGS84ToECEF(msg.latitude, msg.longitude, msg.altitude),ecef_ref_orientation, ecef_ref_point) 
			#print(enu)
			tempdata1 = np.append(tempdata1, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[enu[0]]], axis=1)
			tempdata2 = np.append(tempdata2, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[enu[1]]], axis=1)
			tempdata3 = np.append(tempdata3, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[enu[2]]], axis=1)
		#topicdata
	topicdata.append(tempdata1)
	topicdata.append(tempdata2)
	topicdata.append(tempdata3)
print (len(topicdata))
#print(topicdata[0])
for idx, topic in enumerate(topicdata):
	#plot all data in topicdata
	#print(topic)
	print(idx)
	plt.plot(topic[0,1:], topic[1,1:], color=colors[idx], linewidth=linewidths[idx])
	patches.append(mpatches.Patch(color=colors[idx], label=labels[idx]))



plt.legend(handles=patches, prop={'size': 20})
plt.xlabel('Time (s)', fontsize=24)
plt.ylabel('Position (m)', fontsize=24)
plt.grid(True)
plt.suptitle(title, fontsize=36)
plt.tick_params(axis='both', which='major', labelsize=18)
#this is for VH1
plt.ylim(-7,7)
#this is for real data
#plt.ylim(-20,20)
axes = plt.gca()
ylims = axes.get_ylim()
for topic, msg, unused in bag.read_messages(topics = "/noise_drift_handler/events"):
	if msg.vector.x==3:
		t = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
		index = (np.abs(topicdata[1][0,:]-t)).argmin()
		ymin_rel = (topicdata[1][1,index]-ylims[0])/(ylims[1]-ylims[0])
		plt.axvline(x=msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs, color="black", ymin=ymin_rel, ymax=0.95, linewidth = 2)
		plt.text(msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-1.1, 0.05*ylims[0]+0.95*ylims[1], "gps starts diverging", rotation = 90, fontsize=14)
	elif msg.vector.x==4:
		t = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
		index = (np.abs(topicdata[1][0,:]-t)).argmin()
		ymin_rel = (topicdata[1][1,index]-ylims[0])/(ylims[1]-ylims[0])
		plt.axvline(x=msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs, color="black", ymin=ymin_rel, ymax=0.95, linewidth = 2)
		plt.text(msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-1.1, 0.05*ylims[0]+0.95*ylims[1], "gps stops diverging", rotation = 90, fontsize=14)

for topic, msg, unused in bag.read_messages(topics = "/rovio_divergence_handler/events"):
	if msg.vector.x==1:
		t = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
		index = (np.abs(topicdata[1][0,:]-t)).argmin()
		ymin_rel = (topicdata[1][1,index]-ylims[0])/(ylims[1]-ylims[0])
		plt.axvline(x=msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs, color="black", ymin=ymin_rel, ymax=0.95, linewidth = 2)
		plt.text(msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-1.1, 0.05*ylims[0]+0.95*ylims[1], "rovio diverging", rotation = 90, fontsize=14)
	#elif msg.vector.x==2:
		#plt.axvline(x=msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs, color="black", ymin=0.5, ymax=0.95, linewidth = 2)
		#plt.text(msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-1.1, 0.05*ylims[0]+0.95*ylims[1], "gps stops diverging", rotation = 90, fontsize=14)
mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()
