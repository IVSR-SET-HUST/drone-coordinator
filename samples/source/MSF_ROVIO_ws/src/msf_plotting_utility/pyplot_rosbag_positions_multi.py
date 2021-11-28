#!/usr/bin/env python
 
#code to read data from rosbag and plot them using pyplot
#usage: python pyplot_rosbag.py <rosbag_file1> <rosbag_file2> ...
import rosbag
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

#for now try without sorted assumption (could speed up)
def find_nearest(array,value):
    idx = (np.abs(array-value)).argmin()
    return idx
    
largefont=True
#from matplotlib.axes import Axes as axes
if len(sys.argv)<2:
	print("usage python pyplot_rosbag.py <rosbag_file1> <rosbag_file2> ...")
	sys.exit()

axis = sys.argv[1]
#put topics to plot here
topiclist = []
topiclist.append("/msf_core/state_out")
#topiclist.append("/noise_drift_handler/pose_output")
#topiclist.append("/noise_drift_handler/position_output")
#topiclist.append("/pos_to_error/output")
topiclist.append("/vicon/firefly_sbx/firefly_sbx")
topicdata = []
patches = []
lines = []
colors = ["#000000", "#CC0000", "#FF8000", "#00994C" , "#0000CC"]
#this is for stability
labels = ["Vicon Groundtruth", "Bad Init, No Learn, No Recover", "Good Init, No Learn, No Recover", "Good Init, No Learn, Recover", "Bad Init, Learn, Recover"]
if largefont:
	labels = ["Vicon Groundtruth", "Bad Init", "Good Init", "Good Init, Recover", "Bad Init, Learn, Recover"]
#this is for rotation
labels = ["Vicon Groundtruth", "Simple Init", "Weak Condition", "Strong Condition"]


linewidths = np.array([2.0, 2.0, 2.0, 2.0, 2.0])
if largefont:
	linewidths *= 2.5
titlefont = 28
labelfont = 24
suptitlefont = 36
if largefont:
	titlefont*=1.7
	labelfont*=1.7
	suptitlefont*=1.5

#linestyle = ['-','--','-.','-',':']
linestyle = ['-','-','-','-','-']
#give your plot a title
title = "Comparison "+axis+"-Position"
inputfile = sys.argv[2:]
#do truth here
bag = rosbag.Bag(inputfile[0])
currtopic = topiclist[1]
#only need x
tempdata1=np.array([[0],[0]])
#tempdata2=np.array([[0],[0]])
#tempdata3=np.array([[0],[0]])
for topic, msg, t in bag.read_messages(topics=currtopic):
	#print(msg.pose.pose.position.x)
	if topic == "/vicon/firefly_sbx/firefly_sbx":
		if axis=="x":
			tempdata1 = np.append(tempdata1, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.transform.translation.x]], axis=1)
		elif axis=="y":
			tempdata1 = np.append(tempdata1, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.transform.translation.y]], axis=1)
		elif axis=="z":
			tempdata1 = np.append(tempdata1, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.transform.translation.z]], axis=1)
	#topicdata
topicdata.append(tempdata1)
print (len(topicdata))
#print(topicdata[0])


#remaining
for ifile in inputfile:
	bag = rosbag.Bag(ifile)
	currtopic = topiclist[0]
	#only need x
	tempdata1=np.array([[0],[0]])
	#tempdata2=np.array([[0],[0]])
	#tempdata3=np.array([[0],[0]])
	for topic, msg, t in bag.read_messages(topics=currtopic):
		#print(msg.pose.pose.position.x)
		#case switch how to access x coordinate for all topics
		if topic == "/msf_core/state_out":
			if axis=="x":
				tempdata1 = np.append(tempdata1, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.data[0]]], axis=1)
			elif axis=="y":
				tempdata1 = np.append(tempdata1, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.data[1]]], axis=1)
			elif axis=="z":
				tempdata1 = np.append(tempdata1, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs],[msg.data[2]]], axis=1)
		#topicdata
	topicdata.append(tempdata1)
	print (len(topicdata))
	#print(topicdata[0])
for idx, topic in enumerate(topicdata):
	#plot all data in topicdata
	#print(topic)
	print(idx)
	line, = plt.plot(topic[0,1:], topic[1,1:], color=colors[idx], linewidth=linewidths[idx], linestyle=linestyle[idx])
	patches.append(mpatches.Patch(color=colors[idx], label=labels[idx]))
	lines.append(line)

#compute mean error for all data
errors = []
errorpatches = []
for i in range(1,len(topicdata)):
	errsum = 0
	ndata = 0
	for j in range(topicdata[0].shape[1]):
		idx = find_nearest(topicdata[i][0,:], topicdata[0][0,j])
		errsum+=np.abs(topicdata[0][1,j]-topicdata[i][1,idx])
		ndata+=1
	errors.append(errsum/ndata)
	labeltext="{0:.4f}".format(errors[i-1])+" m"
	errorpatches.append(mpatches.Patch(color=colors[i], label=labeltext))
print(errors)
		
#additional legend
errorleg = plt.legend(handles=errorpatches, prop={'size': labelfont}, loc=2)
errorleg.set_title("Mean Error", prop={'size':titlefont})
ax = plt.gca().add_artist(errorleg)
plt.legend(handles=patches, prop={'size': labelfont})
#plt.legend(handles=lines, prop={'size':20})
plt.xlabel('Time (s)', fontsize=titlefont)
plt.ylabel('Position '+axis+' (m)', fontsize=titlefont)
plt.grid(True)
plt.suptitle(title, fontsize=suptitlefont)
plt.tick_params(axis='both', which='major', labelsize=labelfont)
plt.ylim(-7,7)
axes = plt.gca()
ylims = axes.get_ylim()
for topic, msg, unused in bag.read_messages(topics = "/noise_drift_handler/events"):
	if msg.vector.x==3:
		t = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
		index = (np.abs(topicdata[0][0,:]-t)).argmin()
		ymin_rel = (topicdata[0][1,index]-ylims[0])/(ylims[1]-ylims[0])
		plt.axvline(x=msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs, color="black", ymin=ymin_rel, ymax=0.95, linewidth = linewidths[0])
		if largefont:
			plt.text(msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-3.5, 0.05*ylims[0]+0.95*ylims[1], "gps starts diverging", rotation = 90, fontsize=labelfont)
		else:
			plt.text(msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-1.7, 0.05*ylims[0]+0.95*ylims[1], "gps starts diverging", rotation = 90, fontsize=labelfont)
	elif msg.vector.x==4:
		t = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
		index = (np.abs(topicdata[0][0,:]-t)).argmin()
		ymin_rel = (topicdata[0][1,index]-ylims[0])/(ylims[1]-ylims[0])
		plt.axvline(x=msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs, color="black", ymin=ymin_rel, ymax=0.95, linewidth = linewidths[0])
		if largefont:
			plt.text(msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-3.5, 0.05*ylims[0]+0.95*ylims[1], "gps stops diverging", rotation = 90, fontsize=labelfont)
		else:
			plt.text(msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-1.7, 0.05*ylims[0]+0.95*ylims[1], "gps stops diverging", rotation = 90, fontsize=labelfont)

for topic, msg, unused in bag.read_messages(topics = "/rovio_divergence_handler/events"):
	if msg.vector.x==1:
		t = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
		index = (np.abs(topicdata[0][0,:]-t)).argmin()
		ymin_rel = (topicdata[0][1,index]-ylims[0])/(ylims[1]-ylims[0])
		plt.axvline(x=msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs, color="black", ymin=ymin_rel, ymax=0.95, linewidth = linewidths[0])
		if largefont:
			plt.text(msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-3.5, 0.05*ylims[0]+0.95*ylims[1], "rovio diverging", rotation = 90, fontsize=labelfont)
		else:
			plt.text(msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-1.7, 0.05*ylims[0]+0.95*ylims[1], "rovio diverging", rotation = 90, fontsize=labelfont)
	#elif msg.vector.x==2:
		#plt.axvline(x=msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs, color="black", ymin=0.5, ymax=0.95, linewidth = 2)
		#plt.text(msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-1.1, 0.05*ylims[0]+0.95*ylims[1], "gps stops diverging", rotation = 90, fontsize=14)
#plt.text(0.05,0.95,"Mean Error", fontsize=20, transform=axes.transAxes)
mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()
