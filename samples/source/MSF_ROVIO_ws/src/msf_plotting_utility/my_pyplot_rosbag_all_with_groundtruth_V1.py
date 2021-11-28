import rosbag
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import csv

#put topics to plot here
topic="/rovio/pose_with_covariance_stamped"
filename_groundtruth = '/home/manh/Lab/bag_file/V1_03_difficult/mav0/state_groundtruth_estimate0/data.csv'

#put topics to plot here
topic_msf = "/msf_core/state_out"

topicdata = []
patches = []
colors = ["#CC0000", "#0000CC", "#00994C", "#FF9999", "#9999FF", "#66FF66"]
labels = ["Estimate x", "Estimate y", "Estimate z", "Truth x", "Truth y", "Truth z"]
linewidths = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
#give your plot a title
title = "Error"
#bag = rosbag.Bag(filename_rovio_only)
tempdata1=np.array([[0],[0]])
tempdata2=np.array([[0],[0]])
tempdata3=np.array([[0],[0]])
inputfile = sys.argv[1]
bag = rosbag.Bag(inputfile)
tempdata4=np.array([[0],[0]])
tempdata5=np.array([[0],[0]])
tempdata6=np.array([[0],[0]])
for topic, msg, t in bag.read_messages(topics=topic_msf):
    offset = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
    break
for topic, msg, t in bag.read_messages(topics=topic_msf):
	if topic == "/msf_core/state_out":
		tempdata4 = np.append(tempdata4, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-offset],[msg.data[0]]], axis=1)
		tempdata5 = np.append(tempdata5, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-offset],[msg.data[1]]], axis=1)
		tempdata6 = np.append(tempdata6, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-offset],[msg.data[2]]], axis=1)
	#topicdata
topicdata.append(tempdata4)
topicdata.append(tempdata5)
topicdata.append(tempdata6)

with open(filename_groundtruth) as f:
    reader = csv.reader(f)
    header_row = next(reader)
    for row in reader:
        tempdata1 = np.append(tempdata1, [[(float(row[0])-1.40371588837906E+018)/1000000000+1.62],[float(row[1])]], axis=1)
        tempdata2 = np.append(tempdata2, [[(float(row[0])-1.40371588837906E+018)/1000000000+1.62],[float(row[2])]], axis=1)
        tempdata3 = np.append(tempdata3, [[(float(row[0])-1.40371588837906E+018)/1000000000+1.62],[float(row[3])]], axis=1)
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
plt.ylim(-7,7)
axes = plt.gca()
ylims = axes.get_ylim()

mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()
