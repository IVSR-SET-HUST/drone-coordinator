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
filename_rovio_only = '/home/manh/catkin_ws/MSF_ROVIO_ws/src/msf_plotting_utility/recordings/Rovio_only_V1_03_difficult.bag'
filename_rovio_msf = '/home/manh/catkin_ws/MSF_ROVIO_ws/src/msf_plotting_utility/recordings/MSF_ROVIO_V1_03_difficult_nonzero_nonfixed.bag'
filename_rovio_GPS_msf = '/home/manh/catkin_ws/MSF_ROVIO_ws/src/msf_plotting_utility/recordings/MSF_Rovio_position_V1_03_difficult.bag'

#put topics to plot here
topic_rovio_only = "/rovio/pose_with_covariance_stamped"
topic_msf = "/msf_core/state_out"

topicdata = []
patches = []
colors = ["#CC0000", "#0000CC", "#00994C","#8f9805"]
labels = ["Rovio", "Rovio+MSF", "Rovio+Position+MSF","Ground Truth"]
linewidths = [1.5, 1.5, 1.5, 1.5]
#give your plot a title
title = "Y axis V1 03 difficult"
bag = rosbag.Bag(filename_rovio_only)
tempdata1=np.array([[0],[0]])
tempdata2=np.array([[0],[0]])
tempdata3=np.array([[0],[0]])
tempdata4=np.array([[0],[0]])

for topic, msg, t in bag.read_messages(topics=topic_rovio_only):
    offset1 = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
    break
for topic, msg, t in bag.read_messages(topics=topic_rovio_only):
	if topic == "/rovio/pose_with_covariance_stamped":
		tempdata1 = np.append(tempdata1, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-offset1],[-msg.pose.pose.position.y]], axis=1)
topicdata.append(tempdata1)

bag2 = rosbag.Bag(filename_rovio_msf)
for topic, msg, t in bag2.read_messages(topics=topic_msf):
    offset2 = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
    break
for topic, msg, t in bag2.read_messages(topics=topic_msf):
	if topic == "/msf_core/state_out":
		tempdata2 = np.append(tempdata2, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-offset2],[msg.data[1]]], axis=1)
topicdata.append(tempdata2)

bag3 = rosbag.Bag(filename_rovio_GPS_msf)
for topic, msg, t in bag3.read_messages(topics=topic_msf):
    offset3 = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
    break
for topic, msg, t in bag3.read_messages(topics=topic_msf):
	if topic == "/msf_core/state_out":
		tempdata3 = np.append(tempdata3, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-offset3],[msg.data[1]]], axis=1)
topicdata.append(tempdata3)

with open(filename_groundtruth) as f:
    reader = csv.reader(f)
    header_row = next(reader)
    highs = []
    for row in reader:
        tempdata4 = np.append(tempdata4, [[(float(row[0])-1.40371588837906E+018)/1000000000+1.62],[float(row[2])]], axis=1)
topicdata.append(tempdata4)
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
