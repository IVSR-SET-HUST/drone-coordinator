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
filename_rovio_msf = '/home/manh/catkin_ws/MSF_ROVIO_ws/src/msf_plotting_utility/recordings/MSF_Rovio_V1_03_difficult.bag'
filename_rovio_GPS_msf = '/home/manh/catkin_ws/MSF_ROVIO_ws/src/msf_plotting_utility/recordings/MSF_Rovio_position_V1_03_difficult.bag'

#put topics to plot here
topic_rovio_only = "/rovio/pose_with_covariance_stamped"
topic_msf = "/msf_core/state_out"

topicdata = []
patches = []
colors = ["#CC0000", "#0000CC", "#00994C","#8f9805"]
labels = ["Rovio", "Rovio+MSF", "Rovio_GPS+MSF","Rovio_GPS+MSF"]
linewidths = [1.5, 1.5, 1.5, 1.5]
#give your plot a title
title = "Position Estimated from Rovio"
bag = rosbag.Bag(filename_rovio_only)
tempdata1=np.array([[0],[0]])
tempdata2=np.array([[0],[0]])
tempdata3=np.array([[0],[0]])
tempdata4=np.array([[0],[0]])

for topic, msg, t in bag.read_messages(topics=topic_rovio_only):
    offset = msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs
    break

for topic, msg, t in bag.read_messages(topics=topic_rovio_only):
	if topic == "/rovio/pose_with_covariance_stamped":
		tempdata1 = np.append(tempdata1, [[msg.header.stamp.secs+1e-9*msg.header.stamp.nsecs-offset],[msg.pose.pose.position.y]], axis=1)
topicdata.append(tempdata1)

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
