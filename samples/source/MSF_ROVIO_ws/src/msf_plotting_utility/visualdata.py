import rosbag
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import csv
from matplotlib import pyplot as plt

topicdata = []
patches = []
title = "Ground Truth"
filename = '/home/manh/Lab/bag_file/V1_03_difficult/mav0/state_groundtruth_estimate0/data.csv'
colors = ["#CC0000", "#0000CC", "#00994C"]
labels = ["True x", "True y", "True z"]
linewidths = [1.5, 1.5, 1.5]
tempdata1=np.array([[0],[0]])
tempdata2=np.array([[0],[0]])
tempdata3=np.array([[0],[0]])
ecef_ref_orientation=0
ecef_ref_point=0
with open(filename) as f:
    reader = csv.reader(f)
    header_row = next(reader)
    highs = []
    for row in reader:
        tempdata1 = np.append(tempdata1, [[float(row[0])],[float(row[1])]], axis=1)
        tempdata2 = np.append(tempdata2, [[float(row[0])],[float(row[2])]], axis=1)
        tempdata3 = np.append(tempdata3, [[float(row[0])],[float(row[3])]], axis=1)
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
#this is for VH1
plt.ylim(-7,7)
#this is for real data
#plt.ylim(-20,20)
axes = plt.gca()
ylims = axes.get_ylim()
mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()
