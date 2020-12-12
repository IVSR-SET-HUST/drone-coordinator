import cv2
from cv2 import aruco
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
markerImg = np.zeros((800,800,1), dtype=np.uint8)
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
markerImg = aruco.drawMarker(dictionary,15,700,markerImg,1)

"""aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
fig = plt.figure()
nx = 4
ny = 3
for i in range(1, nx*ny+1):
    ax = fig.add_subplot(ny,nx, i)
    img = aruco.drawMarker(aruco_dict,i, 700, 1)
    plt.imshow(img, cmap=mpl.cm.gray, interpolation="nearest")
    ax.axis("off")

plt.show()"""


cv2.imwrite('marker15.png', markerImg)
cv2.imshow('test', markerImg)
cv2.waitKey(0)
