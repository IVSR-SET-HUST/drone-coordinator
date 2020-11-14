# Installation

Based on [ROVIO Installation](https://github.com/ethz-asl/rovio).
## 1. Prerequisites

- Ubuntu 18.04.

- ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

- python

Install essential package:
```
sudo apt-get install ros-melodic-catkin
sudo apt-get install cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev build-essential
sudo apt-get install python-catkin-pkg
sudo apt-get install python-catkin-tools
sudo apt-get install freeglut3-dev
sudo apt-get install libglew-dev
```
## 2. Build on ROS
Clone the repository and catkin_make:
```
mkdir -p ~/catkin_ws/rovio_ws/src
git clone https://github.com/ANYbotics/kindr.git
catkin build -w ~/catkin_ws/rovio_ws kindr
git clone https://github.com/ethz-asl/rovio.git
cd rovio
git submodule update --init --recursive
cd ~/catkin_ws/rovio_ws
catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
```

# Run ROVIO with EuRoC MAV Dataset

Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Example, your bagfile is saved in ~/Lab/bag_file.

Config file launch with saved data file:
```
gedit ~/catkin_vio/rovio_ws/src/rovio/launch/rovio_rosbag_node.launch
```
change line: <param name="rosbag_filename" value="/root/catkin_vio/src/rovio/MH_01_easy.bag"/>
to:   <param name="rosbag_filename" value="/home/manh/Lab/bag_file/MH_01_easy.bag"/>

Note: Your rosbag container must have folder: rovio.
```
mkdir -p ~Lab/bag_file/rovio
```
Run:
```
source catkin_ws/rovio_ws/devel/setup.bash
roslaunch rovio rovio_rosbag_node.launch 
```
