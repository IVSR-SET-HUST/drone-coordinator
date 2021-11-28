# kimera_rviz_markers

A collection of rviz markers for nice visualization of mainly things related to Visual Inertial Odometry.

# 1. Installation

## A. Prerequisities

- [ROS](https://www.ros.org/install/)

## B. Installation

```bash
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init

# Add workspace to bashrc.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# Clone repo
cd ~/catkin_ws/src
git clone https://github.com/ToniRV/kimera_rviz_markers.git

# Install dependencies from rosinstall file using wstool
wstool init
wstool merge kimera_rviz_markers/install/kimera_rviz_markers.rosinstall
wstool update
```

Finally, compile:

```bash
# Compile code
catkin build

# Refresh workspace
source ~/.bashrc
```

# 2. Usage

## rosrun
```
rosrun kimera_rviz_markers kimera_rviz_markers
```

## roslaunch
```
roslaunch kimera_rviz_markers kimera_rviz_markers
```

## rosservice
