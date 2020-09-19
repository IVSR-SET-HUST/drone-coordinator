# Offboard package for control drone

## Include
- manifests: package.xml
- CMake: CMakeLists.txt
- Hovering: hovering\_node.cpp
- Offboard: offboard\_node.cpp
- Header: offboard.h
-

## Required
- **ros**: Melodic (on Ubuntu 18.04)
- **catkin workspace**: `catkin_ws`
- **mavros**: at `catkin_ws/src/mavros`
- **mavlink**: at `catkin_ws/src/mavlink`

- Remote controller with 3 flight mode: Altitude, Offboard ans Manual
- Wifi connect between Jetson nano and Ground PC. SSH connection

## Build offboard package
**Init offboard package and create hovering\_node.cpp**
- `cd [path/to/catkin_ws]/src/`
- `catkin_create_package offboard rospy roscpp std_msgs mavros_msgs geometry_msgs`
- copy hovering\_node.cpp to `[path/to/catkin_ws]/src/offboard/src/`
- copy and replace package.xml and CMakelists.txt to `[path/to/catkin_ws]/src/offboard`
- `catkin build offboard`
- `source [path/to/catkin_ws]/devel/setup.bash`

**Update offboard\_node.cpp**
- copy offboard.h to `[path/to/catkin_ws]/src/offboard/include/offboard/`
- copy offboard\_node.cpp to `[path/to/catkin_ws]/src/offboard/src/`
- copy and replace CMakelists.txt to `[path/to/catkin_ws]/src/offboard`
- `catkin build offboard`
- `source [path/to/catkin_ws]/devel/setup.bash`

## Usage
**Hovering**
- *connect jetson to pixhawk*              : `roslaunch mavros px4.launch`
- [in other terminal] *run hovering_node*  : `rosrun offboard hovering`
- **check current state and position on screen; then input target (x,y,z)**
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD
- **drone is going to takeoff and reach the target; then hover at that**

**Offboard**
- *connect jetson to pixhawk*              : `roslaunch mavros px4.launch`
- [in other terminal] *run offboard_node*  : `rosrun offboard offboard`
- **check current state and position on screen; then input: number of target and targets (xi,yi,zi)**
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD
- **drone is going to fly in turn through the targets, then return first target**

