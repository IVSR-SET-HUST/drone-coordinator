# Offboard package for control drone

## Include
- manifests: package.xml
- CMake: CMakeLists.txt
- Takeoff source: hovering\_node.cpp
- 


## Required
- **ros**: Melodic (on Ubuntu 18.04)
- **catkin workspace**: `catkin_ws`
- **mavros**: at `catkin_ws/src/mavros`
- **mavlink**: at `catkin_ws/src/mavlink`

## Build offboard
**Init offboard package and create hovering\_node.cpp**
- `cd [path/to/catkin_ws]/src/`
- `catkin_create_package offboard rospy roscpp std_msgs mavros_msgs geometry_msgs`
- copy hovering\_node.cpp to `[path/to/catkin_ws]/src/offboard/src/`
- copy and replace package.xml and CMakelists.txt to `[path/to/catkin_ws]/src/offboard`
- `catkin build offboard`
- `source [path/to/catkin_ws]/devel/setup.bash`

## Usage
- _connect jetson to pixhawk_         : `roslaunch mavros px4.launch`
- _run hovering_node_                 : `rosrun offboard hovering`
- **check current state and position on screen; then input target (x,y,z)**
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD
