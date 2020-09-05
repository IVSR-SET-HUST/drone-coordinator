# Offboard package for control drone

## Include
- Hovering control: takeoff\_node.cpp
- _ongoing_

## Required
- **ros**: Melodic (on Ubuntu 18.04)
- **catkin workspace**: `catkin_ws`
- **mavros**: at `catkin_ws/src/mavros`
- **mavlink**: at `catkin_ws/src/mavlink`

## Build offboard
**Init offboard package and create takeoff\_node.cpp**
- `cd [path/to/catkin_ws]/src/`
- `catkin\_create\_package offboard rospy roscpp std_msgs mavros_msgs geometry_msgs`
- copy takeoff\_node.cpp to `[path/to/catkin_ws]/src/offboard/src/`
- copy and replace package.xml and CMakelists.txt to `[path/to/catkin_ws]/src/offboard`
- `catkin build offboard`
- `source [path/to/catkin_ws]/devel/setup.bash`
