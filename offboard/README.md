# Offboard package for control drone

## Include
- Hovering control: takeoff_node.cpp
- _ongoing_

## Required
- **ros**: Melodic (on Ubuntu 18.04)
- **catkin workspace**: `catkin_ws`
- **mavros**: at `catkin_ws/src/mavros`
- **mavlink**: at `catkin_ws/src/mavlink`

## Build offboard
** Init offboard package and create takeoff_node.cpp **
- `cd _[path/to/catkin_ws]_/src/`
- `catkin_create_package offboard rospy roscpp std_msgs mavros_msgs geometry_msgs`
- copy takeoff_node.cpp to `_[path/to/catkin_ws]_/src/offboard/src/`
- copy package.xml and CMakelists.txt to `_[path/to/catkin_ws]_/src/offboard`
- `catkin build offboard`
