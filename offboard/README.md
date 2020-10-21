# offboard package

## contain
- *include/offboard/offboard.h*: header
- *src/hovering_node.cpp*      : keep drone hovering on input target position
- *src/offboard_node.cpp*      : keep drone flying follow input waypoints
- *src/gps_offb.cpp*           : get current gps position and hovering on a setpoint position
- *src/setmode_offb.cpp*       : set OFFBOARD mode and ARM vehicle in simulation
- *package.xml*                : ros manifests
- *CMakeLists.txt*             : CMakeLists

## required
- **ros**             : Melodic (on Ubuntu 18.04)
- **catkin workspace**: `catkin_ws`
- **mavros**          : [here](https://dev.px4.io/master/en/ros/mavros_installation.html)

- **copy `offboard` directory to `catkin_ws/src/` and build**

## usage
###### hovering node
- *connect jetson to pixhawk*         : `roslaunch mavros px4.launch`
  
  on simulation                       : `roslaunch px4 mavros_posix_sitl.launch`
- *run hovering_node*                 : `rosrun offboard hovering`
- **check current position on screen**

  **input target height for hovering (in meter): z**
  
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD

  on simualation: `rosrun offboard setmode_offb`

###### offboard node
- *connect jetson to pixhawk*         : `roslaunch mavros px4.launch`
  
  on simulation                       : `roslaunch px4 mavros_posix_sitl.launch`
- *run offboard_node*                 : `rosrun offboard offboard`
- **check current pose on screen**

  **input number of target (>0)**
  
  **input target position (in meter): pos_x_i, pos_y_i, pos_z_i**
  
  **input target Yaw rotation (in degree): yaw_i**
  
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD

  on simualation: `rosrun offboard setmode_offb`

###### gps_offb node
- *connect jetson to pixhawk*         : `roslaunch mavros px4.launch`
  
  on simulation                       : `roslaunch px4 mavros_posix_sitl.launch`
- *run gps_offb*                 : `rosrun offboard gps_offb`
- **drone is going to get current gps (global position)**

  ```
    Current GPS position:: [*Latitude*, *Longitude*, *Altitude*]
  ```
- **check global position and input target**
- **input number of goal (>0)** 
- **input goal lat, lon, alt**
```
  Goal (i) position:
  Latitude  i (in degree):
  Longitude i (in degree):
  Altitude  i  (in meter):
```
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD

  on simualation: `rosrun offboard setmode_offb`
