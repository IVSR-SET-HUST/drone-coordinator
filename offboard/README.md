# offboard package

## contain
- *include/offboard/offboard.h*
- *src/hovering_node.cpp*: keep drone hovering on input target position
- *src/offboard_node.cpp*: keep drone flying follow input waypoints
- *src/gps_offb.cpp*: get current gps position and hovering on a setpoint position
- *src/setmode_offb.cpp*: set OFFBOARD mode and ARM vehicle in simulation
- *package.xml*: ros manifests
- *CMakeLists.txt*: CMake

## required
- **ros**: Melodic (on Ubuntu 18.04)
- **px4 Firmware**: v1.10.1
- **catkin workspace**: `catkin_ws`
- **mavros**: at `catkin_ws/src/mavros`
- **mavlink**: at `catkin_ws/src/mavlink`

## usage
###### hovering node
- *connect jetson to pixhawk*         : `roslaunch mavros px4.launch`
- or - *simualation on Gazebo*        : `roslaunch px4 mavros_posix_silt.launch`
- *run hovering_node*                 : `rosrun offboard hovering`
- **check current state and position on screen**

  **input target position: x, y, z**
  
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD
- **on simualation control** `rosrun offboard setmode_offb`

###### offboard node
- *connect jetson to pixhawk*         : `roslaunch mavros px4.launch`
- or - *simualation on Gazebo*        : `roslaunch px4 mavros_posix_silt.launch`
- *run offboard_node*                 : `rosrun offboard offboard`
- **check current state and position on screen**

  **input number of target (>0)**
  
  **input target position: pos_x_i, pos_y_i, pos_z_i**
  
  **input target RPY: roll_i, pitch_i, yaw_i (in degree)**
  
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD
- **on simualation control** `rosrun offboard setmode_offb`

###### gps_offb node
- *connect jetson to pixhawk*         : `roslaunch mavros px4.launch`
- or - *simualation on Gazebo*        : `roslaunch px4 mavros_posix_silt.launch`
- *run gps_offb*                 : `rosrun offboard gps_offb`
- **drone is going to get current gps (global position)**

  ```
    Got global position: [*Latitude*, *Longitude*, *Altitude*]
  ```
- **check global position and input target**

  ```
    Input GPS position

    Latitude  (degree):

    Longitude (degree):

    Altitude  (meter) :

  ```
    
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD
- **on simualation control** `rosrun offboard setmode_offb`
