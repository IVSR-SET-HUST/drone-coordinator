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
- **mavros**          : at `catkin_ws/src/mavros`
- **mavlink**         : at `catkin_ws/src/mavlink`

- **copy `offboard` directory to `catkin_ws/src` and build**

## usage
###### hovering node
- *connect jetson to pixhawk*         : `roslaunch mavros px4.launch`
- *run hovering_node*                 : `rosrun offboard hovering`
- **check current state and position on screen**

  **input target position: x, y, z**
  
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD
- **on simualation control** `rosrun offboard setmode_offb`

###### offboard node
- *connect jetson to pixhawk*         : `roslaunch mavros px4.launch`
- *run offboard_node*                 : `rosrun offboard offboard`
- **check current state and position on screen**

  **input number of target (>0)**
  
  **input target position: pos_x_i, pos_y_i, pos_z_i**
  
  **input target RPY: roll_i, pitch_i, yaw_i (in degree)**
  
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD
- **on simualation control** `rosrun offboard setmode_offb`

###### gps_offb node
- *connect jetson to pixhawk*         : `roslaunch mavros px4.launch`
- *run gps_offb*                 : `rosrun offboard gps_offb`
- **drone is going to get current gps (global position)**

  ```
    ...
  
    goal_position.latitude = global_position.latitude;
    
    goal_position.longitude = global_position.longitude;
    
    goal_position.altitude = global_position.altitude;
    
    ...
  ```
    
  **setpoint is:**
  
  ```
    ...
  
    goal_position.altitude = goal_position.altitude + 2.0;
    
    goal_position.latitude = goal_position.latitude + 0.00001;
    
    goal_position.longitude = goal_position.longitude + 0.00001;
    
    ...
  ```
    
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD
- **on simualation control** `rosrun offboard setmode_offb`
