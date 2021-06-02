# IVSR OFFBOARD package

***
**WARNING**

*OFFBOARD* control is dangerous. 

If you are operating on a real vehicle be sure to have a way of gaining back manual control in case something goes wrong.
***

## Contain
- *include/offboard/offboard.h* : header offboard

- *src/offboard_node.cpp*   : offboard node
- *src/offboard_lib.cpp*    : library for offboard node
- *src/setmode_offb.cpp*    : set OFFBOARD mode and ARM vehicle in simulation

- *config/config.yaml*      : prepared params to load into offboard node

- *launch/offboard.launch*  : launch file

## Required
- **ROS**             : tested on Melodic (Ubuntu 18.04)
- **PX4 Firmware**    : tested on v10.0.1 - setup [here](https://github.com/congtranv/Firmware)
- **Catkin workspace**: `catkin_ws`
- **MAVROS**          : binary installation - setup [here](https://docs.px4.io/master/en/ros/mavros_installation.html#binary-installation-debian-ubuntu)

***
- **git clone `offboard` to `catkin_ws/src/` and build `catkin build`**
***

## Usage
***
*Before run OFFBOARD node, check and modify (if need) the value of parameters in* **config/config.yaml**

*These parameters would be load at first when launch OFFBOARD node*
***

### 1. Simulation
#### 1.1 Run simulation
- run command:
```
roslaunch px4 mavros_posix_sitl.launch
```
- or run script at [here](https://github.com/congtranv/bash):
```
sh px4simulation.sh
```

#### 1.2 Run OFFBOARD node
```
roslaunch offboard offboard.launch
```
There 2 functions:
- HOVERING: drone hover at `z` meters (input from keyboard) in `hover_time` seconds (change in config/config.yaml)
- OFFBOARD: fly with the local/global setpoints that prepared in config/config.yaml or input from keyboard

#### 1.3 ARM and switch to OFFBOARD mode
```
rosrun offboard setmode_offb
```
### 2. Practice
#### 2.1 Connect Companion PC to Pixhawk 4 
- run command:
```
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
```
- or run script:
```
sh connect_px4.sh
```

#### 2.2 Run OFFBOARD node
```
roslaunch offboard offboard.launch
```
There 2 functions:
- HOVERING: drone hover at `z` meters (input from keyboard) in `hover_time` seconds (change in config/config.yaml)
- OFFBOARD: fly with the local/global setpoints that prepared in config/config.yaml or input from keyboard

#### 2.3 ARM and switch to OFFBOARD mode
Use Remote controller to ARM and switch flight mode to OFFBOARD