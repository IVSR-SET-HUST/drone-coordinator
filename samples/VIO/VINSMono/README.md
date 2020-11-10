# Installation

Based on [VINSMono Installation](https://github.com/HKUST-Aerial-Robotics/VINS-Mono).
## 1. Prerequisites
1.1 **Ubuntu** and **ROS**
Ubuntu  18.04.
ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
additional ROS pacakge
```
    sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```

1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html)
```
    sudo apt-get install cmake libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
    mkdir -p ~/catkin_ws
    cd ~/catkin_ws
    git clone https://ceres-solver.googlesource.com/ceres-solver
    cd ceres-solver/
    mkdir build
    cd build
    cmake ..
    make -j8
    sudo make install
```

## 2. Build on ROS
Clone the repository and catkin_make:
```
    mkdir -p ~/catkin_ws/VINSMono_ws/src
    cd ~/catkin_ws/VINSMono_ws/src/
    git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
    cd ..
    catkin_make
```

# Run

## 1. Run VINSMono with EuRoC MAV Dataset

Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets).

Open three terminals, launch the vins_estimator , rviz and play the bag file respectively. Take MH_01 for example
```
    source catkin_ws/VINSMono_ws/devel/setup.bash 
    roslaunch vins_estimator euroc.launch
```
```
    source catkin_ws/VINSMono_ws/devel/setup.bash
    roslaunch vins_estimator vins_rviz.launch
```
```
    rosbag play YOUR_BAG_DIR/MH_01_easy.bag 
```
## 2. Run VINSMono with Real Dataset

**2.1 Setup sensor**
Hardware:
- Laptop Corei5
- Mono Camera: Intel® RealSense™ Depth Camera D435 (using rgb camera at grayscale topic, see [Get grayscale topic]())
- IMU Sensor: PX4 
Setup:
cd ~/catkin_ws/VINSMono_ws/src/VINS-Mono/config
mkdir my_config
Add [my_live_config.yaml]() and [my_live_config_no_extrinsic.yaml]() here.
cd ~/catkin_ws/VINSMono_ws/src/VINS-Mono/vins_estimator/launch
Add [my_live.launch]() and [my_live_no_extrinsic_param.launch]() here.
 **2.2 Run with extrinsic parameter
 
