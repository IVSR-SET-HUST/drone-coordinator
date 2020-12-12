# Installation

Based on [VINSMono Installation](https://github.com/ethz-asl/maplab/wiki/Installation-Ubuntu).
## 1. Prerequisites
Ubuntu  18.04.
ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
Python
Additional ROS & Python pacakge
```
    export UBUNTU_VERSION=bionic
    export ROS_VERSION=melodic
    sudo add-apt-repository "deb http://packages.ros.org/ros/ubuntu $UBUNTU_VERSION main"
    sudo apt install autotools-dev ccache doxygen dh-autoreconf git liblapack-dev libblas-dev libgtest-dev libreadline-dev libssh2-1-dev pylint clang-format python-autopep8 python-catkin-tools python-pip python-git python-setuptools python-termcolor python-wstool libatlas3-base --yes
    sudo pip install requests
```
(OPTIONAL) Install ccache for faster rebuilds
```
    sudo apt install -y ccache &&echo 'export PATH="/usr/lib/ccache:$PATH"' | tee -a ~/.bashrc &&source ~/.bashrc && echo $PATH
    ccache --max-size=10G
```
## 2. Build on ROS
Clone the repository and catkin build:
```
    export CATKIN_WS=~/catkin_ws/maplab_ws
    mkdir -p $CATKIN_WS/src
    cd $CATKIN_WS
    catkin init
    catkin config --merge-devel 
    catkin config --extend /opt/ros/melodic
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
    cd src
    git clone https://github.com/ethz-asl/maplab.git --recursive
    git clone https://github.com/ethz-asl/maplab_dependencies --recursive
    cd $CATKIN_WS
    catkin build maplab
```
# Run

## 1. Run Maplab with EuRoC MAV Dataset

Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets).

Open three terminals, launch the vins_estimator , rviz and play the bag file respectively. Take MH_01 for example
```
    source catkin_ws/maplab_ws/devel/setup.bash
    roscore& rosrun rovioli tutorial_euroc_live save_folder
```
```
    rviz -d YOUR_RVIZ_CONFIG_DIR/maplab.rviz
```
```
    rosbag play YOUR_BAG_DIR/MH_01_easy.bag 
``` 
