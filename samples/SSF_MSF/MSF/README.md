1. install
git clone src to your catkin_ws folder
This src folder contains: 
msf framework: https://github.com/ethz-asl/ethzasl_msf
rovio (vio algorithm): https://github.com/ethz-asl/rovio
python file for visualize result: https://gitlab.ethz.ch/huberya/msf_plotting_utility

cd catkin_ws
catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
catkin build

2. Run
Base on https://github.com/ethz-asl/ethzasl_msf/wiki/Robust-UAV-State-Estimation
2.1. Run MSF+Rovio
Download Dataset: EuroC MAV Dataset: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets​

Vicon Room 1 03 (Download ASL Dataset Format to have full bag file + calibration + groundtruth)

source catkin_ws/devel/setup.bash
roslaunch msf_updates my_pose_sensor_V1_03_difficult_confirmed.launch 

rosparam set /use_sim_time true
rosrun rqt_reconfigure rqt_reconfigure
rosbag play Lab/bag_file/V1_03_difficult.bag --clock --pause

cd catkin_ws/MSF_ROVIO_ws/src/msf_plotting_utility/
./record_msf_only.sh 

(then, your bag file result will be stored in atkin_ws/MSF_ROVIO_ws/src/msf_plotting_utility/recording)
On bagfile play window, press Spacebar to unpause

python my_pyplot_rosbag_positions_confirmed.py ./recordings/_2021-02-25-11-30-46.bag

2.2. Run MSF+Rovio+GPS

source catkin_ws/devel/setup.bash
roslaunch msf_updates my_pose_position_sensor_V1_03_difficult_confirmed.launch

rosparam set /use_sim_time true
rosrun rqt_reconfigure rqt_reconfigure
rosbag play Lab/bag_file/V1_03_difficult.bag --clock --pause

cd catkin_ws/MSF_ROVIO_ws/src/msf_plotting_utility/
./record_msf_only.sh 
On bagfile play window, press Spacebar to unpause

python my_pyplot_rosbag_positions_confirmed.py ./recordings/_2021-02-25-11-30-46.bag

3. Visualize result

In catkin_ws/src/msf_plotting_utility, we have some python file to visualize result.

File my_pyplot_rosbag_all_with_groundtruth_V1.py will plot estimated x, y, z from bag file result and truth x,y,z from groundtruth csv file (in Dataset downloaded, note: ASL Dataset Format)

Command:
cd catkin_ws/src/msf_plotting_utility
python my_pyplot_rosbag_all_with_groundtruth_V1.py ./recordings/_2021-02-25-11-30-46.bag

Your result will be like:

File my_pyplot_rosbag_all_X_axis_V2_confirmed.py, my_pyplot_rosbag_all_Y_axis_V2_confirmed.py, my_pyplot_rosbag_all_Z_axis_V2_confirmed.py will plot correspondingly Estimated X/Y/Z value from Rovio only, Rovio+MSF, Rovio+GPS+MSF, and Truth value.

note: Modify all directory to all bag file on these python files before run.

Command:
cd catkin_ws/src/msf_plotting_utility
python my_pyplot_rosbag_all_X(/Y/Z)_axis_V2_confirmed.py

Your result will be like:
