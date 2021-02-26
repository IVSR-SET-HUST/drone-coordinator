1. install
git clone src to your catkin_ws folder
cd catkin_ws
catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
catkin build

2. Run
Base on https://github.com/ethz-asl/ethzasl_msf/wiki/Robust-UAV-State-Estimation
2.1. Run MSF+Rovio+GPS
Download Dataset: EuroC MAV Dataset: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets​

Vicon Room 1 03 (Download ASL Dataset Format to have full bag file + calibration + groundtruth)

source catkin_ws/devel/setup.bash
roslaunch msf_updates my_pose_sensor_V1_03_difficult_confirmed.launch 

rosparam set /use_sim_time true
rosrun rqt_reconfigure rqt_reconfigure
rosbag play Lab/bag_file/V1_03_difficult.bag --clock --pause

cd catkin_ws/MSF_ROVIO_ws/src/msf_plotting_utility/
./record_msf_only.sh 
On bagfile play window, press Spacebar to unpause

python my_pyplot_rosbag_positions_confirmed.py ./recordings/_2021-02-25-11-30-46.bag
