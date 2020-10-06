#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

#include <iostream>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hovering");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

	// check current state and position
	for(int i = 10; ros::ok() && i > 0; --i)
	{
		ROS_INFO_STREAM("\nCurrent state: \n" << current_state);
		ROS_INFO_STREAM("\nCurrent pose: \n" << current_pose.pose.position);	
		ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position.x = current_pose.pose.position.x;
    target_pose.pose.position.y = current_pose.pose.position.y;
	std::cout << "Input hight target: " << std::endl;
	// std::cout << "x: "; std::cin >> target_pose.pose.position.x;
	// std::cout << "y: "; std::cin >> target_pose.pose.position.y;
	std::cout << "z: "; std::cin >> target_pose.pose.position.z;

    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
		ROS_INFO_STREAM("\nCurrent state: \n" << current_state);
        ROS_INFO_STREAM("\nCurrent position: \n" << current_pose.pose.position);	
	    // ROS_INFO_STREAM("\nTarget position: \n" << target_pose.pose.position);

        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
