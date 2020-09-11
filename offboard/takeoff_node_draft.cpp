#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

#include <iostream>
#include <cmath>

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
    ros::init(argc, argv, "takeoff_node");
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
    while(ros::ok() && current_state.connected)
	{
		ROS_INFO_STREAM("Current state: " << current_state);
		ROS_INFO_STREAM("Current pose: " << current_pose.pose.position);
        ros::spinOnce();
        rate.sleep();
    }

	// ROS_INFO_STREAM("Current state: " << current_state);
	// ROS_INFO_STREAM("Current pose: " << current_pose.pose.position);	
    
	geometry_msgs::PoseStamped target_pose;
    // target_pose.pose.position.x = current_pose.pose.position.x;
    // target_pose.pose.position.y = current_pose.pose.position.y;
    // target_pose.pose.position.z = current_pose.pose.position.z + 2;
	std::cout << "Input target: " << std::endl;
	std::cout << "x: "; std::cin >> target_pose.pose.position.x;
	std::cout << "y: "; std::cin >> target_pose.pose.position.y;
	std::cout << "z: "; std::cin >> target_pose.pose.position.z;
	// ROS_INFO_STREAM("Target pose: " << target_pose.pose.position);    

    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
		ROS_INFO_STREAM("\nCurrent position: \n" << current_pose.pose.position);	
	    ROS_INFO_STREAM("\nCurrent state: \n" << current_state);
		ROS_INFO_STREAM("\nTarget position: \n" << target_pose.pose.position);
        
		// publish target position
		local_pos_pub.publish(target_pose);
			
		float x = std::floor(current_pose.pose.position.x);
		float y = std::floor(current_pose.pose.position.y);
		float z = std::floor(current_pose.pose.position.z);
		//float x = current_pose.pose.position.x;
		//float y = current_pose.pose.position.y;
		//float z = current_pose.pose.position.z;
		if((x == target_pose.pose.position.x) && (y == target_pose.pose.position.y) && ( z == target_pose.pose.position.z)) //break;
		{
			ROS_INFO("\nReached target\n");
			ROS_INFO("\nInput new target:\n");
			// local_pos_pub.publish(target_pose);
			
			std::cout << "Input target: " << std::endl;
			std::cout << "x: "; std::cin >> target_pose.pose.position.x;
			std::cout << "y: "; std::cin >> target_pose.pose.position.y;
			std::cout << "z: "; std::cin >> target_pose.pose.position.z;
	
			//break;
		}
		
        ros::spinOnce();
        rate.sleep();
    }
	
	// ROS_INFO("\nwhile break\n");
    return 0;
}
