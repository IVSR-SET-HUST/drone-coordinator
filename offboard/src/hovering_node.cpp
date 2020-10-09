#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

#include <iostream>

// declare variants
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped target_pose;

// callback functions
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

// main functions
int main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc, argv, "hovering");
    ros::NodeHandle nh;

    // subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);

    // publisher
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ROS_INFO_ONCE("Connecting to FCU ...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

	// check current position
	for(int i = 10; ros::ok() && i > 0; --i)
	{
		ROS_INFO_STREAM("\nCurrent pose: \n" << current_pose.pose.position);	
		ros::spinOnce();
        rate.sleep();
    }

    // input target position, hovering at current (x, y) 
    target_pose.pose.position.x = current_pose.pose.position.x;
    target_pose.pose.position.y = current_pose.pose.position.y;
	std::cout << "Input a height for hovering: " << std::endl;
	std::cout << "z: "; std::cin >> target_pose.pose.position.z;

    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        target_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Ready");

    // publish target, keep drone hovering
    while(ros::ok())
    {
        ROS_INFO_STREAM("\nCurrent position: \n" << current_pose.pose.position);	

        target_pose.header.stamp = ros::Time::now();   
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
