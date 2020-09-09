#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <iostream>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    // init ros node anf node handle
    ros::init(argc, argv, "takeoff_node");
    ros::NodeHandle nh;
    // init subscriber and publisher
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    
    // input target local position from user
    int x,y, z;
    ROS_INFO("Input target local position: ");
    std::cout << "x: "; std::cin >> x;
    std::cout << "y: "; std::cin >> y;
    std::cout << "z: "; std::cin >> z;

    //set target position
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;
    
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        //publish target position to pixhawk
	local_pos_pub.publish(target_pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
