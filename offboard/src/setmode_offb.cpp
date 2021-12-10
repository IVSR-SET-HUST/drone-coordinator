#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "setmode_offb");
    ros::NodeHandle nh;

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool> 
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode> 
            ("mavros/set_mode");
        
    ros::Rate rate(10.0);

    // arm
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) 
    {
        ROS_INFO("Vehicle armed");
    } 
    else 
    {
        ROS_ERROR("Arming failed");
    }

    // set mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.base_mode = 0;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) 
    {
        ROS_INFO("OFFBOARD enabled");
    } 
    else 
    {
        ROS_ERROR("Failed to set OFFBOARD");
    }

    while (ros::ok()) 
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}