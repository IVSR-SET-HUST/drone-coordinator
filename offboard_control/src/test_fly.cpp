// /**
//  * @file offb_node.cpp
//  * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
//  * Stack and tested in Gazebo SITL
//  */
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "offboard_control/pid_controller_base.h"
// #include "offboard_control/uav_controller.h"
// #include "offboard_control/pid_controller_ros.h"

using namespace std;
using namespace Eigen;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_position;
void local_cb( const geometry_msgs::PoseStamped::ConstPtr & msg){
    current_position = *msg;
}

bool check_position(float error,geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target){
    Eigen::Vector3d stop;
    stop << target.pose.position.x - current.pose.position.x,
                    target.pose.position.y - current.pose.position.y,
                    target.pose.position.z - current.pose.position.z;
    double a = stop.norm();
    if (a <= error){
        cout << a << endl;
        return true;
    }
    else
        return false;
}


geometry_msgs::PoseStamped marker_position;
void marker_cb( const geometry_msgs::PoseStamped::ConstPtr & msg){
    marker_position = *msg;
}

bool check_mov;
void check_cb(const std_msgs::Bool msg){
    check_mov = msg.data;
}

// void hover(geometry_msgs::PoseStamped current, ros::Rate rate){
//     ros::Time t_check;
//     t_check = ros::Time::now();
//     while ((ros::Time::now()-t_check)<ros::Duration(3))     
//     {
//         local_pos
//     }
    
// }

int main(int argc, char **argv)
{   
    // UavController da;
    ros::init(argc, argv, "test_fly_node");
    ros::NodeHandle nh;

    ros::Subscriber marker_p = nh.subscribe<geometry_msgs::PoseStamped>
            ("/aruco_marker_pos", 10, marker_cb);

    ros::Subscriber check_move = nh.subscribe<std_msgs::Bool>
            ("/move_position", 10, check_cb);

    ros::Subscriber local_p = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, local_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    //         ("mavros/state", 10, &UavController::state_cb, &da);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher cmd_vel = nh.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // set up pid coefficient
    double kp = 0.4, ki = 0.01, kd = 0.12;
    PidControllerBase pid_cmd;
    pid_cmd.setKp(kp);
    pid_cmd.setKi(ki);
    pid_cmd.setKd(kd);
    pid_cmd.setUMax(0.5);
    pid_cmd.setUMin(-0.5);

    PidControllerBase pid_cmd1;
    pid_cmd1.setKp(kp);
    pid_cmd1.setKi(ki);
    pid_cmd1.setKd(kd);
    pid_cmd1.setUMax(0.5);
    pid_cmd1.setUMin(-0.5);

    PidControllerBase pid_cmd2;
    pid_cmd2.setKp(kp);
    pid_cmd2.setKi(ki);
    pid_cmd2.setKd(kd);
    pid_cmd2.setUMax(0.5);
    pid_cmd2.setUMin(-0.5);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // setup fly points
    int n;
    cout << "Enter the number of points which you want to fly: " << endl;
    cin >> n;
    float fly_point [3][n]; 
    for(int i = 0; i< n; i++){
        for (int j = 0; j < 3; j++)
        {
            cin >> fly_point[j][i];
        }    
    }


    geometry_msgs::PoseStamped set_point;
    set_point.pose.position.x = fly_point[0][0];
    set_point.pose.position.y = fly_point[1][0];
    set_point.pose.position.z = fly_point[2][0];

    geometry_msgs::TwistStamped a;
    
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(set_point);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    std::cout << last_request;

    // ros::spinOnce();
    // rate.sleep();
    float error = 0.2;
    bool stop = false;
    // bool check_takeoff = false;
    // check the uav reach the target or not
    bool check_target = false;
    int i = 1;

    std::cout << "[ INFO] ----- Waiting OFFBOARD switch \n";
    
    while (ros::ok() && !current_state.armed && (current_state.mode != "OFFBOARD"))
    {
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] --------------- READY --------------- \n";

    while(ros::ok() && !stop)
    {
        // local_pos_pub.publish(pose);
        // fly to the first point
        if (i==1)
        {
            while (!check_target && ros::ok())
            {
                // a.twist.linear.x = pid_cmd.compute(set_point.pose.position.x, current_position.pose.position.x);
                // a.twist.linear.y = pid_cmd1.compute(set_point.pose.position.y, current_position.pose.position.y);
                // a.twist.linear.z = pid_cmd2.compute(set_point.pose.position.z, current_position.pose.position.z);
                // cmd_vel.publish(a);
                local_pos_pub.publish(set_point);
                ros::spinOnce();
                rate.sleep();
                check_target = check_position(error, current_position, set_point);
            }
            ros::Time t_check;
            t_check = ros::Time::now();
            while ((ros::Time::now()-t_check)<ros::Duration(2))
            {   
                local_pos_pub.publish(set_point);
                ros::spinOnce();
                rate.sleep();
            }
            if (n == i)
            {
                stop = true;
            }
            else
            {
                ++i;
                check_target = false;
            }
        }
        if (i==2)
        {   
            set_point.pose.position.x = fly_point[0][1];
            set_point.pose.position.y = fly_point[1][1];
            set_point.pose.position.z = fly_point[2][1];
            while (!check_target && ros::ok())
            {   
                // ROS_INFO("Enter loop");
                a.twist.linear.x = pid_cmd.compute(set_point.pose.position.x, current_position.pose.position.x);
                a.twist.linear.y = pid_cmd1.compute(set_point.pose.position.y, current_position.pose.position.y);
                a.twist.linear.z = pid_cmd2.compute(set_point.pose.position.z, current_position.pose.position.z);
                cmd_vel.publish(a);
                ros::spinOnce();
                rate.sleep();
                check_target = check_position(error, current_position, set_point);
            }
            ros::Time t_check;
            t_check = ros::Time::now();
            while ((ros::Time::now()-t_check)<ros::Duration(5))
            {   
                local_pos_pub.publish(set_point);
                ros::spinOnce();
                rate.sleep();
            }
            if (n == i)
            {
                stop = true;
            }
            else
            {
                ++i;
                check_target = false;
            }
            // ++i;
            // check_target = false;
            // stop = true;
        }
        if (i==3)
        {   
            
            // pid_cmd.setUMax(0.5);
            // pid_cmd.setUMin(-0.5);
            // pid_cmd1.setUMax(0.5);
            // pid_cmd1.setUMin(-0.5);
            // set_point.pose.position.x = set_point.pose.position.x+2;
            // set_point.pose.position.y = set_point.pose.position.y+2;
            set_point.pose.position.x = fly_point[0][2];
            set_point.pose.position.y = fly_point[1][2];
            set_point.pose.position.z = fly_point[2][2];
            while (!check_target && ros::ok())
            {   
                // ROS_INFO("Enter loop");
                a.twist.linear.x = pid_cmd.compute(set_point.pose.position.x, current_position.pose.position.x);
                a.twist.linear.y = pid_cmd1.compute(set_point.pose.position.y, current_position.pose.position.y);
                a.twist.linear.z = pid_cmd2.compute(set_point.pose.position.z, current_position.pose.position.z);
                cmd_vel.publish(a);
                ros::spinOnce();
                rate.sleep();
                check_target = check_position(error, current_position, set_point);
            }
            ros::Time t_check;
            t_check = ros::Time::now();
            while ((ros::Time::now()-t_check)<ros::Duration(5))
            {   
                local_pos_pub.publish(set_point);
                ros::spinOnce();
                rate.sleep();
            }
            // ++i;
            // check_target = false;
            stop = true;
        }
    }
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("[ INFO] --------------- LAND ---------------\n");
    }
    return 0;
}
