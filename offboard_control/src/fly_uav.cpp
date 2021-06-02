#include <iomanip>
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
        cout << "Reach target!" << endl;
        cout << a << endl;
        return true;
    }
    else
        return false;
}
// check the x y position of UAV whether is in range or not 
bool check_marker(float error,geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target){
    Eigen::Vector2d stop;
    stop << target.pose.position.x - current.pose.position.x,
                    target.pose.position.y - current.pose.position.y;                
    double a = stop.norm();
    if (a <= error){
        cout << "Reach target!" << endl;
        cout << a << endl;
        cout << a << endl;
        return true;
    }
    else
        return false;
}

bool check_ids;
void check_cb_ids(const std_msgs::Bool msg){
    check_ids = msg.data;
}

void inputPoints(int n, float fly_point[][3])
{
    for (size_t i = 0; i < n; i++)  
    {   
        cout << "Enter position x, y, z: ";
        for (size_t j = 0; j < 3; j++)
        {   
            cin >> fly_point[i][j];
        }
        cout << endl;
    }    
}

bool check_mov;
void check_cb(const std_msgs::Bool msg){
    check_mov = msg.data;
}


geometry_msgs::PoseStamped marker_position;
void marker_cb( const geometry_msgs::PoseStamped::ConstPtr & msg){
    marker_position = *msg;
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "fly_to_marker_node");
    ros::NodeHandle nh;

    ros::Subscriber marker_p = nh.subscribe<geometry_msgs::PoseStamped>
            ("/target_pos", 10, marker_cb);
    ros::Subscriber check_move = nh.subscribe<std_msgs::Bool>
            ("/move_position", 10, check_cb);
    ros::Subscriber check_ids_marker = nh.subscribe<std_msgs::Bool>
            ("/ids_detection", 10, check_cb_ids);
    ros::Subscriber local_p = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, local_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

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
    double kp = 0.4, ki = 0.0, kd = 0.12;

    // set up the velocity of x axis  
    PidControllerBase pid_cmd(kp, ki, kd);
    pid_cmd.setUMax(0.5);
    pid_cmd.setUMin(-0.5);

    PidControllerBase pid_cmd1(kp, ki, kd);
    pid_cmd1.setUMax(0.5);
    pid_cmd1.setUMin(-0.5);

    PidControllerBase pid_cmd2(kp, ki, kd);
    pid_cmd2.setUMax(0.5);
    pid_cmd2.setUMin(-0.5);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // setup fly points 
    int n;
    cout << "Enter the number of points which you want to fly: ";
    cin >> n;
    while (n <=  0)
    {
        cout << "Please input again the number of points which you want to fly: ";
        cin >> n;
        cout << endl;
    }
    cout << endl;
    float fly_point [n][3];
    inputPoints(n, fly_point); 

    geometry_msgs::PoseStamped set_point;
    set_point.pose.position.x = fly_point[0][0];
    set_point.pose.position.y = fly_point[0][1];
    set_point.pose.position.z = fly_point[0][2];

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

    float error;
    // Error position
    cout << "Input error position: ";
    cin >> error;
    cout << endl;

    float error_marker;
    cout << "Input error marker: ";
    cin >> error_marker;
    cout << endl;

    // float error = 0.2;
    // stop while loop condition
    bool stop = false;
    // check target local frame
    bool check_target = false;
    int i = 0;
    // current z position
    double current_z;

    std::cout << "[ INFO] ----- Waiting OFFBOARD switch \n";
    while (ros::ok() && !current_state.armed && (current_state.mode != "OFFBOARD"))
    {
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] --------------- READY --------------- \n";
    while(ros::ok() && !stop)
    {   
        // fly by position control
        if (i==0)
        {
            cout << "Fly to the first points: ";
            cout << set_point.pose.position.x << " ";
            cout << set_point.pose.position.y << " ";
            cout << set_point.pose.position.z << " ";
            cout << endl;
            while (!check_target && ros::ok())
            {
                local_pos_pub.publish(set_point);
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
            ++i;
            if (i == n)
            {   
                // stop = true;
                check_ids = false;
            }
            else
            {
                
                check_target = false;
            }
        }
        // fly by pid control
        for (i = 1; i < n; i++)
        {
            set_point.pose.position.x = fly_point[i][0];
            set_point.pose.position.y = fly_point[i][1];
            set_point.pose.position.z = fly_point[i][2];
            cout << "Fly to the next point: ";
            cout << set_point.pose.position.x << " ";
            cout << set_point.pose.position.y << " ";
            cout << set_point.pose.position.z << " ";
            cout << endl;
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
            check_target = false;
        }
        ros::spinOnce();
        rate.sleep();
        // first case: when uav arrives to destination but it do not have marker.
        // hover and wait id_marker after 10 seconds uav will auto land
        if (check_ids == false)
        {   
            ros::Time t_check;
            t_check = ros::Time::now();
            while ((ros::Time::now()-t_check)<ros::Duration(10))
            {   
                cout << "No ids and hover" << endl;
                local_pos_pub.publish(set_point);
                ros::spinOnce();
                rate.sleep();
            }
        }
        // take the current z position of uav
        current_z = set_point.pose.position.z;
        bool check_target_marker = false;
        if(check_ids == true)
        {   
            // geometry_msgs::PoseStamped previous_points;
            cout << "Fly to marker" << endl;
            while (ros::ok() && !check_target_marker)
            { 
                set_point.pose.position.x = marker_position.pose.position.x;
                set_point.pose.position.y = marker_position.pose.position.y;
                set_point.pose.position.z = current_z;
                // save privious of set point of uav
                // previous_points.pose.position.x = set_point.pose.position.x;
                // previous_points.pose.position.y = set_point.pose.position.y;
                // previous_points.pose.position.z = set_point.pose.position.z;
                cout << "Fly to the next point: ";
                cout << set_point.pose.position.x << " ";
                cout << set_point.pose.position.y << " ";
                cout << set_point.pose.position.z << " ";
                cout << endl;
                // the uav fly toward marker by pid
                while (!check_target && ros::ok())
                {   
                    // ROS_INFO("Enter loop");
                    a.twist.linear.x = pid_cmd.compute(set_point.pose.position.x, current_position.pose.position.x);
                    a.twist.linear.y = pid_cmd1.compute(set_point.pose.position.y, current_position.pose.position.y);
                    a.twist.linear.z = pid_cmd2.compute(set_point.pose.position.z, current_position.pose.position.z);
                    cmd_vel.publish(a);
                    ros::spinOnce();
                    rate.sleep();
                    check_target = check_position(error_marker, current_position, set_point);
                }
                check_target = false;
                // hover above marker in 2 seconds
                ros::Time t_check;
                t_check = ros::Time::now();
                while ((ros::Time::now()-t_check)<ros::Duration(2))
                {   
                    cout << "hover above marker" << endl;
                    local_pos_pub.publish(set_point);
                    ros::spinOnce();
                    rate.sleep();
                }
                check_target_marker = check_marker(error, current_position, marker_position);
                ros::spinOnce();
                rate.sleep();
            }
        }
        bool flag = true;
        geometry_msgs::PoseStamped previous_points;
        while (flag && ros::ok())
        {   
            ros::spinOnce();
            rate.sleep();
            if (check_ids == true)
            {
                if (check_mov == true)
                {
                    set_point.pose.position.x = marker_position.pose.position.x;
                    set_point.pose.position.y = marker_position.pose.position.y;
                    set_point.pose.position.z = current_z;
                    // save privious of set point of uav
                    previous_points.pose.position.x = set_point.pose.position.x;
                    previous_points.pose.position.y = set_point.pose.position.y;
                    previous_points.pose.position.z = set_point.pose.position.z;
                    cout << "Fly to marker position";
                    cout << set_point.pose.position.x << " ";
                    cout << set_point.pose.position.y << " ";
                    cout << set_point.pose.position.z << " ";
                    cout << endl;
                    while (!check_target && ros::ok())
                    {   
                        // ROS_INFO("Enter loop");
                        a.twist.linear.x = pid_cmd.compute(set_point.pose.position.x, current_position.pose.position.x);
                        a.twist.linear.y = pid_cmd1.compute(set_point.pose.position.y, current_position.pose.position.y);
                        a.twist.linear.z = pid_cmd2.compute(set_point.pose.position.z, current_position.pose.position.z);
                        cmd_vel.publish(a);
                        ros::spinOnce();
                        rate.sleep();
                        check_target = check_position(error_marker, current_position, set_point);
                    }
                    check_target = false;
                    // hover above marker in 2 seconds
                    ros::Time t_check;
                    t_check = ros::Time::now();
                    while ((ros::Time::now()-t_check)<ros::Duration(4))
                    {   
                        cout << "hover above marker" << endl;
                        local_pos_pub.publish(set_point);
                        ros::spinOnce();
                        rate.sleep();
                    }
                    ros::spinOnce();
                    rate.sleep();
                    if (current_z <= 1.5)
                    {
                        flag = false;
                    }
                    
                }
                else
                {
                    set_point.pose.position.x = current_position.pose.position.x;
                    set_point.pose.position.y = current_position.pose.position.y;
                    set_point.pose.position.z = current_z - 1.0;
                    // update the altitude
                    current_z = set_point.pose.position.z;
                    // save privious of set point of uav
                    previous_points.pose.position.x = set_point.pose.position.x;
                    previous_points.pose.position.y = set_point.pose.position.y;
                    previous_points.pose.position.z = set_point.pose.position.z;
                    cout << "Decrease to position ";
                    cout << set_point.pose.position.x << " ";
                    cout << set_point.pose.position.y << " ";
                    cout << set_point.pose.position.z << " ";
                    cout << endl;
                    while (!check_target && ros::ok())
                    {   
                        // ROS_INFO("Enter loop");
                        a.twist.linear.x = pid_cmd.compute(set_point.pose.position.x, current_position.pose.position.x);
                        a.twist.linear.y = pid_cmd1.compute(set_point.pose.position.y, current_position.pose.position.y);
                        a.twist.linear.z = pid_cmd2.compute(set_point.pose.position.z, current_position.pose.position.z);
                        cmd_vel.publish(a);
                        ros::spinOnce();
                        rate.sleep();
                        check_target = check_position(error_marker, current_position, set_point);
                    }
                    check_target = false;
                    // hover above marker in 2 seconds
                    ros::Time t_check;
                    t_check = ros::Time::now();
                    while ((ros::Time::now()-t_check)<ros::Duration(2))
                    {   
                        cout << "hover above marker" << endl;
                        local_pos_pub.publish(set_point);
                        ros::spinOnce();
                        rate.sleep();
                    }
                    ros::spinOnce();
                    rate.sleep();
                    if (current_z <= 1.5)
                    {
                        flag = false;
                    }
                }
            }
            else
            {
                cout << "Fly to the privious point: ";
                cout << previous_points.pose.position.x << " ";
                cout << previous_points.pose.position.y << " ";
                cout << previous_points.pose.position.z << " ";
                cout << endl;
                while (!check_target && ros::ok())
                {   
                    a.twist.linear.x = pid_cmd.compute(previous_points.pose.position.x, current_position.pose.position.x);
                    a.twist.linear.y = pid_cmd1.compute(previous_points.pose.position.y, current_position.pose.position.y);
                    a.twist.linear.z = pid_cmd2.compute(previous_points.pose.position.z, current_position.pose.position.z);
                    cmd_vel.publish(a);
                    ros::spinOnce();
                    rate.sleep();
                    check_target = check_position(error, current_position, set_point);
                }
                check_target = false;
                // hover above marker in 2 seconds
                ros::Time t_check;
                t_check = ros::Time::now();
                while ((ros::Time::now()-t_check)<ros::Duration(2))
                {   
                    cout << "hover above marker" << endl;
                    local_pos_pub.publish(set_point);
                    ros::spinOnce();
                    rate.sleep();
                }
                ros::spinOnce();
                rate.sleep();
                if (check_ids==false)
                {
                   flag = false;
                }
            }
        }
        stop = true;
    }
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("[ INFO] --------------- LAND ---------------\n");
    }
    return 0;
}