// #include <eigen3/Eigen/Dense>
// #include <iostream>
// #include <ros/ros.h>
// #include "offboard_control/pid_controller_base.h"
// using namespace std;
// using namespace Eigen;

// int main(int argc, char **argv){
//     ros::init(argc, argv, "test_node");
//     double kp =1.0, ki = 0, kd = 0;
//     PidControllerBase a;
//     a.setKp(kp);
//     double b = a.getKp();
//     cout << b << endl;
// //     Matrix2f a;
// //     a << 1, 2,
// //          4, 5;
// //     cout << a << endl;
// //     return 0;
//      // MatrixXd mat(2,2);
//      // MatrixXd a(2,2);
//      // a << 1, 2, 3, 4;
//      // mat << 1, 2,
//      //      3, 4;
//      // Vector2d u(-1,1), v(2,0);
//      // a = a*mat;
//      // cout << a << endl;
//      // std::cout << "Here is mat*mat:\n" << mat*mat << std::endl;
//      // std::cout << "Here is mat*u:\n" << mat*u << std::endl;
//      // std::cout << "Here is u^T*mat:\n" << u.transpose()*mat << std::endl;
//      // std::cout << "Here is u^T*v:\n" << u.transpose()*v << std::endl;
//      // std::cout << "Here is u*v^T:\n" << u*v.transpose() << std::endl;
//      // std::cout << "Let's multiply mat by itself" << std::endl;
//      // mat = mat*mat;
//      // std::cout << "Now mat is mat:\n" << mat << std::endl;
//      return 0;
// }
// #include <iostream>
// #include <iomanip>
// using namespace std;

// void fillArray(int **array,int row,int col)
// {
//     int i,j;
//     cout<<"Enter Data in array"<<endl;
//     for(i = 0; i < row; i++)
//     {
//         for(j = 0; j < col; j++)
//         {
//             cout<<"Enter element ["<<i<<"]["<<j<<"]: ";
//             cin>>array[i][j];
//         }
//     }
// }
// void showArray(int **array,int row,int col)
// {
//     int i,j;
//     cout<<"Table of contents"<<endl;
//     for(i = 0; i < row; i++)
//     {
//         for(j = 0; j < col; j++)
//         {
//             cout<<setw(3)<<array[i][j];     //show row in one line
//         }
//         cout<<endl;
//     }
// }
// void inputPoints(int n, float fly_point[][3])
// {
//     for (size_t i = 0; i < n; i++)  
//     {   
//         cout << "Enter position x, y, z: ";
//         for (size_t j = 0; j < 3; j++)
//         {   
//             // cout << "Enter position x, y, z: ";
//             cin >> fly_point[i][j];
//         }
//         cout << endl;
//     }
    
// }

// void showOutput(int n, float fly_point[][3])
// {
//     for (size_t i = 0; i < n; i++)  
//     {   
//         // cout << "Enter position x, y, z: ";
//         for (size_t j = 0; j < 3; j++)
//         {   
//             // cout << "Enter position x, y, z: ";
//             // cin >> fly_point[i][j];
//             cout << fly_point[i][j] << " ";
//         }
//         cout << endl;
//     }
// }
// int main()
// {
    // int row,col;
    // cout<<"Enter row: ";
    // cin>>row;
    // cout<<"Enter col: ";
    // cin>>col;

    // int array[row][col];    //declare array
    // fillArray(array,row,col);
    // showArray(array,row,col);
//     int n;
//     cout << "Enter the number of points which you want to fly: ";
//     cin >> n;
//     cout << endl;
//     float fly_point [n][3];
//     inputPoints(n, fly_point); 
//     showOutput(n, fly_point);
//     return 0;
// }

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
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber marker_p = nh.subscribe<geometry_msgs::PoseStamped>
            ("/fly_pos", 10, marker_cb);

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
    double kp = 0.4, ki = 0.0, kd = 0.12;
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
    pid_cmd2.setUMax(1.0);
    pid_cmd2.setUMin(-1.0);

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

    // geometry_msgs::PoseStamped set_point;
    // set_point.pose.position.x = 0;
    // set_point.pose.position.y = 0;
    // set_point.pose.position.z = 7; // change 15 to 7

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
    bool check_takeoff = false;
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
                local_pos_pub.publish(set_point);
                // a.twist.linear.x = pid_cmd.compute(set_point.pose.position.x, current_position.pose.position.x);
                // a.twist.linear.y = pid_cmd1.compute(set_point.pose.position.y, current_position.pose.position.y);
                // a.twist.linear.z = pid_cmd2.compute(set_point.pose.position.z, current_position.pose.position.z);
                // cmd_vel.publish(a);
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
            if (i == n)
            {
                i = i+2;
                check_target = false;
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
            // pid_cmd.setUMax(0.5);
            // pid_cmd.setUMin(-0.5);
            // pid_cmd1.setUMax(0.5);
            // pid_cmd1.setUMin(-0.5);
            // set_point.pose.position.x = set_point.pose.position.x+2;
            // set_point.pose.position.y = set_point.pose.position.y+2;
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
            ++i;
            check_target = false;
        }
        if (i==3)
        {   cout << "Enter third loop" << endl;
            geometry_msgs::PoseStamped target_point;
            target_point.pose.position.x = 0;
            target_point.pose.position.y = 0;
            target_point.pose.position.z = 1.0;

            while (!check_target && ros::ok())
            {   
                if (check_mov){
                    ROS_INFO("Enter if loop");
                    pid_cmd.setUMax(0.3);
                    pid_cmd.setUMin(-0.3);
                    pid_cmd1.setUMax(0.3);
                    pid_cmd1.setUMin(-0.3);
                    pid_cmd2.setUMax(0.1);
                    pid_cmd2.setUMin(-0.1);
                    a.twist.linear.x = pid_cmd.compute(target_point.pose.position.x, marker_position.pose.position.x);
                    a.twist.linear.y = pid_cmd1.compute(target_point.pose.position.y, marker_position.pose.position.y);
                    a.twist.linear.z = pid_cmd2.compute(current_position.pose.position.z, marker_position.pose.position.z);
                    cmd_vel.publish(a);
                    ros::spinOnce();
                    rate.sleep();
                }
                else
                {   
                    ROS_INFO("Enter else loop");
                    pid_cmd.setUMax(0.1);
                    pid_cmd.setUMin(-0.1);
                    pid_cmd1.setUMax(0.1);
                    pid_cmd1.setUMin(-0.1);
                    pid_cmd2.setUMax(0.3);
                    pid_cmd2.setUMin(-0.3);
                    
                    a.twist.linear.x = pid_cmd.compute(target_point.pose.position.x, marker_position.pose.position.x);
                    a.twist.linear.y = pid_cmd1.compute(target_point.pose.position.y, marker_position.pose.position.y);
                    a.twist.linear.z = pid_cmd2.compute(target_point.pose.position.z, marker_position.pose.position.z);
                    cmd_vel.publish(a);
                    ros::spinOnce();
                    rate.sleep();
                }
                check_target = check_position(error, marker_position, target_point);
                ros::spinOnce();
                rate.sleep();
            }
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