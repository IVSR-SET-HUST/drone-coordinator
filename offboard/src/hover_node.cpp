#include "offboard/offboard.h"
#include "offboard/logging.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hover");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, localPose_cb);
    ros::Subscriber batt_sub = nh.subscribe<sensor_msgs::BatteryState> 
            ("mavros/battery", 10, battery_cb);

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Rate rate(20.0);

    while(ros::ok() && !current_state.connected)
	{
        std::cout << "[ INFO] Waiting for FCU connection...\n";
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] FCU connected \n";

    // std::cout << "[ INFO] Waiting for stable initial... \n";
    
    // t_check = ros::Time::now();
    // while (ros::ok() && (ros::Time::now() - t_check) < ros::Duration(20))
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // std::cout << "[ INFO] Stable initial done \n";
    // ros::Duration(1).sleep();
    for(int i = 50; ros::ok() && i > 0; --i)
    {
        ros::spinOnce();
        rate.sleep();
    }
    
    std::printf("\nCurrent local position: [%.3f, %.3f, %.3f]\n", 
                current_pose.pose.position.x, 
                current_pose.pose.position.y, 
                current_pose.pose.position.z);

    // input target position, hovering at current (x, y) 
    target_pose.pose.position.x = current_pose.pose.position.x;
    target_pose.pose.position.y = current_pose.pose.position.y;
	std::cout << "\nInput z for hovering (m): " << std::endl;
	std::cout << "z =  "; std::cin >> target_pose.pose.position.z;

    // send a few setpoints before starting
    std::cout << "[ INFO] Setting OFFBOARD stream...\n";
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        target_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] Set OFFBOARD stream done \n";

    std::cout << "[ INFO] Waiting arm and takeoff... \n";
    while (ros::ok() && !current_state.armed)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // publish target, keep drone hovering
    while(ros::ok())
    {
        std::printf("\nCurrent local position: [%.3f, %.3f, %.3f]\n", 
                        current_pose.pose.position.x, 
                        current_pose.pose.position.y, 
                        current_pose.pose.position.z);	
        batt_percent = current_batt.percentage * 100;
        std::printf("Current Battery: %.1f \n", batt_percent);

        target_pose.header.stamp = ros::Time::now();   
        local_pos_pub.publish(target_pose);

        if (!current_state.armed)
        {
            break;
        }
        else
        {
            ros::spinOnce();
            rate.sleep();
        }
    }

    return 0;
}