#include "offboard/offboard.h"

int main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    // subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber batt_sub = nh.subscribe<sensor_msgs::BatteryState> 
            ("mavros/battery", 10, battery_cb);
    
    // publisher
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected)
	{
        ROS_INFO_ONCE("Waiting for FCU connection ...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

	// check current pose
	for(int i = 100; ros::ok() && i > 0; --i)
	{
		ROS_INFO_STREAM("\nCurrent position: \n" << current_pose.pose.position);

		// tf Quaternion to RPY
		tf::Quaternion qc(
			current_pose.pose.orientation.x,
			current_pose.pose.orientation.y,
			current_pose.pose.orientation.z,
			current_pose.pose.orientation.w);
		tf::Matrix3x3 mc(qc);
		mc.getRPY(r, p, y);

		// print roll, pitch, yaw
		std::cout << "Roll : " << degree(r) << std::endl;
		std::cout << "Pitch: " << degree(p) << std::endl;
		std::cout << "Yaw  : " << degree(y) << std::endl;		

        batt_percent = current_batt.percentage * 100;
        std::printf("Current Battery: %.1f \n", batt_percent);

		ros::spinOnce();
        rate.sleep();
    }

    // set target pose
	input_local_target();
    target_pose.pose.position.x = target_pos[0][0];
    target_pose.pose.position.y = target_pos[0][1];
    target_pose.pose.position.z = target_pos[0][2];
    roll  = radian(target_pos[0][3]);
    pitch = radian(target_pos[0][4]);
    yaw   = radian(target_pos[0][5]);
    q.setRPY(roll, pitch, yaw);
	tf::quaternionTFToMsg(q, target_pose.pose.orientation);

    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        target_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Ready");
    
    int i=0;
    while(ros::ok())
    {
		ROS_INFO_STREAM("\nCurrent position: \n" << current_pose.pose.position);

		// tf Quaternion to RPY
		tf::Quaternion qc(
			current_pose.pose.orientation.x,
			current_pose.pose.orientation.y,
			current_pose.pose.orientation.z,
			current_pose.pose.orientation.w);
		tf::Matrix3x3 mc(qc);
		mc.getRPY(r, p, y);

		// print roll, pitch, yaw
		std::cout << "Roll : " << degree(r) << std::endl;
		std::cout << "Pitch: " << degree(p) << std::endl;
		std::cout << "Yaw  : " << degree(y) << std::endl;
        
        batt_percent = current_batt.percentage * 100;
        std::printf("Current Battery: %.1f \n", batt_percent);

		// publish target position
        if (i < target_num)
        {
            target_pose.pose.position.x = target_pos[i][0];
            target_pose.pose.position.y = target_pos[i][1];
            target_pose.pose.position.z = target_pos[i][2];
            roll  = radian(target_pos[i][3]);
            pitch = radian(target_pos[i][4]);
            yaw   = radian(target_pos[i][5]);
            q.setRPY(roll, pitch, yaw);
	        tf::quaternionTFToMsg(q, target_pose.pose.orientation);

            target_pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(target_pose);
			ros::spinOnce();
        	rate.sleep();
        }
        else
        {
            // return first target
            // target_pose.pose.position.x = target_pos[0][0];
            // target_pose.pose.position.y = target_pos[0][1];
            // target_pose.pose.position.z = target_pos[0][2];
            // roll  = radian(target_pos[0][3]);
            // pitch = radian(target_pos[0][4]);
            // yaw   = radian(target_pos[0][5]);
            // q.setRPY(roll, pitch, yaw);
	        // tf::quaternionTFToMsg(q, target_pose.pose.orientation);
            
            // keep final target
            target_pose.pose.position.x = target_pos[target_num - 1][0];
            target_pose.pose.position.y = target_pos[target_num - 1][1];
            target_pose.pose.position.z = target_pos[target_num - 1][2];
            roll  = radian(target_pos[target_num - 1][3]);
            pitch = radian(target_pos[target_num - 1][4]);
            yaw   = radian(target_pos[target_num - 1][5]);
            q.setRPY(roll, pitch, yaw);
	        tf::quaternionTFToMsg(q, target_pose.pose.orientation);
            
            target_pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(target_pose);
	        
            // loop the waypoints
            // i = 0; 
			ros::spinOnce();
        	rate.sleep();
        }
		
		// check when drone reached target      
		bool check = check_position() && check_orientation();
		std::cout << check << std::endl;
		if(check)
		{
			i = i + 1;
			ros::spinOnce();
		    rate.sleep();
		}
		else 
		{
			continue;
			ros::spinOnce();
		    rate.sleep();
		}

        ros::spinOnce();
        rate.sleep();
    }
	
    return 0;
}
