#include "offboard/offboard.h"

// state callback 
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// pose callback
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected)
	{
        ros::spinOnce();
        rate.sleep();
    }
	
	// check current state and position
	for(int i = 10; ros::ok() && i > 0; --i)
	{
		// ROS_INFO_STREAM("\nCurrent state: \n" << current_state);
		ROS_INFO_STREAM("\nCurrent pose: \n" << current_pose.pose.position);	
		ros::spinOnce();
        rate.sleep();
    }

	input_target();
    target_pose.pose.position.x = target_pos[0][0];
    target_pose.pose.position.y = target_pos[0][1];
    target_pose.pose.position.z = target_pos[0][2];

    // send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    ros::Time last_request = ros::Time::now();
    int i = 0;
    while(ros::ok())
    {
		ROS_INFO_STREAM("\nCurrent position: \n" << current_pose.pose.position);	
	    // ROS_INFO_STREAM("\nCurrent state: \n" << current_state);
		ROS_INFO_STREAM("\nTarget position: \n" << target_pose.pose.position);
        
		// publish target position
        if (i < target_num)
        {
            target_pose.pose.position.x = target_pos[i][0];
            target_pose.pose.position.y = target_pos[i][1];
            target_pose.pose.position.z = target_pos[i][2];
            local_pos_pub.publish(target_pose);
			ros::spinOnce();
        	rate.sleep();
        }
        else
        {
            target_pose.pose.position.x = target_pos[0][0];
            target_pose.pose.position.y = target_pos[0][1];
            target_pose.pose.position.z = target_pos[0][2];
            local_pos_pub.publish(target_pose);
	        // i = 0;
			ros::spinOnce();
        	rate.sleep();
        }
		
		      
		bool check = check_reached();
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
