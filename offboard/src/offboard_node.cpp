#include <offboard/offboard.h>

void stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
}

void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose_ = *msg;
}

void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    current_global_ = *msg;
    global_received_ = true;
}

void velocityBodyCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_vel_ = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", 50, stateCallback);
    local_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 50, localPoseCallback);
    global_pos_sub_ = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 50, globalPositionCallback);
    velocity_body_sub_ = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_body", 50, velocityBodyCallback);

    local_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 100);
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // the setpoint publishing rate MUST be faster than 2Hz 
    ros::Rate rate(10.0);

    // wait for FCU connection
    while(ros::ok() && !current_state_.connected)
	{
        std::printf("\n[ INFO] Waiting for FCU connection \n");
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("[ INFO] FCU connected \n");
    
    // wait for GPS information
    while (ros::ok() && !global_received_) 
    {
        std::printf("[ INFO] Waiting for GPS signal \n");
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("[ INFO] GPS position received \n");
	
    std::printf("\n[ INFO] How do you want to fly?\n");
    char c;
    std::printf("- Choose 1 for HOVERING at Z(m)\n");
    std::printf("- Choose 2 to fly with SETPOINTs\n");
    std::printf("(1/2): ");
    std::cin >> c;
    
    if(c == '1') // Hover at Z (m)
    {
        double x_hover, y_hover, z_hover;
        std::printf("[ INFO] Hover at Z (m)\n");
        std::printf("z = ");
        std::cin >> z_hover; // input z from keyboard
        // hover at above current position
        x_hover = current_pose_.pose.position.x;
        y_hover = current_pose_.pose.position.y;

        target_pose_ = targetTransfer(x_hover, y_hover, z_hover);

        // send a few setpoints before starting
        std::printf("[ INFO] Setting OFFBOARD stream \n");
        for(int i = 50; ros::ok() && i > 0; --i)
        {
            target_pose_.header.stamp = ros::Time::now();
            local_pose_pub_.publish(target_pose_);
            ros::spinOnce();
            rate.sleep();
        }
        std::printf("\n[ INFO] Set OFFBOARD stream done \n");

        std::printf("[ INFO] Waiting OFFBOARD switching \n");
        while (ros::ok() && !current_state_.armed && (current_state_.mode != "OFFBOARD"))
        {
            ros::spinOnce();
            rate.sleep();
        }
        std::printf("\n[ INFO] Takeoff\n");

        takeOff(target_pose_, rate);
        landingAtFinal(targetTransfer(x_hover, y_hover, z_hover), rate);
    }
    else if(c == '2') // Fly with setpoints
    {
        std::printf("[ INFO] Fly with Setpoints\n");
        
        std::printf("\n[ INFO] Waiting for stable initial \n");
        ros::Time t_check;
        t_check = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - t_check) < ros::Duration(20))
        {
            ros::spinOnce();
            rate.sleep();
        }
        std::printf("\n[ INFO] Init stable done \n");

        // get reference point
        global_ref_.latitude = current_global_.latitude;
        global_ref_.longitude = current_global_.longitude;
        global_ref_.altitude = current_global_.altitude;
        
        // calculate offset between local position and converted global position
        geometry_msgs::Point current_enu, goal_enu;
        for(int i = 0; i < 100; i++)
        {
            current_enu = WGS84ToENU(current_global_, global_ref_);
            x_off_[i] = current_pose_.pose.position.x - current_enu.x;
            y_off_[i] = current_pose_.pose.position.y - current_enu.y;
            z_off_[i] = current_pose_.pose.position.z - current_enu.z;

            ros::spinOnce();
            rate.sleep();
        }
        for(int i = 100; i > 0; --i)
        {
            x_offset_ = x_offset_ + x_off_[i]/100;
            y_offset_ = y_offset_ + y_off_[i]/100;
            z_offset_ = z_offset_ + z_off_[i]/100;
        }

        // print information
        std::printf("\n[ INFO] Local position: [%.3f, %.3f, %.3f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
        std::printf("[ INFO] GPS position: [%.8f, %.8f, %.3f]\n", current_global_.latitude, current_global_.longitude, current_global_.altitude);
        std::printf("\n[ DEBUG] global reference: %.8f, %.8f, %.3f\n", global_ref_.latitude, global_ref_.longitude, global_ref_.altitude);
        std::printf("[ DEBUG] offset: %.3f, %.3f, %.3f\n\n", x_offset_, y_offset_, z_offset_);
        
        inputTarget(); // input target: local/global position

        // get first setpoint to set publish stream
        if(input_type_ == true) // local setpoint
        {
            target_pose_ = targetTransfer(x_target_[0], y_target_[0], z_target_[0]);
        }
        else // global setpoint
        {
            goal_enu = WGS84ToENU(goalTransfer(lat_goal_[0], lon_goal_[0], alt_goal_[0]), global_ref_);
            target_pose_ = targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_);
        }

        // send a few setpoints before starting
        std::printf("[ INFO] Setting OFFBOARD stream \n");
        for(int i = 50; ros::ok() && i > 0; --i)
        {
            target_pose_.header.stamp = ros::Time::now();
            local_pose_pub_.publish(target_pose_);
            ros::spinOnce();
            rate.sleep();
        }
        std::printf("\n[ INFO] Set OFFBOARD stream done \n");

        std::printf("[ INFO] Waiting OFFBOARD switching \n");
        while (ros::ok() && !current_state_.armed && (current_state_.mode != "OFFBOARD"))
        {
            ros::spinOnce();
            rate.sleep();
        }

        ros::param::get("z_takeoff", z_takeoff_);
        std::printf("[ INFO] Takeoff to %.3f (m)\n", z_takeoff_);
        takeOff(targetTransfer(current_pose_.pose.position.x, current_pose_.pose.position.y, z_takeoff_), rate); // take off to 3m and ready to fly

        int i = 0; // start with first setpoint in queue
        while(ros::ok())
        {
            if(input_type_) // input_type_ == true: local input
            {
                ros::param::get("desired_velocity", vel_desired_);
                if(i < (target_num_-1)) // start and middle setpoints
                {
                    final_position_ = false;
                    vel_ = velLimit(vel_desired_, current_pose_, targetTransfer(x_target_[i], y_target_[i], z_target_[i])); // drone fly at vel_desired_ (m/s)
                }
                else // final setpoint
                {
                    final_position_ = true;
                    vel_ = velLimit(vel_desired_, current_pose_, targetTransfer(x_target_[target_num_-1], y_target_[target_num_-1], z_target_[target_num_-1])); // drone land at vel_desired_ (m/s)
                }

                // target_pose_ = current_pose_ + delta_s (delta_s = vel_(m/s)*1(s))
                target_pose_ = targetTransfer(current_pose_.pose.position.x + vel_[0], current_pose_.pose.position.y + vel_[1], current_pose_.pose.position.z + vel_[2]);
                target_pose_.header.stamp = ros::Time::now();
                local_pose_pub_.publish(target_pose_); // publish target

                // print information    
                std::printf("\nCurrent local position: [%.3f, %.3f, %.3f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
                std::printf("Target local position: [%.3f, %.3f, %.3f]\n", x_target_[i], y_target_[i], z_target_[i]);
                std::printf("[ DEBUG] Desired velocity: %.3f (m/s)\n", vel_desired_);
                std::printf("[ DEBUG] Body velocity: %.3f (m/s)\n", avgBodyVelocity(current_vel_));

                // calculate distance between current_pose_ and target_pose_
                distance_ = distanceMeasure(current_pose_, targetTransfer(x_target_[i], y_target_[i], z_target_[i]));
                std::printf("Distance to target: %.3f (m) \n", distance_);

                // check when drone reached setpoint
                bool target_reached = checkPosition(check_error_, current_pose_, targetTransfer(x_target_[i], y_target_[i], z_target_[i]));
                std::printf("[ DEBUG] Target reached: %s\n", target_reached ? "true" : "false");

                if(target_reached && !final_position_) // drone reached start or middle setpoint(s)
                {
                    // print information
                    std::printf("\n[ INFO] Reached position: [%.3f, %.3f, %.3f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);   
                    std::printf("- Target position: [%.3f, %.3f, %.3f]\n", x_target_[i], y_target_[i], z_target_[i]);
                    std::printf("- Next target: [%.3f, %.3f, %.3f]\n", x_target_[i+1], y_target_[i+1], z_target_[i+1]);
                    
                    ros::param::get("hover_time", hover_time_);
                    std::printf("- Hovering in %.3f (s)\n", hover_time_);
                    hoverAt(hover_time_, targetTransfer(x_target_[i], y_target_[i], z_target_[i]), rate); // hover at setpoint in hover_time_ second(s)
                    // landingAt(targetTransfer(x_target_[i], y_target_[i], z_target_[i]), rate); // land at each setpoint to unpack - comment if don't want to land at each setpoint
                    i+=1; // next setpoint
                }
                if(target_reached && final_position_) // drone reached final setpoint
                { 
                    // print information
                    std::printf("\n[ INFO] Reached FINAL position: [%.3f, %.3f, %.3f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
                    
                    ros::param::get("hover_time",hover_time_);
                    std::printf("- Hovering in %.3f (s) before land\n", hover_time_);
                    hoverAt(hover_time_, targetTransfer(x_target_[target_num_-1], y_target_[target_num_-1], z_target_[target_num_-1]), rate); // hover at setpoint in hover_time_ second(s)
                    landingAtFinal(targetTransfer(x_target_[target_num_-1], y_target_[target_num_-1], z_target_[target_num_-1]), rate);
                    break;
                }
            }
            else // // input_type_ == false: global input
            {
                // convert global to local position
                if(i < (goal_num_-1)) // start and middle setpoints
                {
                    final_position_ = false;
                    goal_enu = WGS84ToENU(goalTransfer(lat_goal_[i], lon_goal_[i], alt_goal_[i]), global_ref_); 
                }
                else
                {
                    final_position_ = true;
                    goal_enu = WGS84ToENU(goalTransfer(lat_goal_[goal_num_-1], lon_goal_[goal_num_-1], alt_goal_[goal_num_-1]), global_ref_);
                }
                ros::param::get("desired_velocity", vel_desired_);
                vel_ = velLimit(vel_desired_, current_pose_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_)); // drone land at vel_desired_ (m/s)

                // target_pose_ = current_pose_ + delta_s (delta_s = vel_(m/s)*1(s))
                target_pose_ = targetTransfer(current_pose_.pose.position.x + vel_[0], current_pose_.pose.position.y + vel_[1], current_pose_.pose.position.z + vel_[2]);
                target_pose_.header.stamp = ros::Time::now();
                local_pose_pub_.publish(target_pose_); // publish target

                // print information
                std::printf("\nCurrent GPS position: [%.8f, %.8f, %.3f]\n", current_global_.latitude, current_global_.longitude, current_global_.altitude);
                std::printf("Goal GPS position: [%.8f, %.8f, %.3f]\n", lat_goal_[i], lon_goal_[i], alt_goal_[i]);
            
                std::printf("\nCurrent local position: [%.3f, %.3f, %.3f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
                std::printf("Target local position: [%.3f, %.3f, %.3f]\n", targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.x, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.y, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.z);
                std::printf("[ DEBUG] Desired velocity: %.3f (m/s)\n", vel_desired_);
                std::printf("[ DEBUG] Body velocity: %.3f (m/s)\n", avgBodyVelocity(current_vel_));

                // calculate distance between current_pose_ and target_pose_
                distance_ = distanceMeasure(current_pose_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_));
                std::printf("Distance to goal: %.3f (m) \n", distance_);

                // check when drone reached setpoint
                bool target_reached = checkPosition(check_error_, current_pose_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_));
                std::printf("[ DEBUG] Target reached: %s\n", target_reached ? "true" : "false");

                if(target_reached && !final_position_) // drone reached start or middle setpoint(s)
                {
                    // print information
                    std::printf("\n[ INFO] Reached position: [%.8f, %.8f, %.3f]\n", current_global_.latitude, current_global_.longitude, current_global_.altitude);   
                    std::printf("- Goal position: [%.8f, %.8f, %.3f]\n", lat_goal_[i], lon_goal_[i], alt_goal_[i]);
                    std::printf("- Next goal: [%.8f, %.8f, %.3f]\n", lat_goal_[i+1], lon_goal_[i+1], alt_goal_[i+1]);
                    std::printf("\n[ INFO] Local position: [%.3f, %.3f, %.3f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);   
                    std::printf("- Converted target: [%.3f, %.3f, %.3f]\n", targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.x, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.y, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.z);

                    ros::param::get("hover_time",hover_time_);
                    std::printf("- Hovering in %.3f (s)\n", hover_time_);
                    hoverAt(hover_time_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_), rate); // hover at setpoint in hover_time_ second(s)
                    // landingAt(targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_), rate); // land at each setpoint to unpack - comment if don't want to land at each setpoint
                    i+=1; // next setpoint
                }
                if(target_reached && final_position_) // drone reached final setpoint
                {
                    // print information
                    std::printf("\n[ INFO] Reached FINAL position: [%.8f, %.8f, %.3f]\n", current_global_.latitude, current_global_.longitude, current_global_.altitude);

                    ros::param::get("hover_time",hover_time_);
                    std::printf("- Hovering in %.3f (s) before land\n", hover_time_);
                    hoverAt(hover_time_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_), rate); // hover at setpoint in hover_time_ second(s)
                    landingAtFinal(targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_), rate);
                    break;
                }
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
    else
    {
        std::printf("\n[ WARN] Not avaible function - Please Relaunch\n");
    }

    return 0;
}