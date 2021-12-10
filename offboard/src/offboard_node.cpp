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

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool> 
            ("mavros/cmd/arming");

    nh.param<bool>("/offboard_node/delivery", delivery_, delivery_);
    nh.param<bool>("/offboard_node/simulation", simulation_, simulation_);
    nh.param<bool>("/offboard_node/return_home", return_home_, return_home_);
    nh.getParam("/offboard_node/number_of_target", target_num_);
    nh.getParam("/offboard_node/target_error", target_error_);
    nh.getParam("/offboard_node/x_pos", x_target_);
    nh.getParam("/offboard_node/y_pos", y_target_);
    nh.getParam("/offboard_node/z_pos", z_target_);
    nh.getParam("/offboard_node/number_of_goal", goal_num_);
    nh.getParam("/offboard_node/goal_error", goal_error_);
    nh.getParam("/offboard_node/latitude", lat_goal_);
    nh.getParam("/offboard_node/longitude", lon_goal_);
    nh.getParam("/offboard_node/altitude", alt_goal_);
    nh.getParam("/offboard_node/z_takeoff", z_takeoff_);
    nh.getParam("/offboard_node/land_error", land_error_);
    nh.getParam("/offboard_node/takeoff_hover_time", takeoff_time_);
    nh.getParam("/offboard_node/hover_time", hover_);
    nh.getParam("/offboard_node/unpack_time", unpack_time_);
    nh.getParam("/offboard_node/desired_velocity", vel_desired_);
    nh.getParam("/offboard_node/land_velocity", land_vel_);
    nh.getParam("/offboard_node/return_velcity", return_vel_);

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
    if(simulation_)
    {
        std::printf("\n[ NOTICE] Prameter 'simulation' is set true\n");
        std::printf("          OFFBOARD node will automatic ARM and set OFFBOARD mode\n");
        std::printf("          Continue if run a simulation OR SHUTDOWN node if run in drone\n");
        std::printf("          Set parameter 'simulation' to false or not set (default = false)\n");
        std::printf("          and relaunch node for running in drone\n");
        std::printf("          > roslaunch offboard offboard.launch simulation:=false\n");
    }
    else
    {
        std::printf("\n[ NOTICE] Prameter 'simulation' is set false or not set (default = false)\n");
        std::printf("          OFFBOARD node will wait for ARM and set OFFBOARD mode from RC controller\n");
        std::printf("          Continue if run in drone OR shutdown node if run a simulation\n");
        std::printf("          Set parameter 'simulation' to true and relaunch node for simulation\n");
        std::printf("          > roslaunch offboard offboard.launch simulation:=true\n");
    }    
	
    std::printf("\n[ INFO] Please choose mode\n");
    char c;
    std::printf("- Choose 1 for hovering at Z(m)\n");
    std::printf("- Choose 2 to fly with mission\n");
    std::printf("(1/2): ");
    std::cin >> c;
    
    if(c == '1') // Hover at Z (m)
    {
        double x_hover, y_hover, z_hover;
        std::printf("[ INFO] Mode 1: Hovering at Z(m)\n");
        std::printf("        Input Z = ");
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
        std::printf("\n[ INFO] OFFBOARD stream is set\n");

        if(simulation_)
        {
            std::printf("        Ready to takeoff\n");
        }
        else
        {
            std::printf("\n[ INFO] Waiting switching (ARM and OFFBOARD mode) from RC\n");
        }
        
        while (ros::ok() && !current_state_.armed && (current_state_.mode != "OFFBOARD"))
        {
            // std::printf("[ DEBUG] simulation: %s\n", simulation_ ? "true" : "false");
            if(simulation_)
            {
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
                if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) 
                {
                    ROS_INFO("OFFBOARD enabled");
                } 
                else 
                {
                    ROS_ERROR("Failed to set OFFBOARD");
                }
                break;
            }
            else
            {
                ros::spinOnce();
                rate.sleep();
            }
        }

        std::printf("\n[ INFO] Takeoff to %.1f (m)\n", z_hover);

        takeOff(target_pose_, rate);
        landingAtFinal(targetTransfer(x_hover, y_hover, z_hover), rate);
    }
    else if(c == '2') // Fly with setpoints
    {
        std::printf("\n[ INFO] Mode 2: Fly with mission\n");
        std::printf("\n[ INFO] Parameter 'return_home' is set %s\n", return_home_ ? "true":"false");
        std::printf("[ INFO] Parameter 'delivery' is set %s\n", delivery_ ? "true":"false");
        
        std::printf("\n[ INFO] Waiting for stable state\n");
        ros::Time t_check;
        t_check = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - t_check) < ros::Duration(15))
        {
            ros::spinOnce();
            rate.sleep();
        }
        
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
        std::printf("[ INFO] Get a stable state\n");
        std::printf("\n");
        home_pose_ = current_pose_;
        std::printf("\n[ INFO] Got HOME position: [%.1f, %.1f, %.1f]\n", home_pose_.pose.position.x, home_pose_.pose.position.y, home_pose_.pose.position.z);
        std::printf("        latitude : %.8f\n", current_global_.latitude);
        std::printf("        longitude: %.8f\n", current_global_.longitude);
        std::printf("        altitude : %.8f\n", current_global_.altitude);
        std::printf("\n");
        // print information
        std::printf("\n[ INFO] Current Local position: [%.3f, %.3f, %.3f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
        std::printf("[ INFO] Current GPS position: [%.8f, %.8f, %.3f]\n", current_global_.latitude, current_global_.longitude, current_global_.altitude);
        // std::printf("\n[ DEBUG] global reference: %.8f, %.8f, %.3f\n", global_ref_.latitude, global_ref_.longitude, global_ref_.altitude);
        // std::printf("[ DEBUG] offset: %.3f, %.3f, %.3f\n\n", x_offset_, y_offset_, z_offset_);
        
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
        std::printf("\n[ INFO] OFFBOARD stream is set\n");

        if(simulation_)
        {
            std::printf("        Ready to takeoff\n");
        }
        else
        {
            std::printf("\n[ INFO] Waiting switching (ARM and OFFBOARD mode) from RC\n");
        }
        
        while (ros::ok() && !current_state_.armed && (current_state_.mode != "OFFBOARD"))
        {
            // std::printf("[ DEBUG] simulation: %s\n", simulation_ ? "true" : "false");
            if(simulation_)
            {
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
                if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) 
                {
                    ROS_INFO("OFFBOARD enabled");
                } 
                else 
                {
                    ROS_ERROR("Failed to set OFFBOARD");
                }
                break;
            }
            else
            {
                ros::spinOnce();
                rate.sleep();
            }
        }

        // ros::param::get("z_takeoff", z_takeoff_);
        std::printf("\n[ INFO] Takeoff to %.1f (m)\n", z_takeoff_);
        takeOff(targetTransfer(current_pose_.pose.position.x, current_pose_.pose.position.y, z_takeoff_), rate); // take off to 3m and ready to fly

        int i = 0; // start with first setpoint in queue
        while(ros::ok())
        {
            if(input_type_) // input_type_ == true: local input
            {
                // ros::param::get("desired_velocity", vel_desired_);
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
                std::printf("\nCurrent local position: [%.1f, %.1f, %.1f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
                std::printf("Target local position: [%.1f, %.1f, %.1f]\n", x_target_[i], y_target_[i], z_target_[i]);
                // std::printf("[ DEBUG] Desired velocity: %.2f (m/s)\n", vel_desired_);
                // std::printf("[ DEBUG] Body velocity: %.2f (m/s)\n", avgBodyVelocity(current_vel_));

                // calculate distance between current_pose_ and target_pose_
                distance_ = distanceMeasure(current_pose_, targetTransfer(x_target_[i], y_target_[i], z_target_[i]));
                std::printf("Distance to target: %.1f (m) \n", distance_);

                // check when drone reached setpoint
                check_error_ = target_error_;
                bool target_reached = checkPosition(check_error_, current_pose_, targetTransfer(x_target_[i], y_target_[i], z_target_[i]));
                // std::printf("[ DEBUG] Target reached: %s\n", target_reached ? "true" : "false");

                if(target_reached && !final_position_) // drone reached start or middle setpoint(s)
                {
                    // print information
                    std::printf("\n[ INFO] Reached position: [%.1f, %.1f, %.1f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);   
                    std::printf("- Target position: [%.1f, %.1f, %.1f]\n", x_target_[i], y_target_[i], z_target_[i]);
                    std::printf("- Next target: [%.1f, %.1f, %.1f]\n", x_target_[i+1], y_target_[i+1], z_target_[i+1]);
                    
                    // ros::param::get("hover_time", hover_time_);
                    hover_time_ = hover_;
                    std::printf("- Hovering in %.1f (s)\n", hover_time_);
                    
                    if(delivery_)
                    {
                        hoverAt(hover_time_, targetTransfer(x_target_[i], y_target_[i], z_target_[i]), rate); // hover at setpoint in hover_time_ second(s)
                        landingAt(targetTransfer(x_target_[i], y_target_[i], z_target_[i]), rate); // land at each setpoint to unpack - comment if don't want to land at each setpoint
                    }
                    else
                    {
                        hoverAt(hover_time_, targetTransfer(x_target_[i], y_target_[i], z_target_[i]), rate); // hover at setpoint in hover_time_ second(s)
                    }
                    
                    i+=1; // next setpoint
                }
                if(target_reached && final_position_) // drone reached final setpoint
                { 
                    // print information
                    std::printf("\n[ INFO] Reached FINAL position: [%.1f, %.1f, %.1f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
                    
                    if(!return_home_)
                    {
                        // ros::param::get("hover_time",hover_time_);
                        hover_time_ = hover_;
                        std::printf("- Hovering in %.1f (s) before land\n", hover_time_);
                        hoverAt(hover_time_, targetTransfer(x_target_[target_num_-1], y_target_[target_num_-1], z_target_[target_num_-1]), rate); // hover at setpoint in hover_time_ second(s)
                        landingAtFinal(targetTransfer(x_target_[target_num_-1], y_target_[target_num_-1], z_target_[target_num_-1]), rate);
                    }
                    else
                    {
                        if(delivery_)
                        {
                            hoverAt(hover_time_, targetTransfer(x_target_[target_num_-1], y_target_[target_num_-1], z_target_[target_num_-1]), rate); // hover at setpoint in hover_time_ second(s)
                            landingAt(targetTransfer(x_target_[target_num_-1], y_target_[target_num_-1], z_target_[target_num_-1]), rate); // land at each setpoint to unpack - comment if don't want to land at each setpoint
                        }
                        else
                        {
                            hoverAt(hover_time_, targetTransfer(x_target_[target_num_-1], y_target_[target_num_-1], z_target_[target_num_-1]), rate); // hover at setpoint in hover_time_ second(s)
                        }
                        home_pose_.pose.position.z = z_target_[target_num_-1];
                        std::printf("\n[ INFO] Returning home\n");
                        returnHome(home_pose_, rate);
                    }
                    
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
                // ros::param::get("desired_velocity", vel_desired_);
                vel_ = velLimit(vel_desired_, current_pose_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_)); // drone land at vel_desired_ (m/s)

                // target_pose_ = current_pose_ + delta_s (delta_s = vel_(m/s)*1(s))
                target_pose_ = targetTransfer(current_pose_.pose.position.x + vel_[0], current_pose_.pose.position.y + vel_[1], current_pose_.pose.position.z + vel_[2]);
                target_pose_.header.stamp = ros::Time::now();
                local_pose_pub_.publish(target_pose_); // publish target

                // print information
                std::printf("\nCurrent GPS position: [%.8f, %.8f, %.3f]\n", current_global_.latitude, current_global_.longitude, current_global_.altitude);
                std::printf("Goal GPS position: [%.8f, %.8f, %.3f]\n", lat_goal_[i], lon_goal_[i], alt_goal_[i]);
            
                std::printf("\nCurrent local position: [%.1f, %.1f, %.1f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
                std::printf("Target local position: [%.1f, %.1f, %.1f]\n", targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.x, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.y, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.z);
                // std::printf("[ DEBUG] Desired velocity: %.3f (m/s)\n", vel_desired_);
                // std::printf("[ DEBUG] Body velocity: %.3f (m/s)\n", avgBodyVelocity(current_vel_));

                // calculate distance between current_pose_ and target_pose_
                distance_ = distanceMeasure(current_pose_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_));
                std::printf("Distance to goal: %.1f (m) \n", distance_);

                // check when drone reached setpoint
                check_error_ = goal_error_;
                bool target_reached = checkPosition(check_error_, current_pose_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_));
                // std::printf("[ DEBUG] Target reached: %s\n", target_reached ? "true" : "false");

                if(target_reached && !final_position_) // drone reached start or middle setpoint(s)
                {
                    // print information
                    std::printf("\n[ INFO] Reached position: [%.8f, %.8f, %.3f]\n", current_global_.latitude, current_global_.longitude, current_global_.altitude);   
                    std::printf("- Goal position: [%.8f, %.8f, %.3f]\n", lat_goal_[i], lon_goal_[i], alt_goal_[i]);
                    std::printf("- Next goal: [%.8f, %.8f, %.3f]\n", lat_goal_[i+1], lon_goal_[i+1], alt_goal_[i+1]);
                    std::printf("\n[ INFO] Local position: [%.1f, %.1f, %.1f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);   
                    std::printf("- Converted target: [%.1f, %.1f, %.1f]\n", targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.x, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.y, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.z);

                    // ros::param::get("hover_time",hover_time_);
                    hover_time_ = hover_;
                    std::printf("- Hovering in %.1f (s)\n", hover_time_);
                    
                    if(delivery_)
                    {
                        hoverAt(hover_time_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_), rate); // hover at setpoint in hover_time_ second(s)
                        landingAt(targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_), rate); // land at each setpoint to unpack - comment if don't want to land at each setpoint
                    }
                    else
                    {
                        hoverAt(hover_time_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_), rate); // hover at setpoint in hover_time_ second(s)
                    }
                    
                    i+=1; // next setpoint
                }
                if(target_reached && final_position_) // drone reached final setpoint
                {
                    // print information
                    std::printf("\n[ INFO] Reached FINAL position: [%.8f, %.8f, %.3f]\n", current_global_.latitude, current_global_.longitude, current_global_.altitude);

                    if(!return_home_)
                    {
                        // ros::param::get("hover_time",hover_time_);
                        hover_time_ = hover_;
                        std::printf("- Hovering in %.1f (s) before land\n", hover_time_);
                        hoverAt(hover_time_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_), rate); // hover at setpoint in hover_time_ second(s)
                        landingAtFinal(targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_), rate);
                    }
                    else
                    {
                        if(delivery_)
                        {
                            hoverAt(hover_time_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_), rate); // hover at setpoint in hover_time_ second(s)
                            landingAt(targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_), rate); // land at each setpoint to unpack - comment if don't want to land at each setpoint
                        }
                        else
                        {
                            hoverAt(hover_time_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_), rate); // hover at setpoint in hover_time_ second(s)
                        }
                        home_pose_.pose.position.z = goal_enu.z + z_offset_;
                        std::printf("\n[ INFO] Returning home\n");
                        returnHome(home_pose_, rate);
                    }
                    
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
        std::printf("   > roslaunch offboard offboard.launch simulation:=(true/false) delivery:=(true/false) return_home:=(true/false)\n");
        std::printf("   'simulation' for set ARM and switch OFFBOARD mode in auto or receive from RC controller\n");
        std::printf("   simulation:=true, OFFBOARD node will automatic set ARM and switch OFFBOARD mode\n");
        std::printf("   simulation:=false, OFFBOARD node will receive signal to set ARM and switch OFFBOARD mode from RC controller\n");
        std::printf("   'delivery' use in mission mode for landing or hovering at each setpoint\n");
        std::printf("   delivery:=true for landing at each setpoint\n");
        std::printf("   delivery:=false for hovering at each setpoint\n");
        std::printf("   'return_home' use in mission mode for returning HOME or landing at final setpoint\n");
        std::printf("   return_home:=true for returning HOME when reached final setpoint\n");
        std::printf("   return_home:=false for landing at final setpoint\n");
    }

    return 0;
}