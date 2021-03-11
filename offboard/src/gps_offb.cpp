#include "offboard/offboard.h"

int main(int argc, char **argv) 
{

    // initialize ros node
    ros::init(argc, argv, "gps_offb");
    ros::NodeHandle nh;

    // subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State> 
            ("mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix> 
            ("mavros/global_position/global", 10, globalPosition_cb);
    ros::Subscriber gps_pos_sub = nh.subscribe<mavros_msgs::GPSRAW> 
            ("mavros/gpsstatus/gps1/raw", 10, gpsPosition_cb);

    ros::Subscriber batt_sub = nh.subscribe<sensor_msgs::BatteryState> 
            ("mavros/battery", 10, battery_cb);

    // publisher
    ros::Publisher goal_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped> 
            ("mavros/setpoint_position/global", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for fcu connection
    while (ros::ok() && !current_state.connected) 
    {
        ROS_INFO_ONCE("Waiting for FCU connection ...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    // wait for position information
    while (ros::ok() && !global_position_received && !gps_position_received) 
    {
        ROS_INFO_ONCE("Waiting for GPS signal...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("GPS position received");
    ros::Duration(2).sleep();

    // check battery status
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        batt_percent = current_batt.percentage * 100;
        std::printf("Current Battery: %.1f \n", batt_percent);

        double global_position_altitude = double(gps_position.alt)/1000;
        std::printf("Current GPS position: [%f, %f, %.3f]\n", 
                     global_position.latitude, 
                     global_position.longitude, 
                     global_position_altitude);
                     
        ros::spinOnce();
        rate.sleep();
    }

    // set target position
    input_global_target();
    // goal_position.pose.position.latitude  = latitude;
    // goal_position.pose.position.longitude = longitude;
    // goal_position.pose.position.altitude  = altitude;
    goal_position.pose.position.latitude  = goal_pos[0][0];
    goal_position.pose.position.longitude = goal_pos[0][1];
    goal_position.pose.position.altitude  = goal_pos[0][2];

    // send a few setpoints before starting
    for (int i=100; ros::ok() && i>0; --i) 
    {
        goal_position.header.stamp = ros::Time::now();
        goal_pos_pub.publish(goal_position);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Ready");

    // get home, at 3m above the launch position
    double home_lat = global_position.latitude;
    double home_lon = global_position.longitude;
    double home_alt = double(gps_position.alt)/1000 + 3;
    std::printf("Home position: [%f, %f, %f] \n", home_lat, home_lon, home_alt);
    ros::Duration(5).sleep();

    int i=0;
    while (ros::ok()) 
    {
        batt_percent = current_batt.percentage * 100;
        std::printf("Current Battery: %.1f \n", batt_percent);
        if (i < goal_num)
        {
            goal_position.pose.position.latitude  = goal_pos[i][0];
            goal_position.pose.position.longitude = goal_pos[i][1];
            goal_position.pose.position.altitude  = goal_pos[i][2];
            goal_position.header.stamp = ros::Time::now();
            goal_pos_pub.publish(goal_position);
            
            double global_position_altitude = double(gps_position.alt)/1000;
            distance = measureGPS(global_position.latitude, 
                                  global_position.longitude, 
                                  global_position_altitude, 
                                  goal_pos[i][0], goal_pos[i][1], goal_pos[i][2]);
            std::printf("Distance to goal: %.2f m \n", distance);

            ros::spinOnce();
            rate.sleep();
        }
        else
        {
            // return home
            goal_position.pose.position.latitude  = home_lat;
            goal_position.pose.position.longitude = home_lon;
            goal_position.pose.position.altitude  = home_alt;
            goal_position.header.stamp = ros::Time::now();
            goal_pos_pub.publish(goal_position);

            double global_position_altitude = double(gps_position.alt)/1000;
            distance = measureGPS(global_position.latitude, 
                                  global_position.longitude, 
                                  global_position_altitude, 
                                  home_lat, home_lon, home_alt);
            std::printf("Distance to home: %.2f m \n", distance);

            // keep final goal
            // goal_position.pose.position.latitude  = goal_pos[goal_num-1][0];
            // goal_position.pose.position.longitude = goal_pos[goal_num-1][1];
            // goal_position.pose.position.altitude  = goal_pos[goal_num-1][2];
            // goal_position.header.stamp = ros::Time::now();
            // goal_pos_pub.publish(goal_position);
            // ROS_INFO_ONCE("Reached final goal, keep position")

            ros::spinOnce();
            rate.sleep();
        }        
        
        // check GPS reached
        bool check = check_global();
        std::cout << check << std::endl;
		if(check)
		{
            ros::Duration(5).sleep();
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
