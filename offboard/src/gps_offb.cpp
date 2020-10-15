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

    // check battery status
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        batt_percent = current_batt.percentage * 100;
        std::printf("Current Battery: %.1f \n", batt_percent);

        double alt = double(gps_position.alt)/1000;
        std::printf("Current GPS position: [%f, %f, %.3f]\n", 
                     global_position.latitude, 
                     global_position.longitude, 
                     alt);

        ros::spinOnce();
        rate.sleep();
    }

    // set target position
    input_global_target();
    goal_position.pose.position.latitude = latitude;
    goal_position.pose.position.longitude = longitude;
    goal_position.pose.position.altitude = altitude;

    // send a few setpoints before starting
    for (int i=100; ros::ok() && i>0; --i) 
    {
        goal_position.header.stamp = ros::Time::now();
        goal_pos_pub.publish(goal_position);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Ready");

    while (ros::ok()) 
    {
        goal_position.header.stamp = ros::Time::now();
        goal_pos_pub.publish(goal_position);
        double alt = double(gps_position.alt)/1000;
        distance = measureGPS(global_position.latitude, 
                              global_position.longitude, 
                              alt, 
                              latitude, longitude, altitude);
        std::printf("Distance to target: %.2f m \n", distance);
        batt_percent = current_batt.percentage * 100;
        std::printf("Current Battery: %.1f \n", batt_percent);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
