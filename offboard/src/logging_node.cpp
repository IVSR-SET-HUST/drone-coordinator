#include <offboard/offboard.h>
#include <offboard/logging.h>

int main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc, argv, "logging");
    ros::NodeHandle nh;

    // subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, localPose_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix> 
            ("mavros/global_position/global", 10, globalPosition_cb);
    ros::Subscriber gps_pos_sub = nh.subscribe<mavros_msgs::GPSRAW> 
            ("mavros/gpsstatus/gps1/raw", 10, gpsPosition_cb);
    ros::Subscriber rel_alt_sub = nh.subscribe<std_msgs::Float64>
            ("mavros/global_position/rel_alt", 10, relativeAlt_cb);
    ros::Subscriber imu_data_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data_raw", 10, imuData_cb);
    ros::Subscriber mag_data_sub = nh.subscribe<sensor_msgs::MagneticField>
            ("mavros/imu/mag", 10, magData_cb);
    ros::Subscriber static_press_sub = nh.subscribe<sensor_msgs::FluidPressure>
            ("mavros/imu/static_pressure", 10, staticPress_cb);
    ros::Subscriber diff_press_sub = nh.subscribe<sensor_msgs::FluidPressure>
            ("mavros/imu/diff_pressure", 10, diffPress_cb);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
	{
        std::cout << "[ INFO] Waiting for FCU connection...\n";
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] FCU connected \n";

    // wait for GPS information
    while (ros::ok() && !global_position_received && !gps_position_received) 
    {
        std::cout << "[ INFO] Waiting for GPS signal...\n";
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] GPS position received \n";
    std::cout << "[ INFO] Waiting arm and takeoff... \n";
    while (ros::ok() && !current_state.armed)
    {
        ros::spinOnce();
        rate.sleep();
    }

//     refpoint.latitude = global_position.latitude;
//     refpoint.longitude = global_position.longitude;
//     refpoint.altitude = global_position.altitude;
//     std::printf("Reference position: [%f, %f, %.3f]\n", 
//                      refpoint.latitude, 
//                      refpoint.longitude, 
//                      refpoint.altitude);
    
    std::cout << "[ INFO] Logging... \n";
    while (ros::ok())
    {
        // std::printf("Current local position: [%.3f, %.3f, %.3f]\n", 
        //             current_pose.pose.position.x, 
        //             current_pose.pose.position.y, 
        //             current_pose.pose.position.z);
            
        // std::printf("Current global position: [%f, %f, %.3f]\n", 
        //             global_position.latitude, 
        //             global_position.longitude, 
        //             global_position.altitude);

        gps_lat = double(gps_position.lat)/10000000;
        gps_lon = double(gps_position.lon)/10000000;
        gps_alt = double(gps_position.alt)/1000;

        // enu_curr = WGS84ToENU(global_position.latitude,
        //                       global_position.longitude,
        //                       global_position.altitude,
        //                       refpoint.latitude, 
        //                       refpoint.longitude, 
        //                       refpoint.altitude);

        updates("flight", current_pose.pose.position.x,
                          current_pose.pose.position.y,
                          current_pose.pose.position.z,
                          enu_curr.x,
                          enu_curr.y,
                          enu_curr.z,
                          global_position.latitude,
                          global_position.longitude,
                          global_position.altitude,
                          gps_lat, gps_lon, gps_alt, 
                          rel_alt.data);
        updates_sensor("flight", imu_data.angular_velocity.x, 
                                 imu_data.angular_velocity.y,
                                 imu_data.angular_velocity.z,
                                 imu_data.linear_acceleration.x, 
                                 imu_data.linear_acceleration.y, 
                                 imu_data.linear_acceleration.z,
                                 mag_data.magnetic_field.x, 
                                 mag_data.magnetic_field.y, 
                                 mag_data.magnetic_field.z,
                                 static_press.fluid_pressure, 
                                 diff_press.fluid_pressure);
        ros::Duration(0.1).sleep();
        if (!current_state.armed)
        {
            std::cout << "[ INFO] Closed logfile \n";
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