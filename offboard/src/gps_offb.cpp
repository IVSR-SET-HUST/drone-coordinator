#include "offboard/offboard.h"

// callback functions
void globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    global_position = *msg;
    global_position_received = true;
    ROS_INFO_ONCE("Got global position: [%f, %f, %f]", msg->latitude, msg->longitude, msg->altitude);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_offb");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe < mavros_msgs::State > ("mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe < sensor_msgs::NavSatFix > ("mavros/global_position/global", 1, globalPosition_cb);
    
    ros::Publisher goal_pos_pub = nh.advertise < mavros_msgs::GlobalPositionTarget > ("mavros/setpoint_position/global", 10);

    ros::Rate rate(20.0);

    // wait for fcu connection
    while (ros::ok() && !current_state.connected) {
        ROS_INFO_ONCE("Waiting for FCU connection...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    // wait for position information
    while (ros::ok() && !global_position_received) {
        ROS_INFO_ONCE("Waiting for GPS signal...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("GPS position received");

    // set target position
    // goal_position.latitude = global_position.latitude;
    // goal_position.longitude = global_position.longitude;
    // goal_position.altitude = global_position.altitude;
    input_global_target();
    goal_position.latitude = latitude;
    goal_position.longitude = longitude;
    goal_position.altitude = altitude;

    // send a few setpoints before starting
    for (int i=10; ros::ok() && i>0; --i) {
        goal_position.header.stamp = ros::Time::now();
        goal_pos_pub.publish(goal_position);
        ros::spinOnce();
        rate.sleep();
    }

    // take off to 5m above ground
    // goal_position.altitude = goal_position.altitude + 5.0;
    // goal_position.latitude = goal_position.latitude + 0.00001;
    // goal_position.longitude = goal_position.longitude + 0.00001;

    while (ros::ok()) {
        goal_position.header.stamp = ros::Time::now();
        goal_pos_pub.publish(goal_position);
        // std::cout << "Latitude " << global_position.latitude << std::endl;
        // std::cout << "Longitude " << global_position.longitude << std::endl;
        // std::cout << "Altitude " << global_position.altitude << std::endl;
        dist = measureGPS(global_position.latitude, global_position.longitude, latitude, longitude);
        // std::cout << "Distance to target " << dist << " m" << std::endl;
        std::printf("Distance to target: %.2f m \n", dist);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}