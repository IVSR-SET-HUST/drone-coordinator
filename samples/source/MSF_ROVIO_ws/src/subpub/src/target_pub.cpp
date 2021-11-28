#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <cstdlib>

geometry_msgs::PointStamped target; 
nav_msgs::Odometry current;
// khai bao bien de luu gia tri tu subscriber va bien de publish

inline void local_cb(const nav_msgs::Odometry::ConstPtr& msg) // callback gia tri cho subscriber
{
    current = *msg; // gan gia tri vao bien luu da khai bao
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_pub"); // dat ten cho node
    ros::NodeHandle nh;

    //         ten publisher  =  publish <kieu topic> ("ten topic", hang doi)
    ros::Publisher target_pub = nh.advertise<geometry_msgs::PointStamped>("/position_output", 100);
    
    //        ten sunscriber  =  sunscribe <kieu topic> ("ten topic", hang doi, callback)
    ros::Subscriber local_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 100, local_cb);

    ros::Rate loop_rate(5);

    while (ros::ok())
    {        
        // thay doi gia tri cho bien can publish
        target.header = current.header;
        target.point.x = current.pose.pose.position.x;
        target.point.y = current.pose.pose.position.y;
        target.point.z = current.pose.pose.position.z;

        target_pub.publish(target); // publish: bien_publish.publish(gia_tri)

        ros::spinOnce(); // goi callback de cap nhat gia tri
        loop_rate.sleep();
    }
    
    return 0;
}