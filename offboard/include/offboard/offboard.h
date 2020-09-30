#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <iostream>
#include <cmath>

const double PI  =3.141592653589793238463;

bool check_position(void);
bool check_orientation(void);
void input_target(void);
double degree(double);
double radian(double);

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped target_pose;

sensor_msgs::NavSatFix global_position;
bool global_position_received = false;

int target_num;
// float target_pos[10][3];
float target_pos[10][7];
tf::Quaternion q;
double roll, pitch, yaw;
double r, p, y;

bool check_position()
{
	bool reached;
	if(((target_pose.pose.position.x - 0.1) < current_pose.pose.position.x)
	 && (current_pose.pose.position.x < (target_pose.pose.position.x + 0.1)) 
	 && ((target_pose.pose.position.y - 0.1) < current_pose.pose.position.y)
	 && (current_pose.pose.position.y < (target_pose.pose.position.y + 0.1))
	 && ((target_pose.pose.position.z - 0.1) < current_pose.pose.position.z)
	 && (current_pose.pose.position.z < (target_pose.pose.position.z + 0.1)))
	{
		reached = 1;
	}
	else
	{
		reached = 0;
	}
	return reached;
}


bool check_orientation()
{
	bool reached;
	
	// if(((target_pose.pose.orientation.x - 0.1) < current_pose.pose.orientation.x)
	//  && (current_pose.pose.orientation.x < (target_pose.pose.orientation.x + 0.1)) 
	//  && ((target_pose.pose.orientation.y - 0.1) < current_pose.pose.orientation.y)
	//  && (current_pose.pose.orientation.y < (target_pose.pose.orientation.y + 0.1))
	//  && ((target_pose.pose.orientation.z - 0.1) < current_pose.pose.orientation.z)
	//  && (current_pose.pose.orientation.z < (target_pose.pose.orientation.z + 0.1))
	//  && ((target_pose.pose.orientation.w - 0.1) < current_pose.pose.orientation.w)
	//  && (current_pose.pose.orientation.w < (target_pose.pose.orientation.w + 0.1)))
	
	// tf Quaternion to RPY
	tf::Quaternion qc(
		current_pose.pose.orientation.x,
		current_pose.pose.orientation.y,
		current_pose.pose.orientation.z,
		current_pose.pose.orientation.w);
	tf::Matrix3x3 mc(qc);
	double rc, pc, yc;
	mc.getRPY(rc, pc, yc);

	tf::Quaternion qt(
		current_pose.pose.orientation.x,
		current_pose.pose.orientation.y,
		current_pose.pose.orientation.z,
		current_pose.pose.orientation.w);
	tf::Matrix3x3 mt(qt);
	double rt, pt, yt;
	mt.getRPY(rt, pt, yt);
	
	if((((degree(rt)-1)<(degree(rc)))&&(degree(rc)<(degree(rt)+1)))
	 &&(((degree(pt)-1)<(degree(pc)))&&(degree(pc)<(degree(pt)+1)))
	 &&(((degree(yt)-1)<(degree(yc)))&&(degree(yc)<(degree(yt)+1)))) 
	{
		reached = 1;
	}
	else
	{
		reached = 0;
	}
	return reached;
}


void input_target()
{
	std::cout << "Input target(s) position:" << std::endl;
	std::cout << "Number of target(s): "; std::cin >> target_num;
	for (int i = 0; i < target_num; i++)
	{
		std::cout << "Target (" << i+1 << ") position:" <<std::endl; 
		std::cout << "pos_x_" << i+1 << ":"; std::cin >> target_pos[i][0];
		std::cout << "pos_y_" << i+1 << ":"; std::cin >> target_pos[i][1];
		std::cout << "pos_z_" << i+1 << ":"; std::cin >> target_pos[i][2];
		
		// std::cout << "Target (" << i+1 << ") orientation:" <<std::endl; 
		// std::cout << "ort_x_" << i+1 << ":"; std::cin >> target_pos[i][3];
		// std::cout << "ort_y_" << i+1 << ":"; std::cin >> target_pos[i][4];
		// std::cout << "ort_z_" << i+1 << ":"; std::cin >> target_pos[i][5];
		// std::cout << "ort_w_" << i+1 << ":"; std::cin >> target_pos[i][6];
		std::cout << "Target (" << i+1 << ") orientation (degree):" <<std::endl; 
		std::cout << "roll_" << i+1 << ":"; std::cin >> target_pos[i][3];
		std::cout << "pitch_" << i+1 << ":"; std::cin >> target_pos[i][4];
		std::cout << "yaw_" << i+1 << ":"; std::cin >> target_pos[i][5];
	}
}

double degree(double rad)
{
	double radian_to_degree = (rad*180)/PI;
	return radian_to_degree;
}

double radian(double deg)
{
	double degree_to_radian = (deg*PI)/180;
	return degree_to_radian;
}