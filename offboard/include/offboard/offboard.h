/******* ros *******/
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/FluidPressure.h>
/******* local position *******/
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
/******* global position *******/
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPoint.h>

#include <mavros_msgs/GPSRAW.h>
/******* tranformation *******/
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
/******* C++ ******/
#include <iostream>
#include <cmath>
#include <cstdio>
#include <vector>

/****** DEFINE CONSTANTS ******/
const double PI  =3.141592653589793238463;
const double eR  =6378.137; // earth radius in km

const double a = 6378137.0;         // WGS-84 Earth semimajor axis (m)
const double b = 6356752.314245;     // Derived Earth semiminor axis (m)
const double f = (a - b) / a;           // Ellipsoid Flatness
const double f_inv = 1.0 / f;       // Inverse flattening

//const double f_inv = 298.257223563; // WGS-84 Flattening Factor of the Earth 
//const double b = a - a / f_inv;
//const double f = 1.0 / f_inv;

const double a_sq = a * a;
const double b_sq = b * b;
const double e_sq = f * (2 - f);    // Square of Eccentricity

/****** DECLARE VARIANTS ******/
mavros_msgs::State current_state;
mavros_msgs::SetMode set_mode;
std_msgs::Float64 rel_alt;
sensor_msgs::Imu imu_data;
sensor_msgs::MagneticField mag_data;
sensor_msgs::FluidPressure static_press, diff_press;

geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped target_pose;

bool global_position_received = false;
sensor_msgs::NavSatFix global_position;
geographic_msgs::GeoPoseStamped goal_position;

bool gps_position_received = false;
mavros_msgs::GPSRAW gps_position;
double gps_lat, gps_lon, gps_alt;

sensor_msgs::BatteryState current_batt;

tf::Quaternion q; // quaternion to transform to RPY

int target_num; // number of local setpoints
double target_pos[10][7]; // local setpoints list
double roll, pitch, yaw; // roll, pitch, yaw angle
double r, p, y; // roll, pitch, yaw use in transform

int goal_num; // number of global setpoints
double goal_pos[10][3]; // global setpoints list
double distance;

int in_num_of_target;
std::vector<double> in_x_pos;
std::vector<double> in_y_pos;
std::vector<double> in_z_pos;
int in_num_of_goal;
std::vector<double> in_latitude;
std::vector<double> in_longitude;
std::vector<double> in_altitude;
float local_error, global_error;

bool input_type = true; // true == input local || false == input global setpoints
bool final_check = false; // true == reached final point || false == NOT final point
float batt_percent; // baterry capacity

float check_error = 0.1; // default = 0.1 m
ros::Time t_check;

geometry_msgs::Point enu_goal, enu_curr, enu_ref; //Local ENU points: converted from GPS goal and current
geographic_msgs::GeoPoint wgs84_target, wgs84_curr; //Global WGS84 point: convert from ENU target and current
geographic_msgs::GeoPoint refpoint; //Reference point to convert ECEF to ENU and vice versa

double x_offset, y_offset, z_offset;
double x_off[100], y_off[100], z_off[100];
/****** DEFINE FUNCTIONS ******/
/********************************************************************************/
/***** check_position: check when drone reached the target positions       ******/
/***** input: x_target, y_target, ztarget, x_current, y_current, z_current ******/
/***** return: true or false                                               ******/          
/********************************************************************************/
bool check_position(float, double, double, double,
				    double, double, double);

/*******************************************************************************************/
/***** check_orientation: check when drone reached the target orientations            ******/
/***** input: (quaternion target) xt, yt, zt, wt, (quaternion current) xc, yc, zc, wc ******/
/***** return: true or false                                                          ******/
/*******************************************************************************************/
bool check_orientation(double, double, double, double,
				       double, double, double, double);

/**************************************************************************/
/***** check_global: check when drone reached the GPS goal positions ******/
/***** input: (target) lat, lon, alt, (current) lat0, lon0, alt0     ******/
/***** return: true or false                                         ******/       
/**************************************************************************/
bool check_global(double, double, double,
				  double, double, double);

/***************************************************************************/
/***** input_local_target: input the local coodinate (x, y, z) points ******/
/***** and (maybe) input the yaw rotation at each point               ******/
/***************************************************************************/
void input_local_target(void);

/****************************************************************************************/
/***** input_global_target: input the GPS [latitude, longitude, altitude] point(s) ******/
/****************************************************************************************/
void input_global_target(void);

/*******************************************************************/
/***** input_target: choose the local or global input targets ******/
/*******************************************************************/
void input_target(void);

/********************************************************/
/***** degree: convert angle from radian to degree ******/
/***** input: angle in radian                      ******/
/***** return: angle in degree                     ******/
/********************************************************/
double degree(double);

/********************************************************/
/***** radian: convert angle from degree to radian ******/
/***** input: angle in degree                      ******/
/***** return: angle in radian                     ******/
/********************************************************/
double radian(double);

/*********************************************************************************************/
/***** measureGPS: measure the distance between 2 GPS points that use haversine formula ******/
/***** input: (GPS1) lat1, lon1, alt1, (GPS2) lat2, lon2, alt2                          ******/
/***** return: distance in meters                                                       ******/
/*********************************************************************************************/
double measureGPS(double, double, double, double, double, double);
double distanceLocal(double, double, double, double, double, double);

/* WGS84ToECEF: Converts the WGS-84 Geodetic point (latitude, longitude, altitude) **
** to Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z)                      **/
geometry_msgs::Point WGS84ToECEF(double, double, double);

/* ECEFToWGS84: Converts the Earth-Centered Earth-Fixed coordinates (x, y, z) to   **
** WGS-84 Geodetic point (latitude, longitude, altitude)                           **/
geographic_msgs::GeoPoint ECEFToWGS84(double, double, double);

/* ECEFToENU: Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) **
** to East-North-Up coordinates in a Local Tangent Plane that is centered at the   **
** (WGS-84) Geodetic point (lat0, lon0, alt0)                                      **/
geometry_msgs::Point ECEFToENU(double, double, double, double, double, double);

/* ENUToECEF: Converts East-North-Up coordinates (xEast, yNorth, zUp) in a Local   **
** Tangent Plane that is centered at  (WGS-84) Geodetic point (lat0, lon0, alt0)   **
** to the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z)                  **/
geometry_msgs::Point ENUToECEF(double, double, double, double, double, double);

/* WGS84ToENU: Converts the geodetic WGS-84 coordinated (lat, lon, alt) to         **
** East-North-Up coordinates in a Local Tangent Plane that is centered at the      **
** (WGS-84) Geodetic point (lat0, lon0, alt0)                                      **/
geometry_msgs::Point WGS84ToENU(double, double, double, double, double, double);

/* ENUToWGS84: Converts the East-North-Up coordinates in a Local Tangent Plane to  **
** geodetic WGS-84 coordinated (lat, lon, alt) that is centered at the (WGS-84)    **
** Geodetic point (lat0, lon0, alt0)                                               **/
geographic_msgs::GeoPoint ENUToWGS84(double, double, double, double, double, double);

// callback functions
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
void relativeAlt_cb(const std_msgs::Float64::ConstPtr& msg)
{
    rel_alt = *msg;
}
void imuData_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
	imu_data = *msg;
}
void magData_cb(const sensor_msgs::MagneticField::ConstPtr& msg)
{
	mag_data = *msg;
}
void staticPress_cb(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
	static_press = *msg;
}
void diffPress_cb(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
	diff_press = *msg;
}
void localPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}
void globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) 
{
    global_position = *msg;
    global_position_received = true;
}
void gpsPosition_cb(const mavros_msgs::GPSRAW::ConstPtr& msg) 
{
    gps_position = *msg;
	gps_position_received = true;
}
void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg) 
{
    current_batt = *msg;
}