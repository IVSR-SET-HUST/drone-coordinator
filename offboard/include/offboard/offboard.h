#ifndef OFFBOARD_H_
#define OFFBOARD_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <iostream>
#include <cmath>
#include <cstdio>
#include <vector>

/* CONSTANT */

const double PI = 3.141592653589793238463;
const double eR = 6378.137;         // earth radius in km

const double a = 6378137.0;         // WGS-84 Earth semimajor axis (m)
const double b = 6356752.314245;    // Derived Earth semiminor axis (m)
const double f = (a - b) / a;       // Ellipsoid Flatness
const double f_inv = 1.0 / f;       // Inverse flattening

const double a_sq = a * a;
const double b_sq = b * b;
const double e_sq = f * (2 - f);    // Square of Eccentricity

/* VARIABLE */

ros::Subscriber state_sub_;
ros::Subscriber local_pose_sub_;
ros::Subscriber global_pos_sub_;
ros::Subscriber velocity_body_sub_;

ros::Publisher local_pose_pub_;

ros::ServiceClient set_mode_client_;

mavros_msgs::State current_state_;
geometry_msgs::PoseStamped current_pose_;
geometry_msgs::PoseStamped home_pose_;
geometry_msgs::PoseStamped target_pose_;
geometry_msgs::TwistStamped current_vel_;
sensor_msgs::NavSatFix current_global_;
geographic_msgs::GeoPoseStamped goal_global_;
mavros_msgs::SetMode flight_mode_;
sensor_msgs::NavSatFix global_ref_;
geometry_msgs::Point local_offset_;

bool global_received_ = false;
int target_num_;
std::vector<double> x_target_;
std::vector<double> y_target_; 
std::vector<double> z_target_;
int goal_num_;
std::vector<double> lat_goal_; 
std::vector<double> lon_goal_;
std::vector<double> alt_goal_;
bool final_position_ = false; // true = reach final setpoint || false = other setpoints

bool input_type_ = true; //true = local input || false = global input
double check_error_, target_error_, goal_error_, land_error_, distance_;
double x_off_[100], y_off_[100], z_off_[100];
double x_offset_, y_offset_, z_offset_;
double z_takeoff_;

double vel_desired_, land_vel_, return_vel_;
std::vector<double> vel_;
double hover_time_, hover_, takeoff_time_, unpack_time_;
bool delivery_ = false;
bool simulation_ = false;
bool return_home_ = true;

/* FUNCTION DECLARE */

sensor_msgs::NavSatFix goalTransfer(double lat, double lon, double alt); // transfer lat, lon, alt setpoint to same message type with global_position
geometry_msgs::PoseStamped targetTransfer(double x, double y, double z); // transfer x, y, z setpoint to same message type with local_position

double avgBodyVelocity(geometry_msgs::TwistStamped vel);

bool checkPosition(double error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target);
bool checkOrientation(double error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target);

void inputTarget();
void inputLocal();
void inputGlobal();

std::vector<double> velLimit(double v_desired, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target);
void takeOff(geometry_msgs::PoseStamped takeoff_pose, ros::Rate rate);
void hoverAt(double hover_time, geometry_msgs::PoseStamped target, ros::Rate rate);
void landingAt(geometry_msgs::PoseStamped setpoint, ros::Rate rate);
void landingAtFinal(geometry_msgs::PoseStamped setpoint, ros::Rate rate);
void landingWithMarker(geometry_msgs::PoseStamped setpoint, ros::Rate rate);
void returnHome(geometry_msgs::PoseStamped home, ros::Rate rate);

double distanceMeasure(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target);
inline double degreeOf(double rad);
inline double radianOf(double deg);

geometry_msgs::Point WGS84ToECEF(sensor_msgs::NavSatFix wgs84);
geographic_msgs::GeoPoint ECEFToWGS84(geometry_msgs::Point ecef);
geometry_msgs::Point ECEFToENU(geometry_msgs::Point ecef, sensor_msgs::NavSatFix ref);
geometry_msgs::Point ENUToECEF(geometry_msgs::Point enu, sensor_msgs::NavSatFix ref);
geometry_msgs::Point WGS84ToENU(sensor_msgs::NavSatFix wgs84, sensor_msgs::NavSatFix ref);
geographic_msgs::GeoPoint ENUToWGS84(geometry_msgs::Point enu, sensor_msgs::NavSatFix ref);

#endif