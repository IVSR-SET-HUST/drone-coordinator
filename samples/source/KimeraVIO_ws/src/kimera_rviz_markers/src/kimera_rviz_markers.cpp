#include <math.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include "kimera_rviz_markers/kimera_rviz_markers.h"
#include "kimera_rviz_markers/utils.h"

namespace krm {

namespace viz_msgs = visualization_msgs;
using IM = viz_msgs::InteractiveMarker;
using IMC = viz_msgs::InteractiveMarkerControl;

/// Hexacopter marker code adapted from libsfly_viz thanks to Markus Achtelik.
std::vector<viz_msgs::Marker>
makeDroneMarkers(const ros::Time& timestamp,
                 const DroneParams& drone_params) {
  ROS_ASSERT(drone_params.num_rotors >= 2);
  const std::string& frame_id = drone_params.frame_id;

  // Rotor marker template.
  float marker_scale = 1.0;
  viz_msgs::Marker rotor;
  rotor.header.frame_id = frame_id;
  rotor.header.stamp = timestamp;
  rotor.ns = "vehicle_rotor";
  rotor.action = viz_msgs::Marker::ADD;
  rotor.type = viz_msgs::Marker::CYLINDER;
  rotor.scale.x = 0.2 * marker_scale;
  rotor.scale.y = 0.2 * marker_scale;
  rotor.scale.z = 0.01 * marker_scale;
  rotor.color.r = 1.0;
  rotor.color.g = 1.0;
  rotor.color.b = 1.0;
  rotor.color.a = 0.8;
  rotor.pose.position.z = 0;

  // Arm marker template.
  viz_msgs::Marker arm;
  arm.header.frame_id = frame_id;
  arm.header.stamp = timestamp;
  arm.ns = "vehicle_arm";
  arm.action = viz_msgs::Marker::ADD;
  arm.type = viz_msgs::Marker::CUBE;
  arm.scale.x = drone_params.arm_len * marker_scale;
  arm.scale.y = 0.02 * marker_scale;
  arm.scale.z = 0.01 * marker_scale;
  // Put all these hardcoded things in YAML file...
  arm.color.r = 0.0;
  arm.color.g = 0.0;
  arm.color.b = 1.0;
  arm.color.a = 1.0;
  arm.pose.position.z = -0.015 * marker_scale;

  float angle_increment = 2 * M_PI / drone_params.num_rotors;

  std::vector<viz_msgs::Marker> drone_markers;
  for (float angle = angle_increment / 2; angle <= (2 * M_PI);
       angle += angle_increment) {
    rotor.pose.position.x = drone_params.arm_len * cos(angle) * marker_scale;
    rotor.pose.position.y = drone_params.arm_len * sin(angle) * marker_scale;
    rotor.id++;

    arm.pose.position.x = rotor.pose.position.x / 2 ;
    arm.pose.position.y = rotor.pose.position.y / 2 ;
    arm.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    arm.id++;

    drone_markers.push_back(rotor);
    drone_markers.push_back(arm);
  }

  // Body marker template.
  viz_msgs::Marker body;
  body.header.frame_id = frame_id;
  body.header.stamp = timestamp;
  body.ns = "vehicle_body";
  body.action = viz_msgs::Marker::ADD;
  body.type = viz_msgs::Marker::CUBE;
  body.scale.x = drone_params.body_width * 0.75 * marker_scale;
  body.scale.y = drone_params.body_width * 0.75 * marker_scale;
  body.scale.z = drone_params.body_height * 0.75 * marker_scale;
  body.color.r = drone_params.r_color;
  body.color.g = drone_params.g_color;
  body.color.b = drone_params.b_color;
  body.color.a = 1.0;

  drone_markers.push_back(body);

  return drone_markers;
}

viz_msgs::Marker makeMesh() {
  viz_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  //marker.mesh_resource = "package://kimera_rviz_markers/meshes/quadrotor_base.dae";
}

} // End namespace krm.

int main(int argc, char** argv) {
  // Initialize ROS node
  ros::init(argc, argv, "kimera_rviz_markers");

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_ ("~");

  krm::DroneParams quad_params;
  nh_private_.getParam("frame_id", quad_params.frame_id);
  nh_private_.getParam("scale", quad_params.scale);
  nh_private_.getParam("num_rotors", quad_params.num_rotors);
  nh_private_.getParam("arm_len", quad_params.arm_len);
  nh_private_.getParam("body_width", quad_params.body_width);
  nh_private_.getParam("body_height", quad_params.body_height);
  nh_private_.getParam("r_color", quad_params.r_color);
  nh_private_.getParam("g_color", quad_params.g_color);
  nh_private_.getParam("b_color", quad_params.b_color);

  tf::Quaternion quaternion;
  std::vector<visualization_msgs::Marker> drone_markers =
      krm::makeDroneMarkers(ros::Time(), quad_params);
  visualization_msgs::Marker drone_mesh_marker =
      krm::makeMesh();

  ros::Rate rate (10);
  ros::Publisher drone_marker_pub =
      nh_.advertise<visualization_msgs::MarkerArray>("drone_markers", 10, true);
  ros::Publisher drone_mesh_marker_pub =
      nh_.advertise<visualization_msgs::Marker>("drone_mesh", 10, true);

  while(ros::ok()) {
    while (drone_marker_pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    ROS_INFO_ONCE("Publishing drone markers.");
    visualization_msgs::MarkerArray msg;
    msg.markers = drone_markers;
    drone_marker_pub.publish(msg);
    //drone_mesh_marker_pub.publish(drone_mesh_marker);
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("Exiting.");
  return EXIT_SUCCESS;
}
