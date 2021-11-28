#include "kimera_rviz_markers/kimera_rviz_markers.h"

int main(int argc, char** argv) {
  // Initialize ROS node
  ros::init(argc, argv, "drone_marker_pub");

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

  tf::Vector3 position (0, 0, 0);
  ros::Time timestamp = ros::Time();
  std::vector<visualization_msgs::Marker> drone_markers =
      krm::makeDroneMarkers(timestamp, quad_params);

  ros::Rate rate (20);
  ros::Publisher drone_marker_pub =
      nh_.advertise<visualization_msgs::MarkerArray>("drone_markers", 10, true);

  while(ros::ok()) {
    if (drone_marker_pub.getNumSubscribers() > 0) {
      visualization_msgs::MarkerArray msg;
      msg.markers = drone_markers;
      drone_marker_pub.publish(msg);
    }
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("Exiting drone marker publisher.");
  return EXIT_SUCCESS;
}
