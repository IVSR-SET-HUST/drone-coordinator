#pragma once

#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/tf.h>

namespace krm {

struct DroneParams {
  std::string frame_id = "base_link";
  float scale = 0.0;
  int num_rotors = 4u;
  float arm_len = 0.22;
  float body_width = 0.15;
  float body_height = 0.10;
  float r_color = 0.0;
  float g_color = 0.0;
  float b_color = 0.0;
};

// Marker makers
std::vector<visualization_msgs::Marker>
makeDroneMarkers(const ros::Time& timestamp,
                 const DroneParams& drone_params);
visualization_msgs::Marker makeMesh();
}
