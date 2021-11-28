/* @file   KimeraVioRos.cpp
 * @brief  ROS Wrapper for Kimera-VIO
 * @author Antoni Rosinol
 */

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <kimera-vio/pipeline/Pipeline-definitions.h>
#include <kimera-vio/pipeline/Pipeline.h>
#include <kimera-vio/utils/Macros.h>

#include "kimera_vio_ros/RosDataProviderInterface.h"
#include "kimera_vio_ros/RosDisplay.h"
#include "kimera_vio_ros/RosVisualizer.h"

namespace VIO {

class KimeraVioRos {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(KimeraVioRos);
  KIMERA_POINTER_TYPEDEFS(KimeraVioRos);

  KimeraVioRos();
  virtual ~KimeraVioRos() = default;

 public:
  bool runKimeraVio();

 protected:
  bool spin();

  VIO::RosDataProviderInterface::UniquePtr createDataProvider(
      const VioParams& vio_params);

  void connectVIO();

  /**
   * @brief restartKimeraVio Callback for the rosservice to restart the pipeline
   * @param request
   * @param response
   * @return
   */
  bool restartKimeraVio(std_srvs::Trigger::Request& request,
                        std_srvs::Trigger::Response& response);

 protected:
  //! ROS
  ros::NodeHandle nh_private_;

  //! VIO
  VioParams::Ptr vio_params_;
  Pipeline::UniquePtr vio_pipeline_;

  //! Data provider
  RosDataProviderInterface::UniquePtr data_provider_;
  RosDisplay::UniquePtr ros_display_;
  RosVisualizer::UniquePtr ros_visualizer_;

  //! ROS Services
  ros::ServiceServer restart_vio_pipeline_srv_;
  std::atomic_bool restart_vio_pipeline_;
};

}  // namespace VIO
