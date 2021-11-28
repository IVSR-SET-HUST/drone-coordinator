//
// Software License Agreement (BSD License)
//
//==============================================================================
// Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt
//  - Modified:  2020, Antoni Rosinol, MIT
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Based on the point_cloud2 nodelet in the stereo_image_proc package

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

namespace stereo_image_proc {

using namespace sensor_msgs;
using namespace stereo_msgs;

class DepthImageNodelet : public nodelet::Nodelet {
public:
  DepthImageNodelet() = default;
  virtual ~DepthImageNodelet() = default;

protected:
  typedef message_filters::sync_policies::ExactTime<CameraInfo, CameraInfo,
                                                    DisparityImage>
      ExactPolicy;
  typedef message_filters::sync_policies::ApproximateTime<
      CameraInfo, CameraInfo, DisparityImage>
      ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

protected:
  virtual void onInit();

  void connectCb();

  void imageCb(const CameraInfoConstPtr& l_info_msg,
               const CameraInfoConstPtr& r_info_msg,
               const DisparityImageConstPtr& disp_msg);

protected:
  // Subscriptions
  boost::shared_ptr<image_transport::ImageTransport> it_;
  message_filters::Subscriber<CameraInfo> sub_l_info_;
  message_filters::Subscriber<CameraInfo> sub_r_info_;
  message_filters::Subscriber<DisparityImage> sub_disparity_;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  // Publications
  boost::mutex connect_mutex_;
  image_transport::Publisher pub_depth_image_;

  // Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel model_;
  cv::Mat_<cv::Vec3f> points_mat_;  // scratch buffer

};

void DepthImageNodelet::onInit() {
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(private_nh));

  // Synchronize inputs. Topic subscriptions happen on demand in the connection
  // callback. Optionally do approximate synchronization.
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  bool approx;
  private_nh.param("approximate_sync", approx, false);
  if (approx) {
    approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size),
                                                sub_l_info_, sub_r_info_,
                                                sub_disparity_));
    approximate_sync_->registerCallback(
        boost::bind(&DepthImageNodelet::imageCb, this, _1, _2, _3));
  } else {
    exact_sync_.reset(new ExactSync(ExactPolicy(queue_size), sub_l_info_,
                                    sub_r_info_, sub_disparity_));
    exact_sync_->registerCallback(
        boost::bind(&DepthImageNodelet::imageCb, this, _1, _2, _3));
  }

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb =
      boost::bind(&DepthImageNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to
  // pub_points2_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_depth_image_ = it_->advertise("depth_image", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void DepthImageNodelet::connectCb() {
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_depth_image_.getNumSubscribers() == 0) {
    sub_r_info_.unsubscribe();
    sub_disparity_.unsubscribe();
  } else if (!sub_disparity_.getSubscriber()) {
    ros::NodeHandle& nh = getNodeHandle();
    // Queue size 1 should be OK; the one that matters is the synchronizer queue
    // size.
    image_transport::TransportHints hints("raw", ros::TransportHints(),
                                          getPrivateNodeHandle());
    sub_l_info_.subscribe(nh, "left/camera_info", 1);
    sub_r_info_.subscribe(nh, "right/camera_info", 1);
    sub_disparity_.subscribe(nh, "disparity", 1);
  }
}

inline bool isValidPoint(const cv::Vec3f& pt) {
  // Check both for disparities explicitly marked as invalid (where OpenCV maps
  // pt.z to MISSING_Z)
  // and zero disparities (point mapped to infinity).
  return pt[2] != image_geometry::StereoCameraModel::MISSING_Z &&
         !std::isinf(pt[2]);
}

void DepthImageNodelet::imageCb(const CameraInfoConstPtr& l_info_msg,
                                const CameraInfoConstPtr& r_info_msg,
                                const DisparityImageConstPtr& disp_msg) {
  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);

  // Calculate point cloud
  const Image& dimage = disp_msg->image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width,
                             (float*)&dimage.data[0], dimage.step);
  model_.projectDisparityImageTo3d(dmat, points_mat_, true);
  cv::Mat_<cv::Vec3f> mat = points_mat_;

  sensor_msgs::ImagePtr depth_image(new sensor_msgs::Image());

  depth_image->header = disp_msg->header;
  depth_image->width = dimage.width;
  depth_image->height = dimage.height;
  depth_image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_image->step = sizeof(float) * depth_image->width;

  size_t num_pixels = depth_image->width * depth_image->height;

  depth_image->data.resize(num_pixels * sizeof(float));

  std::vector<uint8_t>& data = depth_image->data;

  float nan = std::numeric_limits<float>::quiet_NaN();

  for (size_t i = 0; i < num_pixels; ++i) {
    // data[i*sizeof(float)] =
    if (mat(i)[2] < 1000.0) {
      memcpy(&data[i * sizeof(float)], &mat(i)[2], sizeof(float));
    } else {
      memcpy(&data[i * sizeof(float)], &nan,
             sizeof(float));  // data[i*sizeof(float)]
    }
  }

  pub_depth_image_.publish(depth_image);
}

}  // namespace stereo_image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(stereo_image_proc::DepthImageNodelet, nodelet::Nodelet)
