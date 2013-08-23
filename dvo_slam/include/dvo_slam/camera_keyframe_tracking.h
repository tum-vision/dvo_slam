/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CAMERA_KEYFRAME_TRACKING_H_
#define CAMERA_KEYFRAME_TRACKING_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <image_transport/image_transport.h>

#include <Eigen/Geometry>

#include <dynamic_reconfigure/server.h>

#include <dvo_ros/camera_base.h>
#include <dvo_ros/CameraDenseTrackerConfig.h>
#include <dvo_ros/visualization/ros_camera_trajectory_visualizer.h>

#include <dvo/dense_tracking.h>
#include <dvo/core/intrinsic_matrix.h>
#include <dvo/core/rgbd_image.h>

#include <dvo_slam/keyframe_tracker.h>

namespace dvo_slam
{

class CameraKeyframeTracker : public dvo_ros::CameraBase
{
private:
  typedef dynamic_reconfigure::Server<dvo_ros::CameraDenseTrackerConfig> TrackerReconfigureServer;
  typedef dynamic_reconfigure::Server<dvo_slam::KeyframeSlamConfig> SlamReconfigureServer;

  uint32_t width;
  uint32_t height;

  //boost::shared_ptr<dvo::DenseTracker> tracker;
  dvo::core::IntrinsicMatrix intrinsics;
  boost::shared_ptr<dvo_slam::KeyframeTracker> keyframe_tracker;
  dvo::DenseTracker::Config tracker_cfg;
  dvo_slam::KeyframeTrackerConfig keyframe_tracker_cfg;
  dvo_slam::KeyframeGraphConfig graph_cfg;

  dvo::core::RgbdCameraPyramidPtr camera;
  dvo::core::RgbdImagePyramidPtr current, reference;

  Eigen::Affine3d accumulated_transform;

  ros::Publisher pose_publisher, graph_publisher;
  tf::TransformListener tl;

  TrackerReconfigureServer tracker_reconfigure_server_;
  SlamReconfigureServer slam_reconfigure_server_;

  dvo_ros::visualization::RosCameraTrajectoryVisualizer* vis_;
  dvo_slam::visualization::GraphVisualizer* graph_vis_;

  boost::mutex tracker_mutex_;

  bool hasChanged(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);
  void reset(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);

  void publishTransform(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame);

  void onMapChanged(dvo_slam::KeyframeGraph& map);
public:
  CameraKeyframeTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  virtual ~CameraKeyframeTracker();

  virtual void handleImages(
      const sensor_msgs::Image::ConstPtr& rgb_image_msg,
      const sensor_msgs::Image::ConstPtr& depth_image_msg,
      const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info_msg,
      const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info_msg
  );

  void handleTrackerConfig(dvo_ros::CameraDenseTrackerConfig& config, uint32_t level);
  void handleSlamConfig(dvo_slam::KeyframeSlamConfig& config, uint32_t level);
};

} /* namespace dvo_slam */
#endif /* CAMERA_KEYFRAME_TRACKING_H_ */
