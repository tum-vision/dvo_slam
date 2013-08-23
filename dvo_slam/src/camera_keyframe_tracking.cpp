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

#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <dvo/core/surface_pyramid.h>
#include <dvo/dense_tracking.h>
#include <dvo/util/stopwatch.h>

#include <dvo_slam/config.h>
#include <dvo_slam/camera_keyframe_tracking.h>

#include <dvo_slam/PoseStampedArray.h>
#include <dvo_slam/serialization/map_serializer.h>

#include <dvo_slam/visualization/graph_visualizer.h>

#include <dvo_ros/util/util.h>
#include <dvo_ros/util/configtools.h>
#include <dvo_ros/visualization/ros_camera_trajectory_visualizer.h>


namespace dvo_slam
{

using namespace dvo;
using namespace dvo::core;
using namespace dvo::util;

CameraKeyframeTracker::CameraKeyframeTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
  CameraBase(nh, nh_private),
  tracker_reconfigure_server_(ros::NodeHandle(nh_private, "tracking")),
  slam_reconfigure_server_(ros::NodeHandle(nh_private, "slam")),
  tracker_cfg(dvo::DenseTracker::getDefaultConfig()),
  vis_(new dvo_ros::visualization::RosCameraTrajectoryVisualizer(nh_)),
  graph_vis_(new dvo_slam::visualization::GraphVisualizer(*vis_))
{
  ROS_INFO("CameraDenseTracker::ctor(...)");

  pose_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);
  graph_publisher = nh.advertise<dvo_slam::PoseStampedArray>("graph", 1);

  TrackerReconfigureServer::CallbackType tracker_reconfigure_server_callback = boost::bind(&CameraKeyframeTracker::handleTrackerConfig, this, _1, _2);
  tracker_reconfigure_server_.setCallback(tracker_reconfigure_server_callback);

  SlamReconfigureServer::CallbackType slam_reconfigure_server_callback = boost::bind(&CameraKeyframeTracker::handleSlamConfig, this, _1, _2);
  slam_reconfigure_server_.setCallback(slam_reconfigure_server_callback);

  accumulated_transform.setIdentity();
}

CameraKeyframeTracker::~CameraKeyframeTracker()
{
  delete vis_;
  delete graph_vis_;
}

bool CameraKeyframeTracker::hasChanged(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
{
  return width != camera_info_msg->width || height != camera_info_msg->height;
}

void CameraKeyframeTracker::reset(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
{
  //intrinsics = IntrinsicMatrix::create(camera_info_msg->K[0], camera_info_msg->K[4], camera_info_msg->K[2], camera_info_msg->K[5]);
  intrinsics = IntrinsicMatrix::create(camera_info_msg->P[0], camera_info_msg->P[5], camera_info_msg->P[2], camera_info_msg->P[6]);
  camera.reset(new dvo::core::RgbdCameraPyramid(camera_info_msg->width, camera_info_msg->height, intrinsics));
  camera->build(tracker_cfg.getNumLevels());

  keyframe_tracker.reset(new KeyframeTracker(graph_vis_));
  keyframe_tracker->configureTracking(tracker_cfg);
  keyframe_tracker->configureKeyframeSelection(keyframe_tracker_cfg);
  keyframe_tracker->configureMapping(graph_cfg);
  keyframe_tracker->addMapChangedCallback(boost::bind(&CameraKeyframeTracker::onMapChanged, this, _1));

  static RgbdImagePyramid* const __null__ = 0;

  reference.reset(__null__);
  current.reset(__null__);

  width = camera_info_msg->width;
  height = camera_info_msg->height;

  vis_->reset();
}

void CameraKeyframeTracker::handleTrackerConfig(dvo_ros::CameraDenseTrackerConfig& config, uint32_t level)
{
  if(level == 0) return;

  if(level & dvo_ros::CameraDenseTracker_RunDenseTracking)
  {
    if(config.run_dense_tracking)
    {
      startSynchronizedImageStream();
    }
    else
    {
      stopSynchronizedImageStream();

      // force reset of tracker
      width = 0;
      height = 0;
    }
  }

  if(!config.run_dense_tracking && config.use_dense_tracking_estimate)
  {
    config.use_dense_tracking_estimate = false;
  }

  if(level & dvo_ros::CameraDenseTracker_ConfigParam)
  {
    // fix config, so we don't die by accident
    if(config.coarsest_level < config.finest_level)
    {
      config.finest_level = config.coarsest_level;
    }

    dvo_ros::util::updateConfigFromDynamicReconfigure(config, tracker_cfg);

    // we are called in the ctor as well, but at this point we don't have a tracker instance
    if(keyframe_tracker)
    {
      // lock tracker so we don't reconfigure it while it is running
      boost::mutex::scoped_lock lock(tracker_mutex_);

      keyframe_tracker->configureTracking(tracker_cfg);
      camera->build(tracker_cfg.getNumLevels());
    }

    //ROS_INFO_STREAM("reconfigured tracker, config ( " << tracker_cfg << " )");
  }

  vis_->reset();
}

void CameraKeyframeTracker::handleSlamConfig(dvo_slam::KeyframeSlamConfig& config, uint32_t level)
{
  dvo_slam::updateConfigFromDynamicReconfigure(config, keyframe_tracker_cfg, graph_cfg);

  if(keyframe_tracker)
  {
    boost::mutex::scoped_lock lock(tracker_mutex_);

    keyframe_tracker->configureKeyframeSelection(keyframe_tracker_cfg);
    keyframe_tracker->configureMapping(graph_cfg);

    if(config.graph_opt_final)
    {
      config.graph_opt_final = false;
      keyframe_tracker->finish();
    }
  }

  //ROS_INFO_STREAM("reconfigured SLAM system, frontend config ( " << keyframe_tracker_cfg << " ), backend config  ( " << graph_cfg << " )");
}


void CameraKeyframeTracker::onMapChanged(dvo_slam::KeyframeGraph& map)
{
  dvo_slam::PoseStampedArray msg;
  dvo_slam::serialization::MessageSerializer serializer(msg);
  serializer.serialize(map);

  graph_publisher.publish(msg);
}

void CameraKeyframeTracker::handleImages(
    const sensor_msgs::Image::ConstPtr& rgb_image_msg,
    const sensor_msgs::Image::ConstPtr& depth_image_msg,
    const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info_msg,
    const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info_msg
)
{
  static stopwatch sw_callback("callback");
  sw_callback.start();

  // lock tracker so no one can reconfigure it
  boost::mutex::scoped_lock lock(tracker_mutex_);

  // different size of rgb and depth image
  if(depth_camera_info_msg->width != rgb_camera_info_msg->width || depth_camera_info_msg->height != rgb_camera_info_msg->height)
  {
    ROS_WARN("RGB and depth image have different size!");

    return;
  }

  // something has changed
  if(hasChanged(rgb_camera_info_msg))
  {
    ROS_WARN("RGB image size has changed, resetting tracker!");

    reset(rgb_camera_info_msg);
  }

  cv::Mat intensity, depth;
  cv::Mat rgb_in = cv_bridge::toCvShare(rgb_image_msg)->image;

  if(rgb_in.channels() == 3)
  {
    cv::Mat tmp;
    cv::cvtColor(rgb_in, tmp, CV_BGR2GRAY, 1);

    tmp.convertTo(intensity, CV_32F);
  }
  else
  {
    rgb_in.convertTo(intensity, CV_32F);
  }

  cv::Mat depth_in = cv_bridge::toCvShare(depth_image_msg)->image;

  if(depth_in.type() == CV_16UC1)
  {
    SurfacePyramid::convertRawDepthImageSse(depth_in, depth, 0.001);
  }
  else
  {
    depth = depth_in;
  }

  reference.swap(current);
  current = camera->create(intensity, depth);

  if(rgb_in.channels() == 3)
  {
    rgb_in.convertTo(current->level(0).rgb, CV_32FC3);
  }

  // time delay compensation TODO: use driver settings instead
  std_msgs::Header h = rgb_image_msg->header;
  //h.stamp -= ros::Duration(0.05);

  static Eigen::Affine3d first;

  if(!reference)
  {
    accumulated_transform.setIdentity();
    keyframe_tracker->init(accumulated_transform);

    return;
  }

  Eigen::Affine3d transform, last_transform;

  static stopwatch sw_match("match", 100);
  sw_match.start();

  keyframe_tracker->update(current, h.stamp, accumulated_transform);

  sw_match.stopAndPrint();

  //vis_->trajectory("estimate")->
  //    color(dvo::visualization::Color::red())
  //    .add(accumulated_transform);

  vis_->camera("current")->
      color(dvo::visualization::Color::red()).
      update(current->level(0), accumulated_transform).
      show(dvo::visualization::CameraVisualizer::ShowCamera);

  publishTransform(h, accumulated_transform, "base_link_estimate");

  sw_callback.stopAndPrint();
}

void CameraKeyframeTracker::publishTransform(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame)
{
  static tf::TransformBroadcaster tb;

  tf::StampedTransform tf_transform;
  tf_transform.frame_id_ = "world";
  tf_transform.child_frame_id_ = frame;
  tf_transform.stamp_ = header.stamp;

  tf::TransformEigenToTF(transform, tf_transform);

  tb.sendTransform(tf_transform);

  geometry_msgs::PoseWithCovarianceStamped pose_msg;

  tf::poseTFToMsg(tf_transform, pose_msg.pose.pose);
  pose_msg.header.frame_id = frame;
  pose_msg.header.stamp = header.stamp;

  pose_publisher.publish(pose_msg);
}

} /* namespace dvo_slam */
