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

#ifndef KEYFRAME_TRACKER_H_
#define KEYFRAME_TRACKER_H_

#include <dvo/core/rgbd_image.h>
#include <dvo/dense_tracking.h>

#include <dvo_slam/config.h>
#include <dvo_slam/serialization/map_serializer_interface.h>
#include <dvo_slam/visualization/graph_visualizer.h>

#include <ros/time.h>

namespace dvo_slam
{

class KeyframeTracker
{
public:
  KeyframeTracker(dvo_slam::visualization::GraphVisualizer* visualizer = 0);

  const dvo::DenseTracker::Config& trackingConfiguration() const;
  const dvo_slam::KeyframeTrackerConfig& keyframeSelectionConfiguration() const;
  const dvo_slam::KeyframeGraphConfig& mappingConfiguration() const;

  void configureTracking(const dvo::DenseTracker::Config& cfg);

  void configureKeyframeSelection(const dvo_slam::KeyframeTrackerConfig& cfg);

  void configureMapping(const dvo_slam::KeyframeGraphConfig& cfg);

  void init();
  void init(const Eigen::Affine3d& initial_transformation);

  void update(const dvo::core::RgbdImagePyramid::Ptr& current, const ros::Time& current_time, Eigen::Affine3d& absolute_transformation);

  void forceKeyframe();

  void finish();

  void addMapChangedCallback(const dvo_slam::KeyframeGraph::MapChangedCallback& callback);

  void serializeMap(dvo_slam::serialization::MapSerializerInterface& serializer);
private:
  class Impl;
  boost::shared_ptr<Impl> impl_;
};

} /* namespace dvo_slam */
#endif /* KEYFRAME_TRACKER_H_ */
