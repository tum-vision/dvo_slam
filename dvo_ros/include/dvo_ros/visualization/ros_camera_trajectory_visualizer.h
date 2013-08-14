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

#ifndef ROS_CAMERA_TRAJECTORY_VISUALIZER_H_
#define ROS_CAMERA_TRAJECTORY_VISUALIZER_H_

#include <dvo/visualization/camera_trajectory_visualizer.h>

#include <ros/ros.h>

namespace dvo_ros
{
namespace visualization
{

namespace internal
{
struct RosCameraTrajectoryVisualizerImpl;
} /* namespace internal */

class RosCameraTrajectoryVisualizer : public dvo::visualization::CameraTrajectoryVisualizerInterface
{
public:
  RosCameraTrajectoryVisualizer(ros::NodeHandle& nh);
  virtual ~RosCameraTrajectoryVisualizer();

  virtual dvo::visualization::CameraVisualizer::Ptr camera(std::string name);
  virtual dvo::visualization::TrajectoryVisualizer::Ptr trajectory(std::string name);

  virtual void reset();

  virtual bool native(void*& native_visualizer);
private:
  internal::RosCameraTrajectoryVisualizerImpl* impl_;
};

} /* namespace visualization */
} /* namespace dvo_ros */
#endif /* ROS_CAMERA_TRAJECTORY_VISUALIZER_H_ */
