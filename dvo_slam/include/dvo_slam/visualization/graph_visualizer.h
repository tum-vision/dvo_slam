/**
 *  This file is part of dvo.
 *
 *  Copyright 2013 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
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

#ifndef GRAPH_VISUALIZER_H_
#define GRAPH_VISUALIZER_H_

#include <dvo_ros/visualization/ros_camera_trajectory_visualizer.h>
#include <dvo_slam/keyframe_graph.h>
#include <boost/scoped_ptr.hpp>

namespace dvo_slam
{
namespace visualization
{

namespace internal
{
  class GraphVisualizerImpl;
} /* namespace internal */

class GraphVisualizer
{
public:
  GraphVisualizer(dvo_ros::visualization::RosCameraTrajectoryVisualizer& visualizer);
  virtual ~GraphVisualizer();

  void setGraph(dvo_slam::KeyframeGraph *graph);

  void update();
private:
  boost::scoped_ptr<internal::GraphVisualizerImpl> impl_;
};

} /* namespace visualization */
} /* namespace dvo_slam */
#endif /* GRAPH_VISUALIZER_H_ */
