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

#ifndef KEYFRAME_GRAPH_H_
#define KEYFRAME_GRAPH_H_

#include <dvo_slam/config.h>
#include <dvo_slam/local_map.h>
#include <dvo_slam/keyframe.h>

#include <dvo/dense_tracking.h>
#include <dvo/visualization/camera_trajectory_visualizer.h>

#include <boost/function.hpp>

namespace dvo_slam
{
namespace internal
{

class KeyframeGraphImpl;

typedef boost::scoped_ptr<KeyframeGraphImpl> KeyframeGraphImplPtr;

} /* namespace internal */

class KeyframeGraph
{
public:
  typedef void MapChangedCallbackSignature(KeyframeGraph&);
  typedef boost::function<MapChangedCallbackSignature> MapChangedCallback;

  KeyframeGraph();
  virtual ~KeyframeGraph();

  const dvo_slam::KeyframeGraphConfig& configuration() const;

  void configure(const dvo_slam::KeyframeGraphConfig& config);

  void configureValidationTracking(const dvo::DenseTracker::Config& cfg);

  void add(const LocalMap::Ptr& keyframe);

  void finalOptimization();

  void addMapChangedCallback(const KeyframeGraph::MapChangedCallback& callback);

  const KeyframeVector& keyframes() const;

  const g2o::SparseOptimizer& graph() const;

  cv::Mat computeIntensityErrorImage(int edge_id, bool use_measurement = true) const;

  void debugLoopClosureConstraint(int keyframe1, int keyframe2) const;

private:
  internal::KeyframeGraphImplPtr impl_;
};

} /* namespace dvo_slam */
#endif /* KEYFRAME_GRAPH_H_ */
