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

#ifndef LOCAL_MAP_H_
#define LOCAL_MAP_H_

#include <g2o/core/sparse_optimizer.h>

#include <dvo/core/rgbd_image.h>
#include <dvo_slam/tracking_result_evaluation.h>

namespace dvo_slam
{

namespace internal
{
  struct LocalMapImpl;
} /* namespace internal */

class LocalMap
{
public:
  typedef boost::shared_ptr<LocalMap> Ptr;
  typedef boost::shared_ptr<const LocalMap> ConstPtr;

  virtual ~LocalMap();

  /**
   * Creates a new local map with the given keyframe.
   */
  static LocalMap::Ptr create(const dvo::core::RgbdImagePyramid::Ptr& keyframe, const dvo::core::AffineTransformd& keyframe_pose);

  dvo::core::RgbdImagePyramid::Ptr getKeyframe();

  void setKeyframePose(const dvo::core::AffineTransformd& keyframe_pose);

  dvo::core::RgbdImagePyramid::Ptr getCurrentFrame();

  void getCurrentFramePose(dvo::core::AffineTransformd& current_pose);
  dvo::core::AffineTransformd getCurrentFramePose();

  g2o::SparseOptimizer& getGraph();

  void setEvaluation(dvo_slam::TrackingResultEvaluation::ConstPtr& evaluation);

  dvo_slam::TrackingResultEvaluation::ConstPtr getEvaluation();

  /**
   * Adds a new RGB-D frame to the map, which becomes the active one.
   */
  void addFrame(const dvo::core::RgbdImagePyramid::Ptr& frame);

  /**
   * Adds a measurement connecting the last frame and the active frame to the map.
   */
  void addOdometryMeasurement(const dvo::core::AffineTransformd& pose, const dvo::core::Matrix6d& information);

  /**
   * Adds a measurement connecting the keyframe and the active frame to the map.
   */
  void addKeyframeMeasurement(const dvo::core::AffineTransformd& pose, const dvo::core::Matrix6d& information);

  void optimize();
private:
  LocalMap(const dvo::core::RgbdImagePyramid::Ptr& keyframe, const dvo::core::AffineTransformd& keyframe_pose);

  boost::scoped_ptr<internal::LocalMapImpl> impl_;
};

} /* namespace dvo_slam */
#endif /* LOCAL_MAP_H_ */
