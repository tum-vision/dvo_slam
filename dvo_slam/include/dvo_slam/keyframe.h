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

#ifndef KEYFRAME_H_
#define KEYFRAME_H_

#include <dvo/core/rgbd_image.h>
#include <dvo/util/fluent_interface.h>

#include <dvo_slam/tracking_result_evaluation.h>

#include <ros/time.h>
#include <Eigen/Geometry>
#include <vector>

namespace dvo_slam
{

class Keyframe
{
public:

  Keyframe() : id_(-1) {};
  virtual ~Keyframe() {};

  FI_ATTRIBUTE(Keyframe, short, id)
  FI_ATTRIBUTE(Keyframe, dvo::core::RgbdImagePyramid::Ptr, image)
  FI_ATTRIBUTE(Keyframe, Eigen::Affine3d, pose)
  FI_ATTRIBUTE(Keyframe, dvo_slam::TrackingResultEvaluation::ConstPtr, evaluation)

  ros::Time timestamp() const
  {
    return ros::Time(image()->timestamp());
  }
};

typedef boost::shared_ptr<Keyframe> KeyframePtr;
typedef std::vector<KeyframePtr> KeyframeVector;

} /* namespace dvo_slam */
#endif /* KEYFRAME_H_ */
