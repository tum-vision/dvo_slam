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

#ifndef TRACKING_RESULT_H_
#define TRACKING_RESULT_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dvo/dense_tracking.h>

namespace dvo_slam
{

struct TrackingResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  dvo::core::AffineTransformd Pose;
  dvo::core::Matrix6d Information;
  dvo::DenseTracker::IterationContext* Context;

  bool isNaN() const;

  void setIdentity();
};

} /* namespace dvo_slam */
#endif /* TRACKING_RESULT_H_ */
