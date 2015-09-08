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

#ifndef UTIL_H_
#define UTIL_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


namespace dvo_ros { namespace util {

static void tryGetTransform(tf::Transform& result, tf::TransformListener& tl, std::string target_frame, std::string source_frame, double seconds_to_wait = 5)
{
  if(tl.waitForTransform(target_frame, source_frame, ros::Time(), ros::Duration(seconds_to_wait), ros::Duration(0.2)))
   {
     tf::StampedTransform tmp;

     tl.lookupTransform(target_frame, source_frame, ros::Time(), tmp);

     result = tmp;
   }
   else
   {
     ROS_WARN("using identity!");

     result = tf::Transform::getIdentity();
   }
}

static void tryGetTransform(Eigen::Affine3d& result, tf::TransformListener& tl, std::string target_frame, std::string source_frame, double seconds_to_wait = 5)
{
  tf::Transform tmp;

  tryGetTransform(tmp, tl, target_frame, source_frame, seconds_to_wait);

  tf::transformTFToEigen(tmp, result);
}

} /* namespace util */ } /* namespace dvo_ros */



#endif /* UTIL_H_ */
