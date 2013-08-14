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

#ifndef TOOLS_H_
#define TOOLS_H_

#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>

#include <dvo_benchmark/groundtruth.h>
#include <dvo_benchmark/file_reader.h>

namespace dvo_benchmark
{

static void toPoseTf(const Groundtruth& gt, tf::Pose& pose)
{
  pose.setOrigin(tf::Vector3(gt.PositionX(), gt.PositionY(), gt.PositionZ()));
  pose.setRotation(tf::Quaternion(gt.OrientationX(), gt.OrientationY(), gt.OrientationZ(), gt.OrientationW()));
}

static void toPoseMsg(const Groundtruth& gt, geometry_msgs::Pose& pose)
{
  pose.position.x = gt.PositionX();
  pose.position.y = gt.PositionY();
  pose.position.z = gt.PositionZ();
  pose.orientation.x = gt.OrientationX();
  pose.orientation.y = gt.OrientationY();
  pose.orientation.z = gt.OrientationZ();
  pose.orientation.w = gt.OrientationW();
}

template<typename NumType>
static void toPoseEigen(const Groundtruth& gt, Eigen::Transform<NumType, 3, Eigen::Affine>& pose)
{
  Eigen::Quaterniond rotation(gt.OrientationW(), gt.OrientationX(), gt.OrientationY(), gt.OrientationZ());

  pose = Eigen::Transform<NumType, 3, Eigen::Affine>::Identity();
  pose = rotation.cast<NumType>() * pose;
  pose.translation()(0) = (NumType) gt.PositionX();
  pose.translation()(1) = (NumType) gt.PositionY();
  pose.translation()(2) = (NumType) gt.PositionZ();
}


/**
 * Skips entries until the timestamp of the current entry is larger than the reference timestamp or the end of file has been reached.
 * Afterwards reader.entry() will be the closest entry.
 */
template<class EntryT>
static bool findClosestEntry(FileReader<EntryT>& reader, const ros::Time& reference)
{
  // in case the current timestamp is larger than the reference one, we have the closest entry
  if (reader.entry().Timestamp() >= reference) return true;

  bool success = reader.next();

  while(success && reader.entry().Timestamp() < reference)
  {
    success = reader.next();
  }

  return success;
}

template<class EntryT>
static bool findClosestEntries(FileReader<EntryT>& reader, const ros::Time& reference, EntryT& before, EntryT& after)
{
  if (reader.entry().Timestamp() >= reference)
  {
    throw std::exception();
  }

  EntryT last;
  bool success;

  do
  {
    last = reader.entry();
    success = reader.next();
  }
  while(success && reader.entry().Timestamp() < reference);

  before = last;
  after = reader.entry();

  return success;
}

} /* namespace dvo_benchmark */
#endif /* TOOLS_H_ */
