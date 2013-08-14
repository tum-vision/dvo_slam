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

#ifndef GROUNDTRUTH_H_
#define GROUNDTRUTH_H_

#include <ros/time.h>

#include <dvo/util/fluent_interface.h>

namespace dvo_benchmark
{

class Groundtruth
{
public:
  Groundtruth() {}
  virtual ~Groundtruth() {}

  FI_ATTRIBUTE(Groundtruth, ros::Time, Timestamp);
  FI_ATTRIBUTE(Groundtruth, double, PositionX);
  FI_ATTRIBUTE(Groundtruth, double, PositionY);
  FI_ATTRIBUTE(Groundtruth, double, PositionZ);
  FI_ATTRIBUTE(Groundtruth, double, OrientationX);
  FI_ATTRIBUTE(Groundtruth, double, OrientationY);
  FI_ATTRIBUTE(Groundtruth, double, OrientationZ);
  FI_ATTRIBUTE(Groundtruth, double, OrientationW);

  friend std::ostream& operator <<(std::ostream& out, const Groundtruth& pair);
  friend std::istream& operator >>(std::istream& in, Groundtruth& pair);
};

std::ostream& operator <<(std::ostream& out, const Groundtruth& gt)
{
  out
    << gt.Timestamp_ << " "
    << gt.PositionX_ << " "
    << gt.PositionY_ << " "
    << gt.PositionZ_ << " "
    << gt.OrientationX_ << " "
    << gt.OrientationY_ << " "
    << gt.OrientationZ_ << " "
    << gt.OrientationW_ << std::endl;

  return out;
}

std::istream& operator >>(std::istream& in, Groundtruth& gt)
{
  double timestamp;
  in >> timestamp;
  gt.Timestamp_.fromSec(timestamp);
  in >> gt.PositionX_;
  in >> gt.PositionY_;
  in >> gt.PositionZ_;
  in >> gt.OrientationX_;
  in >> gt.OrientationY_;
  in >> gt.OrientationZ_;
  in >> gt.OrientationW_;

  return in;
}
} /* namespace dvo_benchmark */
#endif /* GROUNDTRUTH_H_ */
