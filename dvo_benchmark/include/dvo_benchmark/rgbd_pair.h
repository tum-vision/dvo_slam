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

#ifndef RGBDPAIR_H_
#define RGBDPAIR_H_

#include <iostream>
#include <ros/time.h>

#include <dvo/util/fluent_interface.h>

namespace dvo_benchmark
{

class RgbdPair
{
public:
  RgbdPair() {};
  virtual ~RgbdPair() {};

  FI_ATTRIBUTE(RgbdPair, ros::Time, RgbTimestamp)
  FI_ATTRIBUTE(RgbdPair, std::string, RgbFile)

  FI_ATTRIBUTE(RgbdPair, ros::Time, DepthTimestamp)
  FI_ATTRIBUTE(RgbdPair, std::string, DepthFile)

  friend std::ostream& operator <<(std::ostream& out, const RgbdPair& pair);
  friend std::istream& operator >>(std::istream& in, RgbdPair& pair);
};

std::ostream& operator <<(std::ostream& out, const RgbdPair& pair)
{
  out
    << pair.RgbTimestamp_ << " "
    << pair.RgbFile_ << " "
    << pair.DepthTimestamp_ << " "
    << pair.DepthFile_ << std::endl;

  return out;
}

std::istream& operator >>(std::istream& in, RgbdPair& pair)
{
  double timestamp;
  in >> timestamp;
  pair.RgbTimestamp_.fromSec(timestamp);
  in >> pair.RgbFile_;
  in >> timestamp;
  pair.DepthTimestamp_.fromSec(timestamp);
  in >> pair.DepthFile_;


  return in;
}

} /* namespace benchmark */
#endif /* RGBDPAIR_H_ */
