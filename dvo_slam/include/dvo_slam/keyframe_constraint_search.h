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

#ifndef KEYFRAME_CONSTRAINT_SEARCH_H_
#define KEYFRAME_CONSTRAINT_SEARCH_H_

#include <dvo_slam/keyframe.h>

namespace dvo_slam
{

class KeyframeConstraintSearchInterface
{
public:
  virtual ~KeyframeConstraintSearchInterface() {}
  virtual void findPossibleConstraints(const KeyframeVector& all, const KeyframePtr& keyframe, KeyframeVector& candidates) = 0;
};
typedef boost::shared_ptr<KeyframeConstraintSearchInterface> KeyframeConstraintSearchInterfacePtr;

class NearestNeighborConstraintSearch : public KeyframeConstraintSearchInterface
{
public:
  NearestNeighborConstraintSearch(float max_distance) :
    max_distance_(max_distance)
  {};
  virtual ~NearestNeighborConstraintSearch() {};

  float maxDistance() const;

  void maxDistance(const float& d);

  virtual void findPossibleConstraints(const KeyframeVector& all, const KeyframePtr& keyframe, KeyframeVector& candidates);
private:
  float max_distance_;
};

} /* namespace dvo_slam */
#endif /* KEYFRAME_CONSTRAINT_SEARCH_H_ */
