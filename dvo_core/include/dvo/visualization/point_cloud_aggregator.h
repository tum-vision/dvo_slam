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

#ifndef POINT_CLOUD_AGGREGATOR_H_
#define POINT_CLOUD_AGGREGATOR_H_

#include <dvo/visualization/async_point_cloud_builder.h>

namespace dvo
{
namespace visualization
{

namespace internal
{
  class PointCloudAggregatorImpl;
} /* namespace internal */

class PointCloudAggregator
{
public:
  typedef boost::function<dvo::visualization::AsyncPointCloudBuilder::PointCloud::Ptr()> PointCloudBuilderCallable;

  PointCloudAggregator();
  virtual ~PointCloudAggregator();

  void add(const std::string& name, const dvo::visualization::AsyncPointCloudBuilder::PointCloud::Ptr& cloud);
  void add(const std::string& name, const PointCloudBuilderCallable& cloud);
  void remove(const std::string& name);

  dvo::visualization::AsyncPointCloudBuilder::PointCloud::Ptr build();
private:
  boost::scoped_ptr<internal::PointCloudAggregatorImpl> impl_;
};

} /* namespace visualization */
} /* namespace dvo */
#endif /* POINT_CLOUD_AGGREGATOR_H_ */
