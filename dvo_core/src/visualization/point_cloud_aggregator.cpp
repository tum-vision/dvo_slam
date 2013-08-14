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

#include <dvo/visualization/point_cloud_aggregator.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

namespace dvo
{
namespace visualization
{

typedef std::map<std::string, PointCloudAggregator::PointCloudBuilderCallable> PointCloudMap;

namespace internal
{
class PointCloudAggregatorImpl
{
public:
  PointCloudAggregatorImpl() {}
  ~PointCloudAggregatorImpl() {}

  PointCloudMap clouds_;
  boost::mutex clouds_mutex_;
};
} /* namespace internal */

dvo::visualization::AsyncPointCloudBuilder::PointCloud::Ptr passthroughPointCloudBuilder(const dvo::visualization::AsyncPointCloudBuilder::PointCloud::Ptr& cloud)
{
  return cloud;
}

PointCloudAggregator::PointCloudAggregator() :
    impl_(new internal::PointCloudAggregatorImpl()) {}
PointCloudAggregator::~PointCloudAggregator() {}

void PointCloudAggregator::add(const std::string& name, const dvo::visualization::AsyncPointCloudBuilder::PointCloud::Ptr& cloud)
{
  PointCloudBuilderCallable callable = boost::bind(&passthroughPointCloudBuilder, cloud);
  add(name, callable);
}

void PointCloudAggregator::add(const std::string& name, const PointCloudBuilderCallable& cloud)
{
  boost::mutex::scoped_lock(impl_->clouds_mutex_);
  impl_->clouds_[name] = cloud;
}

void PointCloudAggregator::remove(const std::string& name)
{
  boost::mutex::scoped_lock(impl_->clouds_mutex_);
  impl_->clouds_.erase(name);
}

AsyncPointCloudBuilder::PointCloud::Ptr PointCloudAggregator::build()
{
  AsyncPointCloudBuilder::PointCloud::Ptr highres_cloud(new AsyncPointCloudBuilder::PointCloud);
  AsyncPointCloudBuilder::PointCloud::Ptr downsampled_cloud(new AsyncPointCloudBuilder::PointCloud);

  PointCloudMap local;
  {
    boost::mutex::scoped_lock(impl_->clouds_mutex_);
    local = impl_->clouds_;
  }

  if(local.empty())
  {
    highres_cloud->points.push_back(AsyncPointCloudBuilder::PointCloud::PointType());
    return highres_cloud;
  }

  pcl::ApproximateVoxelGrid<AsyncPointCloudBuilder::PointCloud::PointType> vg;
  vg.setDownsampleAllData(true);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);

  size_t step = std::max(local.size() / size_t(50), size_t(1));
  size_t k = 0;
  for(PointCloudMap::iterator it = local.begin(); it != local.end(); ++it, ++k)
  {
    if((k % step) == 0)
    {
      (*highres_cloud) += *(it->second());
    }
  }

  vg.setInputCloud(highres_cloud);
  vg.filter(*downsampled_cloud);

  return downsampled_cloud;
}

} /* namespace visualization */
} /* namespace dvo */
