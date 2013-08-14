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

#ifndef ASYNC_POINT_CLOUD_BUILDER_H_
#define ASYNC_POINT_CLOUD_BUILDER_H_

#include <dvo/core/rgbd_image.h>
#include <dvo/core/intrinsic_matrix.h>

#include <boost/function.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace dvo
{
namespace visualization
{

class AsyncPointCloudBuilder
{
public:
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
  typedef boost::function<void (const PointCloud::Ptr& cloud)> DoneCallback;

  struct BuildJob
  {
  public:
    dvo::core::RgbdImage image;
    const Eigen::Affine3d pose;

    BuildJob(const dvo::core::RgbdImage& image, const Eigen::Affine3d pose = Eigen::Affine3d::Identity());

    AsyncPointCloudBuilder::PointCloud::Ptr build();
  private:
    AsyncPointCloudBuilder::PointCloud::Ptr cloud_;
  };

  AsyncPointCloudBuilder();
  virtual ~AsyncPointCloudBuilder();

  void build(const dvo::core::RgbdImage& image, const Eigen::Affine3d pose = Eigen::Affine3d::Identity());

  void done(DoneCallback& callback);

private:
  DoneCallback done_;
};

} /* namespace visualization */
} /* namespace dvo */
#endif /* ASYNC_POINT_CLOUD_BUILDER_H_ */
