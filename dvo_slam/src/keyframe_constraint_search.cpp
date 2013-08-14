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

#include <dvo_slam/keyframe_constraint_search.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ros/time.h>

namespace dvo_slam
{

float NearestNeighborConstraintSearch::maxDistance() const
{
  return max_distance_;
}

void NearestNeighborConstraintSearch::maxDistance(const float& d)
{
  max_distance_ = d;
}

void NearestNeighborConstraintSearch::findPossibleConstraints(const KeyframeVector& all, const KeyframePtr& keyframe, KeyframeVector& candidates)
{
  pcl::PointXYZ search_point;
  search_point.x = keyframe->pose().translation()(0);
  search_point.y = keyframe->pose().translation()(1);
  search_point.z = keyframe->pose().translation()(2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_points(new pcl::PointCloud<pcl::PointXYZ>());

  for(KeyframeVector::const_iterator it = all.begin(); it != all.end(); ++it)
  {
    pcl::PointXYZ p;
    p.x = (*it)->pose().translation()(0);
    p.y = (*it)->pose().translation()(1);
    p.z = (*it)->pose().translation()(2);

    keyframe_points->points.push_back(p);
  }

  std::vector<int> indices;
  std::map<int, KeyframePtr> ids;
  std::vector<float> distances;

  pcl::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>(false));
  kdtree->setInputCloud(keyframe_points);
  kdtree->radiusSearch(search_point, max_distance_, indices, distances);

  for(std::vector<int>::iterator it = indices.begin(); it != indices.end(); ++it)
  {
    candidates.push_back(all[*it]);
  }
}

void DriftAwareNearestNeighborConstraintSearch::findPossibleConstraints(const KeyframeVector& all, const KeyframePtr& keyframe, KeyframeVector& candidates)
{
  pcl::PointXYZ search_point;
  search_point.x = keyframe->pose().translation()(0);
  search_point.y = keyframe->pose().translation()(1);
  search_point.z = keyframe->pose().translation()(2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_points(new pcl::PointCloud<pcl::PointXYZ>());

  for(KeyframeVector::const_iterator it = all.begin(); it != all.end(); ++it)
  {
    if(keyframe->id() == (*it)->id()) continue;

    pcl::PointXYZ p, p2;
    p.x = (*it)->pose().translation()(0);
    p.y = (*it)->pose().translation()(1);
    p.z = (*it)->pose().translation()(2);

    float time_diff = (float)(keyframe->timestamp() - (*it)->timestamp()).toSec();
    float drift_distance = time_diff * drift_per_second_;
    Eigen::Vector3f drift = (search_point.getVector3fMap() - p.getVector3fMap());

    //assert(drift_distance >= 0.0f);

    if(drift.norm() >= drift_distance)
    {
      drift.normalize();
      drift *= drift_distance;
    }
    else
    {
      drift.setZero();
    }

    p2.x = p.x + drift(0);
    p2.y = p.y + drift(1);
    p2.z = p.z + drift(2);

    //assert((search_point.getVector3fMap() - p2.getVector3fMap()).norm() <= (search_point.getVector3fMap() - p.getVector3fMap()).norm());

    keyframe_points->points.push_back(p2);
  }

  std::vector<int> indices;
  std::map<int, KeyframePtr> ids;
  std::vector<float> distances;

  pcl::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>(false));
  kdtree->setInputCloud(keyframe_points);
  kdtree->radiusSearch(search_point, max_distance_, indices, distances);

  for(std::vector<int>::iterator it = indices.begin(); it != indices.end(); ++it)
  {
    candidates.push_back(all[*it]);
  }
}

} /* namespace dvo_slam */
