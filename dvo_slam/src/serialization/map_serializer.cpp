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

#include <dvo_slam/serialization/map_serializer.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <dvo_slam/timestamped.h>

namespace dvo_slam
{
namespace serialization
{



TrajectorySerializer::TrajectorySerializer(std::ostream& stream) :
    stream_(stream)
{
}

TrajectorySerializer::~TrajectorySerializer()
{
}

void TrajectorySerializer::serialize(const dvo_slam::KeyframeGraph& map)
{
  std::map<ros::Time, Eigen::Isometry3d> poses;

  for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = map.graph().vertices().begin(); it != map.graph().vertices().end(); ++it)
  {
    g2o::VertexSE3 *v = (g2o::VertexSE3 *) it->second;

    Timestamped *t = dynamic_cast<Timestamped *>(v->userData());

    assert(t != 0);

    poses[t->timestamp] = v->estimate();
  }

  for (std::map<ros::Time, Eigen::Isometry3d>::iterator it = poses.begin(); it != poses.end(); ++it)
  {
    Eigen::Quaterniond q(it->second.rotation());

    stream_ << it->first << " " << it->second.translation()(0) << " " << it->second.translation()(1) << " " << it->second.translation()(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " << std::endl;
  }
}

EdgeErrorSerializer::EdgeErrorSerializer(std::ostream& stream) :
    stream_(stream)
{
}

EdgeErrorSerializer::~EdgeErrorSerializer()
{
}

void EdgeErrorSerializer::serialize(const dvo_slam::KeyframeGraph& map)
{
  for(g2o::HyperGraph::EdgeSet::const_iterator it = map.graph().edges().begin(); it != map.graph().edges().end(); ++it)
  {
    g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(*it);

    int type = edge->vertices().size() == 2 && edge->level() == 0 && edge->vertex(0)->id() > 0 && edge->vertex(1)->id() > 0 ? 1 : 0;

    Eigen::Vector3d rho;
    if(edge->robustKernel() != 0)
      edge->robustKernel()->robustify(edge->chi2(), rho);
    else
      rho.setZero();

    //if(edge->vertices().size() == 2 && edge->level() == 0 && edge->vertex(0)->id() > 0 && edge->vertex(1)->id() > 0)
      stream_ << type << " " << edge->error().transpose() << " " << edge->chi2() << " " << rho.transpose() <<  std::endl;
  }
}

MessageSerializer::MessageSerializer(dvo_slam::PoseStampedArray& msg) :
    msg_(msg)
{
}

MessageSerializer::~MessageSerializer()
{
}

void MessageSerializer::serialize(const dvo_slam::KeyframeGraph& map)
{
  geometry_msgs::PoseStamped pose;

  for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = map.graph().vertices().begin(); it != map.graph().vertices().end(); ++it)
  {
    g2o::VertexSE3 *v = (g2o::VertexSE3 *) it->second;

    Timestamped *t = dynamic_cast<Timestamped *>(v->userData());

    assert(t != 0);

    Eigen::Isometry3d p = v->estimate();
    Eigen::Quaterniond q(p.rotation());

    pose.header.stamp = t->timestamp;
    pose.pose.position.x = p.translation()(0);
    pose.pose.position.y = p.translation()(1);
    pose.pose.position.z = p.translation()(2);

    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();

    msg_.poses.push_back(pose);
  }
}

} /* namespace serialization */
} /* namespace dvo_slam */
