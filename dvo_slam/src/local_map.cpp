/**
 *  This file is part of dvo.
 *
 *  Copyright 2013 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
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

#include <dvo_slam/local_map.h>
#include <dvo_slam/timestamped.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/edge_se3_offset.h>

namespace dvo_slam
{

namespace internal
{

Eigen::Isometry3d toIsometry(const Eigen::Affine3d& pose)
{
  Eigen::Isometry3d p(pose.rotation());
  p.translation() = pose.translation();

  return p;
}

static Eigen::Affine3d toAffine(const Eigen::Isometry3d& pose)
{
  Eigen::Affine3d p(pose.rotation());
  p.translation() = pose.translation();

  return p;
}

struct LocalMapImpl
{
  typedef g2o::BlockSolver_6_3 BlockSolver;
  typedef g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType> LinearSolver;
  dvo::core::RgbdImagePyramid::Ptr keyframe_, current_;
  g2o::VertexSE3 *keyframe_vertex_, *previous_vertex_, *current_vertex_;

  g2o::SparseOptimizer graph_;
  int max_vertex_id_, max_edge_id_;

  dvo_slam::TrackingResultEvaluation::ConstPtr evaluation_;

  LocalMapImpl(const dvo::core::RgbdImagePyramid::Ptr& keyframe, const dvo::core::AffineTransformd& keyframe_pose) :
    keyframe_(keyframe),
    keyframe_vertex_(0),
    previous_vertex_(0),
    current_vertex_(0),
    max_vertex_id_(1),
    max_edge_id_(1)
  {
    // g2o setup
    graph_.setAlgorithm(
        new g2o::OptimizationAlgorithmLevenberg(
            new BlockSolver(
                new LinearSolver()
            )
        )
    );
    graph_.setVerbose(false);

    keyframe_vertex_ = addFrameVertex(ros::Time(keyframe->timestamp()));
    keyframe_vertex_->setFixed(true);
    keyframe_vertex_->setEstimate(toIsometry(keyframe_pose));
  }

  g2o::VertexSE3* addFrameVertex(const ros::Time& timestamp)
  {
    g2o::VertexSE3* frame_vertex = new g2o::VertexSE3();
    frame_vertex->setId(max_vertex_id_++);
    frame_vertex->setUserData(new Timestamped(timestamp));

    graph_.addVertex(frame_vertex);

    return frame_vertex;
  }

  g2o::EdgeSE3* addTransformationEdge(g2o::VertexSE3 *from, g2o::VertexSE3 *to, const dvo::core::AffineTransformd& transform, const dvo::core::Matrix6d& information)
  {
    assert(from != 0 && to != 0);

    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    edge->setId(max_edge_id_++);
    edge->resize(2);
    edge->setVertex(0, from);
    edge->setVertex(1, to);
    edge->setMeasurement(toIsometry(transform));
    edge->setInformation(information);

    graph_.addEdge(edge);

    return edge;
  }
};

} /* namespace internal */

LocalMap::Ptr LocalMap::create(const dvo::core::RgbdImagePyramid::Ptr& keyframe, const dvo::core::AffineTransformd& keyframe_pose)
{
  LocalMap::Ptr result(new LocalMap(keyframe, keyframe_pose));
  return result;
}

LocalMap::LocalMap(const dvo::core::RgbdImagePyramid::Ptr& keyframe, const dvo::core::AffineTransformd& keyframe_pose) :
    impl_(new internal::LocalMapImpl(keyframe, keyframe_pose))
{
}

LocalMap::~LocalMap()
{
}

dvo::core::RgbdImagePyramid::Ptr LocalMap::getKeyframe()
{
  return impl_->keyframe_;
}

dvo::core::RgbdImagePyramid::Ptr LocalMap::getCurrentFrame()
{
  return impl_->current_;
}

void LocalMap::getCurrentFramePose(dvo::core::AffineTransformd& current_pose)
{
  current_pose = getCurrentFramePose();
}

void LocalMap::setKeyframePose(const dvo::core::AffineTransformd& keyframe_pose)
{
  impl_->keyframe_vertex_->setEstimate(internal::toIsometry(keyframe_pose));

  g2o::OptimizableGraph::EdgeSet& edges = impl_->keyframe_vertex_->edges();

  for(g2o::OptimizableGraph::EdgeSet::iterator it = edges.begin(); it != edges.end(); ++it)
  {
    g2o::EdgeSE3 *e = (g2o::EdgeSE3*)(*it);

    assert(e->vertex(0) == impl_->keyframe_vertex_);

    g2o::VertexSE3 *v = (g2o::VertexSE3*)e->vertex(1);
    v->setEstimate(impl_->keyframe_vertex_->estimate() * e->measurement());
  }
}

dvo::core::AffineTransformd LocalMap::getCurrentFramePose()
{
  return internal::toAffine(impl_->current_vertex_->estimate());
}

g2o::SparseOptimizer& LocalMap::getGraph()
{
  return impl_->graph_;
}

void LocalMap::setEvaluation(dvo_slam::TrackingResultEvaluation::ConstPtr& evaluation)
{
  impl_->evaluation_ = evaluation;
}

dvo_slam::TrackingResultEvaluation::ConstPtr LocalMap::getEvaluation()
{
  return impl_->evaluation_;
}

void LocalMap::addFrame(const dvo::core::RgbdImagePyramid::Ptr& frame)
{
  impl_->current_ = frame;
  impl_->previous_vertex_ = impl_->current_vertex_;
  impl_->current_vertex_ = impl_->addFrameVertex(ros::Time(frame->timestamp()));
}

void LocalMap::addOdometryMeasurement(const dvo::core::AffineTransformd& pose, const dvo::core::Matrix6d& information)
{
  impl_->addTransformationEdge(impl_->previous_vertex_, impl_->current_vertex_, pose, information);
}

void LocalMap::addKeyframeMeasurement(const dvo::core::AffineTransformd& pose, const dvo::core::Matrix6d& information)
{
  impl_->addTransformationEdge(impl_->keyframe_vertex_, impl_->current_vertex_, pose, information);
  impl_->current_vertex_->setEstimate(impl_->keyframe_vertex_->estimate() * internal::toIsometry(pose));
}

void LocalMap::optimize()
{
  impl_->graph_.initializeOptimization();
  impl_->graph_.computeInitialGuess();
  impl_->graph_.optimize(50);
}

} /* namespace dvo_slam */
