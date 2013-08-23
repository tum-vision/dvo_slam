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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <dvo_slam/keyframe_tracker.h>
#include <dvo_slam/keyframe_graph.h>
#include <dvo_slam/local_tracker.h>
#include <dvo_slam/tracking_result_evaluation.h>
#include <dvo_slam/serialization/map_serializer.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>


namespace dvo_slam
{

class KeyframeTracker::Impl
{
  ros::NodeHandle nh_;
  ros::Publisher ll_keyframe_pub_, ll_odometry_pub_, cn_odometry_pub_, cn_keyframe_pub_, de_odometry_pub_, de_keyframe_pub_, md_pub_;

  dvo_slam::visualization::GraphVisualizer* visualizer_;
public:
  friend class ::dvo_slam::KeyframeTracker;

  Impl(dvo_slam::visualization::GraphVisualizer* visualizer) :
    visualizer_(visualizer),
    graph_(),
    lt_()
  {
    if(visualizer_ != 0)
    {
      visualizer_->setGraph(&graph_);
      graph_.addMapChangedCallback(boost::bind(&KeyframeTracker::Impl::onGlobalMapChangedUpdateVisualization, this, _1));
    }

    //graph_.addMapChangedCallback(boost::bind(&KeyframeTracker::Impl::onGlobalMapChangedSaveTrajectory, this, _1));

    lt_.addMapInitializedCallback(boost::bind(&KeyframeTracker::Impl::onMapInitialized, this, _1, _2, _3));
    lt_.addMapCompleteCallback(boost::bind(&KeyframeTracker::Impl::onMapComplete, this, _1, _2));
    lt_.addAcceptCallback(boost::bind(&KeyframeTracker::Impl::onAcceptCriterionTrackingResultEvaluation, this, _1, _2, _3));
    lt_.addAcceptCallback(boost::bind(&KeyframeTracker::Impl::onAcceptCriterionEstimateDivergence, this, _1, _2, _3));
    //lt_.addAcceptCallback(boost::bind(&KeyframeTracker::Impl::onAcceptCriterionEntropyRatio, this, _1, _2, _3));
    lt_.addAcceptCallback(boost::bind(&KeyframeTracker::Impl::onAcceptCriterionDistance, this, _1, _2, _3));
    lt_.addAcceptCallback(boost::bind(&KeyframeTracker::Impl::onAcceptCriterionConstraintRatio, this, _1, _2, _3));
    lt_.addAcceptCallback(boost::bind(&KeyframeTracker::Impl::onAcceptCriterionConditionNumber, this, _1, _2, _3));

    ll_keyframe_pub_ = nh_.advertise<std_msgs::Float64>("/ll/keyframe", 1);
    ll_odometry_pub_ = nh_.advertise<std_msgs::Float64>("/ll/odometry", 1);
    cn_keyframe_pub_ = nh_.advertise<std_msgs::Float64>("/cn/keyframe", 1);
    cn_odometry_pub_ = nh_.advertise<std_msgs::Float64>("/cn/odometry", 1);
    de_keyframe_pub_ = nh_.advertise<std_msgs::Float64>("/de/keyframe", 1);
    de_odometry_pub_ = nh_.advertise<std_msgs::Float64>("/de/odometry", 1);
    md_pub_ = nh_.advertise<std_msgs::Float64>("/md", 1);
  }


  dvo_slam::TrackingResultEvaluation::Ptr evaluation;
  dvo::core::AffineTransformd last_transform_to_keyframe_;

  void onMapInitialized(const LocalTracker& lt, const LocalMap::Ptr& m, const LocalTracker::TrackingResult& r_odometry)
  {
    //cv::imshow("keyframe", m->getKeyframe()->level(0).intensity / 255.0f);
    //cv::waitKey(5);

    //std::cerr << r_odometry.Information << std::endl;

    last_transform_to_keyframe_ = r_odometry.Transformation;

    evaluation.reset(new dvo_slam::LogLikelihoodTrackingResultEvaluation(r_odometry));
  }

  void onMapComplete(const dvo_slam::LocalTracker& lt, const dvo_slam::LocalMap::Ptr& m)
  {
    dvo_slam::TrackingResultEvaluation::ConstPtr const_evaluation(evaluation);
    m->setEvaluation(const_evaluation);
    graph_.add(m);
  }

  bool onAcceptCriterionTrackingResultEvaluation(const LocalTracker& lt, const LocalTracker::TrackingResult& r_odometry, const LocalTracker::TrackingResult& r_keyframe)
  {
    std_msgs::Float64 m1, m2, m3;
    m1.data = -r_odometry.LogLikelihood;
    m2.data = -r_keyframe.LogLikelihood;
    m3.data = evaluation->ratioWithFirst(r_keyframe);

    bool accept = m3.data > cfg_.MinEntropyRatio;

    if(accept)
      evaluation->add(r_keyframe);

    ll_odometry_pub_.publish(m1);
    ll_keyframe_pub_.publish(m3);

    return accept;
  }

  bool onAcceptCriterionEstimateDivergence(const LocalTracker& lt, const LocalTracker::TrackingResult& r_odometry, const LocalTracker::TrackingResult& r_keyframe)
  {
    //std::cerr << "----->test "  << r_keyframe.Pose.translation().norm() << " " << r_odometry.Pose.translation().norm() << std::endl;

    //dvo::core::AffineTransformd diff = last_transform_to_keyframe_.inverse() * r_keyframe.Pose;

    std_msgs::Float64 md;
    //md.data = diff.translation().norm();
    //md_pub_.publish(md);

    bool reject1 = r_odometry.Transformation.translation().norm() > 0.1 || r_keyframe.Transformation.translation().norm() > 1.5 * cfg_.MaxTranslationalDistance;



    // TODO: awful hack
    if(reject1)
    {
      std::cerr << "od before modify: "<< r_odometry.Transformation.translation().norm() << std::endl;
      Eigen::Affine3d& p = const_cast<Eigen::Affine3d&>(r_odometry.Transformation);
      p.setIdentity();
      std::cerr << "od after modify: "<< r_odometry.Transformation.translation().norm() << std::endl;

      dvo::core::Matrix6d& information = const_cast<dvo::core::Matrix6d&>(r_odometry.Information);
      information.setIdentity();
      information *= 0.008 * 0.008;

      std::cerr << "kf before modify: "<< r_keyframe.Transformation.translation().norm() << std::endl;
      Eigen::Affine3d& p2 = const_cast<Eigen::Affine3d&>(r_keyframe.Transformation);
      p2 = last_transform_to_keyframe_;
      std::cerr << "kf after modify: "<< r_keyframe.Transformation.translation().norm() << std::endl;
    }

    last_transform_to_keyframe_ = r_keyframe.Transformation;

    return !reject1;
  }

  bool onAcceptCriterionDistance(const LocalTracker& lt, const LocalTracker::TrackingResult& r_odometry, const LocalTracker::TrackingResult& r_keyframe)
  {
    return r_keyframe.Transformation.translation().norm() < cfg_.MaxTranslationalDistance;
  }

  bool onAcceptCriterionConstraintRatio(const LocalTracker& lt, const LocalTracker::TrackingResult& r_odometry, const LocalTracker::TrackingResult& r_keyframe)
  {
    return (double(r_keyframe.Statistics.Levels.back().Iterations.back().ValidConstraints) / double(r_keyframe.Statistics.Levels.back().ValidPixels)) > cfg_.MinEquationSystemConstraintRatio;
  }

  bool onAcceptCriterionConditionNumber(const LocalTracker& lt, const LocalTracker::TrackingResult& r_odometry, const LocalTracker::TrackingResult& r_keyframe)
  {
    std_msgs::Float64 kappa_odometry, kappa_keyframe, de_odometry, de_keyframe;

    Eigen::SelfAdjointEigenSolver<dvo::core::Matrix6d> eigensolver1(r_odometry.Information);
    dvo::core::Vector6d eigenvalues = eigensolver1.eigenvalues().real();
    std::sort(eigenvalues.data(), eigenvalues.data() + eigenvalues.rows());

    kappa_odometry.data = std::abs(eigenvalues(5) / eigenvalues(0));
    //de_odometry.data = r_odometry.Context->Mean.sum();

    Eigen::SelfAdjointEigenSolver<dvo::core::Matrix6d> eigensolver2(r_keyframe.Information);
    eigenvalues = eigensolver2.eigenvalues().real();
    std::sort(eigenvalues.data(), eigenvalues.data() + eigenvalues.rows());

    kappa_keyframe.data = std::abs(eigenvalues(5) / eigenvalues(0));
    //de_keyframe.data = r_keyframe.Context->Mean.sum();

    cn_odometry_pub_.publish(kappa_odometry);
    cn_keyframe_pub_.publish(kappa_keyframe);

    //de_odometry_pub_.publish(de_odometry);
    //de_keyframe_pub_.publish(de_keyframe);

    return true;
  }

  void onGlobalMapChangedUpdateVisualization(KeyframeGraph& map)
  {
    visualizer_->update();
  }


  void onGlobalMapChangedSaveTrajectory(KeyframeGraph& map)
  {
    static int idx = 1;

    std::stringstream ss;
    ss << "assoc_opt_traj" << idx << ".txt";

    dvo_slam::serialization::FileSerializer<dvo_slam::serialization::TrajectorySerializer> serializer(ss.str());
    serializer.serialize(map);

    ++idx;
  }

  void forceKeyframe()
  {
    lt_.forceCompleteCurrentLocalMap();
  }

  void init(const Eigen::Affine3d& initial_transformation)
  {
    initial_transformation_ = initial_transformation;
    relative_transformation_.setIdentity();
  }

  void update(const dvo::core::RgbdImagePyramid::Ptr& current, const ros::Time& current_time, Eigen::Affine3d& absolute_transformation)
  {
    current->level(0).timestamp = current_time.toSec();

    if(!previous_)
    {
      previous_ = current;
      absolute_transformation = initial_transformation_;
      return;
    }

    if(!lt_.getLocalMap())
    {
      lt_.initNewLocalMap(previous_, current, initial_transformation_);
      lt_.getCurrentPose(absolute_transformation);
      return;
    }

    lt_.update(current, absolute_transformation);
  }

  void finish()
  {
    graph_.finalOptimization();
  }

private:
  KeyframeGraph graph_;
  LocalTracker lt_;
  Eigen::Affine3d initial_transformation_, relative_transformation_, last_absolute_transformation_;
  dvo::core::RgbdImagePyramid::Ptr previous_;

  KeyframeTrackerConfig cfg_;
};

KeyframeTracker::KeyframeTracker(dvo_slam::visualization::GraphVisualizer* visualizer) :
  impl_(new KeyframeTracker::Impl(visualizer))
{
}

void KeyframeTracker::configureTracking(const dvo::DenseTracker::Config& cfg)
{
  impl_->graph_.configureValidationTracking(cfg);
  impl_->lt_.configure(cfg);
}

void KeyframeTracker::configureKeyframeSelection(const dvo_slam::KeyframeTrackerConfig& cfg)
{
  impl_->cfg_ = cfg;
}

void KeyframeTracker::configureMapping(const dvo_slam::KeyframeGraphConfig& cfg)
{
  impl_->graph_.configure(cfg);
}

const dvo::DenseTracker::Config& KeyframeTracker::trackingConfiguration() const
{
  return impl_->lt_.configuration();
}

const dvo_slam::KeyframeTrackerConfig& KeyframeTracker::keyframeSelectionConfiguration() const
{
  return impl_->cfg_;
}

const dvo_slam::KeyframeGraphConfig& KeyframeTracker::mappingConfiguration() const
{
  return impl_->graph_.configuration();
}

void KeyframeTracker::init()
{
  init(Eigen::Affine3d::Identity());
}

void KeyframeTracker::init(const Eigen::Affine3d& initial_transformation)
{
  impl_->init(initial_transformation);
}

void KeyframeTracker::update(const dvo::core::RgbdImagePyramid::Ptr& current, const ros::Time& current_time, Eigen::Affine3d& absolute_transformation)
{
  impl_->update(current, current_time, absolute_transformation);
}

void KeyframeTracker::forceKeyframe()
{
  impl_->forceKeyframe();
}

void KeyframeTracker::finish()
{
  impl_->finish();
}

void KeyframeTracker::addMapChangedCallback(const dvo_slam::KeyframeGraph::MapChangedCallback& callback)
{
  impl_->graph_.addMapChangedCallback(callback);
}

void KeyframeTracker::serializeMap(dvo_slam::serialization::MapSerializerInterface& serializer)
{
  serializer.serialize(impl_->graph_);
}

} /* namespace dvo_slam */
