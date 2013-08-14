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

#include <dvo_slam/local_tracker.h>

#include <dvo/core/point_selection.h>
#include <dvo/core/point_selection_predicates.h>

#include <dvo/util/stopwatch.h>

#include <tbb/parallel_invoke.h>
#include <tbb/tbb_thread.h>

#include <ros/console.h>

namespace dvo_slam
{

namespace internal
{
typedef boost::shared_ptr<dvo::core::PointSelection> PointSelectionPtr;
typedef boost::shared_ptr<dvo::DenseTracker> DenseTrackerPtr;

struct LocalTrackerImpl
{
  friend class LocalTracker;

  DenseTrackerPtr keyframe_tracker_, odometry_tracker_;

  dvo::core::ValidPointAndGradientThresholdPredicate predicate;

  dvo::core::AffineTransformd last_keyframe_pose_;

  PointSelectionPtr keyframe_points_, active_frame_points_;

  bool force_;

  LocalTracker::AcceptSignal accept_;
  LocalTracker::MapInitializedSignal map_initialized_;
  LocalTracker::MapCompleteSignal map_complete_;

  static void match(const DenseTrackerPtr& tracker, const PointSelectionPtr& ref, const dvo::core::RgbdImagePyramid::Ptr& cur, LocalTracker::TrackingResult* r)
  {
    tracker->match(*ref, *cur, *r);
  }
};
} /* namespace internal */

LocalTracker::LocalTracker() :
    impl_(new internal::LocalTrackerImpl())
{
  impl_->keyframe_tracker_.reset(new dvo::DenseTracker());
  impl_->odometry_tracker_.reset(new dvo::DenseTracker());
  impl_->last_keyframe_pose_.setIdentity();
  impl_->force_ = false;
  impl_->keyframe_points_.reset(new dvo::core::PointSelection(impl_->predicate));
  impl_->active_frame_points_.reset(new dvo::core::PointSelection(impl_->predicate));
}

LocalTracker::~LocalTracker()
{
}

dvo_slam::LocalMap::Ptr LocalTracker::getLocalMap() const
{
  return local_map_;
}

void LocalTracker::getCurrentPose(dvo::core::AffineTransformd& pose)
{
  local_map_->getCurrentFramePose(pose);
}

boost::signals2::connection LocalTracker::addAcceptCallback(const AcceptCallback& callback)
{
  return impl_->accept_.connect(callback);
}

boost::signals2::connection LocalTracker::addMapCompleteCallback(const MapCompleteCallback& callback)
{
  return impl_->map_complete_.connect(callback);
}

boost::signals2::connection LocalTracker::addMapInitializedCallback(const MapInitializedCallback& callback)
{
  return impl_->map_initialized_.connect(callback);
}

const dvo::DenseTracker::Config& LocalTracker::configuration() const
{
  return impl_->odometry_tracker_->configuration();
}

void LocalTracker::configure(const dvo::DenseTracker::Config& config)
{
  impl_->keyframe_tracker_->configure(config);
  impl_->odometry_tracker_->configure(config);

  if(impl_->predicate.intensity_threshold != config.IntensityDerivativeThreshold || impl_->predicate.depth_threshold != config.DepthDerivativeThreshold)
  {
    impl_->predicate.intensity_threshold = config.IntensityDerivativeThreshold;
    impl_->predicate.depth_threshold = config.DepthDerivativeThreshold;

    if(local_map_)
    {
      impl_->keyframe_points_->setRgbdImagePyramid(*local_map_->getKeyframe());
    }
  }
}
void LocalTracker::initNewLocalMap(const dvo::core::RgbdImagePyramid::Ptr& keyframe, const dvo::core::RgbdImagePyramid::Ptr& frame, const dvo::core::AffineTransformd& keyframe_pose)
{
  impl_->keyframe_points_->setRgbdImagePyramid(*keyframe);
  impl_->active_frame_points_->setRgbdImagePyramid(*frame);

  TrackingResult r_odometry;
  r_odometry.Transformation.setIdentity();

  impl_->odometry_tracker_->match(*(impl_->keyframe_points_), *frame, r_odometry);
  impl_->last_keyframe_pose_ = r_odometry.Transformation;

  initNewLocalMap(keyframe, frame, r_odometry, keyframe_pose);
}

void LocalTracker::initNewLocalMap(const dvo::core::RgbdImagePyramid::Ptr& keyframe, const dvo::core::RgbdImagePyramid::Ptr& frame, TrackingResult& r_odometry, const dvo::core::AffineTransformd& keyframe_pose)
{
  // TODO: should be side effect free, i.e., not changing r_odometry
  if(r_odometry.isNaN())
  {
    ROS_ERROR("NaN in Map Initialization!");
    r_odometry.setIdentity();
  }

  local_map_ = LocalMap::create(keyframe, keyframe_pose);
  local_map_->addFrame(frame);
  local_map_->addKeyframeMeasurement(r_odometry.Transformation, r_odometry.Information);

  impl_->map_initialized_(*this, local_map_, r_odometry);
}

void LocalTracker::update(const dvo::core::RgbdImagePyramid::Ptr& image, dvo::core::AffineTransformd& pose)
{
  static dvo::util::stopwatch sw_prepare("prepare",100), sw_match("m", 100);
  sw_prepare.start();
  // prepare image
  const dvo::DenseTracker::Config& config = impl_->keyframe_tracker_->configuration();
  image->build(config.getNumLevels());

  for(int idx = config.LastLevel; idx <= config.FirstLevel; ++idx)
  {
    image->level(idx).buildPointCloud();
    image->level(idx).buildAccelerationStructure();
  }
  sw_prepare.stopAndPrint();

  TrackingResult r_odometry, r_keyframe;
  r_odometry.Transformation.setIdentity();
  r_keyframe.Transformation = impl_->last_keyframe_pose_.inverse(Eigen::Isometry);

  // recycle, so we can reuse the allocated memory
  impl_->active_frame_points_->setRgbdImagePyramid(*local_map_->getCurrentFrame());

  // TODO: fix me!
  boost::function<void()> h1 = boost::bind(&internal::LocalTrackerImpl::match, impl_->keyframe_tracker_, impl_->keyframe_points_, image, &r_keyframe);
  boost::function<void()> h2 = boost::bind(&internal::LocalTrackerImpl::match, impl_->odometry_tracker_, impl_->active_frame_points_, image,  &r_odometry);

  sw_match.start();
  tbb::parallel_invoke(h1, h2);
  sw_match.stopAndPrint();

  ROS_WARN_COND(r_odometry.isNaN(), "NAN in Odometry");
  ROS_WARN_COND(r_keyframe.isNaN(), "NAN in Keyframe");

  impl_->force_ = impl_->force_ || r_odometry.isNaN() || r_keyframe.isNaN();

  if(impl_->accept_(*this, r_odometry, r_keyframe) && !impl_->force_)
  {
    local_map_->addFrame(image);
    local_map_->addOdometryMeasurement(r_odometry.Transformation, r_odometry.Information);
    local_map_->addKeyframeMeasurement(r_keyframe.Transformation, r_keyframe.Information);

    impl_->last_keyframe_pose_ = r_keyframe.Transformation;
  }
  else
  {
    impl_->force_ = false;
    impl_->keyframe_points_.swap(impl_->active_frame_points_);

    dvo_slam::LocalMap::Ptr old_map = local_map_;
    dvo::core::AffineTransformd old_pose = old_map->getCurrentFramePose();
    impl_->map_complete_(*this, old_map);

    // TODO: if we have a tracking failure in odometry and in keyframe this initialization makes no sense
    initNewLocalMap(old_map->getCurrentFrame(), image, r_odometry, old_pose);

    impl_->last_keyframe_pose_ = r_odometry.Transformation;
  }

  local_map_->getCurrentFramePose(pose);
}

void LocalTracker::forceCompleteCurrentLocalMap()
{
  impl_->force_ = true;
}

} /* namespace dvo_slam */
