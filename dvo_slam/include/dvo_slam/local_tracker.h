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

#ifndef LOCAL_TRACKER_H_
#define LOCAL_TRACKER_H_

#include <dvo/core/rgbd_image.h>
#include <dvo/dense_tracking.h>

#include <dvo_slam/local_map.h>

#include <boost/scoped_ptr.hpp>
#include <boost/signals2.hpp>

namespace dvo_slam
{

namespace internal
{
  struct LocalTrackerImpl;
} /* namespace internal */

class LocalTracker
{
public:
  typedef ::dvo::DenseTracker::Result TrackingResult;

  struct All
  {
    typedef bool result_type;

    template<typename InputIterator>
    bool operator()(InputIterator first, InputIterator last) const
    {
      int i = 0;
      bool result = true;
      //std::cerr << "votes: ";
      for(;first != last; ++first)
      {
        bool tmp = *first;
        result = result && tmp;

        //std::cerr << tmp << " ";
        i++;
      }

      //std::cerr << std::endl;

      return result;
    }
  };

  typedef boost::signals2::signal<bool (const dvo_slam::LocalTracker&, const dvo_slam::LocalTracker::TrackingResult&, const dvo_slam::LocalTracker::TrackingResult&), dvo_slam::LocalTracker::All> AcceptSignal;
  typedef AcceptSignal::slot_type AcceptCallback;
  typedef boost::signals2::signal<void (const dvo_slam::LocalTracker&, const dvo_slam::LocalMap::Ptr&, const dvo_slam::LocalTracker::TrackingResult&)> MapInitializedSignal;
  typedef MapInitializedSignal::slot_type MapInitializedCallback;
  typedef boost::signals2::signal<void (const dvo_slam::LocalTracker&, const dvo_slam::LocalMap::Ptr&)> MapCompleteSignal;
  typedef MapCompleteSignal::slot_type MapCompleteCallback;

  LocalTracker();
  virtual ~LocalTracker();

  dvo_slam::LocalMap::Ptr getLocalMap() const;

  void getCurrentPose(dvo::core::AffineTransformd& pose);

  void initNewLocalMap(const dvo::core::RgbdImagePyramid::Ptr& keyframe, const dvo::core::RgbdImagePyramid::Ptr& frame, const dvo::core::AffineTransformd& keyframe_pose = dvo::core::AffineTransformd::Identity());

  const dvo::DenseTracker::Config& configuration() const;

  void configure(const dvo::DenseTracker::Config& config);

  void update(const dvo::core::RgbdImagePyramid::Ptr& image, dvo::core::AffineTransformd& pose);

  void forceCompleteCurrentLocalMap();

  boost::signals2::connection addAcceptCallback(const AcceptCallback& callback);
  boost::signals2::connection addMapInitializedCallback(const MapInitializedCallback& callback);
  boost::signals2::connection addMapCompleteCallback(const MapCompleteCallback& callback);
private:
  boost::scoped_ptr<internal::LocalTrackerImpl> impl_;
  dvo_slam::LocalMap::Ptr local_map_;

  void initNewLocalMap(const dvo::core::RgbdImagePyramid::Ptr& keyframe, const dvo::core::RgbdImagePyramid::Ptr& frame, TrackingResult& r_odometry, const dvo::core::AffineTransformd& keyframe_pose);

};

} /* namespace dvo_slam */
#endif /* LOCAL_TRACKER_H_ */
