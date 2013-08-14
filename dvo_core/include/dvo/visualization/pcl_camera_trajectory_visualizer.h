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

#ifndef PCL_CAMERA_TRAJECTORY_VISUALIZER_H_
#define PCL_CAMERA_TRAJECTORY_VISUALIZER_H_

#include <dvo/visualization/camera_trajectory_visualizer.h>

#include <boost/thread/mutex.hpp>

#include <pcl/visualization/pcl_visualizer.h>

namespace dvo
{
namespace visualization
{

struct Switch
{
public:
  Switch(bool initial) :
    value_(initial),
    changed_(false)
  {
  }

  void toggle()
  {
    value_ = !value_;
    changed_ = true;
  }

  bool value() const
  {
    return value_;
  }

  bool changed()
  {
    bool result = changed_;
    changed_ = false;

    return result;
  }
private:
  bool value_, changed_;
};

namespace internal
{
struct PclCameraTrajectoryVisualizerImpl;
} /* namespace internal */

class PclCameraTrajectoryVisualizer : public dvo::visualization::CameraTrajectoryVisualizerInterface
{
public:
  PclCameraTrajectoryVisualizer(bool async = true);
  virtual ~PclCameraTrajectoryVisualizer();


  virtual dvo::visualization::CameraVisualizer::Ptr camera(std::string name);
  virtual dvo::visualization::TrajectoryVisualizer::Ptr trajectory(std::string name);

  virtual void reset();

  void bindSwitchToKey(Switch& s, std::string key);

  void render(int milliseconds = 15);

  boost::mutex& sync();

  pcl::visualization::PCLVisualizer& visualizer();
private:
  internal::PclCameraTrajectoryVisualizerImpl* impl_;
};

} /* namespace visualization */
} /* namespace dvo */
#endif /* PCL_CAMERA_TRAJECTORY_VISUALIZER_H_ */
