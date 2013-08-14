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

#include <dvo/visualization/camera_trajectory_visualizer.h>

namespace dvo
{
namespace visualization
{

class NoopCameraVisualizer : public CameraVisualizer
{
public:
  NoopCameraVisualizer() {}
  virtual ~NoopCameraVisualizer() {};

  virtual void show(Option option = ShowCameraAndCloud) {};
  virtual void hide() {};

  virtual CameraVisualizer& update(const dvo::core::RgbdImage& img, const Eigen::Affine3d& pose)
  {
    return *this;
  }
};

class NoopTrajectoryVisualizer : public TrajectoryVisualizer
{
public:
  NoopTrajectoryVisualizer() {}

  virtual ~NoopTrajectoryVisualizer() {}

  virtual TrajectoryVisualizer& add(const Eigen::Affine3d& pose)
  {
    return *this;
  }
};

NoopCameraTrajectoryVisualizer::NoopCameraTrajectoryVisualizer()
{
}

NoopCameraTrajectoryVisualizer::~NoopCameraTrajectoryVisualizer()
{
}

CameraVisualizer::Ptr NoopCameraTrajectoryVisualizer::camera(std::string name)
{
  static CameraVisualizer::Ptr vis(new NoopCameraVisualizer());

  return vis;
}

TrajectoryVisualizer::Ptr NoopCameraTrajectoryVisualizer::trajectory(std::string name)
{
  static TrajectoryVisualizer::Ptr vis(new NoopTrajectoryVisualizer());

  return vis;
}

void NoopCameraTrajectoryVisualizer::reset()
{
}

} /* namespace visualization */
} /* namespace dvo */
