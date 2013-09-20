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

#ifndef CAMERA_TRAJECTORY_VISUALIZER_H_
#define CAMERA_TRAJECTORY_VISUALIZER_H_


#include <Eigen/Geometry>
#include <boost/function.hpp>

#include <dvo/core/rgbd_image.h>
#include <dvo/util/fluent_interface.h>

namespace dvo
{
namespace visualization
{

struct Color
{
public:
  static const Color& red()
  {
    static Color red(1.0, 0.2, 0.2);
    return red;
  }
  static const Color& green()
  {
    static Color green(0.2, 1.0, 0.2);
    return green;
  }
  static const Color& blue()
  {
    static Color blue(0.2, 0.2, 1.0);
    return blue;
  }

  Color() :
    r(0), g(0), b(0)
  {
  }
  Color(double r, double g, double b) :
    r(r), g(g), b(b)
  {
  }

  double r, g, b;
};

class CameraVisualizer
{
public:
  enum Option
  {
    ShowCameraAndCloud,
    ShowCamera,
    ShowNothing
  };

  typedef boost::shared_ptr<CameraVisualizer> Ptr;

  typedef void OnClickCallbackSignature(const CameraVisualizer&);
  typedef boost::function<OnClickCallbackSignature> OnClickCallback;

  virtual ~CameraVisualizer() {};
  FI_ATTRIBUTE(CameraVisualizer, std::string, name);
  FI_ATTRIBUTE(CameraVisualizer, Color, color);

  virtual CameraVisualizer& onclick(const OnClickCallback& callback) { return *this; }

  virtual void show(Option option = ShowCameraAndCloud) = 0;
  virtual void hide() = 0;
  virtual CameraVisualizer& update(const dvo::core::RgbdImage& img, const Eigen::Affine3d& pose) = 0;
};

class TrajectoryVisualizer
{
public:
  typedef boost::shared_ptr<TrajectoryVisualizer> Ptr;

  virtual ~TrajectoryVisualizer() {};
  FI_ATTRIBUTE(TrajectoryVisualizer, std::string, name);
  FI_ATTRIBUTE(TrajectoryVisualizer, Color, color);

  virtual TrajectoryVisualizer& add(const Eigen::Affine3d& pose) = 0;
};

class CameraTrajectoryVisualizerInterface
{
public:
  virtual ~CameraTrajectoryVisualizerInterface() {};

  virtual CameraVisualizer::Ptr camera(std::string name) = 0;
  virtual TrajectoryVisualizer::Ptr trajectory(std::string name) = 0;

  virtual void reset() = 0;

  virtual bool native(void*& native_visualizer)
  {
    native_visualizer = 0;
    return false;
  }
};

class NoopCameraTrajectoryVisualizer : public CameraTrajectoryVisualizerInterface
{
public:
  NoopCameraTrajectoryVisualizer();
  virtual ~NoopCameraTrajectoryVisualizer();

  virtual CameraVisualizer::Ptr camera(std::string name);
  virtual TrajectoryVisualizer::Ptr trajectory(std::string name);

  virtual void reset();
};

} /* namespace visualization */
} /* namespace dvo */
#endif /* CAMERA_TRAJECTORY_VISUALIZER_H_ */
