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

#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include <opencv2/opencv.hpp>
#include <boost/function.hpp>

#include <dvo/util/fluent_interface.h>

namespace dvo
{
namespace visualization
{

class VisualizerImpl;

class Visualizer
{
public:
  static Visualizer& instance();

  typedef boost::function<void(cv::Mat&)> ImageModifier;

  // with this MatExpr stuff the calculations seem to be performed in the visualizer thread, nice :)
  Visualizer& show(const char* name, const cv::MatExpr& img, ImageModifier modifier = ImageModifier());
  Visualizer& show(std::string& name, const cv::MatExpr& img, ImageModifier modifier = ImageModifier());

  Visualizer& show(const char* name, const cv::Mat& img, ImageModifier modifier = ImageModifier());
  Visualizer& show(std::string& name, const cv::Mat& img, ImageModifier modifier = ImageModifier());

  Visualizer& showHistogram(const char* name, const cv::Mat& img, float binsize, float min, float max);
  Visualizer& showHistogram(std::string& name, const cv::Mat& img, float binsize, float min, float max);

  // if we call waitKey from more than one thread, we can deadlock :(
  Visualizer& useExternalWaitKey(bool value);

  bool useExternalWaitKey() const;

  Visualizer& save(bool value);

  bool save() const;

  FI_ATTRIBUTE(Visualizer, bool, enabled)
private:
  Visualizer();
  ~Visualizer();

  VisualizerImpl* impl_;
};

} /* namespace visualization */
} /* namespace dvo */
#endif /* VISUALIZER_H_ */
