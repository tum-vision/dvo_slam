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

#include <boost/thread.hpp>
#include <fstream>

#include <dvo/visualization/visualizer.h>
#include <dvo/util/histogram.h>
#include "bounded_buffer.h"

namespace dvo
{
namespace visualization
{

static inline void printMinMax(const cv::Mat& img, const char* prefix = "")
{
  double min, max;

  cv::minMaxLoc(img, &min, &max);

  std::cerr << prefix << " min: " << min << " max: " << max << std::endl;
}

static inline void show(const char* name, const cv::Mat& img)
{
  if(img.type() == CV_32F)
  {
    double min, max;

    cv::minMaxLoc(img, &min, &max);

    cv::Mat display = img / max;

    cv::imshow(name, display);
  }
  else
  {
    cv::imshow(name, img);
  }
}

class VisualizerImpl
{
public:
  struct CreateHistogramFunc
  {
    CreateHistogramFunc() {}
    CreateHistogramFunc(VisualizerImpl* impl, std::string& name, const cv::Mat& img, float binsize, float min, float max) :
      impl(impl),
      name(name),
      img(img),
      binsize(binsize),
      min(min),
      max(max)
    { }
    CreateHistogramFunc(const CreateHistogramFunc& other) :
      impl(other.impl),
      name(other.name),
      img(other.img),
      binsize(other.binsize),
      min(other.min),
      max(other.max)
    { }

    ~CreateHistogramFunc() {}

    void operator()()
    {
      cv::Mat mask, hist, hist_vis;

      dvo::util::compute1DHistogram(img, hist, min, max, binsize);

      int bins = hist.size().area();
      int width = bins * (5 + 2) + 10;
      int height = 420;

      hist_vis.create(height, width, CV_8UC3);
      hist_vis.setTo(cv::Scalar(255, 255, 255));

      int x = 5 + 1;

      for(int idx = 0; idx < bins; ++idx, x += 7)
      {
        int binheight = (int) std::min(hist.at<float>(idx), 400.0f);
        cv::rectangle(hist_vis, cv::Rect(x, 10, 5, binheight), cv::Scalar(255, 0, 0), CV_FILLED);
      }

      impl->internalShow(name, cv::MatExpr(hist_vis));
    }

    void save(std::string filename)
    {
      cv::Mat hist;

      dvo::util::compute1DHistogram(img, hist, min, max, 1);

      int bins = hist.size().area();
      float current_bin = min;

      std::ofstream out(filename.c_str());

      for(int idx = 0; idx < bins; ++idx, current_bin += 1)
      {
        out << current_bin << " " << hist.at<float>(idx) << std::endl;
      }

      out.flush();
      out.close();
    }

    VisualizerImpl* impl;
    std::string name;
    cv::Mat img;
    float binsize, min, max;
  };

  struct NamedImage
  {
    NamedImage() {}
    NamedImage(std::string& name, const cv::MatExpr& img, Visualizer::ImageModifier& modifier) :
      name(name),
      img(img),
      modifier(modifier)
    { }

    std::string name;
    cv::MatExpr img;
    Visualizer::ImageModifier modifier;
  };

  typedef std::map<std::string, cv::Size> NameSizeMap;
  typedef std::map<std::string, int> NameSequenceMap;

  VisualizerImpl() :
    external_wait_(false),
    save_(false),
    shutdown_(false),
    image_queue_(30),
    histogram_queue_(30),
    vis_thread_(boost::bind(&VisualizerImpl::visWorker, this)),
    histogram_thread_(&VisualizerImpl::histogramWorker, this)
  {
  }

  ~VisualizerImpl()
  {
    shutdown_ = true;
    image_queue_.shutdown();
    histogram_queue_.shutdown();
    vis_thread_.join();
    histogram_thread_.join();
  }

  void show(std::string& name, const cv::MatExpr& img, Visualizer::ImageModifier modifier)
  {
    if(shutdown_ || !checkImageSize(name, img)) return;

    internalShow(name, img, modifier);
  }

  void showHistogram(std::string& name, const cv::Mat& img, float binsize, float min, float max)
  {
    if(shutdown_ || !checkImageSize(name, img)) return;

    CreateHistogramFunc f(this, name, img, binsize, min, max);

    histogram_queue_.push_front(f);
  }

  void internalShow(std::string& name, const cv::MatExpr& img, Visualizer::ImageModifier modifier = Visualizer::ImageModifier())
  {
    image_queue_.push_front(NamedImage(name, img, modifier));
  }

  bool checkImageSize(std::string& name, const cv::Mat& img)
  {
    NameSizeMap::iterator entry = image_size_lookup_.find(name);

    if(entry == image_size_lookup_.end() || entry->second.width <= img.size().width || entry->second.height <= img.size().height)
    {
      image_size_lookup_[name] = img.size();
      return true;
    }
    else
    {
      return false;
    }
  }

  void visWorker()
  {
    NamedImage named_img;

    while(!shutdown_)
    {
      if(image_queue_.pop_back(&named_img))
      {
        cv::Mat img;

        if(!named_img.modifier.empty())
        {
          ((cv::Mat) named_img.img).copyTo(img);
          named_img.modifier(img);
        }
        else
        {
          img = named_img.img;
        }

        dvo::visualization::show(named_img.name.c_str(), img);

        if(save_)
        {
          if(image_sequence_lookup_.find(named_img.name) == image_sequence_lookup_.end())
          {
            image_sequence_lookup_[named_img.name] = 0;
          }

          std::stringstream filename;
          filename << named_img.name << "_" << image_sequence_lookup_[named_img.name] << ".png";
          printMinMax(img,  named_img.name.c_str());
          double min, max;

          cv::minMaxLoc(img, &min, &max);

          cv::imwrite(filename.str(), img / max * 255);

          image_sequence_lookup_[named_img.name] += 1;
        }

        if(!external_wait_) // if we call waitKey from more than one thread, we can deadlock :(
          cv::waitKey(3);
      }
    }
  }

  void histogramWorker()
  {
    CreateHistogramFunc create_histogram;

    while(!shutdown_)
    {
        if(histogram_queue_.pop_back(&create_histogram))
        {
          create_histogram();

          if(save_)
          {
            std::string name = create_histogram.name + "__hist";

            if(image_sequence_lookup_.find(name) == image_sequence_lookup_.end())
            {
              image_sequence_lookup_[name] = 0;
            }

            std::stringstream filename;
            filename << create_histogram.name << "_" << image_sequence_lookup_[name] << ".txt";

            create_histogram.save(filename.str());

            image_sequence_lookup_[name] += 1;
          }
        }
    }
  }

  bool external_wait_, save_;
private:
  bool shutdown_;

  bounded_buffer<NamedImage> image_queue_;
  bounded_buffer<CreateHistogramFunc> histogram_queue_;

  NameSizeMap image_size_lookup_;
  NameSequenceMap image_sequence_lookup_;

  boost::thread vis_thread_, histogram_thread_;
};

Visualizer& Visualizer::instance()
{
  static Visualizer instance;

  return instance;
}

Visualizer& Visualizer::show(const char* name, const cv::MatExpr& img, ImageModifier modifier)
{
  std::string n(name);

  return show(n, img, modifier);
}

Visualizer& Visualizer::show(std::string& name, const cv::MatExpr& img, ImageModifier modifier)
{
  if(enabled())
    impl_->show(name, img, modifier);

  return *this;
}

Visualizer& Visualizer::show(const char* name, const cv::Mat& img, ImageModifier modifier)
{
  return show(name, cv::MatExpr(img), modifier);
}

Visualizer& Visualizer::show(std::string& name, const cv::Mat& img, ImageModifier modifier)
{
  return show(name, cv::MatExpr(img), modifier);
}

Visualizer& Visualizer::showHistogram(const char* name, const cv::Mat& img, float binsize, float min, float max)
{
  std::string n(name);

  return showHistogram(n, img, binsize, min, max);
}

Visualizer& Visualizer::showHistogram(std::string& name, const cv::Mat& img, float binsize, float min, float max)
{
  if(enabled())
    impl_->showHistogram(name, img, binsize, min, max);

  return *this;
}

Visualizer& Visualizer::useExternalWaitKey(bool value)
{
  impl_->external_wait_ = value;

  return *this;
}

bool Visualizer::useExternalWaitKey() const
{
  return impl_->external_wait_;
}

Visualizer& Visualizer::save(bool value)
{
  impl_->save_ = value;

  return *this;
}

bool Visualizer::save() const
{
  return impl_->save_;
}

Visualizer::Visualizer() :
    impl_(new VisualizerImpl())
{
}

Visualizer::~Visualizer()
{
  delete impl_;
}

} /* namespace visualization */
} /* namespace dvo */
