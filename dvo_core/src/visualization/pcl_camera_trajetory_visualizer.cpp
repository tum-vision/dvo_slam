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

#include <dvo/visualization/pcl_camera_trajectory_visualizer.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <dvo/visualization/async_point_cloud_builder.h>
#include <dvo/visualization/point_cloud_aggregator.h>

namespace dvo
{
namespace visualization
{
namespace internal
{

class PclTrajectoryVisualizer : public dvo::visualization::TrajectoryVisualizer
{
public:
  PclTrajectoryVisualizer(std::string& name)
  {
    name_ = name;
  }

  virtual TrajectoryVisualizer& add(const Eigen::Affine3d& pose)
  {
    PointPair latest;
    latest.second.x = pose.translation()(0);
    latest.second.y = pose.translation()(1);
    latest.second.z = pose.translation()(2);

    if(pairs_.empty())
    {
      ::boost::mutex::scoped_lock lock(m_);

      pairs_.push_back(latest); // add to list - this copies the object
      pairs_.back().first = &(pairs_.back().second); // set pointer to second point in newly allocated object
      last_ = pairs_.begin();
    }

    latest.first = &(pairs_.back().second);

    {
      boost::mutex::scoped_lock lock(m_);
      pairs_.push_back(latest);
    }

    return *this;
  }

  void updateVisualizer(pcl::visualization::PCLVisualizer& visualizer)
  {
    if(pairs_.empty()) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for(list::iterator it = pairs_.begin(); it != pairs_.end(); ++it)
    {
      cloud->points.push_back(it->second);
    }
    for(list::reverse_iterator it = pairs_.rbegin(); it != pairs_.rend(); ++it)
    {
      cloud->points.push_back(it->second);
    }

    visualizer.removeShape(name());
    visualizer.addPolygon<pcl::PointXYZ>(cloud, name());
    visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,  color().r, color().g, color().b, name());
    visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, name());
  }
private:
  class PointPair
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    pcl::PointXYZ second;
    pcl::PointXYZ *first;
  };

  typedef std::list<PointPair, Eigen::aligned_allocator<PointPair> > list;

  list pairs_;
  list::iterator last_;

  boost::mutex m_;
};

class PclCameraVisualizer : public dvo::visualization::CameraVisualizer
{
public:
  PclCameraVisualizer(std::string& name)
  {
    name_ = name;
  }

  virtual ~PclCameraVisualizer()
  {
  }

  virtual void show(Option option = ShowCameraAndCloud)
  {
    visibility_ = option;
  }

  virtual void hide()
  {
    visibility_ = ShowNothing;
  }

  virtual CameraVisualizer& update(const dvo::core::RgbdImage& img, const Eigen::Affine3d& pose)
  {
    ::boost::mutex::scoped_lock lock(m_);

    cloud_.reset(new dvo::visualization::AsyncPointCloudBuilder::BuildJob(img, pose));
    return *this;
  }

  void updateVisualizer(pcl::visualization::PCLVisualizer& visualizer, PointCloudAggregator& aggregator)
  {
    boost::mutex::scoped_lock lock(m_);

    std::string cube_id = name() + "_cube";

    visualizer.removeShape(cube_id);

    if(visibility_ != ShowCameraAndCloud)
    {
      //visualizer.removePointCloud(name());
      aggregator.remove(name());
    }

    if(visibility_ != ShowNothing && cloud_)
    {
      if(visibility_ == ShowCameraAndCloud)
      {
        aggregator.add(name(), cloud_->build());
        //if(!visualizer.updatePointCloud(cloud_->build(), name()))
        //{
        //  visualizer.addPointCloud(cloud_->build(), name());
        //}
        //visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name());
      }
      //visualizer.addCube(cloud_->pose.translation().cast<float>(), Eigen::Quaternionf(cloud_->pose.rotation().cast<float>()), 0.05, 0.05, 0.05, cube_id);
      //visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color().r, color().g, color().b, cube_id);
      //visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cube_id);
    }
  }
private:
  Option visibility_;
  boost::shared_ptr<dvo::visualization::AsyncPointCloudBuilder::BuildJob> cloud_;
  boost::mutex m_;
};

struct PclCameraTrajectoryVisualizerImpl
{
public:
  typedef boost::shared_ptr<PclCameraVisualizer> PclCameraVisualizerPtr;
  typedef boost::shared_ptr<PclTrajectoryVisualizer> PclTrajectoryVisualizerPtr;

  typedef std::map<std::string, PclCameraVisualizerPtr> CameraVisualizerMap;
  typedef std::map<std::string, PclTrajectoryVisualizerPtr> TrajectoryVisualizerMap;

  PclCameraTrajectoryVisualizerImpl(bool render_thread = true)
  {
    if(render_thread)
    {
      visualizer_thread_.reset(new boost::thread(boost::bind(&PclCameraTrajectoryVisualizerImpl::execVisualizerThread, this)));

      // wait for visualizer_ being created
      while(!visualizer_)
      {
        boost::this_thread::yield();
      }
    }
    else
    {
      initVisualizer();
    }
  }

  ~PclCameraTrajectoryVisualizerImpl()
  {
    if(visualizer_ && !visualizer_->wasStopped())
    {
#if ((PCL_MAJOR_VERSION == 1) && (PCL_MINOR_VERSION > 5))
      // only available in PCL >= 1.6
      visualizer_->close();
#endif
    }
    visualizer_thread_->join();
  }

  void bindSwitchToKey(Switch& s, std::string& key)
  {
    visualizer_->registerKeyboardCallback(&PclCameraTrajectoryVisualizerImpl::onSwitchKeyPressed, new SwitchKeyBinding(s, key));
  }

  pcl::visualization::PCLVisualizer& visualizer()
  {
    return *visualizer_;
  }

  CameraVisualizer::Ptr camera(std::string name)
  {
    CameraVisualizerMap::iterator camera = camera_visualizers_.find(name);

    if(camera_visualizers_.end() == camera)
    {
      camera = camera_visualizers_.insert(
          std::make_pair(name, PclCameraVisualizerPtr(new PclCameraVisualizer(name)))
      ).first;
    }

    return camera->second;
  }

  TrajectoryVisualizer::Ptr trajectory(std::string name)
  {
    TrajectoryVisualizerMap::iterator trajectory = trajectory_visualizers_.find(name);

    if(trajectory_visualizers_.end() == trajectory)
    {
      trajectory = trajectory_visualizers_.insert(
          std::make_pair(name, PclTrajectoryVisualizerPtr(new PclTrajectoryVisualizer(name)))
      ).first;
    }

    return trajectory->second;
  }

  void reset()
  {
    camera_visualizers_.clear();
    trajectory_visualizers_.clear();
  }

  void initVisualizer()
  {
    visualizer_.reset(new pcl::visualization::PCLVisualizer("viewer"));
    visualizer_->setBackgroundColor(0.3, 0.3, 0.3);
    visualizer_->initCameraParameters();
  }

  void render(int milliseconds)
  {
    // safeguard against segfault when updating the visualizer from multiple threads
    if(visualizer_thread_ && visualizer_thread_->get_id() != boost::this_thread::get_id())
    {
      static bool warned = false;

      if(!warned)
      {
        warned = true;
        std::cerr << "WARNING: called PclCameraTrajectoryVisualizer::render() from wrong thread!" << std::endl;
      }
      return;
    }

    CameraVisualizerMap local_cameras;
    TrajectoryVisualizerMap local_trajectories;

    {
      boost::mutex::scoped_lock lock(m_);
      local_cameras = camera_visualizers_;
      local_trajectories = trajectory_visualizers_;
    }

    // sync visualizer
    boost::mutex::scoped_lock lock(sync_);

    for(CameraVisualizerMap::iterator it = local_cameras.begin(); it != local_cameras.end(); ++it)
    {
      it->second->updateVisualizer(*visualizer_, cloud_aggregator_);
    }

    AsyncPointCloudBuilder::PointCloud::Ptr aggregated_cloud = cloud_aggregator_.build();

    if(!visualizer_->updatePointCloud(aggregated_cloud, "aggregated_cloud"))
    {
      visualizer_->addPointCloud(aggregated_cloud, "aggregated_cloud");
    }

    for(TrajectoryVisualizerMap::iterator it = local_trajectories.begin(); it != local_trajectories.end(); ++it)
    {
      it->second->updateVisualizer(*visualizer_);
    }

    visualizer_->spinOnce(milliseconds);
  }
  boost::mutex sync_;
private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer_;
  boost::shared_ptr<boost::thread> visualizer_thread_;
  boost::mutex m_;

  CameraVisualizerMap camera_visualizers_;
  TrajectoryVisualizerMap trajectory_visualizers_;

  std::map<std::string, PclCameraVisualizer::Ptr> clouds_;
  PointCloudAggregator cloud_aggregator_;

  void execVisualizerThread()
  {
    initVisualizer();

    while(!visualizer_->wasStopped())
    {
      render(1);
    }
    visualizer_.reset();
  }

  struct SwitchKeyBinding
  {
    Switch& s;
    std::string key;
    SwitchKeyBinding(Switch& s, std::string& key)
      : s(s), key(key)
    {
    }
  };

  static void onSwitchKeyPressed(const pcl::visualization::KeyboardEvent& e, void* data)
  {
    SwitchKeyBinding* binding = (SwitchKeyBinding*) data;
    if(e.keyDown() && e.getKeySym() == binding->key)
    {
      binding->s.toggle();
    }
  }
};
} /* namespace internal */


PclCameraTrajectoryVisualizer::PclCameraTrajectoryVisualizer(bool async) :
    impl_(new internal::PclCameraTrajectoryVisualizerImpl(async))
{
}

PclCameraTrajectoryVisualizer::~PclCameraTrajectoryVisualizer()
{
  delete impl_;
}

dvo::visualization::CameraVisualizer::Ptr PclCameraTrajectoryVisualizer::camera(std::string name)
{
  return impl_->camera(name);
}

dvo::visualization::TrajectoryVisualizer::Ptr PclCameraTrajectoryVisualizer::trajectory(std::string name)
{
  return impl_->trajectory(name);
}

void PclCameraTrajectoryVisualizer::bindSwitchToKey(Switch& s, std::string key)
{
  impl_->bindSwitchToKey(s, key);
}

void PclCameraTrajectoryVisualizer::render(int milliseconds)
{
  impl_->render(milliseconds);
}

boost::mutex& PclCameraTrajectoryVisualizer::sync()
{
  return impl_->sync_;
}

pcl::visualization::PCLVisualizer& PclCameraTrajectoryVisualizer::visualizer()
{
  return impl_->visualizer();
}

void PclCameraTrajectoryVisualizer::reset()
{
  impl_->reset();
}

} /* namespace visualization */
} /* namespace dvo */
