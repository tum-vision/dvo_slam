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

#include <ros/ros.h>

#include <dvo/core/intrinsic_matrix.h>
#include <dvo/core/rgbd_image.h>
#include <dvo/core/surface_pyramid.h>

#include <dvo/visualization/visualizer.h>
#include <dvo/visualization/camera_trajectory_visualizer.h>
#include <dvo/visualization/pcl_camera_trajectory_visualizer.h>

#include <dvo/dense_tracking.h>

#include <dvo_ros/util/configtools.h>
#include <dvo/util/stopwatch.h>
#include <dvo/util/id_generator.h>

#include <dvo_benchmark/file_reader.h>
#include <dvo_benchmark/rgbd_pair.h>
#include <dvo_benchmark/groundtruth.h>
#include <dvo_benchmark/tools.h>

dvo::core::RgbdImagePyramid::Ptr load(std::string rgb_file, std::string depth_file)
{
  cv::Mat rgb, grey, grey_s16, depth, depth_inpainted, depth_mask, depth_mono, depth_float;

  bool rgb_available = false;
  rgb = cv::imread(rgb_file, 1);
  depth = cv::imread(depth_file, -1);

  if(rgb.type() != CV_32FC1)
  {
    if(rgb.type() == CV_8UC3)
    {
      cv::cvtColor(rgb, grey, CV_BGR2GRAY);
      rgb_available = true;
    }
    else
    {
      grey = rgb;
    }

    grey.convertTo(grey_s16, CV_32F);
  }
  else
  {
    grey_s16 = rgb;
  }

  if(depth.type() != CV_32FC1)
  {
    dvo::core::SurfacePyramid::convertRawDepthImageSse(depth, depth_float,  1.035f / 5000.0f);
  }
  else
  {
    depth_float = depth;
  }

  dvo::core::RgbdImagePyramid::Ptr result(new dvo::core::RgbdImagePyramid(grey_s16, depth_float));

  if(rgb_available)
    rgb.convertTo(result->level(0).rgb, CV_32FC3);

  return result;
}

class BenchmarkNode
{
public:
  struct Config
  {
    bool EstimateTrajectory;
    std::string TrajectoryFile;
    bool RenderVideo;
    std::string VideoFolder;

    std::string CameraFile;

    std::string RgbdPairFile;
    std::string GroundtruthFile;

    bool ShowGroundtruth;
    bool ShowEstimate;

    bool EstimateRequired();
    bool VisualizationRequired();
  };

  BenchmarkNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  bool configure();

  void run();

  void createReferenceCamera(dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer, const dvo::core::RgbdImage& img, const dvo::core::IntrinsicMatrix& intrinsics, const Eigen::Affine3d& pose);

  void renderWhileSwitchAndNotTerminated(dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer, const dvo::visualization::Switch& s);

  void processInput(dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer);
private:
  ros::NodeHandle& nh_, nh_private_;
  Config cfg_;

  std::ostream *trajectory_out_;
  dvo_benchmark::FileReader<dvo_benchmark::Groundtruth> *groundtruth_reader_;
  dvo_benchmark::FileReader<dvo_benchmark::RgbdPair> *rgbdpair_reader_;

  dvo::visualization::Switch dump_camera_pose_, load_camera_pose_;
};

bool BenchmarkNode::Config::EstimateRequired()
{
  return EstimateTrajectory || ShowEstimate;
}

bool BenchmarkNode::Config::VisualizationRequired()
{
  return ShowGroundtruth || ShowEstimate || RenderVideo;
}

BenchmarkNode::BenchmarkNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
    nh_(nh),
    nh_private_(nh_private),
    trajectory_out_(0),
    groundtruth_reader_(0),
    rgbdpair_reader_(0),
    dump_camera_pose_(false),
    load_camera_pose_(false)
{
}

bool BenchmarkNode::configure()
{
  // dataset files related stuff
  if(nh_private_.getParam("rgbdpair_file", cfg_.RgbdPairFile))
  {
    rgbdpair_reader_ = new dvo_benchmark::FileReader<dvo_benchmark::RgbdPair>(cfg_.RgbdPairFile);
    rgbdpair_reader_->skipComments();

    if(!rgbdpair_reader_->next())
    {
      std::cerr << "Failed to open '" << cfg_.RgbdPairFile << "'!" << std::endl;
      return false;
    }
  }
  else
  {
    std::cerr << "Missing 'rgbdpair_file' parameter!" << std::endl;
    return false;
  }

  // trajectory estimation related stuff
  nh_private_.param("estimate_trajectory", cfg_.EstimateTrajectory, false);
  if(cfg_.EstimateTrajectory)
  {
    if(nh_private_.getParam("trajectory_file", cfg_.TrajectoryFile) && !cfg_.TrajectoryFile.empty())
    {
      trajectory_out_ = new std::ofstream(cfg_.TrajectoryFile.c_str());

      if(trajectory_out_->fail())
      {
        delete trajectory_out_;

        std::cerr << "Failed to open '" << cfg_.TrajectoryFile << "'!" << std::endl;
        return false;
      }
    }
    else
    {
      trajectory_out_ = &std::cout;
    }
  }

  // video rendering related stuff
  nh_private_.param("render_video", cfg_.RenderVideo, false);
  if(cfg_.RenderVideo)
  {
    if(!nh_private_.getParam("video_folder", cfg_.VideoFolder) || cfg_.VideoFolder.empty())
    {
      std::cerr << "Missing 'video_folder' parameter!" << std::endl;
      return false;
    }
  }

  nh_private_.param("camera_file", cfg_.CameraFile, std::string(""));

  // ground truth related stuff
  nh_private_.param("show_groundtruth", cfg_.ShowGroundtruth, false);
  if(cfg_.ShowGroundtruth)
  {
    if(nh_private_.getParam("groundtruth_file", cfg_.GroundtruthFile))
    {
      groundtruth_reader_ = new dvo_benchmark::FileReader<dvo_benchmark::Groundtruth>(cfg_.GroundtruthFile);
      groundtruth_reader_->skipComments();

      if(!groundtruth_reader_->next())
      {
        std::cerr << "Failed to open '" << cfg_.GroundtruthFile << "'!" << std::endl;
        return false;
      }
    }
    else
    {
      std::cerr << "Missing 'groundtruth_file' parameter!" << std::endl;
      return false;
    }
  }

  nh_private_.param("show_estimate", cfg_.ShowEstimate, false);

  return true;
}

void BenchmarkNode::renderWhileSwitchAndNotTerminated(dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer, const dvo::visualization::Switch& s)
{
  if(cfg_.VisualizationRequired())
  {
    while(s.value() && ros::ok())
    {
      processInput(visualizer);

      // manual render in case we want to render a video
      if(cfg_.RenderVideo)
      {
        ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->render(5);
      }
      else
      {
        ros::Rate(5).sleep();
      }
    }
  }
}

void BenchmarkNode::processInput(dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer)
{
  if(cfg_.VisualizationRequired())
  {
    if(dump_camera_pose_.value())
    {
      if(!cfg_.CameraFile.empty())
      {
        std::cerr << "saving camera pose to '" << cfg_.CameraFile << "'" << std::endl;

        std::vector<pcl::visualization::Camera> cams;
        ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->visualizer().getCameras(cams);

        // same output format as PCLVisualizerInteractorStyle (when pressing 'c' key)
        std::ofstream camera_pose_file(cfg_.CameraFile.c_str());
        camera_pose_file
        << cams[0].clip[0] << "," << cams[0].clip[1] << "/"
        << cams[0].focal[0] << "," << cams[0].focal[1] << "," << cams[0].focal[2] << "/"
        << cams[0].pos[0] << "," << cams[0].pos[1] << "," << cams[0].pos[2] << "/"
        << cams[0].view[0] << "," << cams[0].view[1] << "," << cams[0].view[2] << "/"
        << cams[0].fovy << "/"
        << cams[0].window_size[0] << "," << cams[0].window_size[1] << "/"
        << cams[0].window_pos[0] << "," << cams[0].window_pos[1];
        camera_pose_file.close();
      }
      else
      {
        std::cerr << "Can't save camera pose, set the 'camera_file' parameter!" << std::endl;
      }

      dump_camera_pose_.toggle();
    }

    if(load_camera_pose_.value())
    {
      if(!cfg_.CameraFile.empty())
      {
        std::cerr << "loading camera pose from '" << cfg_.CameraFile << "'" << std::endl;

        char* option_string = "-cam";
        char parameter_string[2048];

        std::ifstream camera_pose_file(cfg_.CameraFile.c_str());
        camera_pose_file.getline(parameter_string, 2048, '\n');
        camera_pose_file.close();

        char** argv = new char*[3];
        argv[1] = option_string;
        argv[2] = parameter_string;

        ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->visualizer().getCameraParameters(3, argv);
        ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->visualizer().updateCamera();
        delete [] argv;
      }
      else
      {
        std::cerr << "Can't load camera pose, set the 'camera_file' parameter!" << std::endl;
      }

      load_camera_pose_.toggle();
    }
  }
}

void BenchmarkNode::createReferenceCamera(dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer, const dvo::core::RgbdImage& img, const dvo::core::IntrinsicMatrix& intrinsics, const Eigen::Affine3d& pose)
{
  dvo::core::RgbdImage first;
  first.rgb = img.rgb.clone();
  first.depth = img.depth.clone();
  first.initialize();

  cv::Vec3f* p = first.rgb.ptr<cv::Vec3f>();

  for(int idx = 0; idx < first.rgb.size().area(); ++idx, ++p)
  {
    p->val[2] = 0.666f * p->val[2];
    p->val[1] = 0.666f * p->val[1];
    p->val[0] = std::min(255.0f, 1.333f * p->val[0]);
  }

  visualizer->camera("reference")->
      color(dvo::visualization::Color::blue()).
      update(first, intrinsics, pose).
      show();
}

void BenchmarkNode::run()
{
  // setup visualizer
  dvo::visualization::Switch pause_switch(true), dummy_switch(true);
  dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer;

  if(cfg_.VisualizationRequired())
  {
    visualizer = new dvo::visualization::PclCameraTrajectoryVisualizer(!cfg_.RenderVideo);
    ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->bindSwitchToKey(pause_switch, "p");
    ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->bindSwitchToKey(dump_camera_pose_, "s");

    if(cfg_.RenderVideo)
    {
      ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->bindSwitchToKey(load_camera_pose_, "l");
      ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->visualizer().getRenderWindow()->SetSize(1280, 960);
    }
  }
  else
  {
    visualizer = new dvo::visualization::NoopCameraTrajectoryVisualizer();
  }

  dvo::util::IdGenerator frame_ids(cfg_.VideoFolder + std::string("/frame_"));

  // configure debugging visualizer
  dvo::visualization::Visualizer::instance()
    .useExternalWaitKey(false)
    .enabled(false)
    .save(false)
  ;

  // setup camera parameters
  // TODO: load from file
  //dvo::core::IntrinsicMatrix intrinsics = dvo::core::IntrinsicMatrix::create(525.0f, 525.0f, 320.0f, 240.0f);
  dvo::core::IntrinsicMatrix intrinsics = dvo::core::IntrinsicMatrix::create(517.3, 516.5, 318.6, 255.3);

  // setup tracker configuration
  dvo_ros::CameraDenseTrackerConfig dynreconfg_cfg = dvo_ros::CameraDenseTrackerConfig::__getDefault__();
  dynreconfg_cfg.__fromServer__(nh_private_);

  dvo::DenseTracker::Config cfg = dvo::DenseTracker::getDefaultConfig();
  dvo_ros::util::updateConfigFromDynamicReconfigure(dynreconfg_cfg, cfg);

  ROS_WARN_STREAM_NAMED("config", "config: \"" << cfg << "\"");

  // setup tracker
  dvo::DenseTracker dense_tracker(intrinsics, cfg);

  // initialize first pose
  Eigen::Affine3d trajectory, relative;

  if(groundtruth_reader_ != 0)
  {
    dvo_benchmark::findClosestEntry(*groundtruth_reader_, rgbdpair_reader_->entry().RgbTimestamp());
    dvo_benchmark::toPoseEigen(groundtruth_reader_->entry(), trajectory);
  }
  else
  {
    trajectory.setIdentity();
  }

  std::string folder = cfg_.RgbdPairFile.substr(0, cfg_.RgbdPairFile.find_last_of("/") + 1);

  std::vector<dvo_benchmark::RgbdPair> pairs;
  rgbdpair_reader_->readAllEntries(pairs);

  dvo::core::RgbdImagePyramid::Ptr reference, current;

  for(std::vector<dvo_benchmark::RgbdPair>::iterator it = pairs.begin(); ros::ok() && it != pairs.end(); ++it)
  {
    reference = current;
    current = load(folder + it->RgbFile(), folder + it->DepthFile());

    if(!reference)
    {
      createReferenceCamera(visualizer, current->level(0), intrinsics, trajectory);
    }

    // pause in the beginning
    renderWhileSwitchAndNotTerminated(visualizer, pause_switch);
    processInput(visualizer);

    if(!reference)
    {
      continue;
    }

    if((it->RgbTimestamp() - pairs.front().RgbTimestamp()).toSec() < 5 || (pairs.back().RgbTimestamp() - it->RgbTimestamp()).toSec() < 5)
    {
      visualizer->camera("reference")->show();
    }
    else
    {
      visualizer->camera("reference")->hide();
    }

    if(cfg_.ShowGroundtruth)
    {
      Eigen::Affine3d groundtruth_pose;

      dvo_benchmark::findClosestEntry(*groundtruth_reader_, it->RgbTimestamp());
      dvo_benchmark::toPoseEigen(groundtruth_reader_->entry(), groundtruth_pose);

      visualizer->trajectory("groundtruth")->
          color(dvo::visualization::Color::green()).
          add(groundtruth_pose);

      visualizer->camera("groundtruth")->
          color(dvo::visualization::Color::green()).
          update(current->level(0), intrinsics, groundtruth_pose).
          show(cfg_.ShowEstimate ? dvo::visualization::CameraVisualizer::ShowCamera : dvo::visualization::CameraVisualizer::ShowCameraAndCloud);
    }

    if(cfg_.EstimateRequired())
    {
      static dvo::util::stopwatch sw_match("match", 100);
      sw_match.start();
      {
        dense_tracker.match(*reference, *current, relative);
      }
      sw_match.stopAndPrint();

      trajectory = trajectory * relative;

      if(cfg_.EstimateTrajectory)
      {
        Eigen::Quaterniond q(trajectory.rotation());

        (*trajectory_out_)
            << it->RgbTimestamp() << " "
            << trajectory.translation()(0) << " "
            << trajectory.translation()(1) << " "
            << trajectory.translation()(2) << " "
            << q.x() << " "
            << q.y() << " "
            << q.z() << " "
            << q.w() << " "
            << std::endl;
      }

      if(cfg_.ShowEstimate)
      {
        visualizer->trajectory("estimate")->
            color(dvo::visualization::Color::red()).
            add(trajectory);

        visualizer->camera("estimate")->
            color(dvo::visualization::Color::red()).
            update(current->level(0), intrinsics, trajectory).
            show(dvo::visualization::CameraVisualizer::ShowCameraAndCloud);
      }
    }

    if(cfg_.RenderVideo)
    {
      ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->render(5);
      ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->visualizer().saveScreenshot(frame_ids.next() + std::string(".png"));
    }
  }

  // keep visualization alive
  renderWhileSwitchAndNotTerminated(visualizer, dummy_switch);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "benchmark", ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  BenchmarkNode benchmark(nh, nh_private);

  if(benchmark.configure())
  {
    benchmark.run();
  }

  return 0;
}
