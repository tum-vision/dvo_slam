

#include <opencv2/opencv.hpp>

#include <vector>
#include <fstream>
#include <ros/ros.h>

#include <dvo/core/rgbd_image.h>
#include <dvo/core/surface_pyramid.h>
#include <dvo/visualization/visualizer.h>
#include <dvo/util/id_generator.h>

#include <dvo/dense_tracking.h>
#include <dvo/dense_tracking_impl.h>

#include <dvo_benchmark/file_reader.h>
#include <dvo_benchmark/rgbd_pair.h>
#include <dvo_benchmark/groundtruth.h>
#include <dvo_benchmark/tools.h>

double calculateSampledFrustumMetric(const Eigen::Affine3d& pose, const dvo::core::IntrinsicMatrix& intrinsics)
{
  Eigen::Matrix<double, 9, 2> pixels;
  pixels <<
  50, 50, 50, 240, 50, 430,
  320, 50, 320, 240, 320, 430,
  590, 50, 590, 240, 590, 430;

  Eigen::Matrix<double, 4, 3 * 9> points;
  for (int d = 1; d < 4; ++d)
  {
    for (int idx = 0; idx < 9; ++idx)
    {
      points(0, (d - 1) * 9 + idx) = (pixels(idx, 0) - intrinsics.ox()) / intrinsics.fx() * d; // x
      points(1, (d - 1) * 9 + idx) = (pixels(idx, 1) - intrinsics.oy()) / intrinsics.fy() * d; // y
      points(2, (d - 1) * 9 + idx) = d;   // z
      points(3, (d - 1) * 9 + idx) = 1.0; // w
    }
  }

  Eigen::Matrix<double, 4, 3 * 9> transformed = pose * points;

  double sum = 0.0;

  for (int idx = 0; idx < points.cols(); ++idx)
  {
    double u1 = points(0, idx) / points(3, idx) * intrinsics.fx() + intrinsics.ox();
    double v1 = points(1, idx) / points(3, idx) * intrinsics.fy() + intrinsics.oy();
    double u2 = transformed(0, idx) / transformed(3, idx) * intrinsics.fx() + intrinsics.ox();
    double v2 = transformed(1, idx) / transformed(3, idx) * intrinsics.fy() + intrinsics.oy();
    sum += std::sqrt((u1 - u2) * (u1 - u2) + (v1 - v2) * (v1 - v2));
  }

  sum /= points.cols();

  //Eigen::Matrix<double, 4, 3 * 9> diff = transformed - points;
  // diff.colwise().norm().sum()

  return sum;
}


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
    dvo::core::SurfacePyramid::convertRawDepthImageSse(depth, depth_float, 1.0f / 5000.0f);
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

struct FindNearestPosePredicate
{
  FindNearestPosePredicate(const dvo_benchmark::RgbdPair& entry) :
    entry_(entry)
  {
  }

  bool operator() (const dvo_benchmark::Groundtruth& left, const dvo_benchmark::Groundtruth& right) const
  {
    return left.Timestamp() <= entry_.RgbTimestamp() && right.Timestamp() >= entry_.RgbTimestamp();
  }
private:
  const dvo_benchmark::RgbdPair& entry_;
};

// Jensen-Bregman LogDet Divergence

template <typename Derived>
double JenBreLogDetDiv(const Eigen::MatrixBase<Derived>& a, const Eigen::MatrixBase<Derived>& b)
{
  return std::log(((a + b) / 2.0).determinant()) - 0.5 * std::log((a * b).determinant());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "experiments", ros::init_options::AnonymousName);

  ros::NodeHandle nh("~");

  std::string dataset;
  if(!nh.getParam("dataset", dataset))
  {
    std::cerr << "Missing 'dataset' parameter!" << std::endl;
    return -1;
  }

  std::string assoc = dataset + "/assoc.txt";
  std::string groundtruth = dataset + "/groundtruth.txt";

  std::vector<dvo_benchmark::RgbdPair> pairs;
  std::vector<dvo_benchmark::Groundtruth> poses;

  dvo::visualization::Visualizer::instance()
    .enabled(true)
    .useExternalWaitKey(true)
    .save(true);

  dvo_benchmark::FileReader<dvo_benchmark::RgbdPair> pair_reader(assoc);
  pair_reader.skipComments();
  pair_reader.readAllEntries(pairs);

  dvo_benchmark::FileReader<dvo_benchmark::Groundtruth> groundtruth_reader(groundtruth);
  groundtruth_reader.skipComments();
  groundtruth_reader.readAllEntries(poses);

  dvo::core::RgbdImagePyramid::Ptr reference, current;
  dvo::core::IntrinsicMatrix intrinsics = dvo::core::IntrinsicMatrix::create(525.0f, 525.0f, 320.0f, 240.0f);

  dvo::DenseTracker::Config cfg = dvo::DenseTracker::getDefaultConfig();
  cfg.UseWeighting = false;
  cfg.UseInitialEstimate = true;
  cfg.FirstLevel = 3;
  cfg.LastLevel = 1;
  cfg.MaxIterationsPerLevel = 100;
  dvo::DenseTracker tracker(intrinsics, cfg);

  Eigen::Affine3d reference_pose, current_pose, relative_pose;
  Eigen::Matrix<double, 6, 6> first_info, current_info;
  Eigen::Matrix2d first_error_precision;
  /*
  for (std::vector<dvo_benchmark::RgbdPair>::iterator it = pairs.begin(); it != pairs.end() && ros::ok(); ++it)
  {
    reference = load(dataset + "/" + it->RgbFile(), dataset + "/" + it->DepthFile());

    std::vector<dvo_benchmark::Groundtruth>::iterator nearest = std::adjacent_find(poses.begin(), poses.end(), FindNearestPosePredicate(*it));

    if(nearest == poses.end()) continue;
    dvo_benchmark::toPoseEigen(*nearest, reference_pose);
    relative_pose.setIdentity();
    bool first = true;

    for (std::vector<dvo_benchmark::RgbdPair>::iterator it_inner = pairs.begin(); it_inner != pairs.end() && ros::ok(); ++it_inner)
    {
      std::vector<dvo_benchmark::Groundtruth>::iterator nearest = std::adjacent_find(poses.begin(), poses.end(), FindNearestPosePredicate(*it_inner));

      if(nearest == poses.end()) continue;
      dvo_benchmark::toPoseEigen(*nearest, current_pose);

      current = load(dataset + "/" + it_inner->RgbFile(), dataset + "/" + it_inner->DepthFile());

      relative_pose = relative_pose.inverse();

      tracker.match(*reference, *current, relative_pose);

<<<<<<< HEAD
  for (std::vector<dvo_benchmark::RgbdPair>::iterator it = pairs.begin() + 49; it != pairs.begin() + 52 && ros::ok(); ++it)
=======
      double translation_dist = (current_pose.inverse() * reference_pose).translation().norm();
      double translation_error = ((current_pose.inverse() * reference_pose) * relative_pose).translation().norm();
      double error_entropy = std::log(tracker.itctx_.Precision.determinant());


      std::cout << translation_dist << " " << translation_error << " " << error_entropy << std::endl;
    }
    std::cerr << (pairs.end() - it) << " ";

  }
  */
  /*
  for (std::vector<dvo_benchmark::RgbdPair>::iterator it = pairs.begin(); it != pairs.end() && ros::ok(); ++it)
  {
    reference = current;
    current = load(dataset + "/" + it->RgbFile(), dataset + "/" + it->DepthFile());

    if(!reference) continue;

    relative_pose.setIdentity();

    tracker.match(*reference, *current, relative_pose);


    std::cerr << tracker.itctx_.NumConstraints << " " << tracker.itctx_.Mean(0) << " " << tracker.itctx_.Mean(1) << " " << tracker.itctx_.Precision(0, 0) << " " << tracker.itctx_.Precision(1, 1) << std::endl;
  }
  */

  for (std::vector<dvo_benchmark::RgbdPair>::iterator it = pairs.begin(); it != pairs.begin() +1 && ros::ok(); ++it)
>>>>>>> b03cf72f26093998d182440c467fe0e5ff327d1c
  {
    reference = load(dataset + "/" + it->RgbFile(), dataset + "/" + it->DepthFile());

    std::vector<dvo_benchmark::Groundtruth>::iterator nearest = std::adjacent_find(poses.begin(), poses.end(), FindNearestPosePredicate(*it));

    if(nearest == poses.end()) continue;
    dvo_benchmark::toPoseEigen(*nearest, reference_pose);
    relative_pose.setIdentity();
    bool first = true;
    int k = 0;
<<<<<<< HEAD

    for (std::vector<dvo_benchmark::RgbdPair>::iterator it_inner = pairs.begin(); it_inner != pairs.end() && ros::ok(); ++it_inner, ++k)
=======
    for (std::vector<dvo_benchmark::RgbdPair>::iterator it_inner = it + 1; it_inner != pairs.end() && ros::ok(); ++it_inner, ++k)
>>>>>>> b03cf72f26093998d182440c467fe0e5ff327d1c
    {
      std::vector<dvo_benchmark::Groundtruth>::iterator nearest = std::adjacent_find(poses.begin(), poses.end(), FindNearestPosePredicate(*it_inner));

      if(nearest == poses.end()) continue;
      dvo_benchmark::toPoseEigen(*nearest, current_pose);

      current = load(dataset + "/" + it_inner->RgbFile(), dataset + "/" + it_inner->DepthFile());

      relative_pose.setIdentity();
      tracker.match(*reference, *current, relative_pose);

      if(first)
      {
        tracker.getInformationEstimate(first_info);
        first_error_precision = tracker.itctx_.Precision;
        first = false;
      }
      tracker.getInformationEstimate(current_info);

      double pose_entropy = std::log(current_info.determinant());
      double loglikelihood = tracker.itctx_.Error;
      double err_i = tracker.itctx_.Mean(0);
      double err_z = tracker.itctx_.Mean(1);
      double translation_error = ((current_pose.inverse() * reference_pose) * relative_pose).translation().norm();
      double translation_dist = (current_pose.inverse() * reference_pose).translation().norm();

      std::cout
          << k << " "
          << pose_entropy << " "
          //<< loglikelihood << " "
          //<< err_i << " "
          //<< err_z << " "
          << translation_error << " "
          << translation_dist << " "
          << std::endl;
    }
    std::cerr << (pairs.begin() + 52 - it) << " ";

  }

    /*
    for (std::vector<dvo_benchmark::RgbdPair>::iterator it_inner = it + 1; it_inner != it + 100 && ros::ok(); ++it_inner, ++k)
    {
      std::vector<dvo_benchmark::Groundtruth>::iterator nearest = std::adjacent_find(poses.begin(), poses.end(), FindNearestPosePredicate(*it_inner));

      if(nearest == poses.end()) continue;
      dvo_benchmark::toPoseEigen(*nearest, current_pose);

      current = load(dataset + "/" + it_inner->RgbFile(), dataset + "/" + it_inner->DepthFile());

      cv::Scalar cur_avg_depth = cv::mean(current->level(0).depth, current->level(0).depth == current->level(0).depth);

      Eigen::Affine3d a1;
      a1.setIdentity();

      Eigen::Matrix<double, 6, 6> fisher;

      tracker.match(*last, *current, a1);
      tracker.getInformationEstimate(fisher);

      current_se3 = Sophus::SE3(a1.rotation(), a1.translation());
      accumulated_odom = accumulated_odom * a1;
      accumulated_se3 = Sophus::SE3(accumulated_odom.rotation(), accumulated_odom.translation());

      propagated_cov = current_se3.Adj() * propagated_cov * current_se3.Adj().transpose() + fisher.inverse();

      relative_pose = relative_pose.inverse();
      tracker.match(*reference, *current, relative_pose);

      if(first)
      {
        tracker.getInformationEstimate(first_info);
        first_error_precision = tracker.itctx_.Precision;
        first = false;
      }
      tracker.getInformationEstimate(current_info);
      keyframe_cov = current_info.inverse();

      current_se3 = Sophus::SE3(relative_pose.rotation(), relative_pose.translation());

      Eigen::Matrix<double, 6, 1> diff = current_se3.log() - accumulated_se3.log();

      double kl_divergence = 0.5 * ((keyframe_cov.inverse() * propagated_cov).trace() + diff.transpose() * keyframe_cov * diff - 2 - std::log(propagated_cov.determinant() / keyframe_cov.determinant()));
      double constraints = tracker.itctx_.NumConstraints;
      double pose_entropy = std::log(current_info.determinant());
      double pose_normalized_entropy = std::log((current_info / constraints).determinant());
      double error_entropy = std::log(tracker.itctx_.Precision.determinant());

<<<<<<< HEAD
      double translation_dist = relative_pose.translation().norm();
      double true_translation_dist = (current_pose.inverse() * reference_pose).translation().norm();
      double translation_error = ((current_pose.inverse() * reference_pose) * relative_pose).translation().norm();


      std::cout
          << ref_avg_depth(0) << " "
          << cur_avg_depth(0) << " "
          << translation_dist << " "
          << translation_error << " "
          << constraints << " "
          << pose_entropy << " "
          << pose_normalized_entropy << " "
          << error_entropy << " "
          << true_translation_dist << " "
          << kl_divergence << " "
          << std::endl;

      last = current;
=======
      //double param_entropy_ratio = std::log(current_info.determinant()) / std::log(first_info.determinant());
      double error_entropy_ratio = std::log(tracker.itctx_.Precision.determinant());// / std::log(first_error_precision.determinant());
      //double param_divergence = JenBreLogDetDiv(first_info, current_info);
      //double error_divergence = JenBreLogDetDiv(first_error_precision, tracker.itctx_.Precision);
      double translation_dist = (current_pose.inverse() * reference_pose).translation().norm();
      double translation_error = ((current_pose.inverse() * reference_pose) * relative_pose).translation().norm();
      double frustum_metric = calculateSampledFrustumMetric(relative_pose, intrinsics);
      //std::cerr << (current_pose.inverse() * reference_pose).matrix() << std::endl << std::endl;
      //std::cerr << relative_pose.matrix() << std::endl << std::endl;
      //std::cerr << ((current_pose.inverse() * reference_pose) * relative_pose).matrix() << std::endl << std::endl;
      //
      //std::string tmp;
      //std::cin >> tmp;

      //std::cout << k << " " << translation_dist << " " << translation_error << " " << param_entropy_ratio << " " << param_divergence << " " << error_entropy_ratio << " " << error_divergence << std::endl;
      std::cout << k << " " << translation_dist << " " << translation_error << " " << error_entropy_ratio << " " << tracker.itctx_.Mean(0) << " " << tracker.itctx_.Mean(1) << " " << frustum_metric << std::endl;
>>>>>>> b03cf72f26093998d182440c467fe0e5ff327d1c
    }
    std::cerr << (pairs.end() - 100 - it) << " ";

  }
<<<<<<< HEAD
  */
=======

>>>>>>> b03cf72f26093998d182440c467fe0e5ff327d1c
  std::cerr << std::endl;
  //while(true)
  //  cv::waitKey(10);
}
