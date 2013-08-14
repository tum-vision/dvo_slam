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

#ifndef DENSE_TRACKER_H_
#define DENSE_TRACKER_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dvo/core/datatypes.h>
#include <dvo/core/intrinsic_matrix.h>
#include <dvo/core/rgbd_image.h>
#include <dvo/core/point_selection.h>
#include <dvo/core/point_selection_predicates.h>
#include <dvo/core/least_squares.h>
#include <dvo/core/weight_calculation.h>

namespace dvo
{

class DenseTracker
{
public:
  struct Config
  {
    int FirstLevel, LastLevel;
    int MaxIterationsPerLevel;
    double Precision;
    double Lambda; // weighting factor for temporal smoothing
    double Mu;     // weighting factor for driving solution close to imu estimate

    bool UseInitialEstimate;
    bool UseWeighting;

    bool UseParallel;

    dvo::core::InfluenceFunctions::enum_t InfluenceFuntionType;
    float InfluenceFunctionParam;

    dvo::core::ScaleEstimators::enum_t ScaleEstimatorType;
    float ScaleEstimatorParam;

    float IntensityDerivativeThreshold;
    float DepthDerivativeThreshold;

    Config();
    size_t getNumLevels() const;

    bool UseTemporalSmoothing() const;

    bool UseEstimateSmoothing() const;

    bool IsSane() const;
  };

  struct IterationContext
  {
    const Config& cfg;

    int Level;
    int Iteration;

    size_t NumConstraints;
    size_t MaxNumConstraints;

    double Error, LastError;
    Eigen::Vector2d Mean;
    Eigen::Matrix2d Precision;

    IterationContext(const Config& cfg);

    // returns true if this is the first iteration
    bool IsFirstIteration() const;

    // returns true if this is the first iteration on the current level
    bool IsFirstIterationOnLevel() const;

    // returns true if this is the first level
    bool IsFirstLevel() const;

    // returns true if this is the last level
    bool IsLastLevel() const;

    bool IterationsExceeded() const;

    // returns LastError - Error
    double ErrorDiff() const;

    double ConstraintRatio() const;
  };

  struct TerminationCriteria
  {
    enum Enum
    {
      IterationsExceeded,
      IncrementTooSmall,
      LogLikelihoodDecreased,
      NumCriteria
    };
  };

  struct IterationStats
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    size_t Id, ValidConstraints;

    double TDistributionLogLikelihood;
    Eigen::Vector2d TDistributionMean;
    Eigen::Matrix2d TDistributionPrecision;

    double PriorLogLikelihood;

    dvo::core::Vector6d EstimateIncrement;
    dvo::core::Matrix6d EstimateInformation;
  };
  typedef std::vector<IterationStats, Eigen::aligned_allocator<IterationStats> > IterationStatsVector;

  struct LevelStats
  {
    size_t Id, MaxValidPixels, ValidPixels;
    TerminationCriteria::Enum TerminationCriterion;
    IterationStatsVector Iterations;
  };
  typedef std::vector<LevelStats> LevelStatsVector;

  struct Stats
  {
    LevelStatsVector Levels;
  };

  struct Result
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    dvo::core::AffineTransformd Transformation;
    dvo::core::Matrix6d Information;
    double LogLikelihood;
  };

  static const Config& getDefaultConfig();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DenseTracker(const Config& cfg = getDefaultConfig());
  DenseTracker(const dvo::DenseTracker& other);

  const Config& configuration() const
  {
    return cfg;
  }

  void configure(const Config& cfg);

  bool match(dvo::core::RgbdImagePyramid& reference, dvo::core::RgbdImagePyramid& current, dvo::core::AffineTransformd& transformation);
  bool match(dvo::core::PointSelection& reference, dvo::core::RgbdImagePyramid& current, dvo::core::AffineTransformd& transformation);

  bool match(dvo::core::RgbdImagePyramid& reference, dvo::core::RgbdImagePyramid& current, dvo::DenseTracker::Result& result, dvo::DenseTracker::Stats& stats);
  bool match(dvo::core::PointSelection& reference, dvo::core::RgbdImagePyramid& current, dvo::DenseTracker::Result& result, dvo::DenseTracker::Stats& stats);

  // TODO: remove
  void updateLastTransform(Eigen::Affine3d& last_transformation);

  // TODO: remove
  void getInformationEstimate(dvo::core::Matrix6d& information) const;

  static inline void computeJacobianOfProjectionAndTransformation(const dvo::core::Vector4& p, dvo::core::Matrix2x6& jacobian);

  static inline void compute3rdRowOfJacobianOfTransformation(const dvo::core::Vector4& p, dvo::core::Vector6& j);

  IterationContext itctx_;
  typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > ResidualVectorType;
  typedef std::vector<float> WeightVectorType;
private:
  Config cfg;

  dvo::core::WeightCalculation weight_calculation_;
  dvo::core::PointSelection reference_selection_;
  dvo::core::ValidPointAndGradientThresholdPredicate selection_predicate_;

  dvo::core::Matrix6d last_a_;

  dvo::core::PointWithIntensityAndDepth::VectorType points, points_error;

  ResidualVectorType residuals;
  WeightVectorType weights;
};

} /* namespace dvo */

template<typename CharT, typename Traits>
std::ostream& operator<< (std::basic_ostream<CharT, Traits> &out, const dvo::DenseTracker::Config &config)
{
  out
  << "First Level = " << config.FirstLevel
  << ", Last Level = " << config.LastLevel
  << ", Max Iterations per Level = " << config.MaxIterationsPerLevel
  << ", Precision = " << config.Precision
  << ", Lambda = " << config.Lambda
  << ", Mu = " << config.Mu
  << ", Use Initial Estimate = " << (config.UseInitialEstimate ? "true" : "false")
  << ", Use Weighting = " << (config.UseWeighting ? "true" : "false")
  << ", Scale Estimator = " << dvo::core::ScaleEstimators::str(config.ScaleEstimatorType)
  << ", Scale Estimator Param = " << config.ScaleEstimatorParam
  << ", Influence Function = " << dvo::core::InfluenceFunctions::str(config.InfluenceFuntionType)
  << ", Influence Function Param = " << config.InfluenceFunctionParam
  << ", Intensity Derivative Threshold = " << config.IntensityDerivativeThreshold
  << ", Depth Derivative Threshold = " << config.DepthDerivativeThreshold
  ;

  return out;
}

template<typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>& operator<< (std::basic_ostream<CharT, Traits> &out, const dvo::DenseTracker::IterationContext &ctx)
{
  out
  << "Level: " << ctx.Level
  << ", Iteration: " << ctx.Iteration
  << ", LastError: " << ctx.LastError
  << ", Error: " << ctx.Error
  << ", ErrorDiff: " << ctx.ErrorDiff()
  ;

  return out;
}

#endif /* DENSE_TRACKER_H_ */
