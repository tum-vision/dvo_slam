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

#ifndef CONFIG_H_
#define CONFIG_H_

#include <ostream>
#include <dvo_slam/KeyframeSlamConfig.h>

namespace dvo_slam
{

struct KeyframeTrackerConfig
{
  bool UseMultiThreading;

  double MaxTranslationalDistance;
  double MaxRotationalDistance;
  double MinEntropyRatio;
  double MinEquationSystemConstraintRatio;

  KeyframeTrackerConfig();
};

struct KeyframeGraphConfig
{
  bool UseRobustKernel;
  bool UseMultiThreading;

  double NewConstraintSearchRadius;
  double NewConstraintMinEntropyRatioCoarse;
  double NewConstraintMinEntropyRatioFine;
  double MinEquationSystemConstraintRatio;

  bool OptimizationUseDenseGraph;
  bool OptimizationRemoveOutliers;
  double OptimizationOutlierWeightThreshold;
  size_t OptimizationIterations;

  bool FinalOptimizationUseDenseGraph;
  bool FinalOptimizationRemoveOutliers;
  double FinalOptimizationOutlierWeightThreshold;
  size_t FinalOptimizationIterations;

  size_t MinConstraintDistance;

  KeyframeGraphConfig();
};

void updateConfigFromDynamicReconfigure(const KeyframeSlamConfig& cfg, KeyframeTrackerConfig& frontend_cfg, KeyframeGraphConfig& backend_cfg);

} /* namespace dvo_slam */

template<typename _CharT, typename _Traits>
std::basic_ostream<_CharT, _Traits>& operator<<(std::basic_ostream<_CharT, _Traits>& out, const dvo_slam::KeyframeTrackerConfig& cfg)
{
  out
    << "UseMultiThreading: " << cfg.UseMultiThreading << " "
    << "MaxTranslationalDistance: " << cfg.MaxTranslationalDistance << " "
    << "MaxRotationalDistance: " << cfg.MaxRotationalDistance << " "
    << "MinEntropyRatio: " << cfg.MinEntropyRatio << " "
    << "MinEquationSystemConstraintRatio: " << cfg.MinEquationSystemConstraintRatio;

  return out;
}

template<typename _CharT, typename _Traits>
std::basic_ostream<_CharT, _Traits>& operator<<(std::basic_ostream<_CharT, _Traits>& out, const dvo_slam::KeyframeGraphConfig& cfg)
{
  out
    << "UseRobustKernel: " << cfg.UseRobustKernel << " "
    << "UseMultiThreading: " << cfg.UseMultiThreading << " "
    << "NewConstraintSearchRadius: " << cfg.NewConstraintSearchRadius << " "
    << "NewConstraintMinEntropyRatioCoarse: " << cfg.NewConstraintMinEntropyRatioCoarse << " "
    << "NewConstraintMinEntropyRatioFine: " << cfg.NewConstraintMinEntropyRatioFine << " "
    << "MinEquationSystemConstraintRatio: " << cfg.MinEquationSystemConstraintRatio << " "
    << "MinConstraintDistance: " << cfg.MinConstraintDistance << " "
    << "OptimizationUseDenseGraph: " << cfg.OptimizationUseDenseGraph << " "
    << "OptimizationIterations: " << cfg.OptimizationIterations << " "
    << "OptimizationRemoveOutliers: " << cfg.OptimizationRemoveOutliers << " "
    << "OptimizationOutlierWeightThreshold: " << cfg.OptimizationOutlierWeightThreshold << " "
    << "FinalOptimizationUseDenseGraph: " << cfg.FinalOptimizationUseDenseGraph << " "
    << "FinalOptimizationIterations: " << cfg.FinalOptimizationIterations << " "
    << "FinalOptimizationRemoveOutliers: " << cfg.FinalOptimizationRemoveOutliers << " "
    << "FinalOptimizationOutlierWeightThreshold: " << cfg.FinalOptimizationOutlierWeightThreshold;

  return out;
}

#endif /* CONFIG_H_ */
