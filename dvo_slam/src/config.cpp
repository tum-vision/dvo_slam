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

#include <limits>
#include <dvo_slam/config.h>

namespace dvo_slam
{

KeyframeTrackerConfig::KeyframeTrackerConfig() :
  UseMultiThreading(true),
  MaxTranslationalDistance(0.2),
  MaxRotationalDistance(std::numeric_limits<double>::max()),
  MinEntropyRatio(0.91),
  MinEquationSystemConstraintRatio(0.33)
{
}

KeyframeGraphConfig::KeyframeGraphConfig() :
    UseRobustKernel(true),
    UseMultiThreading(true),
    NewConstraintSearchRadius(1.0),
    NewConstraintMinEntropyRatioCoarse(0.7),
    NewConstraintMinEntropyRatioFine(0.9),
    MinEquationSystemConstraintRatio(0.2),
    MinConstraintDistance(0),
    OptimizationUseDenseGraph(false),
    OptimizationIterations(20),
    OptimizationRemoveOutliers(false),
    OptimizationOutlierWeightThreshold(0.0),
    FinalOptimizationUseDenseGraph(true),
    FinalOptimizationIterations(5000),
    FinalOptimizationRemoveOutliers(false),
    FinalOptimizationOutlierWeightThreshold(0.0)
{
}

void updateConfigFromDynamicReconfigure(const KeyframeSlamConfig& cfg, KeyframeTrackerConfig& frontend_cfg, KeyframeGraphConfig& backend_cfg)
{
  frontend_cfg.MaxTranslationalDistance = cfg.max_translational_distance;
  frontend_cfg.MaxRotationalDistance = cfg.max_rotational_distance;
  frontend_cfg.MinEntropyRatio = cfg.min_entropy_ratio;
  frontend_cfg.MinEquationSystemConstraintRatio = cfg.min_eq_sys_constraint_ratio;
  frontend_cfg.UseMultiThreading = cfg.use_multithreading;

  backend_cfg.MinConstraintDistance = cfg.graph_opt_min_distance;
  backend_cfg.OptimizationIterations = cfg.graph_opt_iterations;
  backend_cfg.OptimizationUseDenseGraph = cfg.graph_opt_dense_graph;
  backend_cfg.OptimizationRemoveOutliers = cfg.graph_opt_remove_outliers;
  backend_cfg.OptimizationOutlierWeightThreshold = cfg.graph_opt_outlier_weight_threshold;
  backend_cfg.FinalOptimizationRemoveOutliers = cfg.graph_opt_final_remove_outliers;
  backend_cfg.FinalOptimizationOutlierWeightThreshold = cfg.graph_opt_final_outlier_weight_threshold;
  backend_cfg.FinalOptimizationIterations = cfg.graph_opt_final_iterations;
  backend_cfg.FinalOptimizationUseDenseGraph = cfg.graph_opt_final_dense_graph;
  backend_cfg.NewConstraintSearchRadius = cfg.constraint_search_radius;
  backend_cfg.NewConstraintMinEntropyRatioCoarse = cfg.constraint_min_entropy_ratio_coarse;
  backend_cfg.NewConstraintMinEntropyRatioFine = cfg.constraint_min_entropy_ratio_fine;
  backend_cfg.UseRobustKernel = cfg.graph_opt_robust;
  backend_cfg.MinEquationSystemConstraintRatio = cfg.constraint_min_eq_sys_constraint_ratio;
  backend_cfg.UseMultiThreading = cfg.use_multithreading;
}
} /* namespace dvo_slam */

