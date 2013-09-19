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

#include <dvo_slam/constraints/constraint_proposal_voter.h>

#include <sophus/se3.hpp>

namespace dvo_slam
{
namespace constraints
{

CrossValidationVoter::CrossValidationVoter(double threshold) : TranslationThreshold(threshold) {}
CrossValidationVoter::CrossValidationVoter(const CrossValidationVoter& other) : TranslationThreshold(other.TranslationThreshold) {}
CrossValidationVoter::~CrossValidationVoter() {}

void CrossValidationVoter::createAdditionalProposals(ConstraintProposalVector& proposals)
{
  // create cross validation proposals
  size_t old_size = proposals.size();

  for(size_t idx = 0; idx < old_size; ++idx)
  {
    ConstraintProposalPtr proposal = proposals[idx], other = proposal->createInverseProposal();

    proposals.push_back(other);
    pairs_.push_back(std::make_pair(proposal.get(), other.get()));
  }
}

void CrossValidationVoter::removeAdditionalProposals(ConstraintProposalVector& proposals)
{
  for(ProposalPairVector::iterator it = pairs_.begin(); it != pairs_.end(); ++it)
  {
    ConstraintProposal *worse = it->first->TotalScore() >= it->second->TotalScore() && it->first->Accept() ? it->second : it->first;

    for(ConstraintProposalVector::iterator proposal_it = proposals.begin(); proposal_it != proposals.end(); ++proposal_it)
    {
      if(proposal_it->get() == worse)
      {
        proposals.erase(proposal_it);
        break;
      }
    }
  }

  pairs_.clear();
}

ConstraintProposal::Vote CrossValidationVoter::vote(const ConstraintProposal& proposal, bool provide_reason)
{
  ConstraintProposal *inverse = findInverse(&proposal);

  assert(inverse != 0);

  Sophus::SE3d diff((inverse->TrackingResult.Transformation * proposal.TrackingResult.Transformation).matrix());
  double diff_translation_norm = diff.translation().lpNorm<2>();

  ConstraintProposal::Vote v;
  v.Decision = diff_translation_norm <= TranslationThreshold ? ConstraintProposal::Vote::Accept : ConstraintProposal::Vote::Reject;

  if(provide_reason)
  {
    std::stringstream reason;
    reason << "CrossValidation " << diff_translation_norm << " <= " << TranslationThreshold;

    v.Reason = reason.str();
  }

  return v;
}

ConstraintProposal* CrossValidationVoter::findInverse(const ConstraintProposal *proposal)
{
  for(ProposalPairVector::iterator it = pairs_.begin(); it != pairs_.end(); ++it)
  {
    if(it->first == proposal) return it->second;
    if(it->second == proposal) return it->first;
  }

  return static_cast<ConstraintProposal*>(0);
}

TrackingResultEvaluationVoter::TrackingResultEvaluationVoter(double threshold) : RatioThreshold(threshold) {}
TrackingResultEvaluationVoter::~TrackingResultEvaluationVoter() {}

ConstraintProposal::Vote TrackingResultEvaluationVoter::vote(const ConstraintProposal& proposal, bool provide_reason)
{
  double ratio = proposal.Reference->evaluation()->ratioWithAverage(proposal.TrackingResult);

  ConstraintProposal::Vote v;
  v.Decision = ratio >= RatioThreshold ? ConstraintProposal::Vote::Accept : ConstraintProposal::Vote::Reject;
  v.Score = ratio;

  if(provide_reason)
  {
    std::stringstream reason;
    reason << "TrackingResultValidation " << ratio << " >= " << RatioThreshold;

    v.Reason = reason.str();
  }

  return v;
}

ConstraintRatioVoter::ConstraintRatioVoter(double threshold) : RatioThreshold(threshold) {}
ConstraintRatioVoter::~ConstraintRatioVoter() {}

ConstraintProposal::Vote ConstraintRatioVoter::vote(const ConstraintProposal& proposal, bool provide_reason)
{
  const dvo::DenseTracker::LevelStats &l = proposal.TrackingResult.Statistics.Levels.back();
  double ratio = l.HasIterationWithIncrement() ? double(l.LastIterationWithIncrement().ValidConstraints) / double(l.ValidPixels) : 0.0;

  ConstraintProposal::Vote v;
  v.Decision = ratio >= RatioThreshold ? ConstraintProposal::Vote::Accept : ConstraintProposal::Vote::Reject;

  if(provide_reason)
  {
    std::stringstream reason;
    reason << "ConstraintRatio " << ratio << " >= " << RatioThreshold;

    v.Reason = reason.str();
  }

  return v;
}

NaNResultVoter::NaNResultVoter() {}
NaNResultVoter::~NaNResultVoter() {}

ConstraintProposal::Vote NaNResultVoter::vote(const ConstraintProposal& proposal, bool provide_reason)
{
  ConstraintProposal::Vote v;
  v.Decision = proposal.TrackingResult.isNaN() ? ConstraintProposal::Vote::Reject : ConstraintProposal::Vote::Accept;

  if(provide_reason)
  {
    std::stringstream reason;
    reason << "NaNResult " << proposal.TrackingResult.isNaN();

    v.Reason = reason.str();
  }

  return v;
}

OdometryConstraintVoter::OdometryConstraintVoter() {}
OdometryConstraintVoter::~OdometryConstraintVoter() {}

ConstraintProposal::Vote OdometryConstraintVoter::vote(const ConstraintProposal& proposal, bool provide_reason)
{
  bool is_odometry_constraint = std::abs(proposal.Reference->id() - proposal.Current->id()) <= 1;

  ConstraintProposal::Vote v;
  v.Decision = is_odometry_constraint ? ConstraintProposal::Vote::Reject : ConstraintProposal::Vote::Accept;

  if(provide_reason)
  {
    std::stringstream reason;
    reason << "OdometryConstraint " << is_odometry_constraint;

    v.Reason = reason.str();
  }

  return v;
}

} /* namespace constraints */
} /* namespace dvo_slam */
