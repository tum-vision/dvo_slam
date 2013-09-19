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

#include <dvo_slam/constraints/constraint_proposal_validator.h>

#include <boost/bind.hpp>
#include <algorithm>

namespace dvo_slam
{
namespace constraints
{

ConstraintProposalValidator::Stage::Stage(int id) :
  Id(id),
  OnlyKeepBest(false)
{
}

ConstraintProposalValidator::Stage& ConstraintProposalValidator::Stage::keepBest()
{
  OnlyKeepBest = true;
  return *this;
}

ConstraintProposalValidator::Stage& ConstraintProposalValidator::Stage::keepAll()
{
  OnlyKeepBest = false;
  return *this;
}

ConstraintProposalValidator::Stage& ConstraintProposalValidator::Stage::trackingConfig(const dvo::DenseTracker::Config& cfg)
{
  TrackingConfig = cfg;
  return *this;
}

ConstraintProposalValidator::Stage& ConstraintProposalValidator::Stage::addVoter(ConstraintProposalVoter* v)
{
  Voters.push_back(ConstraintProposalVoterPtr(v));
  return *this;
}

ConstraintProposalValidator::ConstraintProposalValidator() {}

ConstraintProposalValidator::Stage& ConstraintProposalValidator::createStage(int id)
{
  stages_.push_back(Stage(id));
  return stages_.back();
}

void ConstraintProposalValidator::validate(ConstraintProposalVector& proposals, bool debug)
{
  for(StageVector::iterator it = stages_.begin(); it != stages_.end(); ++it)
  {
    // reset votes and tracking results
    for(ConstraintProposalVector::iterator proposal_it = proposals.begin(); proposal_it != proposals.end(); ++proposal_it)
    {
      ConstraintProposalPtr& p = *proposal_it;

      p->clearVotes();
      p->TrackingResult.clearStatistics();
    }

    validate(*it, proposals, debug);

    if(debug)
      printVotingResults(*it, proposals);

    // remove rejected proposals
    proposals.erase(std::remove_if(proposals.begin(), proposals.end(), boost::bind(&ConstraintProposal::Reject, _1)), proposals.end());

    // only keep the best result if there are multiple results for one relationship
    if(it->OnlyKeepBest)
      keepBest(proposals);

    // update initial transformations
    for(ConstraintProposalVector::iterator proposal_it = proposals.begin(); proposal_it != proposals.end(); ++proposal_it)
    {
      ConstraintProposalPtr& p = *proposal_it;

      p->InitialTransformation = p->TrackingResult.Transformation.inverse();
    }
  }
}

void ConstraintProposalValidator::keepBest(ConstraintProposalVector& proposals)
{
  for(ConstraintProposalVector::iterator it = proposals.begin(); it != proposals.end(); ++it)
  {
    ConstraintProposalPtr& p = *it;

    for(ConstraintProposalVector::iterator inner_it = it + 1; inner_it != proposals.end();)
    {
      ConstraintProposalPtr& inner_p = *inner_it;

      if(p->isConstraintBetweenSameFrames(*inner_p))
      {
        if(inner_p->TotalScore() > p->TotalScore())
        {
          // copy the better element to the place further in front
          p.swap(inner_p);
        }
        // TODO: maybe this can be reformulated such that there can be a batch erase at the end?!
        inner_it = proposals.erase(inner_it);
      }
      else
      {
        ++inner_it;
      }
    }
  }
}

void ConstraintProposalValidator::validate(Stage& stage, ConstraintProposalVector& proposals, bool debug)
{
  tracker_.configure(stage.TrackingConfig);

  // create additional proposals
  for(ConstraintProposalVoterVector::iterator it = stage.Voters.begin(); it != stage.Voters.end(); ++it)
    (*it)->createAdditionalProposals(proposals);

  // compute tracking result for all proposals
  for(ConstraintProposalVector::iterator it = proposals.begin(); it != proposals.end(); ++it)
  {
    ConstraintProposalPtr& p = (*it);
    p->TrackingResult.Transformation = p->InitialTransformation;
    tracker_.match(*p->Reference->image(), *p->Current->image(), p->TrackingResult);
  }

  // collect votes for all proposals
  for(ConstraintProposalVector::iterator it = proposals.begin(); it != proposals.end(); ++it)
  {
    ConstraintProposal& p = *(*it);

    for(ConstraintProposalVoterVector::iterator voter_it = stage.Voters.begin(); voter_it != stage.Voters.end(); ++voter_it)
    {
      p.Votes.push_back((*voter_it)->vote(p, debug));

      // early abort
      if(p.Votes.back().Decision == ConstraintProposal::Vote::Reject && !debug) break;
    }
  }

  // remove additional proposals
  for(ConstraintProposalVoterVector::reverse_iterator it = stage.Voters.rbegin(); it != stage.Voters.rend(); ++it)
    (*it)->removeAdditionalProposals(proposals);
}

void ConstraintProposalValidator::printVotingResults(const Stage& stage, const ConstraintProposalVector& proposals)
{
  std::cout << "Stage " << stage.Id << ":" << std::endl;

  for(ConstraintProposalVector::const_iterator it = proposals.begin(); it != proposals.end(); ++it)
    (*it)->printVotingResults(std::cout, "  ");
}

} /* namespace constraints */
} /* namespace dvo_slam */
