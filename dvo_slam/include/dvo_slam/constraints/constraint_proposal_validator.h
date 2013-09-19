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

#ifndef CONSTRAINT_PROPOSAL_VALIDATOR_H_
#define CONSTRAINT_PROPOSAL_VALIDATOR_H_

#include <dvo_slam/constraints/constraint_proposal.h>
#include <dvo_slam/constraints/constraint_proposal_voter.h>

namespace dvo_slam
{
namespace constraints
{

struct ConstraintProposalValidator
{
public:
  struct Stage
  {
  private:
    friend struct ConstraintProposalValidator;

    int Id;
    bool OnlyKeepBest;
    dvo::DenseTracker::Config TrackingConfig;
    ConstraintProposalVoterVector Voters;

    Stage(int id);
  public:
    Stage& keepBest();

    Stage& keepAll();

    Stage& trackingConfig(const dvo::DenseTracker::Config& cfg);

    Stage& addVoter(ConstraintProposalVoter* v);
  };

  typedef std::vector<Stage> StageVector;

  ConstraintProposalValidator();

  Stage& createStage(int id);

  void validate(ConstraintProposalVector& proposals, bool debug = false);

  void keepBest(ConstraintProposalVector& proposals);

private:
  StageVector stages_;
  dvo::DenseTracker tracker_;

  void validate(Stage& stage, ConstraintProposalVector& proposals, bool debug = false);

  void printVotingResults(const Stage& stage, const ConstraintProposalVector& proposals);
};

typedef boost::shared_ptr<ConstraintProposalValidator> ConstraintProposalValidatorPtr;


} /* namespace constraints */
} /* namespace dvo_slam */
#endif /* CONSTRAINT_PROPOSAL_VALIDATOR_H_ */
