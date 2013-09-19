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

#ifndef CONSTRAINT_PROPOSAL_VOTER_H_
#define CONSTRAINT_PROPOSAL_VOTER_H_

#include <dvo_slam/constraints/constraint_proposal.h>

namespace dvo_slam
{
namespace constraints
{

struct ConstraintProposalVoter
{
  virtual ~ConstraintProposalVoter() {};

  /**
   * These methods allow voters to get tracking results for additional proposals, which they might need for their
   * decision.
   */
  virtual void createAdditionalProposals(ConstraintProposalVector& proposals) {}
  virtual void removeAdditionalProposals(ConstraintProposalVector& proposals) {}

  /**
   * Vote for the proposal. Has to set the decision.
   * If asked for reason provide detailed explanation of decision.
   */
  virtual ConstraintProposal::Vote vote(const ConstraintProposal& proposal, bool provide_reason) = 0;
};

typedef boost::shared_ptr<ConstraintProposalVoter> ConstraintProposalVoterPtr;
typedef std::vector<ConstraintProposalVoterPtr> ConstraintProposalVoterVector;

struct CrossValidationVoter : public ConstraintProposalVoter
{
  double TranslationThreshold;

  CrossValidationVoter(double threshold);
  CrossValidationVoter(const CrossValidationVoter& other);
  virtual ~CrossValidationVoter();

  virtual void createAdditionalProposals(ConstraintProposalVector& proposals);

  virtual void removeAdditionalProposals(ConstraintProposalVector& proposals);

  virtual ConstraintProposal::Vote vote(const ConstraintProposal& proposal, bool provide_reason);
private:
  typedef std::vector<std::pair<ConstraintProposal*, ConstraintProposal*> > ProposalPairVector;
  ProposalPairVector pairs_;

  ConstraintProposal* findInverse(const ConstraintProposal *proposal);
};

struct TrackingResultEvaluationVoter : public ConstraintProposalVoter
{
  double RatioThreshold;

  TrackingResultEvaluationVoter(double threshold);
  virtual ~TrackingResultEvaluationVoter();

  virtual ConstraintProposal::Vote vote(const ConstraintProposal& proposal, bool provide_reason);
};

struct ConstraintRatioVoter : public ConstraintProposalVoter
{
  double RatioThreshold;

  ConstraintRatioVoter(double threshold);
  virtual ~ConstraintRatioVoter();

  virtual ConstraintProposal::Vote vote(const ConstraintProposal& proposal, bool provide_reason);
};

struct NaNResultVoter : public ConstraintProposalVoter
{
  NaNResultVoter();
  virtual ~NaNResultVoter();

  virtual ConstraintProposal::Vote vote(const ConstraintProposal& proposal, bool provide_reason);
};

struct OdometryConstraintVoter : public ConstraintProposalVoter
{
  OdometryConstraintVoter();
  virtual ~OdometryConstraintVoter();

  virtual ConstraintProposal::Vote vote(const ConstraintProposal& proposal, bool provide_reason);
};

} /* namespace constraints */
} /* namespace dvo_slam */
#endif /* CONSTRAINT_PROPOSAL_VOTER_H_ */
