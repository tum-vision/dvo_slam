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

#ifndef CONSTRAINT_PROPOSAL_H_
#define CONSTRAINT_PROPOSAL_H_

#include <vector>
#include <boost/shared_ptr.hpp>

#include <dvo_slam/keyframe.h>

namespace dvo_slam
{
namespace constraints
{

struct ConstraintProposal
{
  struct Vote
  {
    enum Enum
    {
      Accept,
      Reject
    };

    // hard decision
    Enum Decision;

    // score to select better proposal
    double Score;

    // details for the decision
    std::string Reason;

    Vote() : Decision(Reject), Score(0.0) {}
  };
  typedef std::vector<Vote> VoteVector;

  static boost::shared_ptr<ConstraintProposal> createWithIdentity(const KeyframePtr& reference, const KeyframePtr& current);

  static boost::shared_ptr<ConstraintProposal> createWithRelative(const KeyframePtr& reference, const KeyframePtr& current);

  KeyframePtr Reference, Current;
  Eigen::Affine3d InitialTransformation;
  dvo::DenseTracker::Result TrackingResult;
  VoteVector Votes;

  ConstraintProposal();

  double TotalScore() const;

  bool Accept() const;

  bool Reject() const;

  void clearVotes();

  boost::shared_ptr<ConstraintProposal> createInverseProposal() const;

  bool isConstraintBetweenSameFrames(const ConstraintProposal& other);

  void printVotingResults(std::ostream& out, const std::string indent = "") const;
};

typedef boost::shared_ptr<ConstraintProposal> ConstraintProposalPtr;
typedef std::vector<ConstraintProposalPtr> ConstraintProposalVector;

} /* namespace constraints */
} /* namespace dvo_slam */
#endif /* CONSTRAINT_PROPOSAL_H_ */
