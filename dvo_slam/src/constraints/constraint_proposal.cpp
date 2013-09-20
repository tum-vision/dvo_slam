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

#include <dvo_slam/constraints/constraint_proposal.h>

#include <boost/make_shared.hpp>

namespace dvo_slam
{
namespace constraints
{

boost::shared_ptr<ConstraintProposal> ConstraintProposal::createWithIdentity(const KeyframePtr& reference, const KeyframePtr& current)
{
  boost::shared_ptr<ConstraintProposal> p(new ConstraintProposal());
  p->Reference = reference;
  p->Current = current;
  p->InitialTransformation.setIdentity();

  return p;
}

boost::shared_ptr<ConstraintProposal> ConstraintProposal::createWithRelative(const KeyframePtr& reference, const KeyframePtr& current)
{
  boost::shared_ptr<ConstraintProposal> p(new ConstraintProposal());
  p->Reference = reference;
  p->Current = current;
  p->InitialTransformation = current->pose().inverse() * reference->pose();

  return p;
}

ConstraintProposal::ConstraintProposal() :
  InitialTransformation(Eigen::Affine3d::Identity())
{
}

double ConstraintProposal::TotalScore() const
{
  double s = 0.0;

  for(VoteVector::const_iterator it = Votes.begin(); it != Votes.end(); ++it)
  {
    s += it->Score;
  }

  return s;
}

bool ConstraintProposal::Accept() const
{
  for(VoteVector::const_iterator it = Votes.begin(); it != Votes.end(); ++it)
    if(it->Decision == Vote::Reject) return false;

  return true;
}

bool ConstraintProposal::Reject() const
{
  for(VoteVector::const_iterator it = Votes.begin(); it != Votes.end(); ++it)
    if(it->Decision == Vote::Reject) return true;

  return false;
}

void ConstraintProposal::clearVotes()
{
  Votes.clear();
}

boost::shared_ptr<ConstraintProposal> ConstraintProposal::createInverseProposal() const
{
  boost::shared_ptr<ConstraintProposal> inv(new ConstraintProposal());
  inv->Reference = Current;
  inv->Current = Reference;
  inv->InitialTransformation = InitialTransformation.inverse();

  return inv;
}

bool ConstraintProposal::isConstraintBetweenSameFrames(const ConstraintProposal& other)
{
  return (Reference->id() == other.Reference->id() && Current->id() == other.Current->id()) || (Reference->id() == other.Current->id() && Current->id() == other.Reference->id());
}

void ConstraintProposal::printVotingResults(std::ostream& out, const std::string indent) const
{
  out << indent << "Proposal " << Reference->id() << "->" << Current->id() << " " << (Accept() ? "accept" : "reject") << std::endl;

  for(ConstraintProposal::VoteVector::const_iterator it = Votes.begin(); it != Votes.end(); ++it)
    out << indent << "  " << it->Reason << std::endl;
}

} /* namespace constraints */
} /* namespace dvo_slam */
