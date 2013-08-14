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

#ifndef TRACKING_RESULT_EVALUATION_H_
#define TRACKING_RESULT_EVALUATION_H_

#include <boost/shared_ptr.hpp>

#include <dvo/dense_tracking.h>

namespace dvo_slam
{

class TrackingResultEvaluation
{
public:
  typedef boost::shared_ptr<TrackingResultEvaluation> Ptr;
  typedef boost::shared_ptr<const TrackingResultEvaluation> ConstPtr;

  virtual ~TrackingResultEvaluation() {};

  virtual void add(const dvo::DenseTracker::Result& r);

  virtual double ratioWithFirst(const dvo::DenseTracker::Result& r) const;

  virtual double ratioWithAverage(const dvo::DenseTracker::Result& r) const;

protected:
  TrackingResultEvaluation(double first);

  virtual double value(const dvo::DenseTracker::Result& r) const = 0;
private:
  double first_, average_, n_;
};

class LogLikelihoodTrackingResultEvaluation : public TrackingResultEvaluation
{
public:
  LogLikelihoodTrackingResultEvaluation(const dvo::DenseTracker::Result& r) : TrackingResultEvaluation(value(r)) {};
  virtual ~LogLikelihoodTrackingResultEvaluation() {};
  virtual double value(const dvo::DenseTracker::Result& r) const;
};

class NormalizedLogLikelihoodTrackingResultEvaluation : public TrackingResultEvaluation
{
public:
  NormalizedLogLikelihoodTrackingResultEvaluation(const dvo::DenseTracker::Result& r) : TrackingResultEvaluation(value(r)) {};
  virtual ~NormalizedLogLikelihoodTrackingResultEvaluation() {};
  virtual double value(const dvo::DenseTracker::Result& r) const;
};


class EntropyRatioTrackingResultEvaluation : public TrackingResultEvaluation
{
public:
  EntropyRatioTrackingResultEvaluation(const dvo::DenseTracker::Result& r) : TrackingResultEvaluation(value(r)) {};
  virtual ~EntropyRatioTrackingResultEvaluation() {};
  virtual double value(const dvo::DenseTracker::Result& r) const;
};


} /* namespace dvo_slam */
#endif /* TRACKING_RESULT_EVALUATION_H_ */
