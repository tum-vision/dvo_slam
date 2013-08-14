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

#ifndef MAP_SERIALIZER_H_
#define MAP_SERIALIZER_H_

#include <iostream>

#include <dvo_slam/serialization/map_serializer_interface.h>
#include <dvo_slam/PoseStampedArray.h>

namespace dvo_slam
{
namespace serialization
{

template<typename DelegateType>
class FileSerializer : public MapSerializerInterface
{
public:
  FileSerializer(const std::string& filename) :
    fstream_(filename.c_str()),
    delegate_(fstream_)
  {
    std::cerr << filename << std::endl;
  }
  virtual ~FileSerializer()
  {
    fstream_.flush();
    fstream_.close();
  }

  virtual void serialize(const dvo_slam::KeyframeGraph& map)
  {
    delegate_.serialize(map);
  }
private:
  std::ofstream fstream_;
  DelegateType delegate_;
};

class TrajectorySerializer : public MapSerializerInterface
{
public:
  TrajectorySerializer(std::ostream& stream);
  virtual ~TrajectorySerializer();

  virtual void serialize(const dvo_slam::KeyframeGraph& map);
private:
  std::ostream& stream_;
};

class EdgeErrorSerializer : public MapSerializerInterface
{
public:
  EdgeErrorSerializer(std::ostream& stream);
  virtual ~EdgeErrorSerializer();

  virtual void serialize(const dvo_slam::KeyframeGraph& map);
private:
  std::ostream& stream_;
};

class MessageSerializer : public MapSerializerInterface
{
public:
  MessageSerializer(dvo_slam::PoseStampedArray& msg);
  virtual ~MessageSerializer();

  virtual void serialize(const dvo_slam::KeyframeGraph& map);
private:
  dvo_slam::PoseStampedArray& msg_;
};

} /* namespace serialization */
} /* namespace dvo_slam */
#endif /* MAP_SERIALIZER_H_ */
