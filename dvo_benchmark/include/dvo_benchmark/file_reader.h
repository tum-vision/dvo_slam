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

#ifndef FILEREADER_H_
#define FILEREADER_H_

#include <fstream>

#include <dvo/util/fluent_interface.h>

namespace dvo_benchmark
{

/**
 * EntryT has to support the following operator
 *   std::istream& operator >>(std::istream&, EntryT&);
 */
template<class EntryT>
class FileReader
{
public:
  FileReader(std::string& file) :
    hasEntry_(false),
    file_(file),
    file_stream_(file.c_str())
  {
  }

  virtual ~FileReader()
  {
    file_stream_.close();
  }

  void skip(int num_lines)
  {
    for(int idx = 0; idx < num_lines; ++idx)
    {
      if(!file_stream_.good())  continue;

      file_stream_.ignore(1024, '\n');

      assert(file_stream_.gcount() < 1024);
    }
  }

  void skipComments()
  {
    while(file_stream_.good() && file_stream_.peek() == '#')
    {
      skip(1);
    }
  }

  /**
   * Moves to the next entry in the file. Returns true, if there was a next entry, false otherwise.
   */
  bool next()
  {
    if(file_stream_.good() && !file_stream_.eof())
    {
      file_stream_ >> entry_;
      hasEntry_ = true;

      return true;
    }

    return false;
  }

  /**
   * Read all entries at once.
   */
  void readAllEntries(std::vector<EntryT>& entries)
  {
    if(!hasEntry()) next();

    do
    {
      entries.push_back(entry());
    }
    while(next());
  }

  /**
   * Gets the current entry.
   */
  FI_ATTRIBUTE(FileReader, EntryT, entry);

  /**
   * Determines whether the first entry was read.
   */
  FI_ATTRIBUTE(FileReader, bool, hasEntry);
private:
  std::string file_;
  std::ifstream file_stream_;
};

} /* namespace dvo_benchmark */
#endif /* FILEREADER_H_ */
