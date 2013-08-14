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

#ifndef BOUNDED_BUFFER_H_
#define BOUNDED_BUFFER_H_

#include <boost/thread/condition.hpp>
#include <boost/circular_buffer.hpp>

namespace dvo
{
namespace visualization
{

// TODO: replace with tbb equivalent
// customized bounded buffer from: http://www.boost.org/doc/libs/1_50_0/libs/circular_buffer/doc/circular_buffer.html#boundedbuffer
template <class T>
class bounded_buffer {
public:

   typedef boost::circular_buffer<T> container_type;
   typedef typename container_type::size_type size_type;
   typedef typename container_type::value_type value_type;

   explicit bounded_buffer(size_type capacity) : m_unread(0), m_container(capacity), shutdown_(false) {}

   void push_front(typename container_type::param_value_type item) {
      boost::mutex::scoped_lock lock(m_mutex);
      // m_not_full.wait(lock, boost::bind(&bounded_buffer<value_type>::is_not_full, this)); // don't wait on full
      m_container.push_front(item);
      m_unread = std::min(m_container.capacity(), ++m_unread); // if we don't wait on full, we have to limit the max number of unread values

      lock.unlock();
      m_not_empty.notify_one();
   }

   bool pop_back(value_type* pItem) {
      boost::mutex::scoped_lock lock(m_mutex);
      m_not_empty.wait(lock, boost::bind(&bounded_buffer<value_type>::is_not_empty_or_shutdown, this));

      if(shutdown_) return false; // if we waited, but got shutdown in the meanwhile, we just return because we still have nothing in the container

      *pItem = m_container[--m_unread];
      lock.unlock();
      // m_not_full.notify_one(); // don't wait on full
      return true;
   }

   void shutdown() {
     shutdown_ = true;
     m_not_empty.notify_one();
   }

private:
   bounded_buffer(const bounded_buffer&);              // Disabled copy constructor
   bounded_buffer& operator = (const bounded_buffer&); // Disabled assign operator

   bool is_not_empty() const { return m_unread > 0; }
   bool is_not_full() const { return m_unread < m_container.capacity(); }

   bool is_not_empty_or_shutdown() const { return is_not_empty() || shutdown_; }

   size_type m_unread;
   container_type m_container;
   boost::mutex m_mutex;
   boost::condition m_not_empty;
   bool shutdown_;
   //boost::condition m_not_full; // don't wait on full
};

} /* namespace visualization */
} /* namespace dvo */

#endif /* BOUNDED_BUFFER_H_ */
