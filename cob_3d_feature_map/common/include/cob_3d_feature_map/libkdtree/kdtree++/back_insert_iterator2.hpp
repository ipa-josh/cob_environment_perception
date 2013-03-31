/*
 * back_insert_iterator2.hpp
 *
 *  Created on: Mar 29, 2013
 *      Author: josh
 */

#pragma once

template <class Container>
  class back_insert_iterator2 :
    public std::iterator<std::output_iterator_tag,void,void,void,void>
{
protected:
  Container* container_;
  typename Container::iterator pos;

public:
  typedef Container container_type;
  explicit back_insert_iterator2 (Container& x) : container_(&x), pos(x.begin()) {}

  back_insert_iterator2<Container>& operator= (typename Container::const_reference value)
    { container_->push_back(value); return *this; }
  back_insert_iterator2<Container>& operator* ()
    { return *this; }
  back_insert_iterator2<Container>& operator++ ()
    { ++pos; return *this; }
  back_insert_iterator2<Container> operator++ (int)
    { ++pos; return *this; }

  bool operator==(const back_insert_iterator2 &o) const
	  { return pos==o.pos; }
  bool operator!=(const back_insert_iterator2 &o) const
	  { return !(*this==o); }

  const typename container_type::iterator::value_type &value() const
  	  { return *pos; }
  typename container_type::iterator::value_type &value()
  	  { return *pos; }
  const container_type &container() const
  { return *container_; }
  container_type &container()
  { return *container_; }
};
