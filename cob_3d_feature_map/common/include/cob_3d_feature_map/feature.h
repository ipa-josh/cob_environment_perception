/*
 * feature.h
 *
 *  Created on: 23.03.2013
 *      Author: josh
 */

#ifndef FEATURE_H_
#define FEATURE_H_

#include <cob_3d_feature_map/libkdtree/kdtree++/kdtree.hpp>
#include <boost/shared_ptr.hpp>
#include "id.h"

namespace cob_3d_feature_map {

  template <typename Content>
  class SmartNode {
    typename boost::shared_ptr<Content> c_;
  public:
    typedef typename Content::value_type value_type;

    Content &operator->() {return *c_;}
    const Content &operator->() const {return *c_;}

    boost::shared_ptr<Content> get() {return c_;}

    value_type operator[](size_t n) const
    {
      return (*c_)[n];
    }

    /*typedef double value_type;

    double xyz[3];
    size_t index;

    value_type operator[](size_t n) const
    {
      return xyz[n];
    }

    double distance( const kdtreeNode &node)
    {
      double x = xyz[0] - node.xyz[0];
      double y = xyz[1] - node.xyz[1];
      double z = xyz[2] - node.xyz[2];

   // this is not correct   return sqrt( x*x+y*y+z*z);

   // this is what kdtree checks with find_within_range()
   // the "manhattan distance" from the search point.
   // effectively, distance is the maximum distance in any one dimension.
      return max(fabs(x),max(fabs(y),fabs(z)));

    }*/
  };

  template <typename Feature, typename Relation>
  class Instance_Feature : public IDHandler  {
  public:
    typedef Feature FT;
    typedef Relation RT;
    typedef typename Feature::value_type value_type;

    enum {DIMENSION=Feature::DIMENSION};

  private:
    typedef KDTree::KDTree<Relation::RT::DIMENSION, SmartNode<Relation> > treeType;

    Feature ft_;
    treeType rt_tree_;

  public:

    const Feature &getFt() const {return ft_;}
    const treeType &getRtTree() const {return rt_tree_;}
    ID getId() const {return id_;}

    value_type operator[](size_t n) const
    {
      return ft_[n];
    }
  };
}


#endif /* FEATURE_H_ */
