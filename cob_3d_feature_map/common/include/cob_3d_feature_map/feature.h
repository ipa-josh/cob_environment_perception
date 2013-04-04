/*
 * feature.h
 *
 *  Created on: 23.03.2013
 *      Author: josh
 */

#ifndef FEATURE_H_
#define FEATURE_H_

#include "descriptor.h"

namespace cob_3d_feature_map {

  template <typename Content, size_t const Dimension, typename Type>
  class Feature_kdtree {
  public:
    enum {DIMENSION=Dimension};
    typedef Descriptor1D<2,Type> Descriptor;
    typedef Type value_type;

  protected:
    Content content_;
    Descriptor descr_;

  public:

    const Content &getContent() const
    {return content_;}

    Content &getContent()
    {return content_;}

    const value_type &operator[](size_t n) const
    {
      return content_(n);
    }

    value_type &operator[](size_t n)
    {
      return content_(n);
    }

    Type distance(const Feature_kdtree<Content, Dimension, Type> &node)
    {
      return (content_-node.content_).squaredNorm();
    }

    const Descriptor &getDescriptor() const
    { return descr_; }

    void setDescriptorFromVector(const std::vector<Type> &vec)
    {
      descr_.descr_.resize(vec.size());
      std::reverse_copy(vec.begin(),vec.end(), descr_.descr_.begin());
    }

    bool operator==(const Feature_kdtree &o) const {
      return content_==o.content_;
    }
  };
}

#if 0
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
#endif

#endif /* FEATURE_H_ */