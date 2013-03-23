/*
 * feature.h
 *
 *  Created on: 23.03.2013
 *      Author: josh
 */

#ifndef FEATURE_H_
#define FEATURE_H_

#include <kdtree++/kdtree.hpp>
#include <boost/shared_ptr.hpp>

namespace cob_3d_feature_map {

  template <typename Content>
  class SmartNode {
    typename boost::shared_ptr<Content> c_;
  public:

    Content &operator->() {return *c_;}
    const Content &operator->() const {return *c_;}

    boost::shared_ptr<Content> get() {return c_;}
  };

  template <typename Relation, typename Context>
  class RelationEnd {
    typename boost::shared_ptr<Context> ctxt_;
    Relation rel_;

  public:

  };

  template <typename Feature, typename Relation>
  class Instance_Feature {
    typedef KDTree::KDTree<Relation::DIMENSION, SmartNode<Relation> > treeType;

    Feature ft_;
    treeType rt_tree_;

  public:

    const Feature &getFt() const {return ft_;}
    const treeType &getRtTree() const {return rt_tree_;}
  };
}


#endif /* FEATURE_H_ */
