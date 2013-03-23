/*
 * ft_map.h
 *
 *  Created on: 23.03.2013
 *      Author: josh
 */

#ifndef FT_MAP_H_
#define FT_MAP_H_

#include <kdtree++/kdtree.hpp>

namespace cob_3d_feature_map {

  template <typename Feature>
  class Tree_Feature {

    typedef KDTree::KDTree<Feature::DIMENSION, SmartNode<Feature> > treeType;

    treeType tree_;

  public:

    void insert(const Feature &ft) {
      tree_.insert(ft);
    }

    void find(const std::vector<Feature> &view) {

        for(size_t i=0; i<view.size(); i++) {
          std::vector<Feature> result;
          tree_.find_within_range(view[i].getFt(), view[i].getFt().std(), result);

          for(size_t j=0; j<result.size(); j++) {
            result[j]->pick(view[i].getRtTree());
          }

        }

    }
  };
}


#endif /* FT_MAP_H_ */
