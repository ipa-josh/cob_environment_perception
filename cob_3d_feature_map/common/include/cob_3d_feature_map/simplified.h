/*
 * simplified.h
 *
 *  Created on: 04.04.2013
 *      Author: josh
 */

#ifndef SIMPLIFIED_H_
#define SIMPLIFIED_H_

#include <cob_3d_feature_map/libkdtree/kdtree++/kdtree.hpp>
#include <cob_3d_feature_map/cluster.h>
#include <cob_3d_feature_map/representation.h>
#include <cob_3d_feature_map/feature.h>
#include <cob_3d_feature_map/instance.h>
#include <cob_3d_feature_map/haar.h>
#include <cob_3d_feature_map/correspondence.h>

namespace cob_3d_feature_map {

  template<typename ClusterReprsentation, typename Feature>
  struct SimplifiedInterface {

    typedef cob_3d_feature_map::Instance<ClusterReprsentation, Feature> INST;
    typedef cob_3d_feature_map::Cluster<INST> CL;
    typedef cob_3d_feature_map::CorrespondenceFinding::Correspondence<INST, ClusterReprsentation> COR;
    typedef cob_3d_feature_map::CorrespondenceSet<CL,COR> COR_SET;
    typedef KDTree::KDTree<Feature> treeType;

    /*void add(const boost::shared_ptr<INST> &inst, const boost::shared_ptr<FT> &ft) {

    }*/
  };

  template<const size_t FeatureDim, const size_t ContextDim, typename Type=float>
  struct _SimplifiedInterface_Eigen {
        typedef cob_3d_feature_map::Feature_kdtree<Eigen::Matrix<Type, FeatureDim, 1>, FeatureDim, Type> FT;
        typedef cob_3d_feature_map::ClusterReprsentation<ContextDim> CR;
  };

  template<const size_t FeatureDim, const size_t ContextDim, typename Type=float>
  struct SimplifiedInterface_Eigen : public
  _SimplifiedInterface_Eigen<FeatureDim, ContextDim, Type>,
  SimplifiedInterface<typename _SimplifiedInterface_Eigen<FeatureDim, ContextDim, Type>::CR, typename _SimplifiedInterface_Eigen<FeatureDim, ContextDim, Type>::FT>
  {
  };


}


#endif /* SIMPLIFIED_H_ */
