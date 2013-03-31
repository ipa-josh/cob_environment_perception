/*
 * instance.h
 *
 *  Created on: 26.03.2013
 *      Author: josh
 */

#ifndef INSTANCE_H_
#define INSTANCE_H_

#include "descriptor.h"

namespace cob_3d_feature_map {

  template<typename _ClusterReprsentation, typename Feature>
  class Instance
  {
  public:
    typedef boost::shared_ptr<Feature> FeaturePtr;
    typedef typename Feature::Descriptor DESCRIPTOR;
    typedef _ClusterReprsentation ClusterReprsentation;

  protected:
    std::vector<FeaturePtr> fts_;               /// associated features
    ClusterReprsentation rep_;                  /// summarized representation
    AccumulatedDescriptor<DESCRIPTOR> acc_descr_;     /// accumulated descriptor

  public:

    Instance(const ClusterReprsentation &r):
    	rep_(r)
    {}

    std::vector<FeaturePtr> &getFeatures()
		{ return fts_; }

    void append(const FeaturePtr &ft) {
    	fts_.push_back(ft);
    }

    void updateDescriptor(const FeaturePtr &ft) {
    	acc_descr_.append(ft->getDescriptor());
    }

    const ClusterReprsentation &getRepresentation() const {
      return rep_;
    }

    const AccumulatedDescriptor<DESCRIPTOR> &getAccDescr() const {
      return acc_descr_;
    }

  };

}



#endif /* INSTANCE_H_ */
