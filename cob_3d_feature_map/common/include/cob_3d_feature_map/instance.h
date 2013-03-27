/*
 * instance.h
 *
 *  Created on: 26.03.2013
 *      Author: josh
 */

#ifndef INSTANCE_H_
#define INSTANCE_H_


namespace cob_3d_feature_map {

  template<typename ClusterReprsentation, typename Feature>
  class Instance
  {
  public:
    typedef boost::shared_ptr<Feature> FeaturePtr;
    typedef Feature::Descriptor DESCRIPTOR;

  protected:
    std::vector<FeaturePtr> fts_;               /// associated features
    ClusterReprsentation rep_;                  /// summarized representation
    AccumulatedDescriptor<DESCRIPTOR> acc_descr_;     /// accumulated descriptor

  public:

    const ClusterReprsentation &getRepresentation() const {
      return rep_;
    }

    const AccumulatedDescriptor<DESCRIPTOR> &getAccDescr() const {
      return acc_descr_;
    }

  };

}



#endif /* INSTANCE_H_ */
