/*
 * cluster.h
 *
 *  Created on: 26.03.2013
 *      Author: josh
 */

#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <boost/shared_ptr.hpp>

namespace cob_3d_feature_map {
  /*
   * d(A,B) after "A Metric for Covariance Matrices" W. Förstner and B. Moonen
   * Eigenvalues from (A-B)
   * std::sqrt( sum( std::ln(Eigenvalues)^2 ) )
   */

  template<typename Instance>
  class Cluster
  {
  public:
    typedef boost::shared_ptr<Instance> InstacePtr;
    typedef typename Instance::ClusterReprsentation ClusterReprsentation;
    typedef typename ClusterReprsentation::TYPE TYPE;
    typedef Instance INSTANCE;

  private:
    std::vector<InstacePtr> instances_;                         /// instances of the cluster
    ClusterReprsentation rep_;                                  /// summarized representation

  public:

    std::vector<InstacePtr> &getInstances()
    { return instances_;}
    const std::vector<InstacePtr> &getInstances() const
    { return instances_;}

    const ClusterReprsentation &getRepresentation() const
    { return rep_; }

    void operator+=(const InstacePtr &inst) {
      instances_.push_back(inst);
      //update rep.
      rep_ += inst->getRepresentation();
    }

    TYPE cmp(const Cluster &o) const //-->p[0,1]
    {
      return rep_.cmp(o.rep_);
    }

    ClusterReprsentation split(const size_t ind, const int code) const //-->p[0,1]
    {
      ClusterReprsentation r;
      for(size_t j=0; j<instances_.size(); j++)
        r += instances_[j]->getRepresentation() * instances_[j]->getAccDescr().weight(ind, code);
      return r;
    }

    TYPE split_all(const size_t ind, std::vector<ClusterReprsentation> &ret) const //-->p[0,1]
    {
    	TYPE w=0;
      ret.clear();
      for(int i=0; i<Instance::DESCRIPTOR::CODES; i++) {
        ClusterReprsentation r;
        for(size_t j=0; j<instances_.size(); j++) {
        	TYPE t = instances_[j]->getAccDescr().weight(ind, i);
        	w += t;
          r += instances_[j]->getRepresentation() * t;
        }
        ret.push_back(r);
      }

      return w/Instance::DESCRIPTOR::CODES;
    }

  };

}




#endif /* CLUSTER_H_ */
