/*
 * cluster.h
 *
 *  Created on: 26.03.2013
 *      Author: josh
 */

#ifndef CLUSTER_H_
#define CLUSTER_H_


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
    typedef Instance::ClusterReprsentation ClusterReprsentation;
    typedef ClusterReprsentation::TYPE TYPE;

  private:
    std::vector<InstacePtr> instances_;                         /// instances of the cluster
    ClusterReprsentation rep_;                                  /// summarized representation

  public:

    void operator+=(const InstacePtr &inst) {
      instances_.push_back(inst);
      //update rep.
      rep_ += inst->getRepresentation();
    }

    TYPE cmp(const Cluster &o) //-->p[0,1]
    {
      return rep_.cmp(o.rep_);
    }

    TYPE cmp_split(const std::vector<ClusterReprsentation> &o, const size_t ind) //-->p[0,1]
    {
      TYPE p=0, w=0;
      for(int i=0; i<Instance::DESCRIPTOR::CODES; i++) {
        ClusterReprsentation r = split(ind, i);
        p += (r.getWeight()+o[i].getWeight()) * r.cmp(o[i]);
        w += r.getWeight()+o[i].getWeight();
      }
      return p/w;
    }

    ClusterReprsentation split(const size_t ind, const int code) //-->p[0,1]
    {
      ClusterReprsentation r;
      for(size_t j=0; j<instances_.size(); j++)
        r += instances_[j]->getAccDescr().weight(ind, code) * instances_[j]->getRepresentation();
      return r;
    }

    void split_all(const size_t ind, std::vector<ClusterReprsentation> &ret) //-->p[0,1]
    {
      ret.clear();
      for(int i=0; i<Instance::DESCRIPTOR::CODES; i++) {
        ClusterReprsentation r;
        for(size_t j=0; j<instances_.size(); j++)
          r += instances_[j]->getAccDescr().weight(ind, code) * instances_[j]->getRepresentation();
        ret.push_back(r);
      }
    }

  };

}




#endif /* CLUSTER_H_ */
