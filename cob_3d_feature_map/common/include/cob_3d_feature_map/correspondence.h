/*
 * correspondence.h
 *
 *  Created on: Mar 31, 2013
 *      Author: josh
 */

#ifndef CORRESPONDENCE_H_
#define CORRESPONDENCE_H_

#include <vector>
#include <boost/shared_ptr.hpp>

namespace cob_3d_feature_map {

  namespace CorrespondenceFinding {

    template<typename InstancePtr, class Type>
    struct StupidInitProbability {
      Type operator()(const InstancePtr &a, const InstancePtr &b, const size_t num)
      {
        return 1; // /(Type)num;
      }
    };

    template<typename Correspondence, typename InstancePtr, typename ClusterPtr>
    struct StupidInitCors {
      void operator()(Correspondence &out, const InstancePtr &inst, const ClusterPtr &input)
      {
        out.set(input->getInstances().begin(), input->getInstances().end());
      }
    };

    template<typename Instance, typename ClusterReprsentation,
    class Type = float,
    typename InitP = StupidInitProbability<typename boost::shared_ptr<Instance>, Type>
    >
    //template<typename Instance, typename ClusterReprsentation, typename Type, typename InitP>
    struct Correspondence {
      typedef typename boost::shared_ptr<Instance> InstancePtr;
      typedef typename std::vector<InstancePtr> Vec_InstancePtr;
      typedef ClusterReprsentation REPRESENTATION;
      typedef Type TYPE;

      struct Hypothesis {
        InstancePtr other_;
        Type p_;
        Type p_tmp_;
      };

      InstancePtr origin_;
      std::vector<Hypothesis> hypothesis_;
      Type p_max_;

      Correspondence(const InstancePtr &ft): origin_(ft), p_max_(1)
      {}

      InstancePtr getOrigin() const
      {return origin_;}

      InstancePtr getBestMatch() const
      {
        Type p=0;
        size_t j=(size_t)-1;
        for(size_t i=0; i<hypothesis_.size(); i++)
          if(hypothesis_[i].p_>p) {
            j=i;
            p = hypothesis_[i].p_;
          }
        if(j>=hypothesis_.size())
          return InstancePtr();
        return hypothesis_[j].other_;
      }

      void set(typename Vec_InstancePtr::const_iterator begin, const typename Vec_InstancePtr::const_iterator &end)
      {
        size_t s = end-begin;
        while(begin!=end) {
          Hypothesis h = {*begin, InitP()(*begin, origin_, s), 0};
          hypothesis_.push_back(h);
          ++begin;
        }
      }

      void update(const REPRESENTATION &r1, const REPRESENTATION &r2, const Type &w) {
        //compares relation between origin/hypothesis and r
        for(size_t i=0; i<hypothesis_.size(); i++) {
          hypothesis_[i].p_tmp_ += w*origin_->getRepresentation().cmp3(hypothesis_[i].other_->getRepresentation(), r1, r2);
        }
      }

      void finish(const Type &w) {
        p_max_ = 0;
        for(size_t i=0; i<hypothesis_.size(); i++) {
          hypothesis_[i].p_ *= hypothesis_[i].p_tmp_/w;
          hypothesis_[i].p_tmp_ = 0;
          p_max_ = std::max(p_max_, hypothesis_[i].p_);
        }
      }

      Type getMaxP() const {return p_max_;}
    };

  }

  template<typename Cluster, typename Correspondence,
  typename InitCors = CorrespondenceFinding::StupidInitCors<Correspondence, typename Correspondence::InstancePtr, typename boost::shared_ptr<Cluster> > >
  class CorrespondenceSet {
    std::vector<Correspondence> cors_;
    boost::shared_ptr<Cluster> cl_;

  public:
    typedef Cluster element_type;

    CorrespondenceSet(const boost::shared_ptr<Cluster> &cl):cl_(cl)
    {}

    CorrespondenceSet(const boost::shared_ptr<Cluster> &cl, const boost::shared_ptr<Cluster> &input):cl_(cl)
    {
      for(size_t i=0; i<cl->getInstances().size(); i++) {
        *this += Correspondence(cl->getInstances()[i]);
        InitCors()(cors_.back(), cl->getInstances()[i], input);
      }
    }

    Cluster & operator*() const {return *cl_;}
    Cluster * operator->() const {return cl_.operator->();}
    const boost::shared_ptr<Cluster> &getPtr() const {return cl_;}

    void operator+=(const Correspondence &c) {
      cors_.push_back(c);
    }


    void update(const typename Correspondence::REPRESENTATION &r1, const typename Correspondence::REPRESENTATION &r2, const typename Correspondence::TYPE &w) {
      for(size_t i=0; i<cors_.size(); i++)
        cors_[i].update(r1,r2,w);
    }

    void finish(const typename Correspondence::TYPE &w) {
      for(size_t i=0; i<cors_.size(); i++)
        cors_[i].finish(w);
    }

    typename Correspondence::TYPE getP() const {
      typename Correspondence::TYPE p=0;
      for(size_t i=0; i<cors_.size(); i++)
        p += cors_[i].getMaxP();
      return p/cors_.size();
    }

    const std::vector<Correspondence> &getCors() const
    {return cors_;}
  };

  template<typename Correspondence>
  class CorrespondenceSetDummy {
  public:

    void operator+=(const Correspondence &c) {
    }


    void update(const typename Correspondence::REPRESENTATION &r1, const typename Correspondence::REPRESENTATION &r2, const typename Correspondence::TYPE &w) {
    }

    void finish(const typename Correspondence::TYPE &w) {
    }
  };

}



#endif /* CORRESPONDENCE_H_ */
