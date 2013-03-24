/*
 * context.h
 *
 *  Created on: 23.03.2013
 *      Author: josh
 */

#ifndef CONTEXT_H_
#define CONTEXT_H_

#include <map>

namespace cob_3d_feature_map {

  template<typename Context, typename Relation, typename InstanceId, class View>
  class ContextAcitivity {
    struct S_HISTOGRAM_LOOKUP {
      typedef std::map<InstanceId, size_t> HMAP;
      HMAP lookup_;

      void operator+=(const InstanceId &id) {
        typename HMAP::iterator it = lookup_.find(id);
        if(it==lookup_.end())
          lookup_[id] = lookup_.size();
      }

      size_t size() const {return lookup_.size();}

      size_t operator()(const InstanceId &id) {
        typename HMAP::iterator it = lookup_.find(id);
        assert(it!=lookup_.end());
        return *it;
      }
    };

    struct S_HISTOGRAM {
      std::vector<size_t> hist_;
      const boost::shared_ptr<const S_HISTOGRAM_LOOKUP> lookup_;

      S_HISTOGRAM(const boost::shared_ptr<const S_HISTOGRAM_LOOKUP> &l):lookup_(l)
      {
        hist_.resize(lookup_.size());
      }

      void clear() {
        hist_.clear();
      }

      void operator+=(const InstanceId &id) {
        ++hist_[lookup_(id)];
      }

      void compare(const size_t num, S_HISTOGRAM &o, float &prob_over, float &prob_under) {
        assert(o.hist_.size()==hist_.size());

        for(size_t i=0; i<hist_.size(); i++) {
          if(hist_[i]<o.hist_[i])
            prob_under += o.hist_[i]-hist_[i];
          else
            prob_over += hist_[i]-o.hist_[i];
        }

        prob_under/=num;
        prob_over/=num;
      }
    };

    struct S_ACTIVITY {
      const boost::shared_ptr<Context> ctxt_;
      S_HISTOGRAM hist_;

      S_ACTIVITY(const boost::shared_ptr<Context> &c):ctxt_(c)
      {}

      void operator+=(const InstanceId &id) {
        hist_ += id;
      }
    };

    typedef std::map<typename Context::ID, S_ACTIVITY> AMAP;
    AMAP activity_;
    Relation dist_;
    S_HISTOGRAM_LOOKUP lookup_;
    S_HISTOGRAM hist_view_;

  public:
    ContextAcitivity():
      hist_view_(&lookup_)
    {}

    void init(const View &view) {
      lookup_ = S_HISTOGRAM_LOOKUP();
      for(typename View::const_iterator it = view.begin(); it!=view.end(); it++)
        lookup_ += it->getId();
      hist_view_.clear();
      for(typename View::const_iterator it = view.begin(); it!=view.end(); it++)
        hist_view_ += it->getId();
    }

    void pick(const InstanceId &id, const Relation &r, const boost::shared_ptr<Context> &ctxt) {
      typename AMAP::iterator it = activity_.find(ctxt->getId());
      if(it==activity_.end()) {
        activity_[ctxt->getId()] = S_ACTIVITY(ctxt);
        it = activity_.find(ctxt->getId());
      }

      (*it) += id;
      dist_ += r;
    }

    void cluster() {
      for(typename AMAP::const_iterator it=activity_.begin(); it!=activity_.end(); it++) {
        //generate copy of histogram from view
        S_HISTOGRAM view = hist_view_;

        //region growing until saturation of copied histogram of the view
        size_t num = view.num();
        float prob_over=0, prob_under=0;
        typename AMAP::const_iterator it2 = it;
        do {
          it2->hist_.compare(num, view, prob_over, prob_under);
        } while(false);

      }
    }
  };
}



#endif /* CONTEXT_H_ */
