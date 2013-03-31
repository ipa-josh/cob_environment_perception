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

template<typename FeaturePtr, class Type>
struct StupidInitProbability {
	Type operator()(const FeaturePtr &a, const FeaturePtr &b, const size_t num)
	{
		return 1; // /(Type)num;
	}
};

template<typename Feature, typename ClusterReprsentation,
class Type = float,
typename InitP = StupidInitProbability<typename boost::shared_ptr<Feature>, Type>
>
//template<typename Feature, typename ClusterReprsentation, typename Type, typename InitP>
struct Correspondence {
	typedef typename boost::shared_ptr<Feature> FeaturePtr;
	typedef typename std::vector<FeaturePtr> Vec_FeaturePtr;
	typedef ClusterReprsentation REPRESENTATION;
	typedef Type TYPE;

	struct Hypothesis {
		FeaturePtr other_;
		Type p_;
		Type p_tmp_;
	};

	FeaturePtr origin_;
	std::vector<Hypothesis> hypothesis_;

	Correspondence(const FeaturePtr &ft): origin_(ft)
	{}

	void set(typename Vec_FeaturePtr::const_iterator begin, const typename Vec_FeaturePtr::const_iterator &end)
	{
		size_t s = end-begin;
		while(begin!=end) {
			Hypothesis h = {*begin, InitP(*begin, origin_, s)};
			hypothesis_.push_back(h);
			++begin;
		}
	}

	void update(const REPRESENTATION &r1, const REPRESENTATION &r2, const Type &w) {
		//compares relation between origin/hypothesis and r
		for(size_t i=0; i<hypothesis_.size(); i++) {
			hypothesis_[i].p_tmp_ += w*origin_->getRepresentation().cmp2(hypothesis_[i].other_->getRepresentation(), r1, r2);
		}
	}

	void finish(const Type &w) {
		for(size_t i=0; i<hypothesis_.size(); i++) {
			hypothesis_[i].p_ *= hypothesis_[i].p_tmp_/w;
			hypothesis_[i].p_tmp_ = 0;
		}
	}
};

}

template<typename Correspondence>
class CorrespondenceSet {
	std::vector<Correspondence> cors_;

public:

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
