/*
 * descriptor.h
 *
 *  Created on: 26.03.2013
 *      Author: josh
 */

#ifndef DESCRIPTOR_H_
#define DESCRIPTOR_H_


namespace cob_3d_feature_map {

  template<int Codes, typename Type=float>
  class Descriptor1D {
  public:
    enum {CODES=Codes};
    typedef Type TYPE;

  private:
    std::vector<Type> descr_;

  public:

    inline Type weight(const size_t ind, const int code) const {
      if(ind>=descr_.size())
        return 0;
      return (code?1:-1)*((code%2)-descr_[ind]);
    }

    inline void append(const Type &c) {
      descr_.push_back(c);
    }

    template<typename Descriptor>
    friend class AccumulatedDescriptor;

    template <typename Content, size_t const Dimension, typename _Type>
    friend class Feature_kdtree;
  };

  template<typename Descriptor>
  class AccumulatedDescriptor {
  private:
    std::vector< Descriptor > acc_descr_;

  public:

    inline typename Descriptor::TYPE weight(const size_t ind, const int code) const {
      typename Descriptor::TYPE w=0;
      size_t n = 0;
      for(size_t i=0; i<acc_descr_.size(); i++)
        if(ind<acc_descr_[i].descr_.size()) {
        	w += acc_descr_[i].weight(ind,code);
        	++n;
        }
      return n?w/n:0;
    }

    inline void append(const Descriptor &d) {
      acc_descr_.push_back(d);
    }

  };
}


#endif /* DESCRIPTOR_H_ */
