/*
 * relation.h
 *
 *  Created on: 23.03.2013
 *      Author: josh
 */

#ifndef RELATION_H_
#define RELATION_H_

#include <Eigen/Core>
#include <Eigen/Dense>

namespace cob_3d_feature_map {

  template<int n>
  class RelationDistribution {
  public:
    typedef float value_type;

  private:
    typedef Eigen::Matrix<value_type, n, 1>    VectorU;
    typedef Eigen::Matrix<value_type, n, n>    MatrixU;

    bool computed_;
    size_t size_;
    MatrixU co_;
    VectorU eigenvalues_;
    MatrixU eigenvectors_;

    bool compute();
  public:
    RelationDistribution():
      computed_(false), size_(0), co_(MatrixU::Null()), eigenvalues_(VectorU::Null()), eigenvectors_(MatrixU::Null())
    {}

    enum {DIMENSION=n};

    void operator+=(const MatrixU &m) {computed_=false; ++size_; co_+=m;}
    void operator+=(const VectorU &m) {computed_=false; ++size_; co_+=m*m.transpose();}

    VectorU getEigenvalues() const {if(!computed_) compute(); return eigenvalues_;}
    MatrixU getEigenvectors() const {if(!computed_) compute(); return eigenvectors_;}

    value_type operator[](size_t i) const
    {
      return getEigenvalues()(i);
    }
  };

  template <typename Relation, typename Context>
  class RelationEnd {
   std::vector<typename boost::shared_ptr<Context> > ctxt_;
    Relation rel_;

  public:

    typedef Context CTXT;
    typedef Relation RT;
    typedef typename Relation::value_type value_type;

    value_type operator[](size_t n) const
    {
      return rel_[n];
    }
  };

#include "impl/relation.hpp"
}


#endif /* RELATION_H_ */
