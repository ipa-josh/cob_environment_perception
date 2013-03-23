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
    typedef Eigen::Matrix<float, n, 1>    VectorU;
    typedef Eigen::Matrix<float, n, n>    MatrixU;

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

  };

#include "impl/relation.hpp"
}


#endif /* RELATION_H_ */
