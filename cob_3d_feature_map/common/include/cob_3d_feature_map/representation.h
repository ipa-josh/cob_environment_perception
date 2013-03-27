/*
 * representation.h
 *
 *  Created on: 26.03.2013
 *      Author: josh
 */

#ifndef REPRESENTATION_H_
#define REPRESENTATION_H_


#include <Eigen/Eigenvalues>


namespace cob_3d_feature_map {
  /*
   * d(A,B) after "A Metric for Covariance Matrices" W. Förstner and B. Moonen
   * Eigenvalues from (A-B)
   * std::sqrt( sum( std::ln(Eigenvalues)^2 ) )
   */

  template<int Dimension, typename Type=float>
  class ClusterReprsentation
  {
  public:
    typedef Eigen::Matrix<Type, Dimension, Dimension> MatrixU;
    typedef Eigen::Matrix<Type, Dimension, 1> VectorU;
    typedef Type TYPE;

  private:
    MatrixU tmp_covar_;       // covariance matrix
    VectorU mean_;        // mean matrix
    Type weight_;         // weight or number of data

    /// simalarity with d>=0: d=0 --> equal
    inline static float d(const MatrixU &covar1, const MatrixU &covar2) {
      MatrixU delta = covar1-covar2;
      Eigen::SelfAdjointEigenSolver<MatrixU> es(delta);
      Type d=0.f;
      for(size_t i=0; i<Dimension; i++)
        d += std::pow(std::ln(es.eigenvalues()(i)),2);
      return d;
    }

  public:

    inline void operator+=(const ClusterReprsentation &o) {
      const Type a = weight_/(weight_ + o.weight_), b = o.weight_/(weight_ + o.weight_);
      tmp_covar_ = a*tmp_covar_ + b*o.tmp_covar_;
      mean_  = a* mean_ + b* o.mean_;
      weight_ += o.weight_;
    }

    /// p: [0,1]
    inline Type cmp(const ClusterReprsentation &o) const {
      return 1/std::ln(d(getCoVar(), o.getCoVar())+1);
    }

    inline MatrixU getCoVar() const {
      return covar1 - mean_*mean_.transpose();
    }

    inline Type getWeight() const {
      return weight_;
    }

  };

}


#endif /* REPRESENTATION_H_ */
