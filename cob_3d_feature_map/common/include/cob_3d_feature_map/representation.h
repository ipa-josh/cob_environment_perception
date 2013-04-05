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
    inline static Type d(const MatrixU &covar1, const MatrixU &covar2) {
    	/*Eigen::SelfAdjointEigenSolver<MatrixU> es1(covar1, Eigen::EigenvaluesOnly);
    	Eigen::SelfAdjointEigenSolver<MatrixU> es2(covar2, Eigen::EigenvaluesOnly);

    	const Type a = es1.eigenvalues().dot(es1.eigenvalues()), b = es2.eigenvalues().dot(es2.eigenvalues());
    	Type d = std::sqrt( std::max(a,b) / std::min(a,b))-1;

        / *std::cout<<es1.eigenvalues()<<std::endl;
        std::cout<<es2.eigenvalues()<<std::endl;
        std::cout<<"d: "<<d<<std::endl;*//*

        return d;*/

//      MatrixU delta = covar2.llt().solve(covar1);// = covar1*covar2.inverse(); //TODO: for higher degree use LLT
      Eigen::GeneralizedSelfAdjointEigenSolver<MatrixU> es(covar1,covar2,Eigen::EigenvaluesOnly|Eigen::Ax_lBx);
      //Eigen::SelfAdjointEigenSolver<MatrixU> es(delta,Eigen::EigenvaluesOnly);
      Type d=0;//std::numeric_limits<Type>::max();
      for(size_t i=0; i<Dimension; i++)
        d += std::pow(std::log(std::abs(es.eigenvalues()(i))),2);
      /*std::cout<<covar1<<std::endl;
      std::cout<<covar2<<std::endl;
      //std::cout<<covar2.inverse()<<std::endl;
      //std::cout<<es.eigenvectors()<<std::endl;
      std::cout<<es.eigenvalues()<<std::endl;
      std::cout<<"d: "<<d<<std::endl;*/
      return std::sqrt(d);
    }

    inline static Type d2(const MatrixU &covar1, const MatrixU &covar2) {
    	Eigen::SelfAdjointEigenSolver<MatrixU> es(covar1*covar2.inverse(), Eigen::EigenvaluesOnly);

    	Type d=0;//std::numeric_limits<Type>::max();
      for(size_t i=0; i<Dimension; i++)
        d += std::pow(std::log(std::abs(es.eigenvalues()(i))),2);
      /*std::cout<<covar1<<std::endl;
      std::cout<<covar2<<std::endl;
      //std::cout<<covar2.inverse()<<std::endl;
      //std::cout<<es.eigenvectors()<<std::endl;
      std::cout<<es.eigenvalues()<<std::endl;
      std::cout<<"d: "<<d<<std::endl;*/
      return std::sqrt(d);
    }

  public:
    ClusterReprsentation():tmp_covar_(MatrixU::Zero()), mean_(VectorU::Zero()), weight_(0)
    {}

    inline void operator+=(const VectorU &o) {
    	const Type w = 1;
        weight_ += w;
        const Type a = w/weight_;
      tmp_covar_ = (1-a)*tmp_covar_ + a*(o*o.transpose());
      mean_ = (1-a)*mean_ + a*o;
    }

    inline void operator+=(const ClusterReprsentation &o) {
        const Type a = o.weight_/(o.weight_+weight_);
      if(a!=a || o.weight_==0) return;
      weight_ += o.weight_;
      tmp_covar_ = (1-a)*tmp_covar_ + a*o.tmp_covar_;
      mean_ = (1-a)*mean_ + a*o.mean_;
    }

    inline ClusterReprsentation operator*(const ClusterReprsentation &o) const {
    	ClusterReprsentation r = *this;
      r.weight_ *= o.weight_;
      r.tmp_covar_ = getCoVar() - (getCoVar()*getCoVar())*(getCoVar()+o.getCoVar()).inverse();
      r.mean_ += getCoVar()*(getCoVar()+o.getCoVar()).inverse()*(o.mean_-mean_);
      return r;
    }

    inline ClusterReprsentation operator*(const Type &f) const {
    	ClusterReprsentation r = *this;
    	r.tmp_covar_*=f;
    	r.mean_*=f;
    	r.weight_*=f;
    	return r;
    }

    /// p: [0,1]
    inline Type cmp(const ClusterReprsentation &o) const {
      return 1/(std::log(d(getCoVar(), o.getCoVar())+1)+1);
    }

    inline Type cmp2(const ClusterReprsentation &o, const ClusterReprsentation &r1, const ClusterReprsentation &r2) const {
      const Type dist1 = d2(getCoVar()*r1.getCoVar().inverse(), o.getCoVar()*r2.getCoVar().inverse());
      const Type dist2 = std::pow(std::log(std::sqrt( (mean_-r1.mean_).squaredNorm()/(o.mean_-r2.mean_).squaredNorm() )),2);

      std::cout<<"dist1: "<<dist1<<std::endl;
      std::cout<<"dist2: "<<dist2<<std::endl;

      std::cout<<"R1: "<<r1.getCoVar()<<std::endl;
      std::cout<<"R2: "<<r2.getCoVar()<<std::endl;

      std::cout<<"C1: "<<getCoVar()<<std::endl;
      std::cout<<"C2: "<<o.getCoVar()<<std::endl;

      std::cout<<"C1: "<<getCoVar()*r1.getCoVar().inverse()<<std::endl;
      std::cout<<"C2: "<<o.getCoVar()*r2.getCoVar().inverse()<<std::endl;

      return 1/(std::log(dist1+dist2+1)+1);
    }

    inline Type cmp3(const ClusterReprsentation &o, const ClusterReprsentation &r1, const ClusterReprsentation &r2) const {
      /*const Type d1 = (  getMean()-r1.getMean()).norm();
      const Type d2 = (o.getMean()-r2.getMean()).norm();
      return std::min(d1,d2)/std::max(d1,d2);   //TODO:*/

      const VectorU delta1 = (  getMean()-r1.getMean());
      const VectorU delta2 = (o.getMean()-r2.getMean());

      const Type dn1 = delta1.norm();
      const Type dn2 = delta2.norm();

      const Type d11 = (   getCoVar()*delta1/dn1).norm();
      const Type d12 = (r1.getCoVar()*delta1/dn1).norm();

      const Type d21 = ( o.getCoVar()*delta2/dn2).norm();
      const Type d22 = (r2.getCoVar()*delta2/dn2).norm();

      const Type d = std::sqrt(std::max(d11,d21)) + std::sqrt(std::min(d12,d22));
      return (std::exp( std::pow( (dn1-dn2)/d ,2)/-2 ));// / (std::sqrt(2*M_PI) * d);
    }

    inline MatrixU getCoVar() const {
      return tmp_covar_ - mean_*mean_.transpose();
    }

    inline VectorU getMean() const {
      return mean_;
    }

    inline Type getWeight() const {
      return weight_;
    }

  };

}


#endif /* REPRESENTATION_H_ */
