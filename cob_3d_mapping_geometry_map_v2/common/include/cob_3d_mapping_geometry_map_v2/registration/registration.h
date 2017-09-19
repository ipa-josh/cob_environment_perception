#pragma once

#include "../types/context.h"
#include <pcl/registration/bfgs.h>

namespace cob_3d_geometry_map {
namespace registration {
	
struct registration_error: virtual boost::exception, virtual std::exception { };
	
template<class _Scalar=float>
class CorrespondenceEstimator {
	Context::ConstPtr new_scene_;
	Context::ConstPtr map_;
public:
	typedef _Scalar Scalar;
	typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;
	
	void setNewScene( const Context::ConstPtr &ctxt) {new_scene_ = ctxt;}
	void setMap( const Context::ConstPtr &ctxt) {map_ = ctxt;}
	
	virtual Scalar isCorrespondence(Object::ConstPtr new_obj, Object::ConstPtr map_obj) const {
		return 1.;
	}
	
	virtual void findCorrespondences(const Object::ConstPtr &src, const std::vector<Object::ConstPtr> &tgt, const Matrix4 &transformation, std::vector<int> &cors_idx) {
		const ObjectVolume *vol = dynamic_cast<const ObjectVolume*>(src.get());
		if(!vol) return;
		
		std::vector<Object::Ptr> result;
		Eigen::Affine3f aff; aff.matrix() = transformation.cast<float>();
		map_->volume_search_in_volume(vol->bb_in_context(aff), result);
		
		for(size_t k=0; k<result.size(); k++) {
			if(isCorrespondence(src, result[k])<=0) continue;
			
			for(size_t j=0; j<tgt.size(); j++)
				if(result[k]==tgt[j]) {
					cors_idx.push_back((int)j);
					break;
				}
		}
	}
};

template<class _Scalar=float>
class CovarianceEstimator {	
public:
	typedef _Scalar Scalar;
	typedef Eigen::Matrix<Scalar, 4, 1> Vector4;
	typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
	
	CovarianceEstimator(Object::ConstPtr obj) {
		cov_ = Matrix3::Zero();
		pt_  = Vector4::Zero();
		pt_(3) = 1.0;
		
		const ObjectVolume *vol = dynamic_cast<const ObjectVolume*>(obj.get());
		if(!vol) return;
		
		ObjectVolume::TBB bb = vol->bb_in_pose();
		Eigen::Affine3f aff = cast(vol->pose());
		pt_.template head<3>() = (aff*((bb.min()+bb.max())/2)).cast<Scalar>();
		
		ObjectVolume::TBB::VectorType tl = bb.corner( ObjectVolume::TBB::CornerType::TopLeft );
		ObjectVolume::TBB::VectorType tr = bb.corner( ObjectVolume::TBB::CornerType::TopRight );
		ObjectVolume::TBB::VectorType bl = bb.corner( ObjectVolume::TBB::CornerType::BottomLeft );
		ObjectVolume::TBB::VectorType tlf = bb.corner( ObjectVolume::TBB::CornerType::TopLeftFloor );
		
		Eigen::Matrix3f T;
		T.col(0) = 0.5*(tr-tl);
		T.col(1) = 0.5*(bl-tl);
		T.col(2) = 0.5*(tlf-tl);
		
		cov_ = (aff*(T*T.transpose())).cast<Scalar>();
	}
	
	inline Matrix3 cov() const {return cov_;}
	inline Vector4 pt() const {return pt_;}
	
protected:
	Matrix3 cov_;
	Vector4 pt_;
};

template<class Scalar=float>
class GICP {
public:
	typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
	typedef Eigen::Matrix<Scalar, 4, 1> Vector4;
	typedef Eigen::Matrix<Scalar, 6, 1> Vector6;
	typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
	typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;
	
private:
	
	//member variables
	bool converged_;
	const std::vector<Vector4> *tmp_src_, *tmp_tgt_;
	std::vector<Matrix3> mahalanobis;
	int max_inner_iterations_;
	Scalar rotation_epsilon_;
	Scalar transformation_epsilon_;
	
	/** \brief Set the rotation epsilon (maximum allowable difference between two 
	* consecutive rotations) in order for an optimization to be considered as having 
	* converged to the final solution.
	* \param epsilon the rotation epsilon
	*/
	inline void 
	setRotationEpsilon (double epsilon) { rotation_epsilon_ = epsilon; }

	/** \brief Get the rotation epsilon (maximum allowable difference between two 
	* consecutive rotations) as set by the user.
	*/
	inline double 
	getRotationEpsilon () { return (rotation_epsilon_); }
	
	 /** set maximum number of iterations at the optimization step
	   * \param[in] max maximum number of iterations for the optimizer
	   */
	 void
	 setMaximumOptimizerIterations (int max) { max_inner_iterations_ = max; }

	 ///\return maximum number of iterations at the optimization step
	 int
	 getMaximumOptimizerIterations () { return (max_inner_iterations_); }
	 
	 /** \return trace of mat1^t . mat2 
	   * \param mat1 matrix of dimension nxm
	   * \param mat2 matrix of dimension nxp
	   */
	inline Scalar 
	matricesInnerProd(const Matrix3& mat1, const Matrix3& mat2) const
	{
	Scalar r = 0.;
	size_t n = mat1.rows();
	// tr(mat1^t.mat2)
	for(size_t i = 0; i < n; i++)
	 for(size_t j = 0; j < n; j++)
	   r += mat1 (j, i) * mat2 (i,j);
	return r;
	}

	void computeRDerivative(const Vector6 &x, const Matrix3 &R, Vector6& g) const
	{
	 Matrix3 dR_dPhi;
	 Matrix3 dR_dTheta;
	 Matrix3 dR_dPsi;

	 Scalar phi = x[3], theta = x[4], psi = x[5];

	 Scalar cphi = cos(phi), sphi = sin(phi);
	 Scalar ctheta = cos(theta), stheta = sin(theta);
	 Scalar cpsi = cos(psi), spsi = sin(psi);

	 dR_dPhi(0,0) = 0.;
	 dR_dPhi(1,0) = 0.;
	 dR_dPhi(2,0) = 0.;

	 dR_dPhi(0,1) = sphi*spsi + cphi*cpsi*stheta;
	 dR_dPhi(1,1) = -cpsi*sphi + cphi*spsi*stheta;
	 dR_dPhi(2,1) = cphi*ctheta;

	 dR_dPhi(0,2) = cphi*spsi - cpsi*sphi*stheta;
	 dR_dPhi(1,2) = -cphi*cpsi - sphi*spsi*stheta;
	 dR_dPhi(2,2) = -ctheta*sphi;

	 dR_dTheta(0,0) = -cpsi*stheta;
	 dR_dTheta(1,0) = -spsi*stheta;
	 dR_dTheta(2,0) = -ctheta;

	 dR_dTheta(0,1) = cpsi*ctheta*sphi;
	 dR_dTheta(1,1) = ctheta*sphi*spsi;
	 dR_dTheta(2,1) = -sphi*stheta;

	 dR_dTheta(0,2) = cphi*cpsi*ctheta;
	 dR_dTheta(1,2) = cphi*ctheta*spsi;
	 dR_dTheta(2,2) = -cphi*stheta;

	 dR_dPsi(0,0) = -ctheta*spsi;
	 dR_dPsi(1,0) = cpsi*ctheta;
	 dR_dPsi(2,0) = 0.;

	 dR_dPsi(0,1) = -cphi*cpsi - sphi*spsi*stheta;
	 dR_dPsi(1,1) = -cphi*spsi + cpsi*sphi*stheta;
	 dR_dPsi(2,1) = 0.;

	 dR_dPsi(0,2) = cpsi*sphi - cphi*spsi*stheta;
	 dR_dPsi(1,2) = sphi*spsi + cphi*cpsi*stheta;
	 dR_dPsi(2,2) = 0.;

	 g[3] = matricesInnerProd(dR_dPhi, R);
	 g[4] = matricesInnerProd(dR_dTheta, R);
	 g[5] = matricesInnerProd(dR_dPsi, R);
	}
  
	/// \brief optimization functor structure
	struct OptimizationFunctorWithIndices : public BFGSDummyFunctor<Scalar,6>
	{
		OptimizationFunctorWithIndices (const GICP* gicp)
		  : BFGSDummyFunctor<Scalar,6> (), gicp_(gicp) {}
		  
		Scalar operator() (const Vector6& x) {
			Matrix4 transformation_matrix = Matrix4::Identity();
			gicp_->applyState(transformation_matrix, x);
			Scalar f = 0;
			int m = static_cast<int> (gicp_->tmp_src_->size ());
			for (int i = 0; i < m; ++i)
			{
				// The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
				const Vector4 p_src = (*gicp_->tmp_src_)[i];
				// The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
				const Vector4 p_tgt = (*gicp_->tmp_tgt_)[i];
				Vector4 pp (transformation_matrix * p_src);
				// Estimate the distance (cost function)
				// The last coordiante is still guaranteed to be set to 1.0
				Vector3 res(pp[0] - p_tgt[0], pp[1] - p_tgt[1], pp[2] - p_tgt[2]);
				Vector3 temp (gicp_->mahalanobis[i] * res);
				//increment= res'*temp/num_matches = temp'*M*temp/num_matches (we postpone 1/num_matches after the loop closes)
				f+= Scalar(res.transpose() * temp);
			}
			return f/m;
		}
			
		void  df(const Vector6 &x, Vector6 &df) {
			Matrix4 transformation_matrix = Matrix4::Identity();
			gicp_->applyState(transformation_matrix, x);
			//Zero out g
			df.setZero ();
			//Vector3 g_t = g.head<3> ();
			Matrix3 R = Matrix3::Zero ();
			int m = static_cast<int> (gicp_->tmp_src_->size ());
			for (int i = 0; i < m; ++i)
			{
				// The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
				const Vector4 p_src = (*gicp_->tmp_src_)[i];
				// The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
				const Vector4 p_tgt = (*gicp_->tmp_tgt_)[i];

				Vector4 pp (transformation_matrix * p_src);
				// The last coordiante is still guaranteed to be set to 1.0
				Vector3 res (pp[0] - p_tgt[0], pp[1] - p_tgt[1], pp[2] - p_tgt[2]);
				// temp = M*res
				Vector3 temp (gicp_->mahalanobis[i] * res);
				// Increment translation gradient
				// g.head<3> ()+= 2*M*res/num_matches (we postpone 2/num_matches after the loop closes)
				df.template head<3> ()+= temp;
				// Increment rotation gradient
				pp = p_src;
				Vector3 p_src3 (pp[0], pp[1], pp[2]);
				R+= p_src3 * temp.transpose();
			}
			df.template head<3> ()*= 2.0/m;
			R*= 2.0/m;
			gicp_->computeRDerivative(x, R, df);
		}
		
		void fdf(const Vector6 &x, Scalar &f, Vector6 &df) {
			Matrix4 transformation_matrix = Matrix4::Identity();
			gicp_->applyState(transformation_matrix, x);
			f = 0;
			df.setZero ();
			Matrix3 R = Matrix3::Zero ();
			const int m = static_cast<const int> (gicp_->tmp_src_->size ());
			for (int i = 0; i < m; ++i)
			{
				// The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
				const Vector4 p_src = (*gicp_->tmp_src_)[i];
				// The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
				const Vector4 p_tgt = (*gicp_->tmp_tgt_)[i];
				Vector4 pp (transformation_matrix * p_src);
				// The last coordiante is still guaranteed to be set to 1.0
				Vector3 res (pp[0] - p_tgt[0], pp[1] - p_tgt[1], pp[2] - p_tgt[2]);
				// temp = M*res
				Vector3 temp (gicp_->mahalanobis[i] * res);
				// Increment total error
				f+= Scalar(res.transpose() * temp);
				// Increment translation gradient
				// g.head<3> ()+= 2*M*res/num_matches (we postpone 2/num_matches after the loop closes)
				df.template head<3> ()+= temp;
				pp = p_src;
				Vector3 p_src3 (pp[0], pp[1], pp[2]);
				// Increment rotation gradient
				R+= p_src3 * temp.transpose();
			}
			f/= Scalar(m);
			df.template head<3> ()*= Scalar(2.0/m);
			R*= 2.0/m;
			gicp_->computeRDerivative(x, R, df);
		}

		const GICP *gicp_;
	};
	
	void estimateRigidTransformationBFGS (const std::vector<Vector4> &pt_src,
										  const std::vector<Vector4> &pt_tgt,
										  Matrix4 &transformation_matrix)
	{
	  if (pt_src.size () < 4 || pt_src.size()!=pt_tgt.size())     // need at least 4 samples
	  {
		BOOST_THROW_EXCEPTION ( registration_error{} );
			//NotEnoughPointsException, "[pcl::GeneralizedIterativeClosestPoint::estimateRigidTransformationBFGS] Need at least 4 points to estimate a transform! Source and target have " << indices_src.size () << " points!");
		return;
	  }
	  // Set the initial solution
	  Vector6 x = Vector6::Zero ();
	  x[0] = transformation_matrix (0,3);
	  x[1] = transformation_matrix (1,3);
	  x[2] = transformation_matrix (2,3);
	  x[3] = atan2 (transformation_matrix (2,1), transformation_matrix (2,2));
	  x[4] = asin (-transformation_matrix (2,0));
	  x[5] = atan2 (transformation_matrix (1,0), transformation_matrix (0,0));

	  // Set temporary pointers
	  tmp_src_ = &pt_src;
	  tmp_tgt_ = &pt_tgt;

	  // Optimize using forward-difference approximation LM
	  const Scalar gradient_tol = 1e-2;
	  OptimizationFunctorWithIndices functor(this);
	  BFGS<OptimizationFunctorWithIndices> bfgs (functor);
	  bfgs.parameters.sigma = 0.01;
	  bfgs.parameters.rho = 0.01;
	  bfgs.parameters.tau1 = 9;
	  bfgs.parameters.tau2 = 0.05;
	  bfgs.parameters.tau3 = 0.5;
	  bfgs.parameters.order = 3;

	  int inner_iterations_ = 0;
	  int result = bfgs.minimizeInit (x);
	  result = BFGSSpace::Running;
	  do
	  {
		inner_iterations_++;
		result = bfgs.minimizeOneStep (x);
		if(result)
		{
		  break;
		}
		result = bfgs.testGradient(gradient_tol);
	  } while(result == BFGSSpace::Running && inner_iterations_ < max_inner_iterations_);
	  if(result == BFGSSpace::NoProgress || result == BFGSSpace::Success || inner_iterations_ == max_inner_iterations_)
	  {
		std::cout << "[registration::TransformationEstimationBFGS::estimateRigidTransformation]"<<std::endl;
		std::cout << "BFGS solver finished with exit code "<<result<<std::endl;
		transformation_matrix.setIdentity();
		applyState(transformation_matrix, x);
	  }
	  else
		BOOST_THROW_EXCEPTION ( registration_error{} );
		//PCL_THROW_EXCEPTION(SolverDidntConvergeException, "[pcl::" << getClassName () << "::TransformationEstimationBFGS::estimateRigidTransformation] BFGS solver didn't converge!");
	}
	
	void applyState(Matrix4 &t, const Vector6& x) const
	{
	 // !!! CAUTION Stanford GICP uses the Z Y X euler angles convention
	 Matrix3 R;
	 R = Eigen::AngleAxis<Scalar> (static_cast<Scalar> (x[5]), Vector3::UnitZ ())
	   * Eigen::AngleAxis<Scalar> (static_cast<Scalar> (x[4]), Vector3::UnitY ())
	   * Eigen::AngleAxis<Scalar> (static_cast<Scalar> (x[3]), Vector3::UnitX ());
	 t.template topLeftCorner<3,3> ().matrix () = R * t.template topLeftCorner<3,3> ().matrix ();
	 Vector4 T (static_cast<Scalar> (x[0]), static_cast<Scalar> (x[1]), static_cast<Scalar> (x[2]), 0.0f);
	 t.col (3) += T;
	}

public:

	GICP() : converged_(false), max_inner_iterations_(20), rotation_epsilon_(2e-3), transformation_epsilon_(5e-4)
	{}
	
	bool converged() const  {return converged_;}
	
	template<class CorsEstimator, class CovarianceGen>
	bool compute(CorsEstimator &estimator, Matrix4 transformation, const std::vector<Object::ConstPtr> &src, const std::vector<Object::ConstPtr> &tgt) {
		
		converged_ = false;
		Matrix4 previous_transformation;
		
		while(!converged_) {
			mahalanobis.clear();
			size_t cnt = 0;
			std::vector<int> source_indices, target_indices;
			const Matrix3 R = transformation.template topLeftCorner<3,3>(); //rotational part
			std::vector<Vector4> pt_src, pt_tgt;
			
			for(size_t i=0; i<src.size(); i++) {
				//find correspondences
				std::vector<int> cors_idx;
				estimator.findCorrespondences(src[i], tgt, transformation, cors_idx);
				mahalanobis.resize (mahalanobis.size()+cors_idx.size(), Matrix3::Identity ());
				source_indices.resize(source_indices.size()+cors_idx.size());
				target_indices.resize(target_indices.size()+cors_idx.size());
				
				CovarianceGen cgen1(src[i]);
				const Matrix3 &C1 = cgen1.cov();
				for(size_t j=0; j<cors_idx.size(); j++) {					
					CovarianceGen cgen2(tgt[cors_idx[j]]);
					const Matrix3 &C2 = cgen2.cov();
					Matrix3 &M = mahalanobis[i];
					M = R * C1;
					// temp = M*R' + C2 = R*C1*R' + C2
					Matrix3 temp = M * R.transpose();
					temp+= C2;
					M = temp.inverse ();
					source_indices[cnt] = static_cast<int> (i);
					target_indices[cnt] = cors_idx[j];
					cnt++;
					
					pt_src.push_back(cgen1.pt());
					pt_tgt.push_back(cgen2.pt());
				}
			}
			
			previous_transformation = transformation;
			try
		   {
			 estimateRigidTransformationBFGS(pt_src, pt_tgt, transformation);
			 /* compute the delta from this iteration */
			 Scalar delta = 0.;
			 for(int k = 0; k < 4; k++) {
			   for(int l = 0; l < 4; l++) {
				 Scalar ratio = 1;
				 if(k < 3 && l < 3) // rotation part of the transform
				   ratio = 1./rotation_epsilon_;
				 else
				   ratio = 1./transformation_epsilon_;
				 Scalar c_delta = ratio*fabs(previous_transformation(k,l) - transformation(k,l));
				 if(c_delta > delta)
				   delta = c_delta;
			   }
			 }
		   }
		   catch (...)
		   {
			 std::cout << "[computeTransformation] Optimization issue " << std::endl;
			 return false;
		   }
  
		}
		
		return converged_;
	}
};


template<class TCorrespondenceEstimator, class TCovarianceGen>
class ContextRegistration : public TransformationEstimator {
protected:
	TCorrespondenceEstimator cor_est_;
	nuklei::kernel::se3 initial_guess_;
	
public:
	typedef typename TCorrespondenceEstimator::Scalar Scalar;
	
	ContextRegistration(const nuklei::kernel::se3 &initial_guess) : initial_guess_(initial_guess)
	{}
	
	TCorrespondenceEstimator 		&getCorrespondenceEstimator() 				{return cor_est_;}
	const TCorrespondenceEstimator 	&getCorrespondenceEstimator() const 	{return cor_est_;}

	virtual bool register_scene(const Context::ConstPtr &new_scene, const Context::ConstPtr &map, nuklei::kernel::se3 &tf_out) {
		GICP<Scalar> gicp;
		std::vector<Object::ConstPtr> src, tgt; //TODO: 
		
		typename GICP<Scalar>::Matrix4 transformation; //TODO: 
		return gicp.template compute<TCorrespondenceEstimator, TCovarianceGen>
			(cor_est_, transformation, src, tgt);
	}
};


} //registration
} //cob_3d_geometry_map
