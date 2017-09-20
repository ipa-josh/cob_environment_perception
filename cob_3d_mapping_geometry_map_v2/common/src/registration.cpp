#include <cob_3d_mapping_geometry_map_v2/registration/registration.h>

using namespace cob_3d_geometry_map;

//code to test instiation of templates, does nothing actually
void test_registration_impl() {
	Eigen::Affine3f tf(Eigen::Translation3f(0,0,0));
	
	registration::ContextRegistration<
	 registration::CorrespondenceEstimator<float>,
	 registration::CovarianceEstimator<float>
	> reg;
	
	//(cast(tf));
}
