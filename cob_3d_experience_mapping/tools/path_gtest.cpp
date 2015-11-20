
#include "path_probability.h"
#include <gtest/gtest.h>

TEST(path_probability, index_from_and_to)
{
	PathProbability pp(0.5, 0.5, 32);
	
	for(size_t i=0; i<10000; i++) {
		size_t ind = rand()%pp.possible_paths.size();
		
		double phi = pp.angular(ind);
		double vel = pp.velocity(ind);
		
		EXPECT_EQ(pp(vel, phi), ind);
		
		/*std::cout<<"index in:  "<<ind<<std::endl;
		std::cout<<"alpha:     "<<pp.ind2alpha(ind)<<std::endl;
		std::cout<<"index out: "<<pp.alpha2ind(pp.ind2alpha(ind))<<std::endl;*/
		
		EXPECT_EQ(pp.alpha2ind(pp.ind2alpha(ind)), ind);
	}
	
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
