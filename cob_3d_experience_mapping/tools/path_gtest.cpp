
#include "path_probability.h"
#include <gtest/gtest.h>

class PathObserverTester : public PathObserver
{
public:
	PathObserverTester(const size_t num_steps, const int num_particles) {
		slots_.resize(num_steps);
		for(size_t i=0; i<slots_.size(); i++)
			slots_[i] = Slot((rand()%1000)/1000., (rand()%1000-500)/1000.);
			
		observation_.reset(new Particles::Observation(PathProbability(0.5, 0.5, 8)));
			
		particle_filter_.initialize(num_particles, Particles::State(slots_.begin()));
	}
	
	bool step(size_t step) {
		last_odom_ = slots_[step/10].twist_/10;
		last_time_delta_ = 1;
		
		std::cout<<"odom: "<<last_odom_.transpose()<<std::endl;
		
		best_particle_ = particle_filter_.iterate(*observation_);
		
		std::cout<<"best particles: "<<(best_particle_->pos_it_-slots_.begin())<<" "<<best_particle_->pos_rel_<<std::endl;
		
		return false;
	}
};

TEST(path_probability, particles)
{
	PathObserverTester particle_filter(10, 1000);
	
	for(size_t i=0; i<100; i++)
		particle_filter.step(i);
}

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
