#include <cob_3d_experience_mapping/mapping.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <cob_3d_experience_mapping/visualization/graph.hpp>
#include "../include/ros_node.hpp"
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <gtest/gtest.h>

nav_msgs::Odometry generate_odom(const ros::Time &stamp, const float vel, const float rot, const float vel2=0.f) {
	nav_msgs::Odometry odom;
	odom.header.stamp = stamp;
	odom.twist.twist.linear.x  = vel;
	odom.twist.twist.linear.y  = vel2;
	odom.twist.twist.angular.z = rot;
	return odom;
}

cob_3d_experience_mapping::SensorInfoArray generate_sensor(const int ft_id) {
	cob_3d_experience_mapping::SensorInfoArray sia;
	cob_3d_experience_mapping::SensorInfo si;
	si.id = ft_id;
	sia.infos.push_back(si);
	return sia;
}

TEST(experience_mapping_lemon, init)
{
	ROS_Node<As_Node> sn;
	sn.onInit();
	
	//check init
	EXPECT_EQ(sn.ctxt_.active_states().size(), 1);
	EXPECT_TRUE((bool)sn.ctxt_.current_active_state());
	EXPECT_TRUE((bool)sn.ctxt_.virtual_state());
	EXPECT_EQ(sn.ctxt_.current_active_state()->id(), 1);
}

TEST(experience_mapping_lemon, straight)
{
	ROS_Node<As_Node> sn;
	sn.onInit();
	
	ros::Time time(0);
	std::map<int /*ft id*/, std::vector<int> /*state id*/ > ft2state;
	
	const float noise = 0.025f;
	
	boost::mt19937 rng;
	boost::normal_distribution<float> nd(0.f, noise);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<float> > var_nor(rng, nd);
	
	int num_hops = 0;
	for(int j=0; j<3; j++) {
		for(int i=0; i<100; i++) {
			const int aid = sn.ctxt_.current_active_state()->id();
			bool found = false;
			
			printf("test input %d/%d\n", j,i);
			std::cout<<"expected state id: \t";
			for(size_t k=0; k<ft2state[i].size(); k++) {
				std::cout<<ft2state[i][k]<<" ";
				if(std::abs(ft2state[i][k]-aid)<2) found=true;
			}
			std::cout<<std::endl;
			
			if(j>0) printf("RESULT: %d %d\n", (int)found, j);
			
			EXPECT_TRUE( j==0 || (j>1&&found) || (j==1&&(found||i<50)) );
			EXPECT_GE(sn.ctxt_.current_active_state()->hops(), num_hops);
			if(sn.ctxt_.current_active_state()->hops()<num_hops)
				printf("LOST TRACK\n");
			num_hops = sn.ctxt_.current_active_state()->hops();
			
			ft2state[i].push_back(sn.ctxt_.current_active_state()->id());
			
			sn.on_sensor_info(boost::make_shared<cob_3d_experience_mapping::SensorInfoArray>(generate_sensor(i)));
			sn.on_odom(boost::make_shared<nav_msgs::Odometry>(generate_odom(time, 0.5f + j*var_nor(), 0.3f + (j+1)*var_nor())));
			time += ros::Duration(1);
			
			ft2state[i].push_back(aid);
		}
	}
	
}

TEST(experience_mapping_lemon, random_walk)
{
	ROS_Node<As_Node> sn;
	sn.onInit();
	
	ros::Time time(0);
	std::map<int /*ft id*/, std::vector<int> /*state id*/ > ft2state;
	
	const float noise = 0.025f;
	
	boost::mt19937 rng;
	boost::normal_distribution<float> nd(0.f, noise);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<float> > var_nor(rng, nd);
	
	int num_hops = 0;
	for(int j=0; j<3; j++) {
		srand(0);
		for(int i=0; i<100; i++) {
			const int aid = sn.ctxt_.current_active_state()->id();
			bool found = false;
			
			printf("test input %d/%d\n", j,i);
			std::cout<<"expected state id: \t";
			for(size_t k=0; k<ft2state[i].size(); k++) {
				std::cout<<ft2state[i][k]<<" ";
				if(std::abs(ft2state[i][k]-aid)<2) found=true;
			}
			std::cout<<std::endl;
			
			if(j>0) printf("RESULT: %d %d\n", (int)found, j);
			
			EXPECT_TRUE( j==0 || (j>1&&found) || (j==1&&(found||i<50)) );
			EXPECT_GE(sn.ctxt_.current_active_state()->hops(), num_hops);
			if(sn.ctxt_.current_active_state()->hops()<num_hops)
				printf("LOST TRACK\n");
			num_hops = sn.ctxt_.current_active_state()->hops();
			
			int r = rand();
			
			sn.on_sensor_info(boost::make_shared<cob_3d_experience_mapping::SensorInfoArray>(generate_sensor(i)));
			sn.on_odom(boost::make_shared<nav_msgs::Odometry>(generate_odom(time, 0.5f + j*var_nor(), (r%3-1)*M_PI/2 + j*var_nor(), 0.000001f*i)));
			time += ros::Duration(1);
			
			ft2state[i].push_back(aid);
		}
	}
	
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "exp_mapping_tester", ros::init_options::AnonymousName);
	return RUN_ALL_TESTS();
}
