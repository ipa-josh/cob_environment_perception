#pragma once

#include <nav_msgs/OccupancyGrid.h>


struct PathProbability {
	double phi_res;
	std::vector<double> possible_paths;
};

PathProbability generatePossiblePaths(const nav_msgs::OccupancyGrid &grid, const double max_phi_speed, const size_t path_resolution, const double max_vel, double &within_prob, const double fact_right = 0.7, const double fact_left = 0.75, const double thr = 10.) {
	PathProbability pp;
	
	//init.
	pp.phi_res = max_phi_speed/path_resolution;
	pp.possible_paths.resize(2*path_resolution+1, 0.);
	within_prob = 0;
	
	printf("-----------------------------------------\n");
	
	//inflated obstacles to path (assuming constant speed)
	for(unsigned int x=0; x<grid.info.width; x++) {
		//const double rx = x*grid.info.resolution;
		const double rx = (x-grid.info.width /2.)*grid.info.resolution;// - grid.info.origin.position.x;
			
		for(unsigned int y=0; y<grid.info.height; y++) {
			int8_t o = grid.data[y*grid.info.width+x];
			if(o<0) o=0;
			const double val = std::min(1., o/thr);
			
			const double ry = (y-grid.info.height/2.)*grid.info.resolution;// - grid.info.origin.position.y;
			//const double ry = y*grid.info.resolution;
			if(rx<=0) continue;
			
			if(std::abs(rx)+std::abs(ry)<0.05) {
				within_prob = std::max(within_prob, val);
				printf("O");
			}
			else
				printf("%c", o>thr?'x':' ');
			
			const double phi1 = std::atan2(ry*max_vel,rx);
			//printf(" %f ", phi1);
			const size_t ind = (size_t)(phi1/pp.phi_res + pp.possible_paths.size()/2);
			if(ind<0 || ind>=pp.possible_paths.size()) continue;
			
			pp.possible_paths[ind] = std::max(pp.possible_paths[ind], val);
		}
		printf("\n");
	}
	
	//spread distances to keep track to mid
	for(size_t i=0; i<pp.possible_paths.size()-1; i++)
		pp.possible_paths[i+1] = std::max(pp.possible_paths[i+1], pp.possible_paths[i]*fact_right);
	for(size_t i=pp.possible_paths.size()-1; i>0; i--)
		pp.possible_paths[i-1] = std::max(pp.possible_paths[i-1], pp.possible_paths[i]*fact_left);
		
	return pp;
}
