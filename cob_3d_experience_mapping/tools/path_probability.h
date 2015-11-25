#pragma once

#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>
#include <cob_3d_visualization/simple_marker.h>


struct PathProbability {
	double max_phi_speed, max_vel;
	std::vector<double> possible_paths;
	
	PathProbability(const double phi, const double vel, const size_t res) :
	max_phi_speed(phi), max_vel(vel), possible_paths(2*res+1, 0.)
	{
	}
	
	size_t alpha2ind(const double phi) const {
		assert(phi>=-M_PI_2 && phi<M_PI_2);
		return (phi+M_PI_2)/M_PI * possible_paths.size();
	}
	
	double ind2alpha(const size_t ind) const {
		assert(ind>=0 && ind<possible_paths.size());
		return (ind+0.5)*M_PI/possible_paths.size() - M_PI_2;
	}
	
	double &operator()(const double phi) {return possible_paths[alpha2ind(phi)];}
	double &operator[](const size_t ind) {return possible_paths[ind];}
	double operator()(const double phi) const {return possible_paths[alpha2ind(phi)];}
	double operator[](const size_t ind) const {return possible_paths[ind];}
	
	size_t operator()(const double vel, double phi) const {
		phi = phi*velocity(alpha2ind(phi))/vel;
		if(phi<-M_PI_2) phi = -M_PI_2;
		else if(phi>=M_PI_2) phi = M_PI_2-0.00001;
		return alpha2ind(phi);
	}
	
	double angular(const size_t ind) const {
		assert(ind>=0 && ind<possible_paths.size());
		return ind2alpha(ind);
	}
	
	double velocity(const size_t ind) const {
		assert(ind>=0 && ind<possible_paths.size());
		return std::cos(ind2alpha(ind))*max_vel;
	}
	
	void visualize(const double vel=1) const {
		cob_3d_visualization::RvizMarkerManager::get().clear();
		visualize_partly(vel);
		cob_3d_visualization::RvizMarkerManager::get().publish();
	}
	
	void visualize_partly(const double vel=1) const {
		cob_3d_visualization::RvizMarker scene;
		std::vector<std_msgs::ColorRGBA> colors(possible_paths.size());
		for(size_t i=0; i<colors.size(); i++) {
			colors[i].r = possible_paths[i];
			colors[i].g = 1-possible_paths[i];
			colors[i].b = 0;
			colors[i].a = 1;
		}
		scene.bar_radial(2*vel, Eigen::Vector3f::Zero(), colors, -M_PI_2, M_PI_2, vel);
	}
	
};

//#define DEBUG_OUT_PathProbability

PathProbability generatePossiblePaths(const nav_msgs::OccupancyGrid &grid, const double max_phi_speed, const size_t path_resolution, const double max_vel, double &within_prob, const double fact_right = 0.7, const double fact_left = 0.75, const double thr = 10.) {
	PathProbability pp(max_phi_speed, max_vel, path_resolution);
	
	//init.
	within_prob = 0;
	
#ifdef DEBUG_OUT_PathProbability
	printf("-----------------------------------------\n");
#endif
	
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
			if(rx<=0 || val<=0) continue;
			
			if(std::abs(rx)+std::abs(ry)<0.05) {
				within_prob = std::max(within_prob, val);
#ifdef DEBUG_OUT_PathProbability
				printf("O");
			}
			else
				printf("%c", o>thr?'x':' ');
#else
			}
#endif
				
			const double vel = max_vel * rx / std::sqrt(rx*rx+ry*ry);
			
			const double phi1 = std::atan2(ry,rx*vel);	
			//printf("%f/%f: \t%f %d\n", rx, ry, phi1, (int)pp.alpha2ind(phi1));		
			pp(phi1) = std::max(pp(phi1), val);
		}
#ifdef DEBUG_OUT_PathProbability
		printf("\n");
#endif
	}
	
	//spread distances to keep track to mid
	for(size_t i=0; i<pp.possible_paths.size()-1; i++)
		pp.possible_paths[i+1] = std::max(pp.possible_paths[i+1], pp.possible_paths[i]*fact_right);
	for(size_t i=pp.possible_paths.size()-1; i>0; i--)
		pp.possible_paths[i-1] = std::max(pp.possible_paths[i-1], pp.possible_paths[i]*fact_left);
		
	return pp;
}
