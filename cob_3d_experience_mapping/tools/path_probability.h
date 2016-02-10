#pragma once

#include <deque>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <cob_3d_visualization/simple_marker.h>

#include <boost/random.hpp>
#include <boost/random/triangle_distribution.hpp>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
#include <boost/math/distributions/triangular.hpp>
#include <chrono>
#include <particleplusplus/pfilter.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

struct PathProbability {	
	double max_vel;
	std::vector<double> possible_paths;
	
	PathProbability() {}
	
	PathProbability(const double vel, const size_t res) :
	 max_vel(vel), possible_paths(2*res+1, 0.)
	{
	}
	
	size_t alpha2ind(const double phi) const {
		assert(phi>=-M_PI_2 && phi<=M_PI_2);
		return std::min( (size_t)((phi+M_PI_2)/M_PI * possible_paths.size()), possible_paths.size()-1);
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

PathProbability generatePossiblePaths(const nav_msgs::OccupancyGrid &grid, const std::string fixed_frame_id, const size_t path_resolution, const double max_vel, double &within_prob, const double fact_right = 0.7, const double fact_left = 0.75, const double thr = 10.) {
	PathProbability pp(max_vel, path_resolution);
	
    Eigen::Affine2d trans2d;
    
	static tf::TransformListener listener_(ros::Duration(30.));
	tf::StampedTransform transform;
    try{
      listener_.lookupTransform(grid.header.frame_id, ros::Time(0), //target
                               grid.header.frame_id, grid.header.stamp, //source
                               fixed_frame_id,
                               transform);
                               
		Eigen::Affine3d T;
		tf::transformTFToEigen(transform, T);
		trans2d = Eigen::Translation2d(T.translation().topRows<2>()) *
               T.linear().topLeftCorner<2,2>();
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      
      trans2d = Eigen::Translation2d(0,0);
    }
    
	std::cout<<"trans2d: "<<trans2d.translation().transpose()<<"\n"<<trans2d.rotation()<<std::endl;
	
	//init.
	within_prob = 0;
	
#ifdef DEBUG_OUT_PathProbability
	printf("-----------------------------------------\n");
#endif
	
	//inflated obstacles to path (assuming constant speed)
	for(unsigned int x=0; x<grid.info.width; x++) {
		//const double rx = x*grid.info.resolution;
		Eigen::Vector2d rv;
		rv(0) = (x-grid.info.width /2.)*grid.info.resolution;// - grid.info.origin.position.x;
			
		for(unsigned int y=0; y<grid.info.height; y++) {
			int8_t o = grid.data[y*grid.info.width+x];
			if(o<0) o=0;
			const double val = std::min(1., o/thr);
			
			rv(1) = (y-grid.info.height/2.)*grid.info.resolution;// - grid.info.origin.position.y;
			rv = trans2d*rv;
			
			//const double ry = y*grid.info.resolution;
			if(rv(0)<=0 || val<=0) continue;
			
			if(std::abs(rv(0))+std::abs(rv(1))<0.05) {
				within_prob = std::max(within_prob, val);
#ifdef DEBUG_OUT_PathProbability
				printf("O");
			}
			else
				printf("%c", o>thr?'x':' ');
#else
			}
#endif
				
			const double vel = max_vel * rv(0) / rv.norm();
			
			const double phi1 = std::atan2(rv(1),rv(0)*vel);	
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

struct Slot {
	Eigen::Vector2d twist_;
	
	Slot() {}
	Slot(const double v, const double r) : twist_(v,r) {}
	
	Eigen::Affine2d getT(const double rel=1.) {
		return Eigen::Rotation2D<double>(rel*twist_(1))*Eigen::Translation2d(rel*twist_(0),0);
	}
};

namespace Particles {
	typedef std::deque<Slot> TQueue;
	typedef TQueue::iterator TPos;
	
	struct State {
		double pos_rel_;
		TPos pos_it_;
		
		State() = delete;// : pos_rel_(0) {}
		State(const TPos &pos) : pos_rel_(0), pos_it_(pos) {}
		
		double relative_position() const;
		
		Eigen::Affine2d pose(const TPos &begin) {
			const Eigen::Affine2d T = pos_it_->getT(pos_rel_);
			if(begin==pos_it_) return T;
			return pose(begin, pos_it_-1)*T;
		}
		
		Eigen::Affine2d pose(const TPos &begin, const TPos &last) {
			const Eigen::Affine2d T = pos_it_->getT();
			if(begin==last) return T;
			return pose(begin, last-1)*T;
		}
		
		Eigen::Vector2d _op_min(const TPos &it) const {
			if(it==pos_it_)
				return pos_rel_*pos_it_->twist_;
			return it->twist_+_op_min(it+1);
		}
		
		Eigen::Vector2d operator-(const State &o) const {
			if(o.pos_it_>pos_it_)
				return Eigen::Vector2d(0,0);
			//std::cout<<"op- "<<_op_min(o.pos_it_).transpose()<<" - "<<(o.pos_rel_*o.pos_it_->twist_).transpose()<<std::endl;
			return _op_min(o.pos_it_)-o.pos_rel_*o.pos_it_->twist_;
		}
		
		State moved(double length, TPos end) const {
			State r=*this;
			
			//std::cout<<"moved*: "<<length/pos_it_->twist_.norm()<<" ";
			length += r.pos_rel_*r.pos_it_->twist_.norm();
			//std::cout<<length/pos_it_->twist_.norm()<<std::endl;
			
			while(length*length>=r.pos_it_->twist_.squaredNorm())
			{
				if(r.pos_it_+1==end) {
					r.pos_rel_ = 1;
					return r;
				}
				length -= r.pos_it_->twist_.norm();
				r.pos_it_++;
			}
			r.pos_rel_ = length/r.pos_it_->twist_.norm();
			//std::cout<<"moved: "<<length<<"/"<<r.pos_it_->twist_.norm()<<" "<<r.pos_rel_<<std::endl;
			return r;
		}
			
		geometry_msgs::Twist action(double freq=1) const {
			geometry_msgs::Twist action;
			action.linear.x  = pos_it_->twist_(0)/std::abs(freq);
			action.angular.z = pos_it_->twist_(1)/freq;
			return action;
		}
	};
	
	struct Observation {
		nav_msgs::OccupancyGrid grid_;
		PathProbability prob_;
		double within_prob_;
		
		Observation(const nav_msgs::OccupancyGrid &grid) :
			grid_(grid)
		{
		}
		
		Observation(const PathProbability &prob) :
			prob_(prob), within_prob_(0)
		{
		}
		
		void update(const std::string &fixed_frame) {
			prob_ = generatePossiblePaths(grid_, fixed_frame, 8, 0.5, within_prob_);
		}
		
		double prob(Eigen::Vector2d twist) const;
	};
}

class PathObserver {
public:
	typedef double PrecisionType;
	typedef Particles::State statetype;
	typedef Particles::Observation obsvtype;

protected:
	
	std::deque<Slot> slots_;
	Eigen::Vector2d last_odom_;
	double last_time_delta_, last_time_ts_;
	boost::shared_ptr<Particles::Observation> observation_;
	std::vector<statetype>::iterator best_particle_;
	
	particle_plus_plus::Pfilter<PathObserver, statetype, obsvtype> particle_filter_;
	
	PathObserver() :
		last_time_ts_(-1),
		particle_filter_(this), best_particle_(particle_filter_.end())
	{}
	
public:

	//particle filter
	// See https://www.zybuluo.com/zweng/note/219267 for the explanation.
	// x1 : X_n
	// x2 : X_{n-1}
	PrecisionType state_fn(const statetype &x1, const statetype &x2) {
	  //std::cout<<"dist: "<<((x1-x2)-last_odom_).norm()/last_odom_.norm()<<std::endl;
	  return std::exp( -std::pow( 5*((x1-x2)-last_odom_).norm()/last_odom_.norm(), 2) );//x1.relative_position()>=x2.relative_position()?1:0;//exp(-0.5 * pow((x1 - alpha * x2), 2));
	}

	PrecisionType observe_fn(const statetype &x, const obsvtype &y) {
	  return 1.-0.5*std::pow( y.prob_[y.prob_(last_odom_(0), last_odom_(1))], 2);//y.prob(x.twist_);//1 / exp(x / 2) * exp(-0.5 * pow(y / beta / exp(x / 2), 2));
	}

	PrecisionType proposal_fn(const statetype &x1, const statetype &x2, const obsvtype &y) {
	  return 1;//exp(-0.5 * pow((x1 - alpha * x2), 2));
	}

	boost::mt19937 rng_;
	statetype sampling_fn(const statetype &x, const obsvtype &y) {
	  boost::normal_distribution<double> distribution_normal(0., 0.001);	//TODO: check
	  boost::triangle_distribution<double> distribution_triangular(0.,1.,1.2);
	  
      boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > var_nor(rng_, distribution_normal);
      boost::variate_generator<boost::mt19937&, boost::triangle_distribution<double> > var_tri(rng_, distribution_triangular);
      
      //std::cout<<"rand "<<(var_tri() + var_nor()*last_time_delta_)<<std::endl;
	  
	  return x.moved( (var_tri() + var_nor()*last_time_delta_) * last_odom_.norm(), slots_.end() );///*distribution(generator) +*/ alpha * x;
	}

};
