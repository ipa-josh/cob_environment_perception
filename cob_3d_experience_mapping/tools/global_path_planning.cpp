#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Twist.h>
#include <cob_3d_visualization/simple_marker.h>

#include <ratslam_ros/TopologicalAction.h>
#include <cob_3d_experience_mapping/QueryPath.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/random.hpp>
#include <boost/random/triangle_distribution.hpp>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
#include <boost/math/distributions/triangular.hpp>

#include <cob_3d_experience_mapping/SetGoalAction.h>
#include <actionlib/server/simple_action_server.h>

#include "path_probability.h"
#include <chrono>
#include <particleplusplus/pfilter.h>

typedef actionlib::SimpleActionServer<cob_3d_experience_mapping::SetGoalAction> Server_SetGoal;

/**
 * parse string
 */
template<class T>
void tokenizeV(const std::string &s, std::vector<T> &o, const char escape=':')
{
  typedef boost::tokenizer<boost::escaped_list_separator<char> >  tok_t;

  boost::escaped_list_separator<char> els(escape);
  tok_t tok(s, els);
  
  for(tok_t::iterator j (tok.begin());
      j != tok.end();
      ++j)
  {
    std::string f(*j);
    boost::trim(f);
    o.push_back(boost::lexical_cast<T>(f));
  }
}

	
struct Slot {
	Eigen::Vector2d twist_;
	
	Slot() {}
	Slot(const double v, const double r) : twist_(v,r) {}
	
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
			const Eigen::Affine2d T = Eigen::Translation2d(pos_rel_*pos_it_->twist_(0),0)*Eigen::Rotation2D<double>(pos_rel_*pos_it_->twist_(1));
			if(begin==pos_it_) return T;
			return pose(begin, pos_it_-1)*T;
		}
		
		Eigen::Affine2d pose(const TPos &begin, const TPos &last) {
			const Eigen::Affine2d T = Eigen::Translation2d(pos_it_->twist_(0),0)*Eigen::Rotation2D<double>(pos_it_->twist_(1));
			if(begin==last) return T;
			return pose(begin, last-1)*T;
		}
		
		Eigen::Vector2d _op_min(const TPos &it) const {
			if(it==pos_it_)
				return pos_rel_*pos_it_->twist_;
			return pos_it_->twist_+_op_min(it+1);
		}
		
		Eigen::Vector2d operator-(const State &o) const {
			if(o.pos_it_>pos_it_)
				return Eigen::Vector2d(0,0);
			return _op_min(o.pos_it_)-o.pos_rel_*o.pos_it_->twist_;
		}
		
		State moved(double length, TPos end) const {
			State r=*this;
			length += r.pos_rel_*r.pos_it_->twist_.norm();
			
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
			std::cout<<"moved: "<<length<<"/"<<r.pos_it_->twist_.norm()<<std::endl;
			return r;
		}
			
		geometry_msgs::Twist action(double freq=1) const {
			geometry_msgs::Twist action;
			action.linear.x  = pos_it_->twist_(0)/freq;
			action.angular.z = pos_it_->twist_(1)/freq;
			return action;
		}
	};
	
	struct Observation {
		PathProbability prob_;
		double within_prob_;
		
		Observation(const nav_msgs::OccupancyGrid &grid) :
			prob_(generatePossiblePaths(grid, 0.5, 8, 0.5, within_prob_))
		{
			prob_.visualize(0.5);
		}
		
		double prob(Eigen::Vector2d twist) const;
	};
}

typedef double PrecisionType;
typedef Particles::State statetype;
typedef Particles::Observation obsvtype;

const PrecisionType alpha = 0.91;
const PrecisionType beta = 1.0;

//std::normal_distribution<statetype> distribution(0.0, 1.0);


class MainNode {
	
public:
	//particle filter
	// See https://www.zybuluo.com/zweng/note/219267 for the explanation.
	// x1 : X_n
	// x2 : X_{n-1}
	PrecisionType state_fn(const statetype &x1, const statetype &x2) {
		//last_odom_
	  return std::exp( -((x1-x2)-last_odom_).norm() );//x1.relative_position()>=x2.relative_position()?1:0;//exp(-0.5 * pow((x1 - alpha * x2), 2));
	}

	PrecisionType observe_fn(const statetype &x, const obsvtype &y) {
	  return 1.-0.9*y.prob_[y.prob_(last_odom_(0), last_odom_(1))];//y.prob(x.twist_);//1 / exp(x / 2) * exp(-0.5 * pow(y / beta / exp(x / 2), 2));
	}

	PrecisionType proposal_fn(const statetype &x1, const statetype &x2, const obsvtype &y) {
	  return 1;//exp(-0.5 * pow((x1 - alpha * x2), 2));
	}

	boost::mt19937 rng_;
	statetype sampling_fn(const statetype &x, const obsvtype &y) {
	  boost::normal_distribution<double> distribution_normal(0., 0.001);	//TODO: check
	  boost::triangle_distribution<double> distribution_triangular(0.,1.,1.1);
	  
      boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > var_nor(rng_, distribution_normal);
      boost::variate_generator<boost::mt19937&, boost::triangle_distribution<double> > var_tri(rng_, distribution_triangular);
	  
	  return x.moved( (var_tri() + var_nor()*last_time_delta_) * last_odom_.norm(), slots_.end() );///*distribution(generator) +*/ alpha * x;
	}
	
private:
	
	enum {INVALID_NODE_ID=0};
	

	//parameters
	double fequency_;
	double max_vel_, max_rot_;
	double expected_variance_;
	int num_particles_;
	
	ros::NodeHandle nh_;
	ros::Subscriber sub_action_, sub_costmap_, sub_odom_;
	ros::Publisher pub_odom_;
	ros::ServiceClient srv_query_path_;
	
	int target_id_, max_node_id_;
	
	//data
	particle_plus_plus::Pfilter<MainNode, statetype, obsvtype> particle_filter_;
	
	//action servers
	Server_SetGoal server_set_goal_;
	
	boost::mutex mtx_;
	
	void exe_set_goal(const cob_3d_experience_mapping::SetGoalGoalConstPtr &goal) {
		std::vector<std::string> vgoal;
		//tokenizeV<std::string>(goal->goal, vgoal);
		vgoal.push_back("node");
		vgoal.push_back("1");
		
		
		cob_3d_experience_mapping::SetGoalResult result;
		result.success = false;
		
		if(vgoal.size()!=2) {
			ROS_ERROR("%s", (result.msg = "malformed goal. Example: node:123").c_str());
			server_set_goal_.setSucceeded(result);
		}
		if(vgoal[0]!="node") {
			ROS_ERROR("%s", (result.msg = "malformed goal. Example: node:123").c_str());
			server_set_goal_.setSucceeded(result);
		}
		
		target_id_ = boost::lexical_cast<int>(vgoal[1]);
		
		query_path();
		
	}
	
	bool query_path() {
		assert(target_id_!=INVALID_NODE_ID);
		
		if(target_id_==INVALID_NODE_ID) {
			ROS_WARN("invalid target id set");
			return false;
		}
		
		boost::unique_lock<boost::mutex> scoped_lock(mtx_);
		
		cob_3d_experience_mapping::QueryPath srv;
		srv.request.node_id = target_id_;
		
		if(!srv_query_path_.call(srv)) {
			ROS_ERROR("failed to call query path service");
			return false;
		}
		
		if(srv.response.invert) {
			generate_path(srv.response.actions.rbegin(), srv.response.actions.rend(), -1);
			slots_.insert(slots_.begin(), Slot(0, M_PI));
		}
		else
			generate_path(srv.response.actions.begin(), srv.response.actions.end(), 1);
			
		//init. particle filter
		particle_filter_.initialize(num_particles_, Particles::State(slots_.begin()));
		best_particle_=particle_filter_.end();
		
		return true;
	}
	
	std::deque<Slot> slots_;
	Eigen::Vector2d last_odom_;
	double last_time_delta_, last_time_ts_;
	boost::shared_ptr<Particles::Observation> observation_;
	std::vector<statetype>::iterator best_particle_;
	
	void on_odom(Eigen::Vector2d odom, const double time)
	{
		odom*=time;
		if(odom(1)<-M_PI_2) odom(1) = -M_PI_2;
		else if(odom(1)>=M_PI_2) odom(1) = M_PI_2-0.00001;
		
		boost::unique_lock<boost::mutex> scoped_lock(mtx_);
		
		last_odom_ = odom;
		last_time_delta_ = time;
		
		std::cout<<"odom: "<<last_odom_.transpose()<<std::endl;
		
		if(observation_ && target_id_!=INVALID_NODE_ID) {
			best_particle_ = particle_filter_.iterate(*observation_);
			pub_odom_.publish(best_particle_->action(fequency_));
		}
	}
	
	template<class Iterator>
	void generate_path(const Iterator &begin, const Iterator &end, const float factor=1.f)
	{
		slots_.resize(end-begin);
		
		size_t pos=0;
		for(Iterator it=begin; it!=end; it++) {
			const double vel = factor*it->linear.x;
			const  double rot = factor*it->angular.z;
			
			slots_[pos++] = Slot(vel, rot);
		}
	}
	
	void cb_action(const ratslam_ros::TopologicalActionConstPtr &msg) {
		const int id = msg->dest_id+1;
		const bool reloc = id<max_node_id_;
		max_node_id_ = std::max(id, max_node_id_);
		
		if(target_id_ == INVALID_NODE_ID) return;
		
		if(reloc)
			query_path();
	}
	
public:

	void test() {
		slots_.resize(10);
		for(size_t i=0; i<slots_.size(); i++)
			slots_[i] = Slot((rand()%1000)/1000., (rand()%1000-500)/1000.);
			
		particle_filter_.initialize(num_particles_, Particles::State(slots_.begin()));
		best_particle_=particle_filter_.end();
		
		observation_.reset(new Particles::Observation(nav_msgs::OccupancyGrid()) );
		
		on_odom( Eigen::Vector2d((rand()%1000)/1000., (rand()%1000-500)/1000.), 0.5 );
		
		ROS_INFO("test passed");
	}

	MainNode() :
		fequency_(10.), max_vel_(0.2), max_rot_(0.2),
		expected_variance_(0.01),
		num_particles_(100),
		target_id_(INVALID_NODE_ID), max_node_id_(0),
		particle_filter_(this), best_particle_(particle_filter_.end()),
		server_set_goal_(nh_, "set_goal", boost::bind(&MainNode::exe_set_goal, this, _1), false),
		last_time_ts_(-1)
	{
		ros::param::param<double>("fequency", 		fequency_, fequency_);
		ros::param::param<double>("max_vel", 		max_vel_, max_vel_);
		ros::param::param<double>("max_rot", 		max_rot_, max_rot_);
		ros::param::param<double>("expected_variance",	expected_variance_, expected_variance_);
		ros::param::param<int>   ("num_particles",	num_particles_, num_particles_);
		
		srv_query_path_ = nh_.serviceClient<cob_3d_experience_mapping::QueryPath>("/query_path");
		sub_action_     = nh_.subscribe("action", 1, &MainNode::cb_action, this);
		sub_costmap_ = nh_.subscribe("costmap",      1, &MainNode::on_costmap_update, this);
		sub_odom_     = nh_.subscribe("odom", 1, &MainNode::cb_odom,   this);
		
		pub_odom_    = nh_.advertise<geometry_msgs::Twist>("desired_odom", 5);
		
		server_set_goal_.start();

		cob_3d_visualization::RvizMarkerManager::get()
			.createTopic("global_path_planning_marker")
			.setFrameId("/base_link");
			//.clearOld();
			
		//test();
	}
	
	void cb_odom(const nav_msgs::Odometry::ConstPtr &msg) {
		if(last_time_ts_>0)
			on_odom(
				Eigen::Vector2d(msg->twist.twist.linear.x, msg->twist.twist.angular.z),
				msg->header.stamp.toSec()-last_time_ts_
			);
		
		last_time_ts_ = msg->header.stamp.toSec();
	}
	
	void on_costmap_update(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
		cob_3d_visualization::RvizMarkerManager::get().setFrameId(msg->header.frame_id);
		
		if(target_id_==INVALID_NODE_ID) return;
		
		boost::unique_lock<boost::mutex> scoped_lock(mtx_);
		observation_.reset(new Particles::Observation(*msg) );
	}
	
	double frequency() const {return fequency_;}
			
	void visualize() {
		cob_3d_visualization::RvizMarkerManager::get().clear();
		
		if(best_particle_!=particle_filter_.end()) {
			boost::unique_lock<boost::mutex> scoped_lock(mtx_);
			
			Eigen::Affine2d relation = best_particle_->pose(slots_.begin());
			 
			for(std::vector<statetype>::iterator it=particle_filter_.begin(); it!=particle_filter_.end(); it++)
			{
				const double e = particle_filter_.weight(it-particle_filter_.begin());
				Eigen::Vector3d v3(0,0,0);
				v3.head<2>() = relation*it->pose(slots_.begin()).translation();
				
				std::cout<<"particle("<<e<<"): "<<v3.head<2>().transpose()<<std::endl;
				
				cob_3d_visualization::RvizMarker scene;
				scene.sphere(v3);
				scene.color(1-e,e,0.);
				
				/*cob_3d_visualization::RvizMarker scene;
				scene.arrow(pos, pos_o, 0.03f);
				scene.color(0,0,1,0.5);*/
			}
		}
					
		cob_3d_visualization::RvizMarkerManager::get().publish();
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "local_path_planning");
    
	MainNode mn;
	
	ros::Rate r(mn.frequency());
	while(ros::ok()) {
		mn.visualize();
		ros::spinOnce();
	}
	
	return 0;
}
