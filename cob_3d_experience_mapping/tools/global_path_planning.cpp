#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Twist.h>
#include <cob_3d_visualization/simple_marker.h>

#include <ratslam_ros/TopologicalAction.h>
#include <cob_3d_experience_mapping/QueryPath.h>

#include <Eigen/Core>

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/math/distributions/normal.hpp> // for normal_distribution

#include <cob_3d_experience_mapping/SetGoalAction.h>
#include <actionlib/server/simple_action_server.h>

#include "path_probability.h"
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
		Eigen::Vector2d twist_;
		TPos pos_;
		
		State() {}
		State(const TPos &pos) : pos_(pos) {}
		
		double relative_position() const;
	};
	
	struct Observation {
		double prob(Eigen::Vector2d twist) const;
	};
}

typedef double PrecisionType;
typedef Particles::State statetype;
typedef double obsvtype;

const PrecisionType alpha = 0.91;
const PrecisionType beta = 1.0;

//std::normal_distribution<statetype> distribution(0.0, 1.0);

// See https://www.zybuluo.com/zweng/note/219267 for the explanation.
// x1 : X_n
// x2 : X_{n-1}
PrecisionType f(statetype x1, statetype x2) {
  return x1.relative_position()>=x2.relative_position()?1:0;//exp(-0.5 * pow((x1 - alpha * x2), 2));
}

PrecisionType g(statetype x, obsvtype y) {
  return y.prob(x.twist_);//1 / exp(x / 2) * exp(-0.5 * pow(y / beta / exp(x / 2), 2));
}

PrecisionType q(statetype x1, statetype x2, obsvtype y) {
  return 0;//exp(-0.5 * pow((x1 - alpha * x2), 2));
}

//std::default_random_engine generator(seed);
statetype q_sam(statetype x, obsvtype y) {
  return statetype();///*distribution(generator) +*/ alpha * x;
}

class MainNode {
	
	enum {INVALID_NODE_ID=0};
	

	//parameters
	double fequency_;
	double max_vel_, max_rot_;
	double expected_variance_;
	int num_particles_;
	
	ros::NodeHandle nh_;
	ros::Subscriber sub_action_;
	ros::ServiceClient srv_query_path_;
	
	int target_id_, max_node_id_;
	
	//data
	particle_plus_plus::Pfilter<statetype, obsvtype> particle_filter_;
	
	//action servers
	Server_SetGoal server_set_goal_;
	
	void exe_set_goal(const cob_3d_experience_mapping::SetGoalGoalConstPtr &goal) {
		std::vector<std::string> vgoal;
		tokenizeV<std::string>(goal->goal, vgoal);
		
		cob_3d_experience_mapping::SetGoalResult result;
		result.success = false;
		
		if(vgoal.size()!=2) {
			ROS_ERROR("%s", (result.msg = "malformed goal. Example: node:123").c_str());
			server_set_goal_.setSucceeded(result);
		}
		if(vgoal[0]=="node") {
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
		
		return true;
	}
	
	std::deque<Slot> slots_;
	
	void on_odom(const Eigen::Vector2d &odom, const double time)
	{
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

	MainNode() :
		fequency_(10.), max_vel_(0.2), max_rot_(0.2),
		expected_variance_(0.01),
		num_particles_(100),
		target_id_(INVALID_NODE_ID), max_node_id_(0),
		particle_filter_(f, g, q, q_sam),
		server_set_goal_(nh_, "set_goal", boost::bind(&MainNode::exe_set_goal, this, _1), false)
	{
		ros::param::param<double>("fequency", 		fequency_, fequency_);
		ros::param::param<double>("max_vel", 		max_vel_, max_vel_);
		ros::param::param<double>("max_rot", 		max_rot_, max_rot_);
		ros::param::param<double>("expected_variance",	expected_variance_, expected_variance_);
		ros::param::param<int>   ("num_particles",	num_particles_, num_particles_);
		
		srv_query_path_ = nh_.serviceClient<cob_3d_experience_mapping::QueryPath>("query_path");
		sub_action_     = nh_.subscribe("action", 1, &MainNode::cb_action, this);
		
		server_set_goal_.start();

		cob_3d_visualization::RvizMarkerManager::get()
			.createTopic("local_path_planning_marker")
			.setFrameId("/base_link");
			//.clearOld();
	}
	
	double frequency() const {return fequency_;}
	
	void cycle() {
	}
			
	void visualize() {
		cob_3d_visualization::RvizMarkerManager::get().clear();
		
		for(size_t i=0; i<slots_.size(); i++)
		{
			/*cob_3d_visualization::RvizMarker scene;
			scene.sphere(pos);
			scene.color(1-e,e,0.);
			
			cob_3d_visualization::RvizMarker scene;
			scene.arrow(pos, pos_o, 0.03f);
			scene.color(0,0,1,0.5);*/
		}
					
		cob_3d_visualization::RvizMarkerManager::get().publish();
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "local_path_planning");
    
	MainNode mn;
	
	ros::Rate r(mn.frequency());
	while(ros::ok()) {
		mn.cycle();
		mn.visualize();
		ros::spinOnce();
	}
	
	return 0;
}
