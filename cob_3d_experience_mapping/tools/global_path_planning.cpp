#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Twist.h>

#include <ratslam_ros/TopologicalAction.h>
#include <cob_3d_experience_mapping/QueryPath.h>

#include <Eigen/Core>

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/random/poisson_distribution.hpp>

#include <cob_3d_experience_mapping/SetGoalAction.h>
#include <actionlib/server/simple_action_server.h>

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

class MainNode {
	
	enum {INVALID_NODE_ID=0};
	

	//parameters
	double fequency_;
	double max_vel_, max_rot_;
	
	ros::NodeHandle nh_;
	ros::Subscriber sub_action_;
	ros::ServiceClient srv_query_path_;
	
	int target_id_, max_node_id_;
	
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
		
		if(srv.response.invert)
			generate_path(srv.response.actions.rbegin(), srv.response.actions.rend(), -1);
		else
			generate_path(srv.response.actions.begin(), srv.response.actions.end(), 1);
		
		return true;
	}
	
	struct Slot {
		Eigen::Vector2d twist_;
		
		Slot(const double v, const double r) : twist_(v,r) {}
		
		inline double sim(const Eigen::Vector2d &odom) const {return 1.;}
	};
	
	std::vector<Slot> slots_;
	Eigen::VectorXd probs_;
	
	static void cross_correlation(Eigen::VectorXd in, const Eigen::VectorXd &p, Eigen::VectorXd &out, const size_t offset)
	{
		out.fill(0);
		if(in.rows()<p.size()-off) return;
		
		for(size_t i=off; i<in.rows()-p.size()+off; i++) {
			for(size_t j=0; j<p.size(); j++)
				out(i)+=in(i+j-off)*p(j);
		}
	}
	
	Eigen::VectorXd generate_dist() const {
		Eigen::VectorXd dist;
		//1. create triangular distribution from travelling distance (a=0, b=c=similiar distance)
		//2. cross-correlation with normal distribution with variance = (default odom var. + unsimiliar distance)
		return dist;
	}
	
	void init_prob() {
		//TODO: 
	}
	
	template<class Iterator>
	void generate_path(const Iterator &begin, const Iterator &end, const float factor=1.f)
	{
		slots_.clear();
		
		for(Iterator it=begin; it!=end; it++) {
			double vel = factor*it->linear.x;
			double rot = factor*it->angular.z;
			
			double dur = std::max(std::abs(vel)/max_vel_, std::abs(rot)/max_rot_);
			int slots = (int)(dur/fequency_)+1;
			
			slots_.insert(slots_.end(), slots, Slot(vel/slots, rot/slots));
		}
		
		probs_.setZero(slots_.size());
		init_prob();
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
		target_id_(INVALID_NODE_ID), max_node_id_(0),
		server_set_goal_(nh_, "set_goal", boost::bind(&MainNode::exe_set_goal, this, _1), false)
	{
		ros::param::param<double>("fequency", 		fequency_, fequency_);
		ros::param::param<double>("max_vel", 		max_vel_, max_vel_);
		ros::param::param<double>("max_rot", 		max_rot_, max_rot_);
		
		boost::poisson_distribution<int> pdist(1);
		
		srv_query_path_ = nh_.serviceClient<cob_3d_experience_mapping::QueryPath>("query_path");
		sub_action_     = nh_.subscribe("action", 1, &MainNode::cb_action, this);
		
		server_set_goal_.start();
	}
	
	double frequency() const {return fequency_;}
	
	void cycle() {
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "local_path_planning");
    
	MainNode mn;
	
	ros::Rate r(mn.frequency());
	while(ros::ok()) {
		mn.cycle();
		ros::spinOnce();
	}
	
	return 0;
}
