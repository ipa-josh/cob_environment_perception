#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <boost/thread/mutex.hpp>

#include <ratslam_ros/TopologicalAction.h>
#include <cob_3d_experience_mapping/QueryPath.h>

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <cob_3d_experience_mapping/SetGoalAction.h>
#include <actionlib/server/simple_action_server.h>

#include "path_probability.h"

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


class MainNode : public PathObserver{
	
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
		//srv.response = test_generate_random_path();
		
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
	
	cob_3d_experience_mapping::QueryPath::Response test_generate_random_path() {
		cob_3d_experience_mapping::QueryPath::Response r;
		r.invert = true;
		r.actions.resize(10);
		for(size_t i=0; i<r.actions.size(); i++) {
			r.actions[i].linear.x = (rand()%1001)/1000.*0.5;
			r.actions[i].angular.z = (rand()%1001)/1000.-0.5;
		}
		return r;
	}

	MainNode() :
		fequency_(10.), max_vel_(0.2), max_rot_(0.2),
		expected_variance_(0.01),
		num_particles_(100),
		target_id_(INVALID_NODE_ID), max_node_id_(0),
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
			
			Eigen::Affine2d relation = best_particle_->pose(slots_.begin()).inverse();
			Eigen::Affine2d pos = relation;
			
			for(size_t i=0; i<slots_.size(); i++) {
				Eigen::Vector3d v3_1(0,0,0), v3_2(0,0,0);
				v3_1.head<2>() = pos.translation();
				pos = slots_[i].getT()*pos;
				v3_2.head<2>() = pos.translation();
				
				cob_3d_visualization::RvizMarker scene;
				scene.arrow(v3_1, v3_2, 0.03f);
				scene.color(0,0,1,0.5);
			}
			
			std::cout<<"particle("<<particle_filter_.weight(best_particle_-particle_filter_.begin())<<"): "<<relation.translation().head<2>().transpose()<<std::endl;
			 
			for(std::vector<statetype>::iterator it=particle_filter_.begin(); it!=particle_filter_.end(); it++)
			{
				const double e = particle_filter_.weight(it-particle_filter_.begin());
				Eigen::Vector3d v3(0,0,0);
				v3.head<2>() = relation*it->pose(slots_.begin()).translation();
				
				//std::cout<<"particle("<<e<<"): "<<v3.head<2>().transpose()<<std::endl;
				
				cob_3d_visualization::RvizMarker scene;
				scene.sphere(v3);
				scene.color(1-e,e,0.);
			}
		}
		
		if(observation_)
			observation_->prob_.visualize_partly(0.5);
					
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
		r.sleep();
	}
	
	return 0;
}
