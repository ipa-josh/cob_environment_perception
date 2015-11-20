#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>
#include <cob_3d_visualization/simple_marker.h>
#include "path_probability.h"

class MainNode {

	//parameters
	double max_phi_speed_;
	int path_resolution_;
	double fact_right_;
	double fact_left_;
	double occ_thr_;
	
	ros::NodeHandle nh_;	
	nav_msgs::OccupancyGrid grid_;
	
	ros::Subscriber sub_req_, sub_costmap_;
	ros::Publisher pub_odom_, pub_feature_;
	
	ros::Timer timer_explore_;
	
	boost::mutex mtx_;
	
public:

	MainNode() :
		max_phi_speed_(0.7), path_resolution_(25),
		fact_right_(0.7), fact_left_(0.75), occ_thr_(30.)
	{
		ros::param::param<double>("max_phi_speed", 		max_phi_speed_, max_phi_speed_);
		ros::param::param<double>("fact_right", 		fact_right_, fact_right_);
		ros::param::param<double>("fact_left", 			fact_left_, fact_left_);
		ros::param::param<double>("occ_thr", 			occ_thr_, occ_thr_);
		ros::param::param<int>   ("path_resolution", 	path_resolution_, path_resolution_);
		
		//subscribe to desired odom. message (->action of robot)
		sub_req_     = nh_.subscribe("desired_odom", 1, &MainNode::cb_desired_odom,   this);
		sub_costmap_ = nh_.subscribe("costmap",      1, &MainNode::on_costmap_update, this);
		
		pub_odom_    = nh_.advertise<geometry_msgs::Twist>("action", 5);
		pub_feature_ = nh_.advertise<std_msgs::Float32MultiArray>("feature", 5);
		
		timer_explore_ = nh_.createTimer(ros::Duration(1.), &MainNode::explore, this);
	}
	
	void on_costmap_update(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
		boost::unique_lock<boost::mutex> scoped_lock(mtx_);
		grid_ = *msg;
		
		cob_3d_visualization::RvizMarkerManager::get().setFrameId(msg->header.frame_id);
		
		calc_feature();
	}
	
	void calc_feature() {
		const double vel = 0.3;
		
		//boost::unique_lock<boost::mutex> scoped_lock(mtx_);
		double within_prob;
		PathProbability pp = generatePossiblePaths(grid_, max_phi_speed_, path_resolution_, vel, within_prob, fact_right_, fact_left_, occ_thr_);
		
		std_msgs::Float32MultiArray msg;
		msg.layout.dim.resize(1);
		msg.layout.data_offset = 0;
		msg.layout.dim[0].size = pp.possible_paths.size();
		msg.layout.dim[0].label= "costs per angle";
		
		for(size_t i=0; i<pp.possible_paths.size(); i++)
			msg.data.push_back(pp.possible_paths[i]);
		
		pub_feature_.publish(msg);
	}
	
	ros::Time sleep_;
	void explore(const ros::TimerEvent& event) {
		if(ros::Time::now()<sleep_) return;
		
		const double vel = 0.2;
		const double interval = 10;
		
		boost::unique_lock<boost::mutex> scoped_lock(mtx_);
		double within_prob;
		PathProbability pp = generatePossiblePaths(grid_, max_phi_speed_, path_resolution_, vel, within_prob, fact_right_, fact_left_, occ_thr_);
			
		cob_3d_visualization::RvizMarkerManager::get().clear();
		{
			cob_3d_visualization::RvizMarker scene;
			std::vector<std_msgs::ColorRGBA> colors(pp.possible_paths.size());
			for(size_t i=0; i<colors.size(); i++) {
				colors[i].r = pp.possible_paths[i];
				colors[i].g = 1-pp.possible_paths[i];
				colors[i].b = 0;
				colors[i].a = 1;
			}
			scene.bar_radial(2*vel, Eigen::Vector3f::Zero(), colors, -max_phi_speed_, max_phi_speed_, vel);
		}
		cob_3d_visualization::RvizMarkerManager::get().publish();
		
		geometry_msgs::Twist action;
		
		size_t ind = pp.possible_paths.size();
		double low=0, high=0;
		for(size_t i=0; i<pp.possible_paths.size(); i++) {
			if(pp.possible_paths[i]>=1) continue;
			
			pp.possible_paths[i] += 0.1*std::abs((int)i-(int)pp.possible_paths.size()/2)/(double)pp.possible_paths.size();
			pp.possible_paths[i] *= std::sin(M_PI*( i/(double)pp.possible_paths.size() + ros::Time::now().toSec()/interval ))*0.9 + 0.1;
			if(ind>=pp.possible_paths.size() || pp.possible_paths[i]<pp.possible_paths[ind])
				ind = i;
				
			if(i<pp.possible_paths.size()/2) low *= pp.possible_paths[i];
			else if(i>pp.possible_paths.size()/2) high *= pp.possible_paths[i];
		}
		
		if(ind<pp.possible_paths.size()) {
			action.angular.z = pp.angular(ind);
			if(within_prob<1) action.linear.x  = pp.velocity(ind);
			else if(!action.angular.z) action.angular.z = low<high?max_phi_speed_/2 : -max_phi_speed_/2;
		}
		else {
			if(within_prob>=1) { //we are in an obstacle !
				action.linear.x  = -vel/2; //move back
			}
			else
				action.angular.z = max_phi_speed_/2;
		}
		
		if(!action.linear.x)
			sleep_ = ros::Time::now() + ros::Duration(4.);
			
		action.linear.x  /= 1+3*std::abs(action.angular.z);
		action.angular.z /= 1;
		pub_odom_.publish(action);
	}

	void cb_desired_odom(const nav_msgs::Odometry::ConstPtr &msg) {
		ROS_INFO("cb_desired_odom");
		
		geometry_msgs::Twist action = msg->twist.twist;
		if(msg->twist.twist.linear.x!=0) { //non-special case
			boost::unique_lock<boost::mutex> scoped_lock(mtx_);
			
			PathProbability pp = generatePossiblePaths(grid_, max_phi_speed_, path_resolution_, msg->twist.twist.linear.x, fact_right_, fact_left_, occ_thr_);
 
			const size_t org_ind = pp(msg->twist.twist.linear.x, msg->twist.twist.angular.z);
			if(org_ind<0 || org_ind>=pp.possible_paths.size()) {
				ROS_ERROR("desired rotation speed is not possible, perhaps wrong configuration?");
				return;
			}
			
			if(pp.possible_paths[org_ind]>=1) {
				ROS_ERROR("path is block, waiting...");
				return;
			}
			
			size_t ind = org_ind;
			
			//search for local minima
			while(ind+1<pp.possible_paths.size() && pp.possible_paths[ind]>pp.possible_paths[ind+1]+(ind-org_ind)*(1-fact_right_) )
				++ind;
			while(ind<=org_ind && ind>0          && pp.possible_paths[ind]>pp.possible_paths[ind-1]+(org_ind-ind)*(1-fact_left_) )
				--ind;
			
			action.angular.z = pp.angular(ind);
			action.linear.x  = pp.velocity(ind);
		}
		
		pub_odom_.publish(action);
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "local_path_planning");
    
    cob_3d_visualization::RvizMarkerManager::get().createTopic("local_path_planning_marker").clearOld();
	MainNode mn;
	
	ros::spin();
	
	return 0;
}
