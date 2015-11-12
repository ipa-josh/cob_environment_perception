#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cob_3d_experience_mapping/SensorInfoArray.h>


class MainNode {	
	ros::NodeHandle nh_;	
	
	int feature_num_, feature_depth_; // in bits
	
	ros::Subscriber sub_scan_;
	ros::Publisher pub_feature_;
	
public:

	MainNode() : feature_num_(8), feature_depth_(2)
	{
		ros::param::param<int>   ("feature_depth", 	feature_depth_, feature_depth_);
		ros::param::param<int>   ("feature_num", 	feature_num_, feature_num_);
		
		if(feature_num_*feature_depth_>32) {
			ROS_ERROR("complete feature size has to be less or equal 32 bit. Will stop now!");
			return;
		}
		
		sub_scan_     = nh_.subscribe("scan", 1, &MainNode::cb_scan,   this);
		pub_feature_ = nh_.advertise<cob_3d_experience_mapping::SensorInfoArray>("feature", 5);
	}
	
	void cb_scan(const sensor_msgs::LaserScan::ConstPtr &msg) {
		cob_3d_experience_mapping::SensorInfoArray ft;
		cob_3d_experience_mapping::SensorInfo id; id.id = 0;
		
		const int inc = (int)msg->ranges.size()/feature_num_;
		for(int i=0; i<feature_num_; i++) {
			float avg_dist=0;
			int num=0;
			for(int j=i*inc; j<=(i+1)*inc; j++) {
				if(msg->ranges[j]!=msg->ranges[j] || std::isinf(msg->ranges[j])) continue;
				
				assert(msg->ranges[j]>=msg->range_min && msg->ranges[j]<=msg->range_max);
				if(msg->ranges[j]>=msg->range_min && msg->ranges[j]<=msg->range_max) {
					avg_dist += msg->ranges[j];
					++num;
				}
			}
			if(num>0) avg_dist /= num;
			else avg_dist = msg->range_min;
			
			std::cout<<avg_dist<<" "<<msg->range_max<<" "<<msg->range_min<<" "<<std::endl;
			
			float scaled = (avg_dist-msg->range_min)/(msg->range_max-msg->range_min+0.001f);
			assert(scaled>=0 && scaled<1);
			id.id |= ((uint32_t)(scaled*std::pow(2, feature_depth_)))<<(i*feature_depth_);
		}
		
		ft.infos.push_back(id);
		pub_feature_.publish(ft);
	}
	
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "scan2feature");
    
	MainNode mn;
	
	ros::spin();
	
	return 0;
}
