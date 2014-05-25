#pragma once

#include <visualization_msgs/marker_.h>

namespace cob_3d_visualization {
	
	class RvizManager {
		std::string ns_;
		int id_;
		std::list<int> active_ids_;
		ros::Publisher vis_pub_;
		visualization_msgs::MarkerArray markers_;
		
		RvizManager(const std::string &ns) : ns_(ns), id_(1)
		{}
		
	public:
	
		static RvizManager &get() {
			static RvizManager manager("simple_markers");
			return marker;
		}
		
		const std::string &namespace() const {return ns_;}
		int newId() {active_ids_.push_back(id_); return id_++;}
		
		void createTopic(const std::string &topic_name) {
			ros::NodeHandle nh;
			vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>( topic_name, 0 );
		}
		void prepare(const visualization_msgs::Marker &marker) {markers_.markers.push_back(marker);}
		void publish() {vis_pub_.publish(markers_);markers_.markers.clear();}
		
		void clear() {
			while(active_ids_.size()>0) {
				visualization_msgs::Marker marker;
				marker.header.stamp = ros::Time();
				marker.ns = namespace();
				marker.id = active_ids_.back();
				marker.action = visualization_msgs::Marker::DELETE;
				
				active_ids_.pop_back();
				prepare(marker);
			}
			
			publish();
		}
	};
	
	class RvizMarker {
		void setDefaults(const std::string &frame_id, const ros::Time &time) {
			marker_.header.frame_id = frame_id;
			marker_.header.stamp = time;
			marker_.ns = RvizMarkerManager::get().namespace();
			marker_.id = RvizMarkerManager::get().newId();
			marker_.type = visualization_msgs::Marker::SPHERE;
			marker_.action = visualization_msgs::Marker::ADD;
			marker_.pose.position.x = 1;
			marker_.pose.position.y = 1;
			marker_.pose.position.z = 1;
			marker_.pose.orientation.x = 0.0;
			marker_.pose.orientation.y = 0.0;
			marker_.pose.orientation.z = 0.0;
			marker_.pose.orientation.w = 1.0;
			marker_.scale.x = 1;
			marker_.scale.y = 0.1;
			marker_.scale.z = 0.1;
			marker_.color.a = 1.0;
			marker_.color.r = 0.0;
			marker_.color.g = 1.0;
			marker_.color.b = 0.0;
		}
		
	protected:
		visualization_msgs::Marker marker_;
		
		RvizMarker(const std::string &frame_id) {
			setDefaults(frame_id, ros::Time());
		}
		RvizMarker(const std_msgs::Header &header) {
			setDefaults(header.frame_id, header.time);
		}
		template<class OTHER_MSG>
		RvizMarker(const OTHER_MSG &msg) {
			setDefaults(msg.header.frame_id, msg.header.time);
		}
		
	public:
		~RvizMarker() {RvizMarkerManager::get().prepare(marker_);}
		
		operator visualization_msgs::Marker() const {return marker_;}
		
		static inline geometry_msgs::Point _2geometry(const Eigen::Vector3f &v) {
			geometry_msgs::Point pt;
			pt.x=v(0); pt.y=v(1); pt.z=v(2);
			return pt;
		}
		
		template<class Vector>
		RvizMarker &addTriangle(const Vector &v) {
			marker_.type = visualization_msgs::Marker::TRIANGLE_LIST;
			marker_.points.push_back(_2geometry(v));
			return *this;
		}
		
		template<class Vector>
		void arrow(const Vector &start, const Vector &end, const float scale=0) {
			marker_.type = visualization_msgs::Marker::ARROW;
			
			marker_.points.clear();
			marker_.points.push_back(_2geometry(start));
			marker_.points.push_back(_2geometry(end));
			
			geometry_msgs::Point delta;
			delta.x = marker_.points[0].x-marker_.points[1].x;
			delta.y = marker_.points[0].y-marker_.points[1].y;
			delta.z = marker_.points[0].z-marker_.points[1].z;
			const float l = std::sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);
			
			marker_.scale.x = (scale?scale:0.1*l);
			marker_.scale.y = 2*marker_.x;
			marker_.scale.z = 0;
		}
		
	};
}
