/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: bag_to_pcd.cpp 34671 2010-12-12 01:28:36Z rusu $
 *
 */

/**

\author Radu Bogdan Rusu

@b bag_to_pcd is a simple node that reads in a BAG file and saves all the PointCloud messages to disk in PCD (Point
Cloud Data) format.

 **/

#include <sstream>
#include <boost/filesystem.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/conversions.h>
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/conversions.h"
#include "pcl_ros/transforms.h"
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

typedef sensor_msgs::PointCloud2 PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "bag_to_pcd");
  if (argc < 5)
  {
    std::cerr << "Syntax is: " << argv[0] << " <file_in.bag> <topic> <output_directory> <target_frame>" << std::endl;
    std::cerr << "Example: " << argv[0] << " data.bag /laser_tilt_cloud ./pointclouds /base_link" << std::endl;
    return (-1);
  }

  // TF
  tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_broadcaster;

  rosbag::Bag bag;
  rosbag::View view;
  rosbag::View::iterator view_it;

  try
  {
    bag.open (argv[1], rosbag::bagmode::Read);
  } 
  catch (rosbag::BagException) 
  {
    std::cerr << "Error opening file " << argv[1] << std::endl;
    return (-1);
  }

  //view.addQuery (bag, rosbag::TypeQuery ("sensor_msgs/PointCloud2"));
  view.addQuery (bag, rosbag::TopicQuery(argv[2]));
  view.addQuery (bag, rosbag::TypeQuery ("tf/tfMessage"));
  view_it = view.begin ();

  std::string output_dir = std::string (argv[3]);
  boost::filesystem::path outpath (output_dir);
  if (!boost::filesystem::exists (outpath))
  {
    if (!boost::filesystem::create_directories (outpath))
    {
      std::cerr << "Error creating directory " << output_dir << std::endl;
      return (-1);
    }
    std::cerr << "Creating directory " << output_dir << std::endl;
  }

  // Add the PointCloud2 handler
  std::cerr << "Saving recorded sensor_msgs::PointCloud2 messages on topic " << argv[2] << " to " << output_dir << std::endl;

  PointCloud cloud_t;
  ros::Duration r (0.001);
  // Loop over the whole bag file
  while (view_it != view.end ())
  {
  	try{
		// Handle TF messages first
		tf::tfMessage::ConstPtr tf = view_it->instantiate<tf::tfMessage> ();
		if (tf != NULL)
		{
		  tf_broadcaster.sendTransform (tf->transforms);
		  ros::spinOnce ();
		  r.sleep ();
		}
		else
		{
		  // Get the PointCloud2 message
		  PointCloudConstPtr cloud = view_it->instantiate<PointCloud> ();
		  if (cloud == NULL)
		  {
			++view_it;
			continue;
		  }
		  // Transform it
		  tf::StampedTransform transform;
			//if(argv[4] != cloud->header.frame_id)
		  	tf_listener.lookupTransform(cloud->header.frame_id, argv[4], cloud->header.stamp, transform);
		  Eigen::Matrix4f out_mat;
		  pcl_ros::transformAsMatrix (transform, out_mat);
		  //pcl_ros::transformPointCloud ("/head_axis_link", *cloud, cloud_t, tf_listener);
		  pcl_ros::transformPointCloud (out_mat, *cloud, cloud_t);
                  pcl::PCLPointCloud2 cloud_t2;
                  pcl_conversions::toPCL(cloud_t, cloud_t2);

		  std::cerr << "Got " << cloud_t2.width * cloud_t2.height << " data points in frame " << cloud_t2.header.frame_id << " with the following fields: " << pcl::getFieldsList (cloud_t2) << std::endl;

		  std::stringstream ss;
		  ss << output_dir << "/" << cloud_t2.header.stamp << ".pcd";
		  std::cerr << "Data saved to " << ss.str () << std::endl;
		  pcl::io::savePCDFile (ss.str (), cloud_t2, Eigen::Vector4f::Zero (),
								Eigen::Quaternionf::Identity (), false);
		}
  	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
    // Increment the iterator
    ++view_it;
  }

  return (0);
}
/* ]--- */
