/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_environment_perception
 * \note
 *  ROS package name: cob_3d_mapping_point_map
 *
 * \author
 *  Author: Joshua Hampp, email:joshua.hampp@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 09/2011
 *
 * \brief
 * Description:
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// ROS includes
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// ROS message includes
#include <cob_3d_mapping_msgs/SetPointMap.h>

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>


int main (int argc, char **argv)
{
  if(argc<1) {
    ROS_ERROR("Please specify path to map file\nrosrun cob_env_model set_point_map myfile.pcd");
    return -1;
  }
  ros::init(argc, argv, "set_point_map");

  ros::NodeHandle nh;

  ROS_INFO("Waiting for service server to start.");
  // wait for the server to start
  ros::service::waitForService("/point_map/set_map"); //will wait for infinite time

  ROS_INFO("Server started, sending map.");

  //create message
  cob_3d_mapping_msgs::SetPointMapRequest req;

  pcl::PointCloud<pcl::PointXYZRGB> map;
  if(pcl::io::loadPCDFile(argv[1], map)!=0) {
    ROS_ERROR("Couldn't open pcd file. Sorry.");
    return -1;
  }

  pcl::PCLPointCloud2 map2;
  pcl::toPCLPointCloud2(map, map2);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(map2, cloud_msg);
  //pcl::toROSMsg(map,req.map);
  req.map.header.frame_id ="/map";
  cob_3d_mapping_msgs::SetPointMapResponse resp;

  if (ros::service::call("/point_map/set_map", req,resp))
  {
    ROS_INFO("Service call finished.");
  }
  else
    ROS_INFO("[set point map]: Service call failed.");

  return 0;
}

