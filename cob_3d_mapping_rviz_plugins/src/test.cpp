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
 *  Project name: TODO FILL IN PROJECT NAME HERE
 * \note
 *  ROS stack name: TODO FILL IN STACK NAME HERE
 * \note
 *  ROS package name: TODO FILL IN PACKAGE NAME HERE
 *
 * \author
 *  Author: TODO FILL IN AUTHOR NAME HERE
 * \author
 *  Supervised by: TODO FILL IN CO-AUTHOR NAME(S) HERE
 *
 * \date Date of creation: TODO FILL IN DATE HERE
 *
 * \brief
 *
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
/*
 * test.cpp
 *
 *  Created on: 16.03.2012
 *      Author: josh
 */

#include "ros/ros.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>

#include <sensor_msgs/PointCloud2.h>

#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_mapping_msgs/Shape.h>

#include <ros/time.h>

#include <boost/shared_ptr.hpp>


int main(int argc, char **argv) {

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<cob_3d_mapping_msgs::ShapeArray>("shapes_array",1);

  ros::Rate loop_rate(1);

  while(ros::ok()) {
    cob_3d_mapping_msgs::ShapeArray sa;
    sa.header.frame_id="/map";

    cob_3d_mapping_msgs::Shape s;

    s.params.push_back(0);
    s.params.push_back(0);
    s.params.push_back(1);
    s.params.push_back(0);
    s.centroid.x = 1;
    s.centroid.y = 1;
    s.centroid.z = 1;
    s.header.frame_id="/map";

    s.color.r = 0;
    s.color.g = 0.71;
    s.color.b = 0;
    s.color.a = 1;

    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::PointXYZ pt;

    pt.x=0;
    pt.y=0;
    pt.z=0;
    pc.push_back(pt);

    pt.x=1;
    pt.y=0;
    pt.z=0;
    pc.push_back(pt);

    pt.x=1;
    pt.y=2;
    pt.z=0;
    pc.push_back(pt);

    pt.x=0;
    pt.y=2;
    pt.z=0;
    pc.push_back(pt);

    pt.x=0.5;
    pt.y=1;
    pt.z=0;
    pc.push_back(pt);

    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(pc,pc2);
    s.points.push_back(pc2);
    s.holes.push_back(false);

    sa.shapes.push_back(s);

    s.params[0]=1;
    s.params[1]=0;
    sa.shapes.push_back(s);

    s.params[0]=0;
    s.params[1]=1;
    sa.shapes.push_back(s);

    pub.publish(sa);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

