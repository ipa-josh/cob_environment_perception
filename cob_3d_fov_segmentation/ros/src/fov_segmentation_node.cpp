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
 *  ROS package name: cob_3d_fov_segmentation
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 02/2013
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
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

//##################
//#### includes ####
// ROS includes
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <cob_3d_fov_segmentation/fov_segmentationConfig.h>

// internal includes
#include <cob_3d_mapping_common/ros_msg_conversions.h>
#include "cob_3d_fov_segmentation/fov_segmentation_node.h"

using namespace tf;
using namespace cob_3d_mapping;

FOVSegmentationNode::FOVSegmentationNode ()
{
  config_server_.setCallback (boost::bind (&FOVSegmentationNode::dynReconfCallback, this, _1, _2));
  shape_sub_ = n_.subscribe ("shape_array_in", 10, &FOVSegmentationNode::shapeCallback, this);
  shape_pub_ = n_.advertise<cob_3d_mapping_msgs::ShapeArray> ("shape_array_out", 1);
}

void
FOVSegmentationNode::dynReconfCallback (cob_3d_fov_segmentation::fov_segmentationConfig &config, uint32_t level)
{
  fov_.setSensorFoV_hor (config.sensor_fov_hor_angle);
  fov_.setSensorFoV_ver (config.sensor_fov_ver_angle);
  fov_.setSensorMaxRange (config.sensor_max_range);
  camera_frame_ = config.camera_frame;
  target_frame_ = config.target_frame;

  //new settings -> recalculate
  fov_.computeFieldOfView ();
}

void
FOVSegmentationNode::shapeCallback (const cob_3d_mapping_msgs::ShapeArray::ConstPtr& sa)
{
  StampedTransform st_trf;
  try
  {
    tf_listener_.waitForTransform (target_frame_, camera_frame_, ros::Time (0), ros::Duration (0.1));
    tf_listener_.lookupTransform (target_frame_, camera_frame_, ros::Time (0), st_trf);
  }
  catch (TransformException ex)
  {
    ROS_ERROR ("%s", ex.what ());
  }
  Eigen::Affine3d trafo;
  transformTFToEigen (st_trf, trafo);
  fov_.transformFOV (trafo);
  fov_seg_.setFOV (fov_);
  std::vector<Polygon::Ptr> polys;
  for (unsigned int i = 0; i < sa->shapes.size (); i++)
  {
    Polygon::Ptr p (new Polygon);
    fromROSMsg (sa->shapes[i], *p);
    polys.push_back (p);
  }
  //std::cout << polys[0]->contours[0][0] << std::endl;
  fov_seg_.setShapeArray (polys);
  std::vector<Polygon::Ptr> polys_out;
  fov_seg_.compute (polys_out);
  cob_3d_mapping_msgs::ShapeArray sa_out;
  sa_out.header = sa->header;
  for (unsigned int i = 0; i < polys_out.size (); i++)
  {
    cob_3d_mapping_msgs::Shape s;
    s.header = sa->header;
    toROSMsg (*polys_out[i], s);
    sa_out.shapes.push_back (s);
  }
  shape_pub_.publish (sa_out);
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "fov_segmentation_node");

  FOVSegmentationNode fov;

  ros::Rate loop_rate (10);
  while (ros::ok ())
  {
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}

