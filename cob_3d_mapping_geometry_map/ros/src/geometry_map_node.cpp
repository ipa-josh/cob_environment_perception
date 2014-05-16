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
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_mapping_geometry_map
 *
 * \author
 *  Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 08/2011
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

// external includes
#include <cob_3d_mapping_common/ros_msg_conversions.h>

// internal includes
#include <cob_3d_mapping_geometry_map/geometry_map_nodeConfig.h>
#include <cob_3d_mapping_geometry_map/geometry_map_node.h>

using namespace cob_3d_mapping;

GeometryMapNode::GeometryMapNode()
: map_frame_id_("/map")
{
  config_server_.setCallback(boost::bind(&GeometryMapNode::dynReconfCallback, this, _1, _2));
  shape_sub_ = n_.subscribe("shape_array_in", 10, &GeometryMapNode::shapeCallback, this);
  map_pub_ = n_.advertise<cob_3d_mapping_msgs::ShapeArray>("map",1);
  clear_map_server_ = n_.advertiseService("clear_map", &GeometryMapNode::clearMap, this);
  get_map_server_ = n_.advertiseService("get_map", &GeometryMapNode::getMap, this);
  set_map_server_ = n_.advertiseService("set_map", &GeometryMapNode::setMap, this);
  modify_map_server_ = n_.advertiseService("modify_map", &GeometryMapNode::modifyMap, this);
}

void
GeometryMapNode::dynReconfCallback(cob_3d_mapping_geometry_map::geometry_map_nodeConfig &config, uint32_t level)
{
  ROS_INFO("[geometry_map]: received new parameters");
  geometry_map_.setSaveToFile( config.save_to_file );
  geometry_map_.setFilePath(config.file_path);
  geometry_map_.setMergeThresholds(config.cos_angle, config.d);
  map_frame_id_ = config.map_frame_id;
  //enable_tf_ = config.enable_tf;
  enable_cyl_= config.enable_cyl;
  enable_poly_=config.enable_poly;
}

void
GeometryMapNode::shapeCallback(const cob_3d_mapping_msgs::ShapeArray::ConstPtr& sa)
{
  ROS_INFO("Adding %d new shapes", (int)sa->shapes.size());
  if(sa->header.frame_id != map_frame_id_)
  {
    ROS_ERROR("Frame ID of incoming shape array does not match map frame ID, aborting...");
    return;
  }

  std::vector<Polygon::Ptr> polygon_list;
  std::vector<Cylinder::Ptr> cylinder_list;

  for(size_t i=0; i<sa->shapes.size(); ++i)
  {
    switch (sa->shapes[i].type)
    {
      case cob_3d_mapping_msgs::Shape::POLYGON:
      {
        if(!enable_poly_) continue;
        Polygon::Ptr p(new Polygon);
        fromROSMsg(sa->shapes[i], *p);
        geometry_map_.addMapEntry(p);
        break;
      }
      case cob_3d_mapping_msgs::Shape::CYLINDER:
      {
        if(!enable_cyl_) continue;
        Cylinder::Ptr c(new Cylinder);
        fromROSMsg(sa->shapes[i], *c);
        geometry_map_.addMapEntry(c);
        break;
      }
      default:
        break;
    }
  }

  geometry_map_.cleanUp();
  geometry_map_.incrFrame();

  publishMap();
}

bool
GeometryMapNode::clearMap(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
{
  ROS_INFO("Clearing geometry map...");
  geometry_map_.clearMap();
  cob_3d_mapping_msgs::ShapeArray map_msg;
  map_msg.header.frame_id = map_frame_id_;
  map_msg.header.stamp = ros::Time::now();
  map_pub_.publish(map_msg);
  return true;
}

bool
GeometryMapNode::getMap(cob_3d_mapping_msgs::GetGeometryMap::Request &req, cob_3d_mapping_msgs::GetGeometryMap::Response &res)
{
  std::vector<Polygon::Ptr>* map_polygon = geometry_map_.getMapPolygon();
  std::vector<Polygon::Ptr>* map_cylinder = geometry_map_.getMapCylinder();

  res.map.header.stamp = ros::Time::now();
  res.map.header.frame_id = map_frame_id_;
  for(unsigned int i = 0; i < map_polygon->size(); i++)
  {
    Polygon& sm = *(map_polygon->at(i));
    cob_3d_mapping_msgs::Shape s;
    toROSMsg(sm,s);
    s.header = res.map.header;
    res.map.shapes.push_back(s);
  }

  for(unsigned int i = 0; i < map_cylinder->size(); i++)
  {
    Cylinder& sm = *(boost::dynamic_pointer_cast<Cylinder>(map_cylinder->at(i)));
    cob_3d_mapping_msgs::Shape s;
    toROSMsg(sm,s);
    s.header = res.map.header;
    res.map.shapes.push_back(s);
  }

  return true;
}

bool
GeometryMapNode::setMap(cob_3d_mapping_msgs::SetGeometryMap::Request &req, cob_3d_mapping_msgs::SetGeometryMap::Response &res)
{
  ROS_INFO("Setting map with %d shapes.", (int)req.map.shapes.size());
  geometry_map_.clearMap();
  std::vector<Polygon::Ptr>* map_polygon = geometry_map_.getMapPolygon();
  std::vector<Polygon::Ptr>* map_cylinder = geometry_map_.getMapCylinder();

  for(unsigned int i = 0; i < req.map.shapes.size(); i++)
  {
    if(req.map.shapes[i].type == cob_3d_mapping_msgs::Shape::POLYGON)
    {
      Polygon::Ptr p(new Polygon);
      fromROSMsg(req.map.shapes[i],*p);
      p->merged_ = 9;
      map_polygon->push_back(p);
    }
    else if(req.map.shapes[i].type == cob_3d_mapping_msgs::Shape::CYLINDER)
    {
      Cylinder::Ptr c(new Cylinder);
      fromROSMsg(req.map.shapes[i],*c);
      c->merged_ = 9;
      map_cylinder->push_back(c);
    }
  }
  publishMap();
  return true;
}

//TODO: refactor & test
bool
GeometryMapNode::modifyMap(cob_3d_mapping_msgs::ModifyMap::Request &req,
    cob_3d_mapping_msgs::ModifyMap::Response &res) {

  ROS_INFO ("Modifying the map...") ;
  std::vector<Polygon::Ptr>* map_polygon = geometry_map_.getMapPolygon();

  if(req.action == cob_3d_mapping_msgs::ModifyMapRequest::MODIFY)
  {
    for(unsigned int i = 0; i < map_polygon->size(); i++)
    {
      Polygon& pm = *(map_polygon->at(i));
      for(unsigned int j = 0; j < req.shapes.shapes.size(); j++)
      {
        if (pm.id_ == req.shapes.shapes[j].id)
        {
          Polygon poly;
          fromROSMsg(req.shapes.shapes[j], poly);
          //TODO: only apply transformation, using this, not all data is preserved
          *(map_polygon->at(i)) = poly;
          break;
        }
      }
    }
    publishMap();
  }
  if(req.action == cob_3d_mapping_msgs::ModifyMapRequest::DELETE)
  {
    for(unsigned int i = 0; i < map_polygon->size(); i++)
    {
      Polygon& pm = *(map_polygon->at(i));
      for(unsigned int j = 0; j < req.shapes.shapes.size(); j++)
      {
        if (pm.id_ == req.shapes.shapes[j].id)
        {
          Polygon poly;
          fromROSMsg(req.shapes.shapes[j], poly);
          //TODO: better implement a deleteEntry funtion in GeometryMap
          map_polygon->erase(map_polygon->begin()+i);
          break;
        }
      }
    }
    publishMap();
  }
  return true;
}


void
GeometryMapNode::publishMap()
{
  std::vector<Polygon::Ptr>* map_polygon = geometry_map_.getMapPolygon();
  std::vector<Polygon::Ptr>* map_cylinder = geometry_map_.getMapCylinder();
  ROS_INFO("Map polygon Size : %d", (int)map_polygon->size()) ;
  ROS_INFO("Map cylinder Size : %d", (int)map_cylinder->size()) ;

  geometry_map_.colorizeMap();
  cob_3d_mapping_msgs::ShapeArray map_msg;
  map_msg.header.frame_id = map_frame_id_;
  map_msg.header.stamp = ros::Time::now();

  for(unsigned int i = 0; i < map_polygon->size(); i++)
  {
    Polygon& sm = *(map_polygon->at(i));
    cob_3d_mapping_msgs::Shape s;
    toROSMsg(sm, s);
    s.header = map_msg.header;
    map_msg.shapes.push_back(s);
  }
  for(unsigned int i = 0; i < map_cylinder->size(); i++)
  {
    Cylinder& sm = *(boost::dynamic_pointer_cast<Cylinder>(map_cylinder->at(i)));
    cob_3d_mapping_msgs::Shape s;
    toROSMsg(sm, s);
    s.header = map_msg.header;
    map_msg.shapes.push_back(s);
  }
  map_pub_.publish(map_msg);
}

/*void
GeometryMapNode::fillMarker(Polygon::Ptr p, visualization_msgs::Marker& m, visualization_msgs::Marker& m_t)
{ ROS_DEBUG( "not implemented yet" ); }

void
GeometryMapNode::fillMarker(Cylinder::Ptr c, visualization_msgs::Marker& m, visualization_msgs::Marker& m_t)
{ ROS_DEBUG( "not implemented yet" ); }

void
GeometryMapNode::fillMarker(ShapeCluster::Ptr sc, visualization_msgs::Marker& m, visualization_msgs::Marker& m_t)
{
  m.action = visualization_msgs::Marker::ADD;
  m.type = visualization_msgs::Marker::CUBE;
  m.lifetime = ros::Duration();
  m.header.frame_id = map_frame_id_;

  m.pose.position.x = sc->centroid(0);
  m.pose.position.y = sc->centroid(1);
  m.pose.position.z = sc->centroid(2);
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.scale.x = sc->scale(0);
  m.scale.y = sc->scale(1);
  m.scale.z = sc->scale(2);
  m.color.r = 0.95;
  m.color.g = 0.95;
  m.color.b = 0.95;
  m.color.a = 0.5;
}

void GeometryMapNode::publishPrimitives()
{
  // initialize marker of type cylinder
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.lifetime = ros::Duration();
  marker.header.frame_id = map_frame_id_;

  // get cylinder map array
  std::vector<Cylinder::Ptr>* map_cylinder = geometry_map_.getMap_cylinder();


  int ctr=0;
  int t_ctr=2000;

  for(unsigned int i=0; i<map_cylinder->size(); i++)
  {
    Cylinder& cm = *(map_cylinder->at(i));
    marker.id = cm.id;

    //set primitive color to color of shape
    marker.color.r=cm.color[0];
    marker.color.g=cm.color[1];
    marker.color.b=cm.color[2];
    marker.color.a = 0.8;

    //compute orientation quaternion

    Eigen::Vector3f z_axis=cm.sym_axis;
    //Eigen::Vector3f y_axis= z_axis.unitOrthogonal();
    Eigen::Vector3f y_axis= cm.normal;

    Eigen::Affine3f rot;

    pcl::getTransformationFromTwoUnitVectors(y_axis,z_axis,rot);
    //cm.transform_from_world_to_plane.rotation().eulerAngles(0,1,2);
    Eigen::Vector3f euler = rot.rotation().eulerAngles(0,1,2);
    tf::Quaternion orientation= tf::createQuaternionFromRPY(euler(0),euler(1),euler(2));

    //set cylinder orientation
    marker.pose.orientation.x = orientation[0];
    marker.pose.orientation.y = orientation[1];
    marker.pose.orientation.z = orientation[2];
    marker.pose.orientation.w = orientation[3];
    //set cylinder position
    marker.pose.position.x = cm.origin_[0];
    marker.pose.position.y = cm.origin_[1];
    marker.pose.position.z = cm.origin_[2];
    // TODO<<<<WATCH OUT<<<<< presentation configuration - hard coded limits >>>>>>>>>>>>>>>>>>
    marker.pose.position.z = cm.origin_[2]-0.03;



    //set shape of cylinder
    marker.scale.x = cm.r_ *2;
    marker.scale.y = cm.r_ *2;
    marker.scale.z =  (cm.h_max_ - cm.h_min_);


    marker.id = t_ctr;
    std::stringstream ss;
    ss << ctr;
    marker.text = ss.str();
    ctr++;
    t_ctr++;

    primitive_pub_.publish(marker);

  }
}*/

int main (int argc, char** argv)
{
  ros::init (argc, argv, "geometry_map_node");

  GeometryMapNode gmn;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce ();
    loop_rate.sleep();
  }
}

