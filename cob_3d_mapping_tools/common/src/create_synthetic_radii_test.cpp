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
 *  ROS package name: cob_3d_mapping_tools
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 11/2011
 *
 * \brief
 * Description:
 *
 * ToDo:
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

#include "cob_3d_mapping_tools/point_generator.h"
#include <cob_3d_mapping_common/label_defines.h>

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <math.h>

using namespace std;
using namespace pcl;

typedef visualization::PointCloudColorHandlerRGBField<PointXYZRGBA> ColorHdlRGBA;
typedef Eigen::Vector3f Vec;

string file;
bool visualize;
string folder;
float noise;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("out,o", value<string>(&file)->default_value(""),
     "output file, set to \"\" for no output")
    ("noise,n", value<float>(&noise)->default_value(0.00f),
     "gaussian noise level")
    ("vis,v", value<bool>(&visualize)->default_value(true), 
     "enable visualization")
    ("folder,f", value<string>(&folder)->default_value(""), 
     "save each form separated, ignores -o")
    ;

  positional_options_description p_opt;
  p_opt.add("out", 1);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help"))
  {
    cout << "creates a synthetic scene consisting of spheres and cylinders with various sizes. "
	 << "The sizes are hardcoded." << endl;
    cout << options << endl;
    exit(0);
  }
}

/*! 
 * @brief converts a float to string with a defined precision.
 */
string fl2label(const float & f, const size_t & precision)
{
  if (f == 0) 
    return string(precision, '0');

  stringstream ss;
  ss << f;
  string s = ss.str();
  s = s.substr(2,s.length());
  if (precision > s.length())
    s += string(precision - s.length(), '0');
  return (s);
}

/*!
 * @brief creates a synthetic scene consisting of spheres and cylinders with various sizes
 */
int main(int argc, char** argv)
{
  readOptions(argc, argv);

  float step = 0.005f;
  float tmp[] = {0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.07f, 0.08f, 0.09f, 0.10f, 
		 0.11f, 0.12f, 0.13f, 0.14f, 0.15f};
  vector<float> radii(tmp, tmp + sizeof(tmp) / sizeof(float));
  vector<PointCloud<PointXYZRGBA>::Ptr > pcs;
  vector<PointCloud<PointXYZRGBA>::Ptr > pcc;
  PointCloud<PointXYZRGBA>::Ptr pc_ptr;
  PointCloud<PointXYZRGBA>::Ptr scene(new PointCloud<PointXYZRGBA>);

  for (size_t i=0; i<radii.size(); i++)
  {
    pc_ptr.reset(new PointCloud<PointXYZRGBA>);
    pcs.push_back(pc_ptr);
    pc_ptr.reset(new PointCloud<PointXYZRGBA>);
    pcc.push_back(pc_ptr);
  }

  cob_3d_mapping_tools::PointGenerator<PointXYZRGBA> pg;
  if (noise != 0) pg.setGaussianNoise(noise);

  Vec pos_s(-0.2f,0.0f,1.0f);
  Vec pos_c(0.2f,0.0f,1.0f);
  for (size_t i=0; i<radii.size(); i++)
  {
    pos_s = pos_s + Vec( -(radii[i] - radii[i-1]), 0.0f, radii[i] + 0.05f);
    pg.setOutputCloud(pcs[i]);
    pg.generateSphere(step, pos_s, -pos_s, radii[i], 0.45 * M_PI);
    *scene += *pcs[i];

    pos_c = pos_c + Vec(0.0f, 0.0f, radii[i] + 0.05f);
    pg.setOutputCloud(pcc[i]);
    Vec dir = pos_c.cross(Vec::UnitY());
    dir = 2 * radii[i] * dir.normalized();
    pg.generateCylinder(step, pos_c, -dir, -Vec::UnitY() * radii[i], M_PI);
    *scene += *pcc[i];
  }
  pc_ptr.reset(new PointCloud<PointXYZRGBA>);
  pg.setOutputCloud(pc_ptr);
  pg.generateEdge(step, Vec(0.1f,0.0f,0.9f), -Vec::UnitX() * 0.2f, Vec::UnitZ() * 0.075f);
  *scene += *pc_ptr;

  if (folder != "")
  {
    for (size_t i=0; i<radii.size(); i++)
    {
      io::savePCDFileASCII(folder + "cyl_" + fl2label(radii[i], 2) + "r.pcd", *pcc[i]);
      io::savePCDFileASCII(folder + "sph_" + fl2label(radii[i], 2) + "r.pcd", *pcs[i]);
    }
    io::savePCDFileASCII(folder + "edge_.pcd", *pc_ptr);
    cout << "Saved " << 2*radii.size() << " Point Clouds to: " << folder << endl;
  }
  else if (file != "")
  {
    io::savePCDFileASCII(file, *scene);
    cout << "Saved Point Cloud to: " << file << endl;
  }

  if (visualize)
  {
    visualization::PCLVisualizer v;
    v.setBackgroundColor(255, 255, 255);
    v.addCoordinateSystem(0.05);
    ColorHdlRGBA hdl(scene);
    v.addPointCloud<PointXYZRGBA>(scene, hdl, "cloud");

    while(!v.wasStopped())
    {
      v.spinOnce(100);
      usleep(100000);
    }
  }
  else
  {
    cout << "Visualization disabled." << endl;
  }
  return 0;

}
