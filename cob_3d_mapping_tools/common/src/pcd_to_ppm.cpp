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

// Boost:
#include <boost/program_options.hpp>
// PCL:
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "cob_3d_mapping_common/io.h"

using namespace std;
using namespace pcl;

vector<string> file_o(2, "");
string file_i = "";

float min_z, max_z;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in", value<string>(&file_i), "input pcd file")
    ("out", value< vector<string> >(&file_o), "output files, first rgb, [second depth]")
    ("min,m", value<float>(&min_z)->default_value(FLT_MAX), "min value of depth range")
    ("max,M", value<float>(&max_z)->default_value(FLT_MIN), "max value of depth range")
    ;

  positional_options_description p_opt;
  p_opt.add("in", 1).add("out", 2);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help"))
  {
    cout << "Reads a point cloud and extracts color and depth values as .ppm image" << endl;
    cout << options << endl;
    exit(0);
  }
  if (file_i == "")
  {
    cout << "no input and output file defined " << endl << options << endl;
    exit(0);
  }
  if (file_o[0] == "")
  {
    cout << "no output file defined " << endl << options << endl;
    exit(0);
  }
}

/*! 
 * @brief Reads a point cloud and extracts color and depth values as .ppm image 
 */
int main(int argc, char** argv)
{
  readOptions(argc, argv);
  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>());

  PCDReader r;
  if (r.read(file_i, *p) == -1) return(0);
  
  cout << "Loaded pointcloud with " << p->width << " x " << p->height << " points." << endl;

  cob_3d_mapping_common::PPMWriter w;
  if (w.writeRGB(file_o[0], *p) == -1) return(0);
  cout << "Extracted RGB image..." << endl;

  if (file_o[1] != "")
  {
    if (min_z != FLT_MAX)
      w.setMinZ(min_z);
    if (max_z != FLT_MIN)
      w.setMaxZ(max_z);

    if (w.writeDepth(file_o[1], *p) == -1) return(0);
    if (w.writeDepthLinear(file_o[1] + "_linear", *p) == -1) return(0);
    cout << "Extracted depth image..." << endl;
  }

  cout << "Done!" << endl;

  return(0);
}
