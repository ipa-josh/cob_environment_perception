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
 * \date Date of creation: 08/2012
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

#ifndef COB_3D_MAPPING_TOOLS_GUI_TOOLS_H_
#define COB_3D_MAPPING_TOOLS_GUI_TOOLS_H_

#include <highgui.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/function.hpp>

#include "cob_3d_mapping_tools/gui/types.h"

namespace Gui
{
  namespace Tools
  {
    template<typename T>
      inline T min3(const T& a, const T& b, const T& c) { return ( a < b ? std::min<T>(a,c) : std::min<T>(b,c) ); }
    template<typename T>
      inline T max3(const T& a, const T& b, const T& c) { return ( a > b ? std::max<T>(a,c) : std::max<T>(b,c) ); }

    class DominantColor
    {
      private:
      enum { HIST_SIZE = 180 }; // uint8_t limits hue to 0..255

      public:
      DominantColor() : sum_colors_(0), sum_sat_(0), sum_val_(0), hue_histogram_(HIST_SIZE,0), sat_values_(HIST_SIZE,0)
      { }

      ~DominantColor() { }

      void addColor(uint8_t r, uint8_t g, uint8_t b)
      {
        int h,s,v;
        rgb2hsv(r,g,b,h,s,v);
        int pos = incrBin(h);
        sat_values_[pos] += s;
        sum_sat_ += s;
        sum_val_ += v;
        ++sum_colors_;
      }

      inline void getColor(uint8_t& r, uint8_t& g, uint8_t& b) const
      {
        if(!sum_colors_) { r=0; g=0; b=0; return; }
        int pos = getMaxBin();
        hsv2rgb( round(pos*bin_size+bin_center), sat_values_[pos]/hue_histogram_[pos], sum_val_/sum_colors_, r,g,b );
      }

      inline int incrBin(int h)
      {
        int bin = h*inv_bin_size;
        std::cout << "incBin:"<<bin<<" H:"<<(int)h<<"inv_bin"<<inv_bin_size<<std::endl;
        hue_histogram_[(size_t)bin] += 1;
        return bin;
      }

      inline int getMaxBin() const
      {
        int max_bin=0, max_size=0;
        for(int i=0;i<HIST_SIZE;++i)
        {
          if (hue_histogram_[i] >= max_size)
          {
            max_size = hue_histogram_[i];
            max_bin = i;
          }
        }
        std::cout<<"max_(bin/size): "<<max_bin<<"/"<<max_size<<std::endl;
        return max_bin;
      }

      inline void rgb2hsv(uint8_t r, uint8_t g, uint8_t b, int& h, int& s, int& v) const
      {
        int rgb_min = min3(r,g,b);
        int rgb_max = max3(r,g,b);
        int delta = rgb_max - rgb_min;
        v = rgb_max;
        if (v == 0) { s = h = 0; return; }
        s = round(float(delta) / float(v) * 100.0f);
        v /= 2.55f;
        float h_tmp;
        if (s == 0) { h = 0; return; }
        if      ((int)r == rgb_max)   h_tmp =     (float(g - b)) / (float(delta));
        else if ((int)g == rgb_max)   h_tmp = 2.0f + (float(b - r)) / (float(delta));
        else                          h_tmp = 4.0f + (float(r - g)) / (float(delta));

        h = h_tmp * 60.0f;
        if(h<0) h+=360;
      }

      inline void hsv2rgb(int h, int s, int v, uint8_t& r, uint8_t& g, uint8_t& b) const
      {
        if (s == 0) { r = g = b = v; return; }

        float hh = h / 60.0f; // sector 0..5
        int i = floor(hh);
        float f = hh - i;
        v = round(v*2.55f);
        int p = v * (100 - s) * 0.01f;
        int q = v * (100 - s * f) * 0.01f;
        int t = v * (100 - s * (1.0f - f)) * 0.01f;

        switch(i)
        {
        case 0:
        { r = v; g = t; b = p; break; }
        case 1:
        { r = q; g = v; b = p; break; }
        case 2:
        { r = p; g = v; b = t; break; }
        case 3:
        { r = p; g = q; b = v; break; }
        case 4:
        { r = t; g = p; b = v; break; }
        default:// case 5:
        { r = v; g = p; b = q; break; }
        }
      }

      private:
      int sum_colors_;
      int sum_sat_;
      int sum_val_;
      std::vector<int> hue_histogram_;
      std::vector<int> sat_values_;

      static const float inv_bin_size = 1.0f / 360.0f * HIST_SIZE;
      static const float bin_size = 360.0f / HIST_SIZE;
      static const float bin_center = ((360.0f / HIST_SIZE) - 1.0f) * 0.5f;
    };


    uint32_t getGradientColor(float, float, float, cv::Vec3b&);
    uint32_t getGradientColor(double, cv::Vec3b&);

    /* --- converter base class --- */
    template<typename PointT>
    class PointConverterBase
    {
      public:
      PointConverterBase(const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
        : cloud_(cloud) { }
      virtual ~PointConverterBase() { }

      virtual cv::Vec3b operator() (int)=0;

      protected:
      typename pcl::PointCloud<PointT>::ConstPtr cloud_;
    };

    /* --- RGB converter --- */
    template<typename PointT>
    class PointConverterRGB : public PointConverterBase<PointT>
    {
      public:
      PointConverterRGB(const typename pcl::PointCloud<PointT>::ConstPtr& cloud) : PointConverterBase<PointT>(cloud) { }

      cv::Vec3b operator() (int idx)
      {
        cv::Vec3b rgb;
        rgb[0] = (*this->cloud_)[idx].b;
        rgb[1] = (*this->cloud_)[idx].g;
        rgb[2] = (*this->cloud_)[idx].r;
        return rgb;
      }
    };

    /* --- Depth converter --- */
    template<typename PointT>
    class PointConverterDepth : public PointConverterBase<PointT>
    {
      public:
      PointConverterDepth(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, float z_min, float z_max)
        : PointConverterBase<PointT>(cloud), z_min_(z_min), z_max_(z_max)
      { }

      cv::Vec3b operator() (int idx)
      {
        cv::Vec3b bgr;
        getGradientColor((*this->cloud_)[idx].z, z_min_, z_max_, bgr);
        return bgr;
      }

      private:
      float z_min_;
      float z_max_;
    };

    template<typename PointT>
    void convertPointCloud2Cv(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, cvImagePtr& image,
                                boost::function<cv::Vec3b (int)>& converter);

    template<typename PointT>
    void getMinMaxZ(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, float& z_min, float& z_max);
  }
}

#endif
